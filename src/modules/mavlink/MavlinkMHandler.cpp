/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#include "MavlinkMHandler.hpp"
#include "mavlink_m_profile.h"
#include "mavlink_main.h"

extern "C" {
#include <lib/crc/crc.h>
}
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static_assert(MAVLINK_MSG_ID_TRACK_IDENTITY == 53000, "unexpected MAVLink-M TRACK_IDENTITY id");
static_assert(MAVLINK_MSG_ID_TARGET_CUE == 53001, "unexpected MAVLink-M TARGET_CUE id");
static_assert(MAVLINK_MSG_ID_TARGET_HANDOVER == 53002, "unexpected MAVLink-M TARGET_HANDOVER id");
static_assert(MAVLINK_MSG_ID_PARTICIPANT_POSITION == 53003, "unexpected MAVLink-M PARTICIPANT_POSITION id");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK == 53004, "unexpected MAVLink-M ACK id");
static_assert(MAVLINK_MSG_ID_TRACK_IDENTITY_MIN_LEN == 141 && MAVLINK_MSG_ID_TRACK_IDENTITY_LEN == 141,
	      "unexpected finalized TRACK_IDENTITY layout");
static_assert(MAVLINK_MSG_ID_TARGET_CUE_MIN_LEN == 68 && MAVLINK_MSG_ID_TARGET_CUE_LEN == 68,
	      "unexpected finalized TARGET_CUE layout");
static_assert(MAVLINK_MSG_ID_TARGET_HANDOVER_MIN_LEN == 207 && MAVLINK_MSG_ID_TARGET_HANDOVER_LEN == 207,
	      "unexpected finalized TARGET_HANDOVER layout");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_MIN_LEN == 69 && MAVLINK_MSG_ID_MAVLINK_M_ACK_LEN == 69,
	      "unexpected finalized ACK layout");
static_assert(MAV_CMD_USER_1 == 31010, "unexpected owner-control command id");

namespace
{

constexpr uint64_t Unix2020Usec = 1'577'836'800ULL * 1'000'000ULL;
constexpr uint64_t MavlinkSigningEpochSeconds = 1'420'070'400ULL;
constexpr int32_t ActionReceiptOnly = 0;
constexpr int32_t ActionRepositionCurrentAltitude = 1;
constexpr int32_t ActionInterceptCueAltitude = 2;
constexpr uint16_t OwnerDecisionCommand = MAV_CMD_USER_1;
constexpr uint8_t OwnerDecisionAccept = 1;
constexpr uint8_t OwnerDecisionReject = 2;
constexpr uint32_t OwnerDecisionTaskMessage = MAVLINK_MSG_ID_TARGET_CUE;
// PX4 Commander only acknowledges MAV_CMD_DO_REPOSITION when param2 requests
// AUTO_LOITER. The MAVLink-M gate already requires AUTO_LOITER, so setting this
// flag does not broaden mode authority; it makes Commander and Navigator agree
// on the guarded one-shot command.
constexpr float RepositionChangeModeFlag = 1.f;
// MAVLink 2 strips zero-valued payload suffixes even for non-extension fields.
// Match AAGS by requiring the semantic prefix and letting the generated,
// zero-initializing decoders reconstruct optional suffixes.
constexpr uint8_t TrackIdentityRequiredWireLength =
	static_cast<uint8_t>(offsetof(mavlink_track_identity_t, origin_sysid));
constexpr uint8_t TargetCueRequiredWireLength =
	static_cast<uint8_t>(offsetof(mavlink_target_cue_t, cue_type));
constexpr uint8_t TargetHandoverRequiredWireLength =
	static_cast<uint8_t>(offsetof(mavlink_target_handover_t, confidence_score));
constexpr uint8_t OwnerDecisionRequiredWireLength =
	static_cast<uint8_t>(offsetof(mavlink_command_long_t, confirmation));
static_assert(TrackIdentityRequiredWireLength == 58, "unexpected TRACK_IDENTITY required prefix");
static_assert(TargetCueRequiredWireLength == 45, "unexpected TARGET_CUE required prefix");
static_assert(TargetHandoverRequiredWireLength == 76, "unexpected TARGET_HANDOVER required prefix");
static_assert(OwnerDecisionRequiredWireLength == 32, "unexpected COMMAND_LONG required prefix");

constexpr uint8_t sanitize_action(int32_t action)
{
	return action >= ActionReceiptOnly && action <= ActionInterceptCueAltitude
	       ? static_cast<uint8_t>(action) : ActionReceiptOnly;
}

constexpr int32_t sanitize_max_age(int32_t maximum_age_s)
{
	return maximum_age_s >= 0 && maximum_age_s <= 600 ? maximum_age_s : 30;
}

static_assert(sanitize_action(0) == 0, "receipt-only mode must remain inert");
static_assert(sanitize_action(1) == 1, "current-altitude reposition mode must remain available");
static_assert(sanitize_action(2) == 2, "explicit cue-altitude intercept mode must remain available");
static_assert(sanitize_action(3) == 0, "unknown action modes must fail closed");
static_assert(sanitize_action(15) == 0, "obsolete command bitmask must migrate fail-closed");
static_assert(sanitize_max_age(-1) == 30, "negative replay window must fail closed");
static_assert(sanitize_max_age(601) == 30, "oversized replay window must fail closed");
static_assert(sanitize_max_age(0) == 0 && sanitize_max_age(600) == 600,
	      "documented replay-window bounds must remain valid");

#ifdef PX4_STORAGEDIR
constexpr const char *StatePath = PX4_STORAGEDIR "/mavlink_m_state.bin";
constexpr const char *StateTempPath = PX4_STORAGEDIR "/.mavlink_m_state.tmp";
constexpr const char *SigningKeyPath = PX4_STORAGEDIR "/mavlink_m_signing.key";
#endif

template<typename T>
void get_parameter(param_t parameter, T &value)
{
	if (parameter != PARAM_INVALID) {
		(void)param_get(parameter, &value);
	}
}

bool finite_or_nan(float value)
{
	return PX4_ISFINITE(value) || std::isnan(value);
}

bool exact_unsigned_value(float value, uint32_t maximum, uint32_t *result)
{
	if (result == nullptr || !PX4_ISFINITE(value) || value < 0.f
	    || value > static_cast<float>(maximum)
	    || fabsf(value - roundf(value)) > 0.f) {
		return false;
	}

	*result = static_cast<uint32_t>(value);
	return true;
}

bool exact_unsigned_word(float value, uint16_t *word)
{
	uint32_t result = 0;

	if (word == nullptr || !exact_unsigned_value(value, UINT16_MAX, &result)) {
		return false;
	}

	*word = static_cast<uint16_t>(result);
	return true;
}

bool valid_target_class(uint8_t target_class)
{
	// 255 is the generated enum sentinel. Preserve forward compatibility with
	// new shared classes in the reserved gaps without treating the sentinel as
	// an operational classification.
	return target_class != UINT8_MAX;
}

bool valid_target_force(uint8_t target_force)
{
	return target_force == MAVLINK_M_TARGET_FORCE_UNKNOWN
	       || target_force == MAVLINK_M_TARGET_FORCE_NEUTRAL
	       || target_force == MAVLINK_M_TARGET_FORCE_FRIENDLY
	       || target_force == MAVLINK_M_TARGET_FORCE_FOE
	       || target_force == MAVLINK_M_TARGET_FORCE_EXTRATERRESTRIAL;
}

bool all_zero(const uint8_t *bytes, size_t length)
{
	for (size_t i = 0; i < length; ++i) {
		if (bytes[i] != 0) {
			return false;
		}
	}

	return true;
}

int hex_nibble(char value)
{
	if (value >= '0' && value <= '9') {
		return value - '0';
	}

	if (value >= 'a' && value <= 'f') {
		return value - 'a' + 10;
	}

	if (value >= 'A' && value <= 'F') {
		return value - 'A' + 10;
	}

	return -1;
}

} // namespace

MavlinkMHandler::MavlinkMHandler(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	_param_mode = param_find("MAV_M_MODE");
	_param_link_id = param_find("MAV_M_LNK_ID");
	_param_instance = param_find("MAV_M_INST");
	_param_source_system = param_find("MAV_M_SRC_SYS");
	_param_source_component = param_find("MAV_M_SRC_CMP");
	_param_control_instance = param_find("MAV_M_CTL_INST");
	_param_control_system = param_find("MAV_M_CTL_SYS");
	_param_control_component = param_find("MAV_M_CTL_CMP");
	_param_control_link_id = param_find("MAV_M_CTL_LNK");
	_param_same_endpoint = param_find("MAV_M_SAME_EP");
	_param_rc_channel = param_find("MAV_M_RC_CH");
	_param_rc_reject = param_find("MAV_M_RC_REJ");
	_param_rc_accept = param_find("MAV_M_RC_ACC");
	_param_max_age = param_find("MAV_M_MAX_AGE");
	_param_action = param_find("MAV_M_ACTION");
	update_parameters();
}

void MavlinkMHandler::configure_receiver_status(mavlink_status_t *status)
{
	_receiver_status = status;
	(void)configure_signing();
}

void MavlinkMHandler::update_parameters()
{
	const int32_t previous_mode = _mode;
	const int32_t previous_instance = _instance;
	const int32_t previous_source_system = _source_system;
	const int32_t previous_source_component = _source_component;
	const int32_t previous_control_instance = _control_instance;
	const int32_t previous_control_system = _control_system;
	const int32_t previous_control_component = _control_component;
	const int32_t previous_same_endpoint = _same_endpoint;
	const int32_t previous_rc_channel = _rc_channel;
	const int32_t previous_rc_reject = _rc_reject;
	const int32_t previous_rc_accept = _rc_accept;

	get_parameter(_param_mode, _mode);
	get_parameter(_param_link_id, _link_id);
	get_parameter(_param_instance, _instance);
	get_parameter(_param_source_system, _source_system);
	get_parameter(_param_source_component, _source_component);
	get_parameter(_param_control_instance, _control_instance);
	get_parameter(_param_control_system, _control_system);
	get_parameter(_param_control_component, _control_component);
	get_parameter(_param_control_link_id, _control_link_id);
	get_parameter(_param_same_endpoint, _same_endpoint);
	get_parameter(_param_rc_channel, _rc_channel);
	get_parameter(_param_rc_reject, _rc_reject);
	get_parameter(_param_rc_accept, _rc_accept);
	get_parameter(_param_max_age, _max_age_s);
	const int32_t requested_max_age = _max_age_s;
	_max_age_s = sanitize_max_age(requested_max_age);

	if (requested_max_age != _max_age_s && _param_max_age != PARAM_INVALID) {
		PX4_WARN("resetting invalid MAV_M_MAX_AGE=%ld to %ld seconds",
			 static_cast<long>(requested_max_age), static_cast<long>(_max_age_s));
		(void)param_set_no_notification(_param_max_age, &_max_age_s);
	}

	get_parameter(_param_action, _action_mode);
	// Fail closed when migrating a persisted value from the obsolete bitmask
	// implementation. Only an explicit documented value enables navigation.
	const int32_t requested_action = _action_mode;
	_action_mode = sanitize_action(requested_action);

	if (requested_action != _action_mode && _param_action != PARAM_INVALID) {
		PX4_WARN("resetting obsolete MAV_M_ACTION=%ld to receipt-only",
			 static_cast<long>(requested_action));
		(void)param_set_no_notification(_param_action, &_action_mode);
	}

	const bool mode_valid = _mode >= 0 && _mode <= 2;
	const bool instance_valid = _instance >= 0 && _instance < MAVLINK_COMM_NUM_BUFFERS;
	const bool source_valid = _source_system >= 1 && _source_system <= UINT8_MAX
				  && _source_component >= 1 && _source_component <= UINT8_MAX;
	const bool signing_link_valid = _mode != 2 || (_link_id >= 0 && _link_id <= UINT8_MAX);
	_endpoint_configuration_valid = mode_valid && instance_valid && source_valid && signing_link_valid;
	const bool same_endpoint_parameter_valid = _same_endpoint == 0 || _same_endpoint == 1;
	const bool same_endpoint = _same_endpoint == 1;
	const bool control_instance_valid = _control_instance >= 0
					    && _control_instance < MAVLINK_COMM_NUM_BUFFERS;
	const bool control_source_valid = _control_system >= 1 && _control_system <= UINT8_MAX
					  && _control_component >= 1 && _control_component <= UINT8_MAX;
	const bool same_route = _control_instance == _instance;
	const bool same_authority = _control_system == _source_system
				    && _control_component == _source_component;
	// SAME_EP is deliberately all-or-nothing. Partial overlap would make it
	// ambiguous which endpoint owns the durable task and its authoritative ACK.
	const bool control_endpoint_relationship_valid = same_endpoint
			? same_route && same_authority
			: !same_route && !same_authority;
	const bool control_signing_link_valid = _mode != 2
						|| (_control_link_id >= 0 && _control_link_id <= UINT8_MAX
						    && (same_endpoint
							? _control_link_id == _link_id
							: _control_link_id != _link_id));
	_control_configuration_valid = mode_valid && same_endpoint_parameter_valid
				       && control_instance_valid && control_source_valid
				       && control_endpoint_relationship_valid
				       && control_signing_link_valid;

	if (!_endpoint_configuration_valid && _mode != 0) {
		PX4_WARN("invalid MAVLink-M route/source/signing parameter; endpoint disabled");
	}

	if (!_control_configuration_valid && _mode != 0
	    && (_control_instance >= 0 || _control_system != 0)) {
		PX4_WARN("invalid MAVLink-M owner-control route/source/signing parameter; network decisions disabled");
	}

	if (previous_control_instance != _control_instance
	    || previous_control_system != _control_system
	    || previous_control_component != _control_component
	    || previous_same_endpoint != _same_endpoint) {
		_control_status = mavlink_m_target_status_s{};
		_last_control_status_send = 0;
	}

	if (previous_mode != _mode || previous_rc_channel != _rc_channel
	    || previous_rc_reject != _rc_reject || previous_rc_accept != _rc_accept) {
		// A newly selected switch/channel must establish a fresh center latch;
		// it may not inherit the old control's center-to-high transition.
		_last_rc_position = RcPosition::Unknown;
		_rc_center_latched = false;
	}

	if (_state_loaded && (previous_instance != _instance
			       || previous_source_system != _source_system
			       || previous_source_component != _source_component)) {
		_active = Assignment{};

		for (Assignment &assignment : _inbox) {
			assignment = Assignment{};
		}

		for (Assignment &assignment : _terminal) {
			assignment = Assignment{};
		}

		for (ControlRecord &control : _controls) {
			control = ControlRecord{};
		}

		for (TrackIdentity &identity : _track_identities) {
			identity = TrackIdentity{};
		}

		_state_loaded = false;
		_source_last_seen = 0;
		_last_rc_position = RcPosition::Unknown;
		_rc_center_latched = false;
	}

	(void)configure_signing();
}

bool MavlinkMHandler::enabled() const
{
	return _mavlink != nullptr && _endpoint_configuration_valid
	       && (_mode == 1 || _mode == 2)
	       && _mavlink->get_instance_id() == _instance
	       && (!signing_required() || _signing_ready);
}

bool MavlinkMHandler::control_enabled() const
{
	return _mavlink != nullptr && _control_configuration_valid
	       && (_mode == 1 || _mode == 2)
	       && _mavlink->get_instance_id() == _control_instance
	       && (!signing_required() || _signing_ready);
}

bool MavlinkMHandler::signing_required() const
{
	return _mode == 2;
}

bool MavlinkMHandler::source_matches(const mavlink_message_t &message) const
{
	return message.sysid == static_cast<uint8_t>(_source_system)
	       && message.compid == static_cast<uint8_t>(_source_component);
}

bool MavlinkMHandler::control_source_matches(const mavlink_message_t &message) const
{
	return message.sysid == static_cast<uint8_t>(_control_system)
	       && message.compid == static_cast<uint8_t>(_control_component);
}

bool MavlinkMHandler::task_message_allowed(const mavlink_message_t &message) const
{
	return enabled() && source_matches(message);
}

bool MavlinkMHandler::handle_message(const mavlink_message_t &message)
{
	if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG
	    && (_mode == 1 || _mode == 2) && _control_configuration_valid
	    && message.len >= OwnerDecisionRequiredWireLength
	    && message.len <= MAVLINK_MSG_ID_COMMAND_LONG_LEN) {
		mavlink_command_long_t command{};
		mavlink_msg_command_long_decode(&message, &command);

		if (command.command == OwnerDecisionCommand) {
			return handle_control_command(message);
		}
	}

	if (!enabled()) {
		return false;
	}

	// Any frame from the configured endpoint refreshes the informational
	// source-fresh flag. Cue acceptance itself remains governed by the cue's
	// persisted validity window, not by a removed private capability message.
	if (source_matches(message)) {
		_source_last_seen = hrt_absolute_time();
	}

	if (message.msgid != MAVLINK_MSG_ID_TRACK_IDENTITY
	    && message.msgid != MAVLINK_MSG_ID_TARGET_CUE
	    && message.msgid != MAVLINK_MSG_ID_TARGET_HANDOVER) {
		return false;
	}

	if (!task_message_allowed(message)) {
		return false;
	}

	if (message.msgid == MAVLINK_MSG_ID_TRACK_IDENTITY) {
		handle_track_identity(message);

	} else if (message.msgid == MAVLINK_MSG_ID_TARGET_CUE) {
		handle_target_cue(message);

	} else {
		handle_target_handover(message);
	}

	return true;
}

bool MavlinkMHandler::handle_control_command(const mavlink_message_t &message)
{
	mavlink_command_long_t command{};
	mavlink_msg_command_long_decode(&message, &command);
	const uint8_t vehicle_system = static_cast<uint8_t>(_mavlink->get_system_id());
	const uint8_t vehicle_component = static_cast<uint8_t>(_mavlink->get_component_id());

	// A USER_1 command for another exact component is not ours. A broadcast or
	// all-components form is ours but deliberately denied: a cue decision must
	// name one exact owned autopilot.
	if (command.target_system != 0 && command.target_system != vehicle_system) {
		return false;
	}

	if (command.target_system != vehicle_system
	    || command.target_component != vehicle_component) {
		send_control_command_ack(message, MAV_RESULT_DENIED);
		return true;
	}

	// Every handler instance sees the same route parameters. Consuming and
	// denying USER_1 here prevents a wrong-link request from falling through to
	// PX4's generic vehicle_command path.
	if (!control_enabled() || !control_source_matches(message)) {
		send_control_command_ack(message, MAV_RESULT_DENIED);
		return true;
	}

	uint16_t action_word = 0;
	uint16_t cue_low = 0;
	uint16_t cue_high = 0;
	uint32_t task_message = 0;
	const bool command_shape_valid =
		command.confirmation == 0
		&& exact_unsigned_word(command.param1, &action_word)
		&& exact_unsigned_word(command.param2, &cue_low)
		&& exact_unsigned_word(command.param3, &cue_high)
		&& exact_unsigned_value(command.param4, 0x00ffffffU, &task_message)
		&& task_message == OwnerDecisionTaskMessage
		&& fabsf(command.param5) <= 0.f
		&& fabsf(command.param6) <= 0.f
		&& fabsf(command.param7) <= 0.f
		&& (action_word == OwnerDecisionAccept || action_word == OwnerDecisionReject);
	const uint32_t cue_id = static_cast<uint32_t>(cue_low)
				| (static_cast<uint32_t>(cue_high) << 16U);

	if (!command_shape_valid || cue_id == 0) {
		send_control_command_ack(message, MAV_RESULT_FAILED);
		return true;
	}

	// A control request is valid only for the exact cue PX4 is currently
	// advertising on this owner link. This prevents a blind or stale command
	// from accepting a different queued assignment.
	_target_status_sub.update(&_control_status);
	const bool status_fresh = _control_status.timestamp != 0
				  && hrt_elapsed_time(&_control_status.timestamp) < 1'000'000;
	const bool cue_matches = status_fresh
				 && _control_status.message_id == OwnerDecisionTaskMessage
				 && _control_status.instance_id == cue_id;
	const bool state_matches =
		(action_word == OwnerDecisionAccept
		 && _control_status.state == mavlink_m_target_status_s::STATE_PENDING)
		|| (action_word == OwnerDecisionReject
		    && (_control_status.state == mavlink_m_target_status_s::STATE_PENDING
			|| _control_status.state == mavlink_m_target_status_s::STATE_ACTIVE));

	if (!cue_matches || !state_matches) {
		send_control_command_ack(message, MAV_RESULT_DENIED);
		return true;
	}

	mavlink_m_task_decision_s decision{};
	decision.timestamp = hrt_absolute_time();
	decision.task_msgid = OwnerDecisionTaskMessage;
	decision.task_instance = cue_id;
	decision.target_system = vehicle_system;
	decision.action = action_word == OwnerDecisionAccept
			  ? mavlink_m_task_decision_s::ACTION_ACCEPT
			  : mavlink_m_task_decision_s::ACTION_REJECT;

	if (!_task_decision_pub.publish(decision)) {
		send_control_command_ack(message, MAV_RESULT_FAILED);
		return true;
	}

	// This ACK confirms only that the authorized local decision was queued.
	// The cue source receives the authoritative MAVLINK_M_ACK after the cue
	// instance durably commits the resulting state transition.
	send_control_command_ack(message, MAV_RESULT_ACCEPTED);
	return true;
}

void MavlinkMHandler::handle_track_identity(const mavlink_message_t &message)
{
	if (message.len < TrackIdentityRequiredWireLength
	    || message.len > MAVLINK_MSG_ID_TRACK_IDENTITY_LEN) {
		return;
	}

	mavlink_track_identity_t track{};
	mavlink_msg_track_identity_decode(&message, &track);

	const uint64_t now_usec = utc_now_usec();
	const uint64_t tolerance_usec = _max_age_s > 0
					? static_cast<uint64_t>(_max_age_s) * 1'000'000ULL : 0;

	// A zero target-set is explicitly valid on the wire, but the current cue
	// has no track_uid and therefore offers no safe correlation key.
	if (track.target_set_id == 0) {
		return;
	}

	if (track.origin_sysid == 0 || all_zero(track.track_uid, sizeof(track.track_uid))
	    || track.time_usec < Unix2020Usec || track.first_detected_usec < Unix2020Usec
	    || track.first_detected_usec > track.time_usec
	    || now_usec < Unix2020Usec
	    || (tolerance_usec > 0 && (track.time_usec > now_usec + tolerance_usec
					 || now_usec > track.time_usec + tolerance_usec))
	    || (!std::isnan(track.id_confidence)
		&& (!PX4_ISFINITE(track.id_confidence) || track.id_confidence < 0.f || track.id_confidence > 1.f))
	    || !valid_target_class(track.target_class) || !valid_target_force(track.target_force)
	    || track.origin_sensor >= MAVLINK_M_ID_METHOD_ENUM_END
	    || track.id_method >= MAVLINK_M_ID_METHOD_ENUM_END
	    || track.pid_status >= MAVLINK_M_PID_STATUS_ENUM_END
	    || track.track_rel >= MAVLINK_M_TRACK_REL_ENUM_END
	    || track.external_track_type >= MAVLINK_M_TRACK_NUMBER_TYPE_ENUM_END
	    || track.stanag_identity >= MAVLINK_M_STANAG_IDENTITY_ENUM_END
	    || track.environment >= MAVLINK_M_ENVIRONMENT_ENUM_END
	    || (track.atr_confidence_pct > 100 && track.atr_confidence_pct != 255)
	    || track.atr_conf_tier >= MAVLINK_M_ATR_CONFIDENCE_ENUM_END
	    || track.sidc_context >= MAVLINK_M_SIDC_CONTEXT_ENUM_END) {
		PX4_WARN("ignoring invalid MAVLink-M track identity");
		return;
	}

	TrackIdentity identity{};
	identity.time_usec = track.time_usec;
	identity.first_detected_usec = track.first_detected_usec;
	identity.target_set_id = track.target_set_id;
	identity.confidence = track.id_confidence;
	identity.source_system = message.sysid;
	identity.source_component = message.compid;
	identity.origin_system = track.origin_sysid;
	memcpy(identity.track_uid, track.track_uid, sizeof(identity.track_uid));

	TrackIdentity *slot = nullptr;
	TrackIdentity *oldest = &_track_identities[0];

	for (TrackIdentity &stored : _track_identities) {
		if (stored.target_set_id == identity.target_set_id
		    && stored.source_system == identity.source_system
		    && stored.source_component == identity.source_component) {
			if (memcmp(stored.track_uid, identity.track_uid, sizeof(identity.track_uid)) != 0) {
				if (track_identity_fresh(stored, identity.time_usec)) {
					PX4_WARN("MAVLink-M target-set identity collision");
					return;
				}

				// An expired cached identity must not pin a target-set forever.
				// A fresh identity may safely replace that stale cache entry.
				slot = &stored;
				break;
			}

			if (identity.time_usec < stored.time_usec) {
				return;
			}

			slot = &stored;
			break;
		}

		if (stored.target_set_id == 0 && slot == nullptr) {
			slot = &stored;
		}

		if (stored.time_usec < oldest->time_usec) {
			oldest = &stored;
		}
	}

	if (slot == nullptr) {
		slot = oldest;
	}

	*slot = identity;

	const Assignment active_before = _active;
	const Assignment inbox_before[InboxCapacity] {_inbox[0], _inbox[1]};
	bool changed = false;

	auto enrich = [this, &identity, &changed](Assignment &assignment) {
		if (assignment.state != AssignmentState::Empty
		    && track_identity_usable(assignment, identity)
		    && (all_zero(assignment.track_uid, sizeof(assignment.track_uid))
			|| memcmp(assignment.track_uid, identity.track_uid, sizeof(identity.track_uid)) == 0)) {
			apply_track_identity(assignment, identity);
			changed = true;
		}
	};

	enrich(_active);

	for (Assignment &assignment : _inbox) {
		enrich(assignment);
	}

	if (changed && !save_state()) {
		_active = active_before;
		_inbox[0] = inbox_before[0];
		_inbox[1] = inbox_before[1];
		PX4_WARN("MAVLink-M track enrichment was not persisted");
	}
}

void MavlinkMHandler::handle_target_cue(const mavlink_message_t &message)
{
	if (message.len < TargetCueRequiredWireLength
	    || message.len > MAVLINK_MSG_ID_TARGET_CUE_LEN) {
		return;
	}

	mavlink_target_cue_t cue{};
	mavlink_msg_target_cue_decode(&message, &cue);

	Assignment assignment{};
	assignment.time_usec = cue.time_usec;
	const uint64_t configured_lifetime_usec = _max_age_s > 0
			? static_cast<uint64_t>(_max_age_s) * 1'000'000ULL : 0;
	assignment.valid_until_usec = configured_lifetime_usec > 0
			&& cue.time_usec <= UINT64_MAX - configured_lifetime_usec
			? cue.time_usec + configured_lifetime_usec : 0;
	assignment.message_id = MAVLINK_MSG_ID_TARGET_CUE;
	assignment.instance_id = cue.cue_id;
	assignment.target_set_id = cue.target_set_id;
	assignment.payload_crc = crc32_signature(0, message.len,
			reinterpret_cast<const uint8_t *>(message.payload64));
	assignment.lat = cue.lat;
	assignment.lon = cue.lon;
	assignment.alt = cue.alt;
	assignment.vx = cue.vx;
	assignment.vy = cue.vy;
	assignment.vz = cue.vz;
	assignment.confidence = cue.confidence_score;
	assignment.source_system = message.sysid;
	assignment.source_component = message.compid;
	assignment.origin_system = cue.origin_sysid;
	assignment.target_class = cue.target_class;
	assignment.target_force = cue.target_force;
	assignment.cue_type = cue.cue_type;
	memcpy(assignment.name, cue.name, sizeof(cue.name));
	assignment.name[sizeof(assignment.name) - 1] = '\0';

	if (cue.origin_sysid == 0) {
		send_ack(assignment, MAVLINK_M_ACK_FAILED, "cue origin system invalid");
		return;
	}

	if (assignment.cue_type != MAVLINK_M_CUE_TYPE_INVESTIGATE
	    && assignment.cue_type != MAVLINK_M_CUE_TYPE_OBSERVE
	    && assignment.cue_type != MAVLINK_M_CUE_TYPE_MARK) {
		send_ack(assignment, MAVLINK_M_ACK_UNSUPPORTED, "unsupported cue type");
		return;
	}

	if (!valid_target_class(assignment.target_class) || !valid_target_force(assignment.target_force)) {
		send_ack(assignment, MAVLINK_M_ACK_FAILED, "invalid classification enum");
		return;
	}

	if (all_zero(assignment.track_uid, sizeof(assignment.track_uid))) {
		if (const TrackIdentity *identity = find_track_identity(assignment)) {
		apply_track_identity(assignment, *identity);
		}
	}

	store_assignment(assignment);
}

void MavlinkMHandler::handle_target_handover(const mavlink_message_t &message)
{
	if (message.len < TargetHandoverRequiredWireLength
	    || message.len > MAVLINK_MSG_ID_TARGET_HANDOVER_LEN) {
		return;
	}

	mavlink_target_handover_t handover{};
	mavlink_msg_target_handover_decode(&message, &handover);

	Assignment assignment{};
	assignment.time_usec = handover.time_usec;
	assignment.valid_until_usec = handover.valid_until_usec;
	const uint64_t configured_lifetime_usec = _max_age_s > 0
			? static_cast<uint64_t>(_max_age_s) * 1'000'000ULL : 0;

	if (configured_lifetime_usec > 0
	    && handover.time_usec <= UINT64_MAX - configured_lifetime_usec) {
		const uint64_t configured_deadline = handover.time_usec + configured_lifetime_usec;
		assignment.valid_until_usec = assignment.valid_until_usec == 0
					      ? configured_deadline
					      : math::min(assignment.valid_until_usec, configured_deadline);
	}
	assignment.track_identity_time_usec = handover.detected_first_usec;
	assignment.message_id = MAVLINK_MSG_ID_TARGET_HANDOVER;
	// The finalized handover has no private handover_id. target_set_id is its
	// documented ACK correlation instance.
	assignment.instance_id = handover.target_set_id;
	assignment.target_set_id = handover.target_set_id;
	assignment.payload_crc = crc32_signature(0, message.len,
			reinterpret_cast<const uint8_t *>(message.payload64));
	assignment.lat = handover.lat;
	assignment.lon = handover.lon;
	assignment.alt = handover.alt;
	assignment.vx = handover.vx;
	assignment.vy = handover.vy;
	assignment.vz = handover.vz;
	assignment.confidence = handover.confidence_score;
	assignment.source_system = message.sysid;
	assignment.source_component = message.compid;
	assignment.origin_system = message.sysid;
	assignment.target_class = handover.target_class;
	assignment.target_force = handover.target_force;
	memcpy(assignment.track_uid, handover.track_uid, sizeof(assignment.track_uid));
	memcpy(assignment.name, handover.target_name, sizeof(assignment.name));
	assignment.name[sizeof(assignment.name) - 1] = '\0';

	if (handover.target_set_id == 0 || handover.valid_until_usec == 0
	    || handover.detected_first_usec < Unix2020Usec
	    || handover.detected_first_usec > handover.time_usec
	    || !valid_target_class(handover.target_class) || !valid_target_force(handover.target_force)
	    || handover.match_media_type >= MAVLINK_M_MATCH_MEDIA_TYPE_ENUM_END) {
		send_ack(assignment, MAVLINK_M_ACK_FAILED, "invalid handover fields");
		return;
	}

	store_assignment(assignment);
}

bool MavlinkMHandler::validate_common(const Assignment &assignment, const char **reason, uint8_t *ack_result) const
{
	if (assignment.instance_id == 0) {
		*reason = "zero assignment instance";
		*ack_result = MAVLINK_M_ACK_FAILED;
		return false;
	}

	if (assignment.lat < -900'000'000 || assignment.lat > 900'000'000
	    || assignment.lon < -1'800'000'000 || assignment.lon > 1'800'000'000
	    || assignment.lat == INT32_MAX || assignment.lon == INT32_MAX) {
		*reason = "invalid WGS84 coordinate";
		*ack_result = MAVLINK_M_ACK_FAILED;
		return false;
	}

	if (!finite_or_nan(assignment.alt) || !finite_or_nan(assignment.vx)
	    || !finite_or_nan(assignment.vy) || !finite_or_nan(assignment.vz)
	    || (!std::isnan(assignment.confidence)
		&& (!PX4_ISFINITE(assignment.confidence) || assignment.confidence < 0.f || assignment.confidence > 1.f))) {
		*reason = "invalid numeric field";
		*ack_result = MAVLINK_M_ACK_FAILED;
		return false;
	}

	return validate_time(assignment.time_usec, assignment.valid_until_usec, reason, ack_result);
}

bool MavlinkMHandler::validate_time(uint64_t time_usec, uint64_t valid_until_usec, const char **reason,
				    uint8_t *ack_result) const
{
	const uint64_t now_usec = utc_now_usec();

	// MAV_M_MAX_AGE=0 may disable replay-window comparison, but it must never
	// turn an unset/non-UTC wire timestamp into a valid assignment identity.
	if (time_usec < Unix2020Usec) {
		*reason = "assignment timestamp invalid";
		*ack_result = MAVLINK_M_ACK_FAILED;
		return false;
	}

	if (_max_age_s > 0) {
		if (now_usec < Unix2020Usec) {
			*reason = "vehicle UTC unavailable";
			*ack_result = MAVLINK_M_ACK_FAILED;
			return false;
		}

		const uint64_t tolerance_usec = static_cast<uint64_t>(_max_age_s) * 1'000'000ULL;

		if (time_usec > now_usec + tolerance_usec) {
			*reason = "assignment timestamp in future";
			*ack_result = MAVLINK_M_ACK_FAILED;
			return false;
		}

		if (now_usec > time_usec + tolerance_usec) {
			*reason = "assignment stale";
			*ack_result = MAVLINK_M_ACK_EXPIRED;
			return false;
		}
	}

	if (valid_until_usec != 0) {
		if (valid_until_usec < time_usec) {
			*reason = "expiry precedes assignment";
			*ack_result = MAVLINK_M_ACK_FAILED;
			return false;
		}

		if (now_usec < Unix2020Usec) {
			*reason = "vehicle UTC unavailable";
			*ack_result = MAVLINK_M_ACK_FAILED;
			return false;
		}

		if (now_usec > valid_until_usec) {
			*reason = "assignment expired";
			*ack_result = MAVLINK_M_ACK_EXPIRED;
			return false;
		}
	}

	return true;
}

MavlinkMHandler::Assignment *MavlinkMHandler::find_duplicate(const Assignment &candidate)
{
	auto matches = [&candidate](Assignment &stored) {
		return stored.state != AssignmentState::Empty
		       && stored.source_system == candidate.source_system
		       && stored.source_component == candidate.source_component
		       && stored.message_id == candidate.message_id
		       && stored.instance_id == candidate.instance_id;
	};

	if (matches(_active)) {
		return &_active;
	}

	for (Assignment &stored : _inbox) {
		if (matches(stored)) {
			return &stored;
		}
	}

	for (Assignment &stored : _terminal) {
		if (matches(stored)) {
			return &stored;
		}
	}

	return nullptr;
}

MavlinkMHandler::Assignment *MavlinkMHandler::find_task(uint32_t message_id, uint32_t instance_id)
{
	auto matches = [message_id, instance_id](Assignment &assignment) {
		return assignment.state != AssignmentState::Empty
		       && assignment.message_id == message_id && assignment.instance_id == instance_id;
	};

	if (matches(_active)) {
		return &_active;
	}

	for (Assignment &assignment : _inbox) {
		if (matches(assignment)) {
			return &assignment;
		}
	}

	for (Assignment &assignment : _terminal) {
		if (matches(assignment)) {
			return &assignment;
		}
	}

	return nullptr;
}

MavlinkMHandler::ControlRecord *MavlinkMHandler::find_control(uint32_t control_id)
{
	for (ControlRecord &control : _controls) {
		if (control.control_id == control_id
		    && control.source_system == static_cast<uint8_t>(_source_system)
		    && control.source_component == static_cast<uint8_t>(_source_component)) {
			return &control;
		}
	}

	return nullptr;
}

bool MavlinkMHandler::track_identity_fresh(const TrackIdentity &identity, uint64_t reference_time_usec) const
{
	if (_max_age_s <= 0) {
		return true;
	}

	const uint64_t now_usec = utc_now_usec();

	if (now_usec < Unix2020Usec || identity.time_usec < Unix2020Usec
	    || reference_time_usec < Unix2020Usec) {
		return false;
	}

	const uint64_t tolerance_usec = static_cast<uint64_t>(_max_age_s) * 1'000'000ULL;
	const uint64_t age_delta = now_usec >= identity.time_usec
				   ? now_usec - identity.time_usec : identity.time_usec - now_usec;
	const uint64_t reference_delta = reference_time_usec >= identity.time_usec
					 ? reference_time_usec - identity.time_usec
					 : identity.time_usec - reference_time_usec;
	return age_delta <= tolerance_usec && reference_delta <= tolerance_usec;
}

bool MavlinkMHandler::track_identity_usable(const Assignment &assignment, const TrackIdentity &identity) const
{
	return identity.target_set_id == assignment.target_set_id
	       && identity.source_system == assignment.source_system
	       && identity.source_component == assignment.source_component
	       && identity.origin_system == assignment.origin_system
	       && track_identity_fresh(identity, assignment.time_usec);
}

const MavlinkMHandler::TrackIdentity *MavlinkMHandler::find_track_identity(const Assignment &assignment) const
{
	if (assignment.target_set_id == 0) {
		return nullptr;
	}

	for (const TrackIdentity &identity : _track_identities) {
		if (track_identity_usable(assignment, identity)) {
			return &identity;
		}
	}

	return nullptr;
}

void MavlinkMHandler::apply_track_identity(Assignment &assignment, const TrackIdentity &identity)
{
	assignment.track_identity_time_usec = identity.time_usec;
	memcpy(assignment.track_uid, identity.track_uid, sizeof(assignment.track_uid));
}

MavlinkMHandler::Assignment *MavlinkMHandler::find_pending(uint32_t message_id, uint32_t instance_id)
{
	for (Assignment &assignment : _inbox) {
		if (assignment.state == AssignmentState::Pending
		    && (message_id == 0 || assignment.message_id == message_id)
		    && (instance_id == 0 || assignment.instance_id == instance_id)) {
			return &assignment;
		}
	}

	return nullptr;
}

MavlinkMHandler::Assignment *MavlinkMHandler::find_free_inbox()
{
	for (Assignment &assignment : _inbox) {
		if (assignment.state == AssignmentState::Empty) {
			return &assignment;
		}
	}

	return nullptr;
}

void MavlinkMHandler::store_assignment(const Assignment &candidate)
{
	const char *reason = nullptr;
	uint8_t result = MAVLINK_M_ACK_FAILED;

	if (!validate_common(candidate, &reason, &result)) {
		send_ack(candidate, result, reason);
		return;
	}

	if (Assignment *duplicate = find_duplicate(candidate)) {
		if (duplicate->payload_crc != candidate.payload_crc) {
			send_ack(candidate, MAVLINK_M_ACK_FAILED, "immutable instance collision");

		} else {
			send_ack(*duplicate, duplicate->last_ack_result, "duplicate idempotent replay");
		}

		return;
	}

	Assignment *slot = find_free_inbox();

	if (slot == nullptr) {
		send_ack(candidate, MAVLINK_M_ACK_FAILED, "assignment queue full");
		return;
	}

	const Assignment previous = *slot;
	*slot = candidate;
	slot->state = AssignmentState::Pending;
	slot->last_ack_result = MAVLINK_M_ACK_RECEIVED;
	slot->restored = 0;

	if (!save_state()) {
		*slot = previous;
		send_ack(candidate, MAVLINK_M_ACK_FAILED, "durable storage failed");
		return;
	}

	send_ack(*slot, MAVLINK_M_ACK_RECEIVED, "stored pending pilot decision");
	publish_status();
}

void MavlinkMHandler::remove_inbox(Assignment *assignment)
{
	if (assignment == &_inbox[0]) {
		_inbox[0] = _inbox[1];
		_inbox[1] = Assignment{};

	} else if (assignment == &_inbox[1]) {
		_inbox[1] = Assignment{};
	}
}

void MavlinkMHandler::remember_terminal(const Assignment &assignment, AssignmentState state, uint8_t result)
{
	for (unsigned i = TerminalCapacity - 1; i > 0; --i) {
		_terminal[i] = _terminal[i - 1];
	}

	_terminal[0] = assignment;
	_terminal[0].state = state;
	_terminal[0].last_ack_result = result;
}

void MavlinkMHandler::undo_remember_terminal(const Assignment &evicted)
{
	for (unsigned i = 0; i + 1 < TerminalCapacity; ++i) {
		_terminal[i] = _terminal[i + 1];
	}

	_terminal[TerminalCapacity - 1] = evicted;
}

void MavlinkMHandler::accept_pending(uint32_t message_id, uint32_t instance_id)
{
	Assignment *pending = find_pending(message_id, instance_id);

	if (pending == nullptr) {
		PX4_WARN("MAVLink-M accept ignored: no matching pending task");
		return;
	}

	const Assignment decided = *pending;
	const char *reason = nullptr;
	uint8_t result = MAVLINK_M_ACK_FAILED;

	// Decisions are local, but their subject still has to be valid at the exact
	// decision instant. This closes the race between deadline expiry and an RC
	// or console acceptance arriving in the same receiver iteration.
	if (!validate_common(decided, &reason, &result)) {
		if (result == MAVLINK_M_ACK_EXPIRED) {
			expire_assignments(utc_now_usec());

			// A persistence failure rolls expiry back. Never accept in that
			// case; still tell the sender that the decision arrived too late.
			if (find_pending(decided.message_id, decided.instance_id) != nullptr) {
				send_ack(decided, result, reason);
			}

		} else {
			send_ack(decided, result, reason);
		}

		return;
	}

	// One explicit local acceptance may own the active cue at a time. A second
	// acceptance cannot mean "supersede" because the finalized shared profile
	// has no supersession/lifecycle message. Keep the new cue Pending and its
	// duplicate ACK at RECEIVED so clearing the active cue can never dispatch it
	// without another deliberate local acceptance.
	if (_active.state != AssignmentState::Empty) {
		PX4_WARN("MAVLink-M accept blocked: active task %lu/%lu; pending task %lu/%lu requires a fresh decision",
			 static_cast<unsigned long>(_active.message_id),
			 static_cast<unsigned long>(_active.instance_id),
			 static_cast<unsigned long>(decided.message_id),
			 static_cast<unsigned long>(decided.instance_id));
		publish_status();
		return;
	}

	const Assignment active_before = _active;
	const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
	_active = decided;
	_active.state = AssignmentState::Active;
	_active.last_ack_result = MAVLINK_M_ACK_ACCEPTED;
	++_active.status_sequence;
	remove_inbox(pending);

	if (!save_state()) {
		_active = active_before;
		_inbox[0] = inbox_before[0];
		_inbox[1] = inbox_before[1];
		send_ack(decided, MAVLINK_M_ACK_FAILED, "decision storage failed");
		return;
	}

	Assignment acknowledged = _active;
	acknowledged.last_ack_result = MAVLINK_M_ACK_ACCEPTED;
	send_ack(acknowledged, MAVLINK_M_ACK_ACCEPTED, "local operator accepted active target");
	(void)command_active_assignment();

	publish_status();
}

void MavlinkMHandler::reject_pending_or_abort_active(uint32_t message_id, uint32_t instance_id)
{
	Assignment *pending = find_pending(message_id, instance_id);

	if (pending != nullptr) {
		Assignment decided = *pending;
		++decided.status_sequence;
		const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
		const Assignment terminal_evicted = _terminal[TerminalCapacity - 1];
		remove_inbox(pending);
		remember_terminal(decided, AssignmentState::Rejected, MAVLINK_M_ACK_REJECTED);

		if (!save_state()) {
			_inbox[0] = inbox_before[0];
			_inbox[1] = inbox_before[1];
			undo_remember_terminal(terminal_evicted);
			send_ack(decided, MAVLINK_M_ACK_FAILED, "decision storage failed");
			return;
		}

		send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "local operator rejected assignment");
		publish_status();
		return;
	}

	const bool active_matches = _active.state == AssignmentState::Active
		&& (message_id == 0 || _active.message_id == message_id)
		&& (instance_id == 0 || _active.instance_id == instance_id);

	if (active_matches) {
		Assignment decided = _active;
		++decided.status_sequence;
		const Assignment active_before = _active;
		const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
		const Assignment terminal_evicted = _terminal[TerminalCapacity - 1];
		_active = Assignment{};
		remember_terminal(decided, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);

		if (!save_state()) {
			_active = active_before;
			_inbox[0] = inbox_before[0];
			_inbox[1] = inbox_before[1];
			undo_remember_terminal(terminal_evicted);
			send_ack(decided, MAVLINK_M_ACK_FAILED, "abort storage failed");
			return;
		}

		send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "local operator aborted active target");
		(void)cancel_assignment_commands(_terminal[0]);

		publish_status();

	} else {
		PX4_WARN("MAVLink-M reject ignored: no matching pending or active task");
	}
}

void MavlinkMHandler::update_local_decision()
{
	// Drain the bounded queue every receiver iteration. A stale entry must not
	// hide a newer operator decision behind it until a later scheduling cycle.
	for (unsigned i = 0; i < mavlink_m_task_decision_s::ORB_QUEUE_LENGTH; ++i) {
		mavlink_m_task_decision_s decision{};

		if (!_task_decision_sub.update(&decision)) {
			return;
		}

		const hrt_abstime now = hrt_absolute_time();
		const uint64_t age_usec = now >= decision.timestamp ? now - decision.timestamp : UINT64_MAX;
		const uint8_t vehicle_system = static_cast<uint8_t>(_mavlink->get_system_id());

		if (decision.timestamp == 0 || decision.timestamp <= _last_task_decision
		    || age_usec > 1'000'000 || decision.target_system != vehicle_system) {
			PX4_WARN("MAVLink-M ignored local decision ts=%llu last=%llu age=%llu target=%u vehicle=%u",
				 static_cast<unsigned long long>(decision.timestamp),
				 static_cast<unsigned long long>(_last_task_decision),
				 static_cast<unsigned long long>(age_usec),
				 static_cast<unsigned>(decision.target_system),
				 static_cast<unsigned>(vehicle_system));
			continue;
		}

		_last_task_decision = decision.timestamp;

		if (decision.action == mavlink_m_task_decision_s::ACTION_ACCEPT) {
			accept_pending(decision.task_msgid, decision.task_instance);

		} else if (decision.action == mavlink_m_task_decision_s::ACTION_REJECT) {
			reject_pending_or_abort_active(decision.task_msgid, decision.task_instance);

		} else {
			PX4_WARN("MAVLink-M ignored unknown local decision action");
		}
	}
}

MavlinkMHandler::RcPosition MavlinkMHandler::classify_rc(uint16_t pwm) const
{
	if (_rc_reject >= _rc_accept || pwm < 750 || pwm > 2250) {
		return RcPosition::Unknown;
	}

	if (pwm <= _rc_reject) {
		return RcPosition::Reject;
	}

	if (pwm >= _rc_accept) {
		return RcPosition::Accept;
	}

	return RcPosition::Center;
}

void MavlinkMHandler::update_rc()
{
	input_rc_s input{};

	if (!_input_rc_sub.update(&input)) {
		return;
	}

	if (_rc_channel <= 0 || _rc_channel > input_rc_s::RC_INPUT_MAX_CHANNELS
	    || _rc_channel > input.channel_count || input.rc_failsafe || input.rc_lost
	    || input.input_source == input_rc_s::RC_INPUT_SOURCE_UNKNOWN
	    || input.input_source == input_rc_s::RC_INPUT_SOURCE_MAVLINK
	    || hrt_elapsed_time(&input.timestamp_last_signal) > 500'000) {
		_last_rc_position = RcPosition::Unknown;
		_rc_center_latched = false;
		return;
	}

	const RcPosition position = classify_rc(input.values[_rc_channel - 1]);

	if (position == RcPosition::Unknown) {
		_last_rc_position = RcPosition::Unknown;
		_rc_center_latched = false;
		return;
	}

	if (position == RcPosition::Center) {
		_rc_center_latched = true;
	}

	if (position != _last_rc_position && _rc_center_latched && _last_rc_position == RcPosition::Center) {
		if (position == RcPosition::Accept) {
			accept_pending();
			_rc_center_latched = false;

		} else if (position == RcPosition::Reject) {
			reject_pending_or_abort_active();
			_rc_center_latched = false;
		}
	}

	_last_rc_position = position;
}

bool MavlinkMHandler::vehicle_ready_for_reposition() const
{
	return _vehicle_status.timestamp != 0
	       && hrt_elapsed_time(&_vehicle_status.timestamp) < 2'000'000
	       && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
	       && !_vehicle_status.failsafe
	       && _vehicle_status.failure_detector_status == vehicle_status_s::FAILURE_NONE;
}

bool MavlinkMHandler::vehicle_airborne_for_reposition() const
{
	return _vehicle_land_detected.timestamp != 0
	       && hrt_elapsed_time(&_vehicle_land_detected.timestamp) < 2'000'000
	       && !_vehicle_land_detected.freefall
	       && !_vehicle_land_detected.landed
	       && !_vehicle_land_detected.maybe_landed
	       && !_vehicle_land_detected.ground_contact;
}

bool MavlinkMHandler::publish_vehicle_command(const Assignment &assignment, uint32_t command, float param1,
		float param2, float param3, float param4, double param5, double param6, float param7,
		uint8_t target_component)
{
	if (_mavlink == nullptr || assignment.source_system == 0 || assignment.source_component == 0) {
		return false;
	}

	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.command = command;
	vehicle_command.param1 = param1;
	vehicle_command.param2 = param2;
	vehicle_command.param3 = param3;
	vehicle_command.param4 = param4;
	vehicle_command.param5 = param5;
	vehicle_command.param6 = param6;
	vehicle_command.param7 = param7;
	vehicle_command.target_system = static_cast<uint8_t>(_mavlink->get_system_id());
	vehicle_command.target_component = target_component == CommandTargetLocalComponent
					   ? static_cast<uint8_t>(_mavlink->get_component_id())
					   : target_component;
	vehicle_command.source_system = assignment.source_system;
	vehicle_command.source_component = assignment.source_component;
	vehicle_command.from_external = true;

	return _vehicle_command_pub.publish(vehicle_command);
}

bool MavlinkMHandler::command_active_assignment()
{
	// TARGET_CUE is explicitly non-kinetic. The only optional vehicle action is
	// a one-shot reposition attempt for INVESTIGATE, called only by the explicit
	// local acceptance that immediately promotes this exact cue to active. This
	// path is never called by periodic update or restore. MAVLink-M defines no
	// INTERCEPT cue enum: cue-altitude use is therefore a separate, local PX4
	// policy applied only after the finalized INVESTIGATE cue is accepted.
	if (_active.state != AssignmentState::Active
	    || _active.message_id != MAVLINK_MSG_ID_TARGET_CUE
	    || _active.cue_type != MAVLINK_M_CUE_TYPE_INVESTIGATE
	    || (_action_mode != ActionRepositionCurrentAltitude
		&& _action_mode != ActionInterceptCueAltitude)
	    || (_active.command_flags & CommandNav) != 0) {
		return false;
	}

	const char *freshness_reason = nullptr;
	uint8_t freshness_result = MAVLINK_M_ACK_FAILED;

	// Durable acceptance can include an fsync/rename. Recheck immediately
	// before publication so a cue crossing its deadline during that commit can
	// be accepted for the audit trail but can never cause late motion.
	if (!validate_common(_active, &freshness_reason, &freshness_result)) {
		PX4_WARN("MAVLink-M reposition suppressed: %s",
			 freshness_reason != nullptr ? freshness_reason : "cue no longer valid");
		return false;
	}

	// Acceptance persistence can also span a vehicle-state transition. Pull the
	// newest sample after the commit and immediately before the flight-state
	// gate rather than relying on update()'s pre-decision cache.
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_global_position_sub.update(&_global_position);

	if (!vehicle_ready_for_reposition() || !vehicle_airborne_for_reposition()) {
		if (_last_reposition_block_log == 0
		    || hrt_elapsed_time(&_last_reposition_block_log) > 2'000'000) {
			PX4_INFO("MAVLink-M reposition not issued; accept a fresh cue while already airborne, armed, and in Hold");
			_last_reposition_block_log = hrt_absolute_time();
		}

		return false;
	}

	if (_global_position.timestamp == 0
	    || hrt_elapsed_time(&_global_position.timestamp) >= 2'000'000
	    || !PX4_ISFINITE(_global_position.lat)
	    || !PX4_ISFINITE(_global_position.lon)
	    || !PX4_ISFINITE(_global_position.alt)) {
		PX4_WARN("MAVLink-M reposition not issued; fresh aircraft AMSL position unavailable");
		return false;
	}

	const double target_lat = _active.lat * 1e-7;
	const double target_lon = _active.lon * 1e-7;
	// Normal point-to-point navigation creates a target-position setpoint at
	// the aircraft's acceptance-time AMSL altitude. This is the explicit
	// "invisible waypoint" that prevents a ground-target altitude from causing
	// an unintended climb or descent.
	float target_alt = _global_position.alt;

	if (_action_mode == ActionInterceptCueAltitude && PX4_ISFINITE(_active.alt)) {
		target_alt = _active.alt;
	}

	if (publish_vehicle_command(_active, vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
				    NAN, RepositionChangeModeFlag, NAN, NAN,
				    target_lat, target_lon, target_alt)) {
		_active.command_flags |= CommandNav;

		if (!save_state()) {
			PX4_WARN("MAVLink-M command state was not persisted");
		}

		publish_status();
		return true;
	}

	return false;
}

bool MavlinkMHandler::cancel_assignment_commands(const Assignment &assignment)
{
	if (assignment.command_flags == 0) {
		return false;
	}

	// Revoking future action authority must not revoke the ability to stop a
	// reposition that this endpoint already issued and durably marked.
	_vehicle_status_sub.update(&_vehicle_status);
	_global_position_sub.update(&_global_position);

	if ((assignment.command_flags & CommandNav) != 0
	    && vehicle_ready_for_reposition()
	    && _global_position.timestamp != 0 && hrt_elapsed_time(&_global_position.timestamp) < 2'000'000
	    && PX4_ISFINITE(_global_position.lat) && PX4_ISFINITE(_global_position.lon)
	    && PX4_ISFINITE(_global_position.alt)) {
		return publish_vehicle_command(assignment, vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
					       NAN, RepositionChangeModeFlag, NAN, NAN,
					       _global_position.lat, _global_position.lon,
					       _global_position.alt);
	}

	return false;
}

bool MavlinkMHandler::assignment_expired_at(const Assignment &assignment, uint64_t now_usec) const
{
	if (assignment.valid_until_usec != 0 && now_usec > assignment.valid_until_usec) {
		return true;
	}

	if (_max_age_s <= 0) {
		return false;
	}

	const uint64_t maximum_age_usec = static_cast<uint64_t>(_max_age_s) * 1'000'000ULL;
	return assignment.time_usec <= UINT64_MAX - maximum_age_usec
	       && now_usec > assignment.time_usec + maximum_age_usec;
}

void MavlinkMHandler::expire_assignments(uint64_t now_usec)
{
	if (now_usec < Unix2020Usec) {
		return;
	}

	bool changed = false;
	Assignment expired_assignments[3] {};
	Assignment terminal_evicted[3] {};
	unsigned expired_count = 0;
	const Assignment active_before = _active;
	const Assignment inbox_before[2] {_inbox[0], _inbox[1]};

	if (_active.state == AssignmentState::Active
	    && assignment_expired_at(_active, now_usec)) {
		Assignment expired = _active;
		++expired.status_sequence;
		_active = Assignment{};
		terminal_evicted[expired_count] = _terminal[TerminalCapacity - 1];
		remember_terminal(expired, AssignmentState::Expired, MAVLINK_M_ACK_EXPIRED);
		expired_assignments[expired_count++] = expired;
		changed = true;
	}

	for (unsigned i = 0; i < InboxCapacity;) {
		if (_inbox[i].state != AssignmentState::Empty
		    && assignment_expired_at(_inbox[i], now_usec)) {
			Assignment expired = _inbox[i];
			++expired.status_sequence;
			remove_inbox(&_inbox[i]);
			terminal_evicted[expired_count] = _terminal[TerminalCapacity - 1];
			remember_terminal(expired, AssignmentState::Expired, MAVLINK_M_ACK_EXPIRED);
			expired_assignments[expired_count++] = expired;
			changed = true;

		} else {
			++i;
		}
	}

	if (changed) {
		if (!save_state()) {
			_active = active_before;
			_inbox[0] = inbox_before[0];
			_inbox[1] = inbox_before[1];

			for (unsigned i = expired_count; i > 0; --i) {
				undo_remember_terminal(terminal_evicted[i - 1]);
			}

		} else {
			for (unsigned i = 0; i < expired_count; ++i) {
				send_ack(expired_assignments[i], MAVLINK_M_ACK_EXPIRED, "assignment expired");
				(void)cancel_assignment_commands(expired_assignments[i]);
			}
		}

		publish_status();
	}
}

void MavlinkMHandler::send_ack(const Assignment &assignment, uint8_t result, const char *reason)
{
	if (!enabled()) {
		return;
	}

	_mavlink->set_proto_version(2);

	mavlink_mavlink_m_ack_t ack{};
	ack.time_usec = utc_now_usec();
	ack.ack_msgid = assignment.message_id;
	ack.ack_instance = assignment.instance_id;
	ack.origin_sysid = assignment.source_system;
	ack.ack_sysid = mavlink_system.sysid;
	ack.result = result;
	strncpy(ack.reason, reason != nullptr ? reason : "", sizeof(ack.reason) - 1);
	mavlink_msg_mavlink_m_ack_send_struct(_mavlink->get_channel(), &ack);
}

void MavlinkMHandler::send_control_command_ack(const mavlink_message_t &request, uint8_t result)
{
	if (_mavlink == nullptr) {
		return;
	}

	mavlink_command_long_t command{};
	mavlink_msg_command_long_decode(&request, &command);
	_mavlink->set_proto_version(2);

	mavlink_command_ack_t ack{};
	ack.command = command.command;
	ack.result = result;
	ack.progress = UINT8_MAX;
	ack.target_system = request.sysid;
	ack.target_component = request.compid;
	mavlink_msg_command_ack_send_struct(_mavlink->get_channel(), &ack);
}

void MavlinkMHandler::send_control_status()
{
	if (!control_enabled()) {
		return;
	}

	_target_status_sub.update(&_control_status);
	const bool status_fresh = _control_status.timestamp != 0
				  && hrt_elapsed_time(&_control_status.timestamp) < 1'000'000;
	const bool cue_status = status_fresh
				&& _control_status.message_id == OwnerDecisionTaskMessage
				&& _control_status.instance_id != 0;
	const bool pending = cue_status
			     && _control_status.state == mavlink_m_target_status_s::STATE_PENDING;
	const bool active = cue_status
			    && _control_status.state == mavlink_m_target_status_s::STATE_ACTIVE;
	const bool terminal = cue_status
			      && (_control_status.state == mavlink_m_target_status_s::STATE_REJECTED
				  || _control_status.state == mavlink_m_target_status_s::STATE_ABORTED
				  || _control_status.state == mavlink_m_target_status_s::STATE_EXPIRED);
	const uint32_t cue_id = cue_status ? _control_status.instance_id : 0;
	int32_t cue_bits = 0;
	static_assert(sizeof(cue_bits) == sizeof(cue_id), "owner cue ID must preserve 32 raw bits");
	memcpy(&cue_bits, &cue_id, sizeof(cue_bits));
	const uint32_t time_boot_ms = static_cast<uint32_t>(hrt_absolute_time() / 1000);

	auto send_int = [&](const char *name, int32_t value) {
		mavlink_named_value_int_t item{};
		item.time_boot_ms = time_boot_ms;
		strncpy(item.name, name, sizeof(item.name));
		item.value = value;
		mavlink_msg_named_value_int_send_struct(_mavlink->get_channel(), &item);
	};
	auto send_float = [&](const char *name, float value) {
		mavlink_named_value_float_t item{};
		item.time_boot_ms = time_boot_ms;
		strncpy(item.name, name, sizeof(item.name));
		item.value = value;
		mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &item);
	};

	_mavlink->set_proto_version(2);

	if (pending || active) {
		// Send a correlated, receiver-local snapshot before the actionable
		// status. AAGS does not expose Accept until it has AAGS_DID and the
		// coordinate fields for the same cue on this exact owner link.
		send_int("AAGS_DID", cue_bits);
		send_int("AAGS_LAT", _control_status.lat);
		send_int("AAGS_LON", _control_status.lon);
		send_float("AAGS_ALT", _control_status.alt_msl_m);
		const int32_t source = (static_cast<int32_t>(_control_status.source_system) << 8)
				       | static_cast<int32_t>(_control_status.source_component);
		send_int("AAGS_SRC", source);
		const int32_t configuration = (static_cast<int32_t>(_control_status.action_mode) << 8)
					      | static_cast<int32_t>(_control_status.cue_type);
		send_int("AAGS_CFG", configuration);
		send_int(pending ? "AAGS_PEND" : "AAGS_ACTV", cue_bits);

	} else if (terminal) {
		send_int("AAGS_DONE", cue_bits);

	} else {
		send_int("AAGS_IDLE", 0);
	}

	_last_control_status_send = hrt_absolute_time();
}

float MavlinkMHandler::wrap_180(float angle_deg)
{
	while (angle_deg > 180.f) {
		angle_deg -= 360.f;
	}

	while (angle_deg < -180.f) {
		angle_deg += 360.f;
	}

	return angle_deg;
}

void MavlinkMHandler::send_osd_vector()
{
	Assignment *display = find_pending();

	if (display == nullptr && _active.state == AssignmentState::Active) {
		display = &_active;
	}

	if (display == nullptr || _global_position.timestamp == 0
	    || hrt_elapsed_time(&_global_position.timestamp) > 2'000'000
	    || !PX4_ISFINITE(_global_position.lat) || !PX4_ISFINITE(_global_position.lon)) {
		return;
	}

	const double target_lat = display->lat * 1e-7;
	const double target_lon = display->lon * 1e-7;
	const float bearing_deg = math::degrees(get_bearing_to_next_waypoint(
				  _global_position.lat, _global_position.lon, target_lat, target_lon));
	float relative_deg = NAN;

	if (_attitude.timestamp != 0 && hrt_elapsed_time(&_attitude.timestamp) < 2'000'000) {
		const float yaw_deg = math::degrees(matrix::Eulerf(matrix::Quatf(_attitude.q)).psi());
		relative_deg = wrap_180(bearing_deg - yaw_deg);
	}

	mavlink_debug_vect_t osd{};
	osd.time_usec = hrt_absolute_time();
	strncpy(osd.name, "AAGS_TGT", sizeof(osd.name));
	osd.x = bearing_deg;
	osd.y = relative_deg;
	osd.z = get_distance_to_next_waypoint(_global_position.lat, _global_position.lon, target_lat, target_lon);
	mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &osd);
}

void MavlinkMHandler::publish_status()
{
	Assignment *display = find_pending();

	if (display == nullptr && _active.state == AssignmentState::Active) {
		display = &_active;
	}

	if (display == nullptr) {
		for (Assignment &assignment : _inbox) {
			if (assignment.state == AssignmentState::Queued) {
				display = &assignment;
				break;
			}
		}
	}

	if (display == nullptr && _terminal[0].state != AssignmentState::Empty) {
		display = &_terminal[0];
	}

	mavlink_m_target_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = mavlink_m_target_status_s::STATE_IDLE;
	status.bearing_deg = NAN;
	status.relative_bearing_deg = NAN;
	status.range_m = NAN;
	status.alt_msl_m = NAN;
	status.source_fresh = _source_last_seen != 0 && hrt_elapsed_time(&_source_last_seen) < SourceFreshTimeout;

	for (const Assignment &assignment : _inbox) {
		if (assignment.state != AssignmentState::Empty) {
			++status.queue_depth;
		}
	}

	if (display != nullptr) {
		status.assignment_time_usec = display->time_usec;
		status.track_identity_time_usec = display->track_identity_time_usec;
		status.message_id = display->message_id;
		status.instance_id = display->instance_id;
		status.target_set_id = display->target_set_id;
		status.source_system = display->source_system;
		status.source_component = display->source_component;
		status.origin_system = display->origin_system;
		status.target_class = display->target_class;
		status.target_force = display->target_force;
		status.cue_type = display->cue_type;
		status.action_mode = static_cast<uint8_t>(_action_mode);
		status.command_flags = display->command_flags;
		status.prompt = display->state == AssignmentState::Pending;
		status.restored = display->restored != 0;
		status.lat = display->lat;
		status.lon = display->lon;
		status.alt_msl_m = display->alt;
		memcpy(status.track_uid, display->track_uid, sizeof(status.track_uid));
		memcpy(status.name, display->name, sizeof(status.name) - 1);

		switch (display->state) {
		case AssignmentState::Pending: status.state = mavlink_m_target_status_s::STATE_PENDING; break;
		case AssignmentState::Active: status.state = mavlink_m_target_status_s::STATE_ACTIVE; break;
		case AssignmentState::Queued: status.state = mavlink_m_target_status_s::STATE_QUEUED; break;
		case AssignmentState::Rejected: status.state = mavlink_m_target_status_s::STATE_REJECTED; break;
		case AssignmentState::Aborted: status.state = mavlink_m_target_status_s::STATE_ABORTED; break;
		case AssignmentState::Expired: status.state = mavlink_m_target_status_s::STATE_EXPIRED; break;
		case AssignmentState::Empty: break;
		}

		if (_global_position.timestamp != 0 && hrt_elapsed_time(&_global_position.timestamp) < 2'000'000
		    && PX4_ISFINITE(_global_position.lat) && PX4_ISFINITE(_global_position.lon)) {
			const double target_lat = display->lat * 1e-7;
			const double target_lon = display->lon * 1e-7;
			status.position_valid = true;
			status.bearing_deg = math::degrees(get_bearing_to_next_waypoint(
						      _global_position.lat, _global_position.lon, target_lat, target_lon));
			status.range_m = get_distance_to_next_waypoint(
					 _global_position.lat, _global_position.lon, target_lat, target_lon);

			if (_attitude.timestamp != 0 && hrt_elapsed_time(&_attitude.timestamp) < 2'000'000) {
				const float yaw_deg = math::degrees(matrix::Eulerf(matrix::Quatf(_attitude.q)).psi());
				status.relative_bearing_deg = wrap_180(status.bearing_deg - yaw_deg);
			}
		}

		status.track_identity_valid = !all_zero(display->track_uid, sizeof(display->track_uid));
	}

	_status_pub.publish(status);
	_last_status_publish = status.timestamp;
}

void MavlinkMHandler::update()
{
	_global_position_sub.update(&_global_position);
	_attitude_sub.update(&_attitude);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_status_sub.update(&_vehicle_status);

	const bool cue_receiver_enabled = enabled();
	const bool owner_control_enabled = control_enabled();

	if (!cue_receiver_enabled && !owner_control_enabled) {
		return;
	}

	// All MAVLink-M message IDs are above the MAVLink 1 range.
	_mavlink->set_proto_version(2);

	const hrt_abstime now = hrt_absolute_time();

	if (cue_receiver_enabled) {
		if (!_state_loaded) {
			(void)load_state();
			_state_loaded = true;
		}

		const uint64_t now_usec = utc_now_usec();
		expire_assignments(now_usec);
		update_rc();
		update_local_decision();

		if (now - _last_osd_send >= 200'000) {
			send_osd_vector();
			_last_osd_send = now;
		}

		if (now - _last_status_publish >= 200'000) {
			publish_status();
		}
	}

	if (owner_control_enabled && now - _last_control_status_send >= 200'000) {
		send_control_status();
	}
}

bool MavlinkMHandler::configure_signing()
{
	if (_mavlink == nullptr) {
		return false;
	}

	mavlink_status_t *tx_status = _mavlink->get_status();
	const bool selected_cue_link = _endpoint_configuration_valid
				       && _mode == 2
				       && _mavlink->get_instance_id() == _instance;
	const bool selected_control_link = _control_configuration_valid
					   && _mode == 2
					   && _mavlink->get_instance_id() == _control_instance;
	const bool selected_physical_link = selected_cue_link || selected_control_link;

	if (!selected_physical_link) {
		if (tx_status->signing == &_signing) {
			tx_status->signing = nullptr;
			tx_status->signing_streams = nullptr;
		}

		if (_receiver_status != nullptr && _receiver_status->signing == &_signing) {
			_receiver_status->signing = nullptr;
			_receiver_status->signing_streams = nullptr;
		}

		_signing_ready = false;
		return _mode != 2;
	}

	if (!_signing_ready) {
		uint8_t key[32] {};
		const uint64_t timestamp = signing_timestamp();

		if (timestamp == 0 || !load_signing_key(key)) {
			if (tx_status->signing == &_signing) {
				tx_status->signing = nullptr;
				tx_status->signing_streams = nullptr;
			}

			if (_receiver_status != nullptr && _receiver_status->signing == &_signing) {
				_receiver_status->signing = nullptr;
				_receiver_status->signing_streams = nullptr;
			}

			PX4_ERR("MAVLink-M signed mode unavailable: key/time invalid");
			return false;
		}

		_signing = mavlink_signing_t{};
		_signing_streams = mavlink_signing_streams_t{};
		_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
		_signing.link_id = static_cast<uint8_t>(selected_control_link
						       ? _control_link_id : _link_id);
		_signing.timestamp = timestamp;
		memcpy(_signing.secret_key, key, sizeof(_signing.secret_key));
		_signing.accept_unsigned_callback = nullptr;
		_signing_ready = true;
		PX4_INFO("MAVLink-M signed physical link active");
	}

	tx_status->signing = &_signing;
	tx_status->signing_streams = &_signing_streams;

	if (_receiver_status != nullptr) {
		_receiver_status->signing = &_signing;
		_receiver_status->signing_streams = &_signing_streams;
	}

	return true;
}

bool MavlinkMHandler::load_signing_key(uint8_t key[32]) const
{
#ifdef PX4_STORAGEDIR
	struct stat info {};

	if (::stat(SigningKeyPath, &info) != 0 || (info.st_size != 32 && info.st_size != 64)) {
		return false;
	}

#ifdef __PX4_POSIX
	if ((info.st_mode & (S_IRWXG | S_IRWXO)) != 0) {
		PX4_ERR("MAVLink-M signing key must be owner-only");
		return false;
	}
#endif

	const int fd = ::open(SigningKeyPath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	uint8_t encoded[64] {};
	size_t total = 0;

	while (total < static_cast<size_t>(info.st_size)) {
		const ssize_t count = ::read(fd, encoded + total, static_cast<size_t>(info.st_size) - total);

		if (count <= 0) {
			::close(fd);
			return false;
		}

		total += static_cast<size_t>(count);
	}

	::close(fd);

	if (total == 32) {
		memcpy(key, encoded, 32);
		return !all_zero(key, 32);
	}

	for (size_t i = 0; i < 32; ++i) {
		const int high = hex_nibble(static_cast<char>(encoded[i * 2]));
		const int low = hex_nibble(static_cast<char>(encoded[i * 2 + 1]));

		if (high < 0 || low < 0) {
			memset(key, 0, 32);
			return false;
		}

		key[i] = static_cast<uint8_t>((high << 4) | low);
	}

	return !all_zero(key, 32);
#else
	(void)key;
	return false;
#endif
}

uint64_t MavlinkMHandler::signing_timestamp()
{
	struct timespec ts {};

	if (clock_gettime(CLOCK_REALTIME, &ts) != 0
	    || ts.tv_sec <= static_cast<time_t>(MavlinkSigningEpochSeconds)) {
		return 0;
	}

	return (static_cast<uint64_t>(ts.tv_sec) - MavlinkSigningEpochSeconds) * 100'000ULL
	       + static_cast<uint64_t>(ts.tv_nsec) / 10'000ULL;
}

uint64_t MavlinkMHandler::utc_now_usec()
{
	struct timespec ts {};

	if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
		return 0;
	}

	const int64_t seconds = static_cast<int64_t>(ts.tv_sec);

	if (seconds < 0) {
		return 0;
	}

	return static_cast<uint64_t>(seconds) * 1'000'000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1'000ULL;
}

uint32_t MavlinkMHandler::state_crc(const PersistedState &state) const
{
	PersistedState copy = state;
	copy.crc = 0;
	return crc32_signature(0, sizeof(copy), reinterpret_cast<const uint8_t *>(&copy));
}

bool MavlinkMHandler::load_state()
{
#ifdef PX4_STORAGEDIR
	const int fd = ::open(StatePath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	PersistedState state{};
	const ssize_t bytes = ::read(fd, &state, sizeof(state));
	::close(fd);

	if (bytes != static_cast<ssize_t>(sizeof(state)) || state.magic != PersistenceMagic
	    || state.version != PersistenceVersion || state.size != sizeof(state)
	    || state.crc != state_crc(state)) {
		PX4_WARN("ignoring invalid MAVLink-M persisted state");
		return false;
	}

	if (state.mavlink_instance != static_cast<uint8_t>(_instance)
	    || state.source_system != static_cast<uint8_t>(_source_system)
	    || state.source_component != static_cast<uint8_t>(_source_component)
	    || strncmp(state.profile_hash, AAGS_MAVLINK_M_CORE_XML_SHA256, sizeof(state.profile_hash)) != 0) {
		PX4_WARN("ignoring MAVLink-M state for different endpoint/profile");
		return false;
	}

	_active = state.active;
	memcpy(_inbox, state.inbox, sizeof(_inbox));
	memcpy(_terminal, state.terminal, sizeof(_terminal));
	memcpy(_controls, state.controls, sizeof(_controls));

	if (_active.state != AssignmentState::Empty) {
		_active.restored = 1;
	}

	bool migrated_legacy_queue = false;

	for (Assignment &assignment : _inbox) {
		if (assignment.state == AssignmentState::Queued) {
			// Version 6 firmware could durably ACK a second cue as accepted
			// while leaving it queued. The finalized profile has no safe
			// implicit-promotion contract, so migrate it back to RECEIVED and
			// require a fresh local decision after the active cue clears.
			assignment.state = AssignmentState::Pending;
			assignment.last_ack_result = MAVLINK_M_ACK_RECEIVED;
			++assignment.status_sequence;
			migrated_legacy_queue = true;
			PX4_WARN("restored legacy queued MAVLink-M task as pending");
		}

		if (assignment.state != AssignmentState::Empty) {
			assignment.restored = 1;
		}

		assignment.name[sizeof(assignment.name) - 1] = '\0';
	}

	_active.name[sizeof(_active.name) - 1] = '\0';

	for (Assignment &assignment : _terminal) {
		assignment.name[sizeof(assignment.name) - 1] = '\0';
	}

	if (migrated_legacy_queue && !save_state()) {
		PX4_WARN("failed to persist legacy MAVLink-M queue migration");
	}

	PX4_INFO("restored MAVLink-M assignment state");
	return true;
#else
	return false;
#endif
}

bool MavlinkMHandler::save_state()
{
#ifdef PX4_STORAGEDIR
	PersistedState state{};
	state.magic = PersistenceMagic;
	state.version = PersistenceVersion;
	state.size = static_cast<uint16_t>(sizeof(state));
	state.mavlink_instance = static_cast<uint8_t>(_instance);
	state.source_system = static_cast<uint8_t>(_source_system);
	state.source_component = static_cast<uint8_t>(_source_component);
	strncpy(state.profile_hash, AAGS_MAVLINK_M_CORE_XML_SHA256, sizeof(state.profile_hash) - 1);
	state.active = _active;
	memcpy(state.inbox, _inbox, sizeof(_inbox));
	memcpy(state.terminal, _terminal, sizeof(_terminal));
	memcpy(state.controls, _controls, sizeof(_controls));
	state.crc = state_crc(state);

	const int fd = ::open(StateTempPath, O_WRONLY | O_CREAT | O_TRUNC, 0600);

	if (fd < 0) {
		PX4_ERR("MAVLink-M state open failed (%d)", errno);
		return false;
	}

	const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&state);
	size_t remaining = sizeof(state);
	bool success = true;

	while (remaining > 0) {
		const ssize_t written = ::write(fd, bytes, remaining);

		if (written <= 0) {
			success = false;
			break;
		}

		bytes += written;
		remaining -= written;
	}

	if (success && fsync(fd) != 0) {
		success = false;
	}

	::close(fd);

	if (!success || rename(StateTempPath, StatePath) != 0) {
		(void)unlink(StateTempPath);
		PX4_ERR("MAVLink-M state commit failed (%d)", errno);
		return false;
	}

	return true;
#else
	PX4_ERR("MAVLink-M durable storage unavailable");
	return false;
#endif
}
