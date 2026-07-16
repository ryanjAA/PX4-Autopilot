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
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static_assert(MAVLINK_MSG_ID_TRACK_IDENTITY == 54000, "unexpected MAVLink-M TRACK_IDENTITY id");
static_assert(MAVLINK_MSG_ID_TARGET_CUE == 54001, "unexpected MAVLink-M TARGET_CUE id");
static_assert(MAVLINK_MSG_ID_TARGET_HANDOVER == 54002, "unexpected MAVLink-M TARGET_HANDOVER id");
static_assert(MAVLINK_MSG_ID_PARTICIPANT_POSITION == 54003, "unexpected MAVLink-M PARTICIPANT_POSITION id");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK == 54004, "unexpected MAVLink-M ACK id");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_TASK_STATUS == 54005, "unexpected MAVLink-M task status id");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL == 54006, "unexpected MAVLink-M task control id");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY == 54007, "unexpected MAVLink-M capability id");
static_assert(MAVLINK_MSG_ID_TARGET_CUE_MIN_LEN == 68 && MAVLINK_MSG_ID_TARGET_CUE_LEN == 94,
	      "unexpected private TARGET_CUE layout");
static_assert(MAVLINK_MSG_ID_TARGET_HANDOVER_MIN_LEN == 191 && MAVLINK_MSG_ID_TARGET_HANDOVER_LEN == 213,
	      "unexpected private TARGET_HANDOVER layout");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_MIN_LEN == 69 && MAVLINK_MSG_ID_MAVLINK_M_ACK_LEN == 71,
	      "unexpected private ACK layout");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_TASK_STATUS_LEN == 65, "unexpected private task status layout");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL_LEN == 69, "unexpected private task control layout");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY_LEN == 121, "unexpected private capability layout");

namespace
{

constexpr uint64_t Unix2020Usec = 1'577'836'800ULL * 1'000'000ULL;
constexpr uint64_t MavlinkSigningEpochSeconds = 1'420'070'400ULL;
constexpr uint64_t CapabilityLifetimeUsec = 15'000'000ULL;
constexpr uint8_t CueAddressedLength = 78;
constexpr uint8_t HandoverAddressedLength = 197;
constexpr uint8_t TaskStatusRequiredLength = 25;
constexpr uint8_t TaskControlRequiredLength = 29;
constexpr uint8_t CapabilityRequiredLength = 116;
constexpr uint32_t RequiredSourceCapabilities = MAVLINK_M_CAPABILITY_APPLICATION_RECEIPT
		| MAVLINK_M_CAPABILITY_LOCAL_DECISION
		| MAVLINK_M_CAPABILITY_TASK_STATUS
		| MAVLINK_M_CAPABILITY_TASK_CONTROL
		| MAVLINK_M_CAPABILITY_HANDOVER
		| MAVLINK_M_CAPABILITY_INERT_ONLY;

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

bool valid_target_class(uint8_t target_class)
{
	return target_class <= MAVLINK_M_TARGET_CLASS_IFV_9
	       || target_class == MAVLINK_M_TARGET_CLASS_ROAD
	       || target_class == MAVLINK_M_TARGET_CLASS_STRUCTURE;
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
	_param_rc_channel = param_find("MAV_M_RC_CH");
	_param_rc_reject = param_find("MAV_M_RC_REJ");
	_param_rc_accept = param_find("MAV_M_RC_ACC");
	_param_max_age = param_find("MAV_M_MAX_AGE");
	update_parameters();
}

void MavlinkMHandler::configure_receiver_status(mavlink_status_t *status)
{
	_receiver_status = status;
	(void)configure_signing();
}

void MavlinkMHandler::update_parameters()
{
	const int32_t previous_instance = _instance;
	const int32_t previous_source_system = _source_system;
	const int32_t previous_source_component = _source_component;

	get_parameter(_param_mode, _mode);
	get_parameter(_param_link_id, _link_id);
	get_parameter(_param_instance, _instance);
	get_parameter(_param_source_system, _source_system);
	get_parameter(_param_source_component, _source_component);
	get_parameter(_param_rc_channel, _rc_channel);
	get_parameter(_param_rc_reject, _rc_reject);
	get_parameter(_param_rc_accept, _rc_accept);
	get_parameter(_param_max_age, _max_age_s);

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
		_source_capability_valid_until_usec = 0;
		_source_capability_flags = 0;
		_last_rc_position = RcPosition::Unknown;
		_rc_center_latched = false;
	}

	(void)configure_signing();
}

bool MavlinkMHandler::enabled() const
{
	return _mavlink != nullptr && (_mode == 1 || _mode == 2)
	       && _mavlink->get_instance_id() == _instance
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

bool MavlinkMHandler::local_address_matches(uint8_t target_system, uint8_t target_component) const
{
	return target_system != 0 && target_component != 0 && _mavlink != nullptr
	       && target_system == static_cast<uint8_t>(_mavlink->get_system_id())
	       && target_component == static_cast<uint8_t>(_mavlink->get_component_id());
}

bool MavlinkMHandler::capability_current() const
{
	const uint64_t now_usec = utc_now_usec();
	return _source_last_seen != 0 && hrt_elapsed_time(&_source_last_seen) < SourceFreshTimeout
	       && now_usec >= Unix2020Usec && now_usec <= _source_capability_valid_until_usec
	       && (_source_capability_flags & RequiredSourceCapabilities) == RequiredSourceCapabilities
	       && (!signing_required()
		   || (_source_capability_flags & MAVLINK_M_CAPABILITY_SIGNING_REQUIRED) != 0);
}

bool MavlinkMHandler::task_message_allowed(const mavlink_message_t &message) const
{
	return enabled() && source_matches(message) && capability_current();
}

void MavlinkMHandler::handle_message(const mavlink_message_t &message)
{
	if (!enabled()) {
		return;
	}

	if (message.msgid != MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY
	    && message.msgid != MAVLINK_MSG_ID_TARGET_CUE
	    && message.msgid != MAVLINK_MSG_ID_TARGET_HANDOVER
	    && message.msgid != MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL
	    && message.msgid != MAVLINK_MSG_ID_MAVLINK_M_TASK_STATUS) {
		return;
	}

	if (!source_matches(message)) {
		return;
	}

	if (message.msgid == MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY) {
		handle_capability(message);
		return;
	}

	if (!task_message_allowed(message)) {
		return;
	}

	_source_last_seen = hrt_absolute_time();

	if (message.msgid == MAVLINK_MSG_ID_TARGET_CUE) {
		handle_target_cue(message);

	} else if (message.msgid == MAVLINK_MSG_ID_TARGET_HANDOVER) {
		handle_target_handover(message);

	} else if (message.msgid == MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL) {
		handle_task_control(message);

	} else {
		handle_task_status(message);
	}
}

void MavlinkMHandler::handle_capability(const mavlink_message_t &message)
{
	if (message.len < CapabilityRequiredLength) {
		return;
	}

	mavlink_mavlink_m_capability_t capability{};
	mavlink_msg_mavlink_m_capability_decode(&message, &capability);
	uint8_t expected_hash[32] {};
	const uint64_t now_usec = utc_now_usec();

	if (!profile_hash_bytes(expected_hash) || now_usec < Unix2020Usec
	    || capability.time_usec < Unix2020Usec
	    || capability.valid_until_usec <= capability.time_usec
	    || now_usec > capability.valid_until_usec
	    || capability.time_usec > now_usec + CapabilityLifetimeUsec
	    || capability.valid_until_usec > capability.time_usec + CapabilityLifetimeUsec
	    || capability.endpoint_type != MAVLINK_M_ENDPOINT_AAGS
	    || capability.endpoint_system == 0 || capability.endpoint_component == 0
	    || capability.endpoint_system != message.sysid || capability.endpoint_component != message.compid
	    || capability.protocol_major != AAGS_MAVLINK_M_PROTOCOL_MAJOR
	    || capability.protocol_minor != AAGS_MAVLINK_M_PROTOCOL_MINOR
	    || memcmp(capability.profile_sha256, expected_hash, sizeof(expected_hash)) != 0
	    || strncmp(capability.profile_id, AAGS_MAVLINK_M_PROFILE_ID, sizeof(capability.profile_id)) != 0
	    || strncmp(capability.profile_version, AAGS_MAVLINK_M_PROFILE_VERSION,
		       sizeof(capability.profile_version)) != 0
	    || (capability.capability_flags & RequiredSourceCapabilities) != RequiredSourceCapabilities
	    || (signing_required()
		&& (capability.capability_flags & MAVLINK_M_CAPABILITY_SIGNING_REQUIRED) == 0)) {
		return;
	}

	_source_capability_valid_until_usec = capability.valid_until_usec;
	_source_capability_flags = capability.capability_flags;
	_source_last_seen = hrt_absolute_time();
}

void MavlinkMHandler::handle_track_identity(const mavlink_message_t &message)
{
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
				PX4_WARN("MAVLink-M target-set identity collision");
				return;
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
		    && assignment.target_set_id == identity.target_set_id
		    && assignment.source_system == identity.source_system
		    && assignment.source_component == identity.source_component
		    && assignment.origin_system == identity.origin_system
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
	if (message.len < CueAddressedLength) {
		return;
	}

	mavlink_target_cue_t cue{};
	mavlink_msg_target_cue_decode(&message, &cue);

	if (!local_address_matches(cue.target_system, cue.target_component)) {
		return;
	}

	Assignment assignment{};
	assignment.time_usec = cue.time_usec;
	assignment.valid_until_usec = cue.valid_until_usec;
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
	memcpy(assignment.track_uid, cue.track_uid, sizeof(assignment.track_uid));
	memcpy(assignment.name, cue.name, sizeof(cue.name));
	assignment.name[sizeof(assignment.name) - 1] = '\0';

	if (cue.origin_sysid == 0 || cue.valid_until_usec == 0) {
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
	if (message.len < HandoverAddressedLength) {
		return;
	}

	mavlink_target_handover_t handover{};
	mavlink_msg_target_handover_decode(&message, &handover);

	if (!local_address_matches(handover.target_system, handover.target_component)) {
		return;
	}

	Assignment assignment{};
	assignment.time_usec = handover.time_usec;
	assignment.valid_until_usec = handover.valid_until_usec;
	assignment.track_identity_time_usec = handover.detected_first_usec;
	assignment.message_id = MAVLINK_MSG_ID_TARGET_HANDOVER;
	assignment.instance_id = handover.handover_id;
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

	if (handover.handover_id == 0 || handover.valid_until_usec == 0
	    || handover.detected_first_usec < Unix2020Usec
	    || handover.detected_first_usec > handover.time_usec
	    || !valid_target_class(handover.target_class) || !valid_target_force(handover.target_force)
	    || handover.match_media_type >= MAVLINK_M_MATCH_MEDIA_TYPE_ENUM_END) {
		send_ack(assignment, MAVLINK_M_ACK_FAILED, "invalid handover fields");
		return;
	}

	store_assignment(assignment);
}

void MavlinkMHandler::handle_task_control(const mavlink_message_t &message)
{
	if (message.len < TaskControlRequiredLength) {
		return;
	}

	mavlink_mavlink_m_task_control_t control{};
	mavlink_msg_mavlink_m_task_control_decode(&message, &control);

	if (!local_address_matches(control.target_system, control.target_component)) {
		return;
	}

	Assignment response{};
	response.time_usec = control.time_usec;
	response.message_id = MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL;
	response.instance_id = control.control_id;
	response.source_system = message.sysid;
	response.source_component = message.compid;
	response.origin_system = message.sysid;

	const uint32_t payload_crc = crc32_signature(0, message.len,
			reinterpret_cast<const uint8_t *>(message.payload64));

	if (ControlRecord *prior = find_control(control.control_id)) {
		if (prior->payload_crc == payload_crc && prior->task_msgid == control.task_msgid
		    && prior->task_instance == control.task_instance
		    && prior->replacement_instance == control.replacement_instance
		    && prior->action == control.action) {
			send_ack(response, prior->ack_result, "duplicate idempotent control");

		} else {
			send_ack(response, MAVLINK_M_ACK_FAILED, "immutable control collision");
		}

		return;
	}

	const char *time_reason = nullptr;
	uint8_t time_result = MAVLINK_M_ACK_FAILED;

	if (control.control_id == 0
	    || (control.task_msgid != MAVLINK_MSG_ID_TARGET_CUE
		&& control.task_msgid != MAVLINK_MSG_ID_TARGET_HANDOVER)
	    || control.task_instance == 0
	    || control.action > MAVLINK_M_TASK_CONTROL_SUPERSEDE
	    || (control.action == MAVLINK_M_TASK_CONTROL_CANCEL && control.replacement_instance != 0)
	    || (control.action == MAVLINK_M_TASK_CONTROL_SUPERSEDE
		&& (control.replacement_instance == 0 || control.replacement_instance == control.task_instance))
	    || !validate_time(control.time_usec, 0, &time_reason, &time_result)) {
		send_ack(response, time_result, time_reason != nullptr ? time_reason : "invalid task control");
		return;
	}

	Assignment *task = find_task(control.task_msgid, control.task_instance);
	Assignment *replacement = control.action == MAVLINK_M_TASK_CONTROL_SUPERSEDE
				  ? find_task(control.task_msgid, control.replacement_instance) : nullptr;
	uint8_t result = MAVLINK_M_ACK_ACCEPTED;
	const char *reason = control.action == MAVLINK_M_TASK_CONTROL_CANCEL
			     ? "task cancelled" : "task superseded";

	if (task == nullptr) {
		result = MAVLINK_M_ACK_REJECTED;
		reason = "unknown task";

	} else if (control.action == MAVLINK_M_TASK_CONTROL_SUPERSEDE
		   && (replacement == nullptr || replacement->state == AssignmentState::Rejected
		       || replacement->state == AssignmentState::Aborted
		       || replacement->state == AssignmentState::Expired)) {
		result = MAVLINK_M_ACK_REJECTED;
		reason = "replacement task unavailable";
	}

	const Assignment active_before = _active;
	const Assignment inbox_before[InboxCapacity] {_inbox[0], _inbox[1]};
	const Assignment terminal_evicted = _terminal[TerminalCapacity - 1];
	const ControlRecord control_evicted = _controls[ControlCapacity - 1];
	Assignment status_assignment{};
	bool send_aborted_status = false;
	bool terminal_changed = false;

	if (result == MAVLINK_M_ACK_ACCEPTED && task != nullptr
	    && task->state != AssignmentState::Rejected && task->state != AssignmentState::Aborted
	    && task->state != AssignmentState::Expired) {
		status_assignment = *task;
		++status_assignment.status_sequence;

		if (task == &_active) {
			_active = Assignment{};

		} else if (task == &_inbox[0] || task == &_inbox[1]) {
			remove_inbox(task);
		}

		remember_terminal(status_assignment, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);
		promote_queued();
		send_aborted_status = true;
		terminal_changed = true;
	}

	for (unsigned i = ControlCapacity - 1; i > 0; --i) {
		_controls[i] = _controls[i - 1];
	}

	_controls[0].control_id = control.control_id;
	_controls[0].task_msgid = control.task_msgid;
	_controls[0].task_instance = control.task_instance;
	_controls[0].replacement_instance = control.replacement_instance;
	_controls[0].payload_crc = payload_crc;
	_controls[0].source_system = message.sysid;
	_controls[0].source_component = message.compid;
	_controls[0].action = control.action;
	_controls[0].ack_result = result;

	if (!save_state()) {
		_active = active_before;
		_inbox[0] = inbox_before[0];
		_inbox[1] = inbox_before[1];

		if (terminal_changed) {
			undo_remember_terminal(terminal_evicted);
		}

		for (unsigned i = 0; i + 1 < ControlCapacity; ++i) {
			_controls[i] = _controls[i + 1];
		}

		_controls[ControlCapacity - 1] = control_evicted;
		send_ack(response, MAVLINK_M_ACK_FAILED, "control storage failed");
		return;
	}

	send_ack(response, result, reason);

	if (send_aborted_status) {
		send_task_status(status_assignment, MAVLINK_M_TASK_STATE_ABORTED, reason, control.reason_code);
	}

	if (_active.state == AssignmentState::Active
	    && (_active.instance_id != active_before.instance_id
		|| _active.message_id != active_before.message_id)) {
		send_task_status(_active, MAVLINK_M_TASK_STATE_ACTIVE,
				 "previous task ended; accepted queued task is active");
	}

	publish_status();
}

void MavlinkMHandler::handle_task_status(const mavlink_message_t &message)
{
	if (message.len < TaskStatusRequiredLength) {
		return;
	}

	mavlink_mavlink_m_task_status_t status{};
	mavlink_msg_mavlink_m_task_status_decode(&message, &status);

	if (!local_address_matches(status.target_system, status.target_component)
	    || status.task_instance == 0 || status.status_sequence == 0
	    || (status.task_msgid != MAVLINK_MSG_ID_TARGET_CUE
		&& status.task_msgid != MAVLINK_MSG_ID_TARGET_HANDOVER)
	    || status.state == MAVLINK_M_TASK_STATE_UNKNOWN
	    || status.state > MAVLINK_M_TASK_STATE_EXPIRED) {
		return;
	}

	const char *reason = nullptr;
	uint8_t result = MAVLINK_M_ACK_FAILED;
	(void)validate_time(status.time_usec, 0, &reason, &result);
	// Remote status is deliberately informational. It never mutates local pilot
	// decisions, navigation state, flight mode, arming state, or payload state.
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

	if (_max_age_s > 0) {
		if (now_usec < Unix2020Usec) {
			*reason = "vehicle UTC unavailable";
			*ack_result = MAVLINK_M_ACK_FAILED;
			return false;
		}

		if (time_usec < Unix2020Usec) {
			*reason = "assignment timestamp invalid";
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

const MavlinkMHandler::TrackIdentity *MavlinkMHandler::find_track_identity(const Assignment &assignment) const
{
	if (assignment.target_set_id == 0) {
		return nullptr;
	}

	for (const TrackIdentity &identity : _track_identities) {
		if (identity.target_set_id == assignment.target_set_id
		    && identity.source_system == assignment.source_system
		    && identity.source_component == assignment.source_component
		    && identity.origin_system == assignment.origin_system) {
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

MavlinkMHandler::Assignment *MavlinkMHandler::find_pending()
{
	for (Assignment &assignment : _inbox) {
		if (assignment.state == AssignmentState::Pending) {
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

void MavlinkMHandler::promote_queued()
{
	if (_active.state != AssignmentState::Empty) {
		return;
	}

	for (Assignment &assignment : _inbox) {
		if (assignment.state == AssignmentState::Queued) {
			_active = assignment;
			_active.state = AssignmentState::Active;
			++_active.status_sequence;
			remove_inbox(&assignment);
			return;
		}
	}
}

void MavlinkMHandler::accept_pending()
{
	Assignment *pending = find_pending();

	if (pending == nullptr) {
		return;
	}

	if (_source_last_seen == 0 || hrt_elapsed_time(&_source_last_seen) >= SourceFreshTimeout) {
		PX4_WARN("MAVLink-M accept blocked: source endpoint stale");
		return;
	}

	const Assignment decided = *pending;
	const Assignment active_before = _active;
	const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
	Assignment *accepted = nullptr;

	if (_active.state == AssignmentState::Empty) {
		_active = decided;
		_active.state = AssignmentState::Active;
		_active.last_ack_result = MAVLINK_M_ACK_ACCEPTED;
		++_active.status_sequence;
		remove_inbox(pending);
		accepted = &_active;

	} else {
		pending->state = AssignmentState::Queued;
		pending->last_ack_result = MAVLINK_M_ACK_ACCEPTED;
		accepted = pending;
	}

	if (!save_state()) {
		_active = active_before;
		_inbox[0] = inbox_before[0];
		_inbox[1] = inbox_before[1];
		send_ack(decided, MAVLINK_M_ACK_FAILED, "decision storage failed");
		return;
	}

	Assignment acknowledged = *accepted;
	acknowledged.last_ack_result = MAVLINK_M_ACK_ACCEPTED;
	send_ack(acknowledged, MAVLINK_M_ACK_ACCEPTED,
		 _active.instance_id == decided.instance_id ? "pilot accepted active target" : "pilot accepted queued target");

	if (_active.instance_id == decided.instance_id) {
		send_task_status(acknowledged, MAVLINK_M_TASK_STATE_ACTIVE, "pilot accepted observation task");
	}

	publish_status();
}

void MavlinkMHandler::reject_pending_or_abort_active()
{
	Assignment *pending = find_pending();

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

		send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "pilot rejected assignment");
		send_task_status(_terminal[0], MAVLINK_M_TASK_STATE_ABORTED, "pilot rejected observation task");
		publish_status();
		return;
	}

	if (_active.state == AssignmentState::Active) {
		Assignment decided = _active;
		++decided.status_sequence;
		const Assignment active_before = _active;
		const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
		const Assignment terminal_evicted = _terminal[TerminalCapacity - 1];
		_active = Assignment{};
		remember_terminal(decided, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);
		promote_queued();

		if (!save_state()) {
			_active = active_before;
			_inbox[0] = inbox_before[0];
			_inbox[1] = inbox_before[1];
			undo_remember_terminal(terminal_evicted);
			send_ack(decided, MAVLINK_M_ACK_FAILED, "abort storage failed");
			return;
		}

		send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "pilot aborted active target");
		send_task_status(_terminal[0], MAVLINK_M_TASK_STATE_ABORTED, "pilot aborted observation task");

		if (_active.state == AssignmentState::Active) {
			send_task_status(_active, MAVLINK_M_TASK_STATE_ACTIVE,
					 "previous task ended; accepted queued task is active");
		}

		publish_status();
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

	if (_active.state == AssignmentState::Active && _active.valid_until_usec != 0
	    && now_usec > _active.valid_until_usec) {
		Assignment expired = _active;
		++expired.status_sequence;
		_active = Assignment{};
		terminal_evicted[expired_count] = _terminal[TerminalCapacity - 1];
		remember_terminal(expired, AssignmentState::Expired, MAVLINK_M_ACK_EXPIRED);
		expired_assignments[expired_count++] = expired;
		changed = true;
	}

	for (unsigned i = 0; i < InboxCapacity;) {
		if (_inbox[i].state != AssignmentState::Empty && _inbox[i].valid_until_usec != 0
		    && now_usec > _inbox[i].valid_until_usec) {
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
		promote_queued();

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
				send_task_status(expired_assignments[i], MAVLINK_M_TASK_STATE_EXPIRED,
						 "observation task expired");
			}

			if (_active.state == AssignmentState::Active
			    && (_active.instance_id != active_before.instance_id
				|| _active.message_id != active_before.message_id)) {
				send_task_status(_active, MAVLINK_M_TASK_STATE_ACTIVE,
						 "previous task expired; accepted queued task is active");
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
	ack.target_system = assignment.source_system;
	ack.target_component = assignment.source_component;
	strncpy(ack.reason, reason != nullptr ? reason : "", sizeof(ack.reason) - 1);
	mavlink_msg_mavlink_m_ack_send_struct(_mavlink->get_channel(), &ack);
}

void MavlinkMHandler::send_task_status(const Assignment &assignment, uint8_t state, const char *reason,
				       uint16_t reason_code)
{
	if (!enabled() || assignment.source_system == 0 || assignment.source_component == 0
	    || assignment.instance_id == 0 || assignment.status_sequence == 0) {
		return;
	}

	_mavlink->set_proto_version(2);
	mavlink_mavlink_m_task_status_t status{};
	status.time_usec = utc_now_usec();
	status.task_msgid = assignment.message_id;
	status.task_instance = assignment.instance_id;
	status.status_sequence = assignment.status_sequence;
	status.reason_code = reason_code;
	status.target_system = assignment.source_system;
	status.target_component = assignment.source_component;
	status.state = state;
	strncpy(status.reason, reason != nullptr ? reason : "", sizeof(status.reason) - 1);
	mavlink_msg_mavlink_m_task_status_send_struct(_mavlink->get_channel(), &status);
}

void MavlinkMHandler::send_capability()
{
	if (!enabled()) {
		return;
	}

	uint8_t hash[32] {};
	const uint64_t now_usec = utc_now_usec();

	if (now_usec < Unix2020Usec || !profile_hash_bytes(hash)) {
		return;
	}

	_mavlink->set_proto_version(2);
	mavlink_mavlink_m_capability_t capability{};
	capability.time_usec = now_usec;
	capability.valid_until_usec = now_usec + CapabilityLifetimeUsec;
	capability.capability_flags = MAVLINK_M_CAPABILITY_CUE_RECEIVE
				      | MAVLINK_M_CAPABILITY_APPLICATION_RECEIPT
				      | MAVLINK_M_CAPABILITY_LOCAL_DECISION
				      | MAVLINK_M_CAPABILITY_TASK_STATUS
				      | MAVLINK_M_CAPABILITY_TASK_CONTROL
				      | MAVLINK_M_CAPABILITY_HANDOVER
				      | MAVLINK_M_CAPABILITY_INERT_ONLY;

	if (signing_required()) {
		capability.capability_flags |= MAVLINK_M_CAPABILITY_SIGNING_REQUIRED;
	}

	capability.endpoint_type = MAVLINK_M_ENDPOINT_PX4_PILOT;
	capability.endpoint_system = static_cast<uint8_t>(_mavlink->get_system_id());
	capability.endpoint_component = static_cast<uint8_t>(_mavlink->get_component_id());
	capability.protocol_major = AAGS_MAVLINK_M_PROTOCOL_MAJOR;
	capability.protocol_minor = AAGS_MAVLINK_M_PROTOCOL_MINOR;
	memcpy(capability.profile_sha256, hash, sizeof(hash));
	strncpy(capability.profile_id, AAGS_MAVLINK_M_PROFILE_ID, sizeof(capability.profile_id) - 1);
	strncpy(capability.profile_version, AAGS_MAVLINK_M_PROFILE_VERSION, sizeof(capability.profile_version) - 1);
	mavlink_msg_mavlink_m_capability_send_struct(_mavlink->get_channel(), &capability);
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

	if (!enabled()) {
		return;
	}

	// All MAVLink-M message IDs are above the MAVLink 1 range.
	_mavlink->set_proto_version(2);

	if (!_state_loaded) {
		(void)load_state();
		_state_loaded = true;
	}

	update_rc();
	const hrt_abstime now = hrt_absolute_time();
	const uint64_t now_usec = utc_now_usec();
	expire_assignments(now_usec);

	if (_last_capability_send == 0 || now - _last_capability_send >= CapabilityInterval) {
		send_capability();
		_last_capability_send = now;
	}

	if (now - _last_osd_send >= 200'000) {
		send_osd_vector();
		_last_osd_send = now;
	}

	if (now - _last_status_publish >= 200'000) {
		publish_status();
	}
}

bool MavlinkMHandler::configure_signing()
{
	if (_mavlink == nullptr) {
		return false;
	}

	mavlink_status_t *tx_status = _mavlink->get_status();
	const bool selected_physical_link = _mode == 2 && _mavlink->get_instance_id() == _instance;

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
		_signing.link_id = static_cast<uint8_t>(_link_id);
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

bool MavlinkMHandler::profile_hash_bytes(uint8_t hash[32])
{
	constexpr const char *encoded = AAGS_MAVLINK_M_CORE_XML_SHA256;

	if (sizeof(AAGS_MAVLINK_M_CORE_XML_SHA256) != 65) {
		return false;
	}

	for (size_t i = 0; i < 32; ++i) {
		const int high = hex_nibble(encoded[i * 2]);
		const int low = hex_nibble(encoded[i * 2 + 1]);

		if (high < 0 || low < 0) {
			return false;
		}

		hash[i] = static_cast<uint8_t>((high << 4) | low);
	}

	return true;
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

	for (Assignment &assignment : _inbox) {
		if (assignment.state != AssignmentState::Empty) {
			assignment.restored = 1;
		}

		assignment.name[sizeof(assignment.name) - 1] = '\0';
	}

	_active.name[sizeof(_active.name) - 1] = '\0';

	for (Assignment &assignment : _terminal) {
		assignment.name[sizeof(assignment.name) - 1] = '\0';
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
