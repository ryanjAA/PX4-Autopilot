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
static_assert(vehicle_command_s::VEHICLE_CMD_PX4_MAVLINK_M_FLY_THROUGH > UINT16_MAX,
	      "MAVLink-M fly-through command must remain PX4-internal only");

namespace
{

constexpr uint64_t Unix2020Usec = 1'577'836'800ULL * 1'000'000ULL;
constexpr uint64_t MavlinkSigningEpochSeconds = 1'420'070'400ULL;
constexpr int32_t ActionReceiptOnly = 0;
constexpr int32_t ActionRepositionCurrentAltitude = 1;
constexpr int32_t ActionInterceptCueAltitude = 2;
constexpr int32_t MaximumUdpPeerLimit = 8;
constexpr int32_t DefaultUdpPeerTimeoutS = 30;
constexpr uint16_t OwnerDecisionCommand = MAV_CMD_USER_1;
constexpr uint8_t OwnerDecisionAccept = 1;
constexpr uint8_t OwnerDecisionReject = 2;
constexpr uint32_t OwnerDecisionTaskMessage = MAVLINK_MSG_ID_TARGET_CUE;
// PX4 Commander only acknowledges MAV_CMD_DO_REPOSITION when param2 requests
// AUTO_LOITER. The MAVLink-M gate already requires AUTO_LOITER, so setting this
// flag does not broaden mode authority; it makes Commander and Navigator agree
// on the guarded one-shot command.
constexpr float RepositionChangeModeFlag = 1.f;
constexpr float DefaultInterceptRadiusM = 25.f;
constexpr float DefaultInterceptDwellS = 3.f;
constexpr float DefaultInterceptDeltaZM = 100.f;
constexpr float DefaultInterceptClearanceM = 30.f;
constexpr float DefaultLoiterRadiusM = 80.f;
constexpr float FixedWingLoiterMarginM = 10.f;
constexpr float SetpointHorizontalToleranceM = 2.f;
constexpr float SetpointAltitudeToleranceM = 1.f;
constexpr float RecoveryAltitudeToleranceM = 5.f;
constexpr hrt_abstime SetpointApplyTimeout = 2'000'000;
constexpr hrt_abstime CompletionAckTimeout = 500'000;
constexpr uint32_t InterceptTokenMaximum = INT32_MAX;
// MAVLink 2 strips zero-valued payload suffixes even for non-extension fields.
// Match AAGS by requiring the semantic prefix and letting the generated,
// zero-initializing decoders reconstruct optional suffixes.
constexpr uint8_t TrackIdentityRequiredWireLength =
	static_cast<uint8_t>(offsetof(mavlink_track_identity_t, origin_sysid));
constexpr uint8_t TargetCueRequiredWireLength =
	static_cast<uint8_t>(offsetof(mavlink_target_cue_t, cue_type));
constexpr uint8_t TargetHandoverRequiredWireLength =
	static_cast<uint8_t>(offsetof(mavlink_target_handover_t, confidence_score));
// The command field is enough to identify MAV_CMD_USER_1 as protected. A
// truncated request is still consumed, then denied because its exact vehicle
// target fields decode as zero. It must never fall through to generic command
// handling merely because those required target bytes are absent.
constexpr uint8_t OwnerDecisionCommandWireLength =
	static_cast<uint8_t>(offsetof(mavlink_command_long_t, target_system));
static_assert(TrackIdentityRequiredWireLength == 58, "unexpected TRACK_IDENTITY required prefix");
static_assert(TargetCueRequiredWireLength == 45, "unexpected TARGET_CUE required prefix");
static_assert(TargetHandoverRequiredWireLength == 76, "unexpected TARGET_HANDOVER required prefix");
static_assert(OwnerDecisionCommandWireLength == 30, "unexpected COMMAND_LONG command prefix");

constexpr uint8_t sanitize_action(int32_t action)
{
	return action >= ActionReceiptOnly && action <= ActionInterceptCueAltitude
	       ? static_cast<uint8_t>(action) : ActionReceiptOnly;
}

constexpr bool valid_system_selector(int32_t selector)
{
	return selector == -1 || (selector >= 1 && selector <= UINT8_MAX);
}

constexpr bool system_selector_matches(int32_t selector, uint8_t system_id)
{
	return system_id != 0 && (selector == -1 || selector == system_id);
}

constexpr uint8_t persisted_system_selector(int32_t selector)
{
	// MAVLink system zero is invalid for an AAGS endpoint, so it is an
	// unambiguous on-disk representation of the runtime -1 wildcard.
	return selector == -1 ? 0 : static_cast<uint8_t>(selector);
}

constexpr int32_t sanitize_max_age(int32_t maximum_age_s)
{
	return maximum_age_s >= 0 && maximum_age_s <= 600 ? maximum_age_s : 30;
}

constexpr int32_t sanitize_udp_peer_limit(int32_t peer_limit)
{
	return peer_limit >= 0 && peer_limit <= MaximumUdpPeerLimit
	       ? peer_limit : 0;
}

constexpr int32_t sanitize_udp_peer_timeout(int32_t timeout_s)
{
	return timeout_s >= 5 && timeout_s <= 300
	       ? timeout_s : DefaultUdpPeerTimeoutS;
}

constexpr bool is_esad_control_message(uint32_t message_id)
{
#if defined(MAVLINK_MSG_ID_ESAD_ARMING)

	if (message_id == MAVLINK_MSG_ID_ESAD_ARMING) {
		return true;
	}

#endif
#if defined(MAVLINK_MSG_ID_ESAD_CONFIG)

	if (message_id == MAVLINK_MSG_ID_ESAD_CONFIG) {
		return true;
	}

#endif
	return false;
}

constexpr float absolute_value(float value)
{
	return value < 0.f ? -value : value;
}

constexpr bool approximately_equal(float first, float second)
{
	return absolute_value(first - second) < 0.001f;
}

bool float_value_changed(float previous, float current)
{
	if (PX4_ISFINITE(previous) != PX4_ISFINITE(current)) {
		return true;
	}

	return PX4_ISFINITE(previous) && fabsf(previous - current) > FLT_EPSILON;
}

constexpr float sanitize_intercept_radius(float radius_m)
{
	return radius_m >= 1.f && radius_m <= 500.f ? radius_m : DefaultInterceptRadiusM;
}

constexpr float sanitize_intercept_dwell(float dwell_s)
{
	return dwell_s >= 0.f && dwell_s <= 60.f ? dwell_s : DefaultInterceptDwellS;
}

constexpr float sanitize_intercept_delta_z(float delta_z_m)
{
	return delta_z_m >= 0.f && delta_z_m <= 1'000.f ? delta_z_m : DefaultInterceptDeltaZM;
}

constexpr float sanitize_intercept_clearance(float clearance_m)
{
	return (clearance_m >= 0.f && clearance_m <= 1'000.f) || approximately_equal(clearance_m, -1.f)
	       ? clearance_m : DefaultInterceptClearanceM;
}

constexpr float effective_intercept_radius(float configured_radius_m, bool fixed_wing, float loiter_radius_m)
{
	const float fixed_wing_radius_m = absolute_value(loiter_radius_m) + FixedWingLoiterMarginM;
	return fixed_wing && fixed_wing_radius_m > configured_radius_m ? fixed_wing_radius_m : configured_radius_m;
}

constexpr bool intercept_completion_allowed(bool task_valid, bool source_fresh, bool vehicle_safe,
		bool setpoint_owned, bool fly_through_complete, bool dwell_complete, bool altitude_within_limit)
{
	return task_valid && source_fresh && vehicle_safe && setpoint_owned
	       && fly_through_complete && dwell_complete && altitude_within_limit;
}

static_assert(sanitize_action(0) == 0, "receipt-only mode must remain inert");
static_assert(sanitize_action(1) == 1, "current-altitude reposition mode must remain available");
static_assert(sanitize_action(2) == 2, "explicit cue-altitude intercept mode must remain available");
static_assert(sanitize_action(3) == 0, "unknown action modes must fail closed");
static_assert(sanitize_action(15) == 0, "obsolete command bitmask must migrate fail-closed");
static_assert(valid_system_selector(-1), "the documented AAGS wildcard must remain valid");
static_assert(valid_system_selector(1), "the lowest MAVLink system ID must remain valid");
static_assert(valid_system_selector(255), "the highest MAVLink system ID must remain valid");
static_assert(!valid_system_selector(0), "the reserved MAVLink system ID must remain invalid");
static_assert(!valid_system_selector(256), "out-of-range MAVLink system IDs must remain invalid");
static_assert(system_selector_matches(-1, 1), "the AAGS wildcard must match the lowest MAVLink system ID");
static_assert(system_selector_matches(-1, 255), "the AAGS wildcard must match the highest MAVLink system ID");
static_assert(!system_selector_matches(-1, 0), "the AAGS wildcard must reject the reserved MAVLink system ID");
static_assert(system_selector_matches(253, 253), "an exact AAGS selector must match the configured system");
static_assert(!system_selector_matches(253, 254), "an exact AAGS selector must reject a different system");
static_assert(persisted_system_selector(-1) == 0 && persisted_system_selector(255) == 255,
	      "wildcard persistence encoding must not collide with a valid system ID");
static_assert(sanitize_max_age(-1) == 30, "negative replay window must fail closed");
static_assert(sanitize_max_age(601) == 30, "oversized replay window must fail closed");
static_assert(sanitize_max_age(0) == 0 && sanitize_max_age(600) == 600,
	      "documented replay-window bounds must remain valid");
static_assert(sanitize_udp_peer_limit(-1) == 0
	      &&sanitize_udp_peer_limit(0) == 0
	      &&sanitize_udp_peer_limit(1) == 1
	      &&sanitize_udp_peer_limit(MaximumUdpPeerLimit) == MaximumUdpPeerLimit
	      &&sanitize_udp_peer_limit(MaximumUdpPeerLimit + 1) == 0,
	      "UDP peer limit must remain inside the fixed table");
static_assert(sanitize_udp_peer_timeout(4) == DefaultUdpPeerTimeoutS
	      &&sanitize_udp_peer_timeout(5) == 5
	      &&sanitize_udp_peer_timeout(300) == 300
	      &&sanitize_udp_peer_timeout(301) == DefaultUdpPeerTimeoutS,
	      "UDP peer timeout must remain bounded");
static_assert(approximately_equal(sanitize_intercept_radius(0.f), DefaultInterceptRadiusM)
	      &&approximately_equal(sanitize_intercept_radius(500.f), 500.f),
	      "intercept radius bounds must fail closed");
static_assert(approximately_equal(sanitize_intercept_dwell(-1.f), DefaultInterceptDwellS)
	      &&approximately_equal(sanitize_intercept_dwell(60.f), 60.f),
	      "intercept dwell bounds must fail closed");
static_assert(approximately_equal(sanitize_intercept_delta_z(-1.f), DefaultInterceptDeltaZM)
	      &&approximately_equal(sanitize_intercept_delta_z(1'000.f), 1'000.f),
	      "intercept altitude-change bounds must fail closed");
static_assert(approximately_equal(sanitize_intercept_clearance(-1.f), -1.f)
	      &&approximately_equal(sanitize_intercept_clearance(0.f), 0.f)
	      &&approximately_equal(sanitize_intercept_clearance(1'000.f), 1'000.f)
	      &&approximately_equal(sanitize_intercept_clearance(-0.5f), DefaultInterceptClearanceM)
	      &&approximately_equal(sanitize_intercept_clearance(1'001.f), DefaultInterceptClearanceM),
	      "intercept terrain-clearance override and bounds must fail closed");
static_assert(approximately_equal(effective_intercept_radius(25.f, false, 80.f), 25.f)
	      &&approximately_equal(effective_intercept_radius(25.f, true, 80.f), 90.f)
	      &&approximately_equal(effective_intercept_radius(120.f, true, 80.f), 120.f),
	      "fixed-wing post-crossing dwell must include loiter radius and margin");
static_assert(intercept_completion_allowed(true, true, true, true, true, true, true),
	      "all intercept completion gates should permit completion");
static_assert(!intercept_completion_allowed(true, false, true, true, true, true, true)
	      && !intercept_completion_allowed(true, true, false, true, true, true, true)
	      && !intercept_completion_allowed(true, true, true, false, true, true, true)
	      && !intercept_completion_allowed(true, true, true, true, false, true, true)
	      && !intercept_completion_allowed(true, true, true, true, true, false, true)
	      && !intercept_completion_allowed(true, true, true, true, true, true, false),
	      "every intercept completion gate must fail closed");

#ifdef PX4_STORAGEDIR
constexpr const char *StatePath = PX4_STORAGEDIR "/mavlink_m_state.bin";
constexpr const char *StateTempPath = PX4_STORAGEDIR "/.mavlink_m_state.tmp";
constexpr const char *SigningKeyPath = PX4_STORAGEDIR "/mavlink_m_signing.key";

bool sync_persistence_directory(const char *operation)
{
	const int directory_fd = ::open(PX4_STORAGEDIR, O_RDONLY | O_DIRECTORY);

	if (directory_fd < 0) {
		PX4_ERR("MAVLink-M %s directory open failed (%d)", operation, errno);
		return false;
	}

	const bool directory_synced = fsync(directory_fd) == 0;
	const int directory_sync_error = errno;
	::close(directory_fd);

	if (!directory_synced) {
		PX4_ERR("MAVLink-M %s directory sync failed (%d)", operation, directory_sync_error);
		return false;
	}

	return true;
}
#endif

bool invalidate_persisted_state()
{
#ifdef PX4_STORAGEDIR
	bool directory_changed = false;

	if (::unlink(StatePath) == 0) {
		directory_changed = true;

	} else if (errno != ENOENT) {
		PX4_ERR("MAVLink-M state invalidation failed (%d)", errno);
		return false;
	}

	if (::unlink(StateTempPath) == 0) {
		directory_changed = true;

	} else if (errno != ENOENT) {
		PX4_WARN("MAVLink-M temporary state cleanup failed (%d)", errno);
	}

	if (directory_changed && !sync_persistence_directory("state invalidation")) {
		return false;
	}

#endif
	return true;
}

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

MavlinkMHandler::MavlinkMHandler(Mavlink &mavlink) :
	_mavlink(&mavlink)
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
	_param_peer_limit = param_find("MAV_M_PEERS");
	_param_peer_timeout = param_find("MAV_M_P_TMO");
	_param_rc_channel = param_find("MAV_M_RC_CH");
	_param_rc_reject = param_find("MAV_M_RC_REJ");
	_param_rc_accept = param_find("MAV_M_RC_ACC");
	_param_max_age = param_find("MAV_M_MAX_AGE");
	_param_action = param_find("MAV_M_ACTION");
	_param_intercept_radius = param_find("MAV_M_INT_RAD");
	_param_intercept_dwell = param_find("MAV_M_INT_DWL");
	_param_intercept_delta_z = param_find("MAV_M_INT_DZ");
	_param_intercept_clearance = param_find("MAV_M_INT_CLR");
	_param_nav_loiter_radius = param_find("NAV_LOITER_RAD");
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
	const int32_t previous_action_mode = _action_mode;
	const float previous_intercept_radius = _intercept_radius_m;
	const float previous_intercept_dwell = _intercept_dwell_s;
	const float previous_intercept_delta_z = _intercept_delta_z_m;
	const float previous_intercept_clearance = _intercept_clearance_m;
	const float previous_nav_loiter_radius = _nav_loiter_radius_m;
	// Clearing a persisted predecessor-stop marker must use the endpoint header
	// that owns the currently loaded state. Do this before reading any new route
	// or source selector from the parameter store.
	const bool deferred_navigation_stopped_before_parameter_read =
		!_state_loaded || stop_deferred_navigation();

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
	get_parameter(_param_peer_limit, _peer_limit);
	const int32_t requested_peer_limit = _peer_limit;
	_peer_limit = sanitize_udp_peer_limit(requested_peer_limit);

	if (requested_peer_limit != _peer_limit && _param_peer_limit != PARAM_INVALID) {
		PX4_WARN("resetting invalid MAV_M_PEERS=%ld to %ld",
			 static_cast<long>(requested_peer_limit), static_cast<long>(_peer_limit));
		(void)param_set_no_notification(_param_peer_limit, &_peer_limit);
	}

	get_parameter(_param_peer_timeout, _peer_timeout_s);
	const int32_t requested_peer_timeout = _peer_timeout_s;
	_peer_timeout_s = sanitize_udp_peer_timeout(requested_peer_timeout);

	if (requested_peer_timeout != _peer_timeout_s && _param_peer_timeout != PARAM_INVALID) {
		PX4_WARN("resetting invalid MAV_M_P_TMO=%ld to %ld seconds",
			 static_cast<long>(requested_peer_timeout), static_cast<long>(_peer_timeout_s));
		(void)param_set_no_notification(_param_peer_timeout, &_peer_timeout_s);
	}

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

	get_parameter(_param_intercept_radius, _intercept_radius_m);
	const float requested_intercept_radius = _intercept_radius_m;
	_intercept_radius_m = sanitize_intercept_radius(requested_intercept_radius);

	if (float_value_changed(requested_intercept_radius, _intercept_radius_m)
	    && _param_intercept_radius != PARAM_INVALID) {
		PX4_WARN("resetting invalid MAV_M_INT_RAD to %.1f m", static_cast<double>(_intercept_radius_m));
		(void)param_set_no_notification(_param_intercept_radius, &_intercept_radius_m);
	}

	get_parameter(_param_intercept_dwell, _intercept_dwell_s);
	const float requested_intercept_dwell = _intercept_dwell_s;
	_intercept_dwell_s = sanitize_intercept_dwell(requested_intercept_dwell);

	if (float_value_changed(requested_intercept_dwell, _intercept_dwell_s)
	    && _param_intercept_dwell != PARAM_INVALID) {
		PX4_WARN("resetting invalid MAV_M_INT_DWL to %.1f s", static_cast<double>(_intercept_dwell_s));
		(void)param_set_no_notification(_param_intercept_dwell, &_intercept_dwell_s);
	}

	get_parameter(_param_intercept_delta_z, _intercept_delta_z_m);
	const float requested_intercept_delta_z = _intercept_delta_z_m;
	_intercept_delta_z_m = sanitize_intercept_delta_z(requested_intercept_delta_z);

	if (float_value_changed(requested_intercept_delta_z, _intercept_delta_z_m)
	    && _param_intercept_delta_z != PARAM_INVALID) {
		PX4_WARN("resetting invalid MAV_M_INT_DZ to %.1f m", static_cast<double>(_intercept_delta_z_m));
		(void)param_set_no_notification(_param_intercept_delta_z, &_intercept_delta_z_m);
	}

	get_parameter(_param_intercept_clearance, _intercept_clearance_m);
	const float requested_intercept_clearance = _intercept_clearance_m;
	_intercept_clearance_m = sanitize_intercept_clearance(requested_intercept_clearance);

	if (float_value_changed(requested_intercept_clearance, _intercept_clearance_m)
	    && _param_intercept_clearance != PARAM_INVALID) {
		PX4_WARN("resetting invalid MAV_M_INT_CLR to %.1f m",
			 static_cast<double>(_intercept_clearance_m));
		(void)param_set_no_notification(_param_intercept_clearance, &_intercept_clearance_m);
	}

	get_parameter(_param_nav_loiter_radius, _nav_loiter_radius_m);

	if (!PX4_ISFINITE(_nav_loiter_radius_m)
	    || _nav_loiter_radius_m < 25.f || _nav_loiter_radius_m > 1'000.f) {
		_nav_loiter_radius_m = DefaultLoiterRadiusM;
	}

	const bool endpoint_identity_changed = previous_instance != _instance
					       || previous_source_system != _source_system
					       || previous_source_component != _source_component;

	if (_state_loaded && endpoint_identity_changed) {
		const bool navigation_issued = (_active.command_flags & CommandNav) != 0;
		const bool navigation_stopped = deferred_navigation_stopped_before_parameter_read
						&& (!navigation_issued
						    || (assignment_cancellation_ready(_active)
							&& cancel_assignment_commands(_active)));
		const bool state_invalidated = navigation_stopped && invalidate_persisted_state();

		if (!state_invalidated) {
			// Do not erase the only task record or permit an old state file to be
			// resurrected later when navigation could not be stopped or invalidation
			// failed. Restore the previous endpoint selector and require a retry.
			_instance = previous_instance;
			_source_system = previous_source_system;
			_source_component = previous_source_component;
			(void)param_set(_param_instance, &_instance);
			(void)param_set(_param_source_system, &_source_system);
			(void)param_set(_param_source_component, &_source_component);
			PX4_ERR("MAVLink-M endpoint change blocked: active navigation or state could not be cleared");

		} else {
			clear_intercept_tracking();
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
			memset(_source_last_seen, 0, sizeof(_source_last_seen));
			_last_rc_position = RcPosition::Unknown;
			_rc_center_latched = false;
		}
	}

	const bool intercept_policy_changed = previous_action_mode != _action_mode
					      || previous_mode != _mode
					      || previous_instance != _instance
					      || previous_source_system != _source_system
					      || previous_source_component != _source_component
					      || float_value_changed(previous_intercept_radius, _intercept_radius_m)
					      || float_value_changed(previous_intercept_dwell, _intercept_dwell_s)
					      || float_value_changed(previous_intercept_delta_z, _intercept_delta_z_m)
					      || float_value_changed(previous_intercept_clearance, _intercept_clearance_m)
					      || float_value_changed(previous_nav_loiter_radius, _nav_loiter_radius_m);

	if (_intercept_phase != InterceptPhase::None
	    && _intercept_phase != InterceptPhase::Aborted
	    && intercept_policy_changed) {
		abort_intercept("navigation policy changed", true);
	}

	const bool mode_valid = _mode >= 0 && _mode <= 2;
	const bool instance_valid = _instance >= 0 && _instance < MAVLINK_COMM_NUM_BUFFERS;
	const bool source_valid = valid_system_selector(_source_system)
				  && _source_component >= 1 && _source_component <= UINT8_MAX;
	const bool signing_link_valid = _mode != 2 || (_link_id >= 0 && _link_id <= UINT8_MAX);
	_endpoint_configuration_valid = mode_valid && instance_valid && source_valid && signing_link_valid;
	const bool same_endpoint_parameter_valid = _same_endpoint == 0 || _same_endpoint == 1;
	const bool same_endpoint = _same_endpoint == 1;
	const bool control_instance_valid = _control_instance >= 0
					    && _control_instance < MAVLINK_COMM_NUM_BUFFERS;
	const bool control_source_valid = valid_system_selector(_control_system)
					  && _control_component >= 1 && _control_component <= UINT8_MAX;
	const bool same_route = _control_instance == _instance;
	const bool same_authority_selector = _control_system == _source_system
					     && _control_component == _source_component;
	const bool exact_authority_collision = _control_system != -1 && _source_system != -1
					       && same_authority_selector;
	// On one multi-client physical route, the MAVLink source system/component
	// still identifies the independent cue-source and owner-control roles. This
	// permits any authorized station to submit while one exact station retains
	// decision authority. On distinct routes, retain the legacy exact-identity
	// collision check so two physical endpoints cannot claim the same authority.
	const bool control_endpoint_relationship_valid = same_endpoint
			? same_route
			: !same_route && !exact_authority_collision;
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

	// A parameter update can change whether this receiver owns either route,
	// even when the instance number itself is unchanged. Reapply the small,
	// bounded peer configuration every time rather than risk leaving a former
	// route in Direct or Gateway mode after an authority parameter is changed.
	_udp_peer_configuration_dirty = true;

	(void)configure_signing();
}

bool MavlinkMHandler::cue_physical_route_selected() const
{
	return _mavlink != nullptr && (_mode == 1 || _mode == 2)
	       && _instance >= 0 && _instance < MAVLINK_COMM_NUM_BUFFERS
	       && _mavlink->get_instance_id() == _instance;
}

bool MavlinkMHandler::control_physical_route_selected() const
{
	return _mavlink != nullptr && (_mode == 1 || _mode == 2)
	       && _control_instance >= 0 && _control_instance < MAVLINK_COMM_NUM_BUFFERS
	       && _mavlink->get_instance_id() == _control_instance;
}

bool MavlinkMHandler::cue_route_selected() const
{
	return _endpoint_configuration_valid && cue_physical_route_selected();
}

bool MavlinkMHandler::control_route_selected() const
{
	return _control_configuration_valid && control_physical_route_selected();
}

bool MavlinkMHandler::signing_active() const
{
	return _signing_ready && _mavlink != nullptr && _receiver_status != nullptr
	       && _mavlink->get_status()->signing == &_signing
	       && _mavlink->get_status()->signing_streams == &_signing_streams
	       && _receiver_status->signing == &_signing
	       && _receiver_status->signing_streams == &_signing_streams;
}

bool MavlinkMHandler::ingress_locked() const
{
	// A selected physical route remains claimed even when another endpoint
	// parameter is corrupt. Invalid identity or signing-link configuration must
	// never turn a protected physical ingress back into an ordinary MAVLink
	// command route.
	return signing_required() && (cue_physical_route_selected() || control_physical_route_selected())
	       && !signing_active();
}

bool MavlinkMHandler::generic_ingress_allowed(const mavlink_message_t &message,
		bool udp_endpoint_authorized) const
{
	// Authorized ESAD control is deliberately received on MAV_M_INST and then
	// forwarded to MAV_M_ESAD_I. It is the only targetless control traffic that
	// may cross a cue route when cue and owner-control routes are separate.
	if (is_esad_control_message(message.msgid)) {
		return esad_control_allowed(message, udp_endpoint_authorized);
	}

	if (!cue_physical_route_selected() && !control_physical_route_selected()) {
		return true;
	}

	// Peer discovery is handled before this gate. Cue-source task traffic and
	// its liveness heartbeat are consumed by handle_message(). Everything that
	// remains, including commands, missions, parameters, FTP, TIMESYNC, manual
	// control, RC override, and forwarding, belongs only to the configured
	// owner-control identity on the configured owner route.
	return control_enabled() && udp_endpoint_authorized
	       && control_source_matches(message)
	       && signing_link_matches(message, _control_link_id);
}

bool MavlinkMHandler::enabled() const
{
	return cue_route_selected() && (!signing_required() || signing_active());
}

bool MavlinkMHandler::control_enabled() const
{
	return control_route_selected() && (!signing_required() || signing_active());
}

bool MavlinkMHandler::accepts_udp_peer_heartbeat(const mavlink_message_t &message) const
{
#if defined(MAVLINK_UDP)
	const bool cue_route = enabled();
	const bool control_route = control_enabled();
	const bool cue_component = cue_route
				   && message.compid == static_cast<uint8_t>(_source_component);
	// ESAD control is intentionally received on the cue physical route even
	// when cue submission and owner decisions use separate MAVLink instances.
	// Admit the exact owner component there as a read-only Direct peer so its
	// later ESAD frame can reach the full identity and link gate. Registration
	// alone never grants task, generic-command, or ESAD authority.
	const bool esad_control_on_cue_route = cue_route && _control_configuration_valid;
	const bool control_component = (control_route || esad_control_on_cue_route)
				       && message.compid == static_cast<uint8_t>(_control_component);

	if ((!cue_route && !control_route) || _peer_limit == 0
	    || _mavlink->get_protocol() != Protocol::UDP
	    || message.msgid != MAVLINK_MSG_ID_HEARTBEAT
	    || message.sysid == 0
	    || (!cue_component && !control_component)) {
		return false;
	}

	// Source and control components may intentionally overlap. In that case a
	// heartbeat signed for either configured role is sufficient to register its
	// exact UDP tuple. The subsequent message-specific gate still requires the
	// one correct role and link.
	const bool cue_link_matches = cue_component && signing_link_matches(message, _link_id);
	const bool control_link_matches = control_component && signing_link_matches(message, _control_link_id);

	if (!cue_link_matches && !control_link_matches) {
		return false;
	}

	mavlink_heartbeat_t heartbeat{};
	mavlink_msg_heartbeat_decode(&message, &heartbeat);
	return heartbeat.type == MAV_TYPE_GCS;
#else
	(void)message;
	return false;
#endif
}

void MavlinkMHandler::apply_udp_peer_configuration()
{
#if defined(MAVLINK_UDP)

	if (_mavlink != nullptr) {
		_mavlink->configure_mavlink_m_udp_peers(
			(cue_route_selected() || control_route_selected())
			&& _mavlink->get_protocol() == Protocol::UDP,
			static_cast<unsigned>(_peer_limit), udp_peer_timeout());
	}

#endif
	_udp_peer_configuration_dirty = false;
}

bool MavlinkMHandler::signing_required() const
{
	return _mode == 2;
}

bool MavlinkMHandler::signing_link_matches(const mavlink_message_t &message, int32_t expected_link_id) const
{
	return !signing_required()
	       || ((message.incompat_flags & MAVLINK_IFLAG_SIGNED) != 0
		   && expected_link_id >= 0 && expected_link_id <= UINT8_MAX
		   && message.signature[0] == static_cast<uint8_t>(expected_link_id));
}

bool MavlinkMHandler::source_matches(const mavlink_message_t &message) const
{
	return system_selector_matches(_source_system, message.sysid)
	       && message.compid == static_cast<uint8_t>(_source_component);
}

bool MavlinkMHandler::control_source_matches(const mavlink_message_t &message) const
{
	return system_selector_matches(_control_system, message.sysid)
	       && message.compid == static_cast<uint8_t>(_control_component);
}

bool MavlinkMHandler::source_recent(uint8_t source_system) const
{
	return source_system != 0 && _source_last_seen[source_system] != 0
	       && hrt_elapsed_time(&_source_last_seen[source_system]) < SourceFreshTimeout;
}

bool MavlinkMHandler::task_message_allowed(const mavlink_message_t &message,
		bool udp_endpoint_authorized) const
{
	return enabled() && udp_endpoint_authorized && source_matches(message)
	       && signing_link_matches(message, _link_id);
}

bool MavlinkMHandler::esad_control_allowed(const mavlink_message_t &message,
		bool udp_endpoint_authorized) const
{
	// ESAD control is targetless. It is admitted only on MAV_M_INST, using the
	// separately configured owner identity and signing link as its authority.
	return enabled() && _control_configuration_valid && udp_endpoint_authorized
	       && control_source_matches(message)
	       && signing_link_matches(message, _control_link_id);
}

bool MavlinkMHandler::handle_message(const mavlink_message_t &message,
				     bool udp_endpoint_authorized)
{
	if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT
	    && message.len >= MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN
	    && task_message_allowed(message, udp_endpoint_authorized)) {
		mavlink_heartbeat_t heartbeat{};
		mavlink_msg_heartbeat_decode(&message, &heartbeat);

		// A healthy cue source normally sends task traffic only when a target
		// changes. Sustain its movement authority with the standard GCS
		// heartbeat, but only after the same route, endpoint, identity, component,
		// and signing-link checks used for task messages have all passed.
		if (heartbeat.type == MAV_TYPE_GCS) {
			_source_last_seen[message.sysid] = hrt_absolute_time();
		}

		// HEARTBEAT still belongs to PX4's generic handler and stream logic.
		return false;
	}

	if (is_esad_control_message(message.msgid)) {
		// Returning false is deliberate only for an authorized frame: it permits
		// the normal forwarding path to carry the byte-identical signed message to
		// the selected ESAD output. Every unauthorized form is consumed here.
		return !esad_control_allowed(message, udp_endpoint_authorized);
	}

	if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG
	    && message.len >= OwnerDecisionCommandWireLength) {
		mavlink_command_long_t command{};
		mavlink_msg_command_long_decode(&message, &command);

		if (command.command == OwnerDecisionCommand) {
			return handle_control_command(message, udp_endpoint_authorized);
		}
	}

	const bool task_message = message.msgid == MAVLINK_MSG_ID_TRACK_IDENTITY
				  || message.msgid == MAVLINK_MSG_ID_TARGET_CUE
				  || message.msgid == MAVLINK_MSG_ID_TARGET_HANDOVER;

	if (!task_message) {
		return false;
	}

	if (!task_message_allowed(message, udp_endpoint_authorized)) {
		return true;
	}

	_source_last_seen[message.sysid] = hrt_absolute_time();

	if (message.msgid == MAVLINK_MSG_ID_TRACK_IDENTITY) {
		handle_track_identity(message);

	} else if (message.msgid == MAVLINK_MSG_ID_TARGET_CUE) {
		handle_target_cue(message);

	} else {
		handle_target_handover(message);
	}

	return true;
}

bool MavlinkMHandler::handle_control_command(const mavlink_message_t &message,
		bool udp_endpoint_authorized)
{
	mavlink_command_long_t command{};
	mavlink_msg_command_long_decode(&message, &command);
	const uint8_t vehicle_system = static_cast<uint8_t>(_mavlink->get_system_id());
	const uint8_t vehicle_component = static_cast<uint8_t>(_mavlink->get_component_id());

	// A USER_1 command for another exact component is not ours. A broadcast or
	// all-components form is ours but deliberately denied: a cue decision must
	// name one exact owned autopilot.
	if (command.target_system != 0 && command.target_system != vehicle_system) {
		return true;
	}

	if (command.target_system != vehicle_system
	    || command.target_component != vehicle_component) {
		send_control_command_ack(message, MAV_RESULT_DENIED);
		return true;
	}

	// Every handler instance sees the same route parameters. Consuming and
	// denying USER_1 here prevents a wrong-link request from falling through to
	// PX4's generic vehicle_command path.
	if (!control_enabled() || !udp_endpoint_authorized
	    || !control_source_matches(message)
	    || !signing_link_matches(message, _control_link_id)) {
		send_control_command_ack(message, MAV_RESULT_DENIED);
		return true;
	}

	uint16_t action_word = 0;
	uint16_t cue_low = 0;
	uint16_t cue_high = 0;
	uint32_t task_message = 0;
	uint16_t requested_effect = 0;
	const bool command_shape_valid =
		command.confirmation == 0
		&& exact_unsigned_word(command.param1, &action_word)
		&& exact_unsigned_word(command.param2, &cue_low)
		&& exact_unsigned_word(command.param3, &cue_high)
		&& exact_unsigned_value(command.param4, 0x00ffffffU, &task_message)
		&& task_message == OwnerDecisionTaskMessage
		&& exact_unsigned_word(command.param5, &requested_effect)
		&& requested_effect <= ActionInterceptCueAltitude
		&& fabsf(command.param6) <= 0.f
		&& fabsf(command.param7) <= 0.f
		&& (action_word == OwnerDecisionAccept || action_word == OwnerDecisionReject)
		&& (action_word != OwnerDecisionReject
		    || requested_effect == mavlink_m_task_decision_s::EFFECT_DEFAULT);
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

	// MAV_M_ACTION is a local permission ceiling, not the per-cue selection.
	// An explicit request may narrow that authority but may never broaden it.
	if (action_word == OwnerDecisionAccept
	    && requested_effect != mavlink_m_task_decision_s::EFFECT_DEFAULT
	    && (_control_status.cue_type != MAVLINK_M_CUE_TYPE_INVESTIGATE
		|| requested_effect > sanitize_action(_action_mode)
		|| (requested_effect == mavlink_m_task_decision_s::EFFECT_INTERCEPT
		    && !PX4_ISFINITE(_control_status.alt_msl_m)))) {
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
	decision.requested_effect = static_cast<uint8_t>(requested_effect);

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

	auto enrich = [this, &identity, &changed](Assignment & assignment) {
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
	auto matches = [&candidate](Assignment & stored) {
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
	auto matches = [message_id, instance_id](Assignment & assignment) {
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
		    && system_selector_matches(_source_system, control.source_system)
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

	// Owner decisions carry message ID and instance but no cue-source identity.
	// Under a system wildcard, two sources using the same instance would make a
	// later Accept/Reject ambiguous. Fail the second source closed instead of
	// allowing an operator command to select whichever queue slot happens first.
	if (find_task(candidate.message_id, candidate.instance_id) != nullptr) {
		send_ack(candidate, MAVLINK_M_ACK_FAILED, "assignment instance ambiguous across sources");
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

void MavlinkMHandler::accept_pending(uint32_t message_id, uint32_t instance_id, uint8_t requested_effect)
{
	Assignment *pending = find_pending(message_id, instance_id);

	if (pending == nullptr) {
		PX4_WARN("MAVLink-M accept ignored: no matching pending task");
		return;
	}

	const Assignment decided = *pending;

	// A delayed safety hold for a superseded task must never be allowed to race
	// a newly accepted command. Resolve and durably clear that obligation first.
	if (!stop_deferred_navigation()) {
		send_ack(decided, MAVLINK_M_ACK_RECEIVED,
			 "acceptance blocked: previous navigation stop pending");
		publish_status();
		return;
	}

	const char *reason = nullptr;
	uint8_t result = MAVLINK_M_ACK_FAILED;
	const uint8_t permission_ceiling = sanitize_action(_action_mode);

	if (requested_effect > mavlink_m_task_decision_s::EFFECT_INTERCEPT) {
		send_ack(decided, MAVLINK_M_ACK_RECEIVED, "movement blocked: invalid requested effect");
		return;
	}

	if (requested_effect != mavlink_m_task_decision_s::EFFECT_DEFAULT
	    && (decided.message_id != MAVLINK_MSG_ID_TARGET_CUE
		|| decided.cue_type != MAVLINK_M_CUE_TYPE_INVESTIGATE)) {
		send_ack(decided, MAVLINK_M_ACK_RECEIVED, "movement blocked: requested effect requires INVESTIGATE cue");
		return;
	}

	if (requested_effect == mavlink_m_task_decision_s::EFFECT_INTERCEPT
	    && !PX4_ISFINITE(decided.alt)) {
		send_ack(decided, MAVLINK_M_ACK_RECEIVED, "movement blocked: intercept requires finite cue altitude");
		return;
	}

	const uint8_t execution_effect =
		requested_effect == mavlink_m_task_decision_s::EFFECT_DEFAULT
		? permission_ceiling : requested_effect;

	if (requested_effect != mavlink_m_task_decision_s::EFFECT_DEFAULT
	    && requested_effect > permission_ceiling) {
		send_ack(decided, MAVLINK_M_ACK_RECEIVED, "movement blocked: requested effect exceeds MAV_M_ACTION");
		return;
	}

	// Decisions are local, but their subject still has to be valid at the exact
	// decision instant. This closes the race between deadline expiry and an RC
	// or console acceptance arriving in the same receiver iteration.
	if (!validate_common(decided, &reason, &result)) {
		if (result == MAVLINK_M_ACK_EXPIRED) {
			expire_assignments(utc_now_usec());

			// A persistence failure rolls expiry back. Never accept in that
			// case. Report the still-durable Pending state rather than a
			// terminal EXPIRED result that PX4 failed to persist.
			if (find_pending(decided.message_id, decided.instance_id) != nullptr) {
				send_ack(decided, MAVLINK_M_ACK_RECEIVED,
					 "acceptance blocked: expiry storage failed; cue remains pending");
			}

		} else {
			send_ack(decided, MAVLINK_M_ACK_RECEIVED,
				 reason != nullptr ? reason : "acceptance blocked: cue remains pending");
		}

		return;
	}

	Assignment candidate = decided;
	candidate.state = AssignmentState::Active;
	candidate.execution_effect = execution_effect;
	const bool movement_requested = assignment_requests_movement(candidate);
	const bool superseding = _active.state == AssignmentState::Active;

	if (movement_requested) {
		const char *movement_reason = nullptr;

		// A movement decision is not an acceptance unless it can take effect at
		// this exact instant. Keep the cue Pending so the operator can restore a
		// safe flight state and make a new, explicit acceptance decision.
		if (!movement_acceptance_ready(candidate, &movement_reason)) {
			PX4_WARN("MAVLink-M movement acceptance blocked: %s",
				 movement_reason != nullptr ? movement_reason : "navigation unavailable");
			send_ack(decided, MAVLINK_M_ACK_RECEIVED,
				 movement_reason != nullptr ? movement_reason : "movement command unavailable");
			publish_status();
			return;
		}
	}

	// A nonmoving replacement of an issued navigation task must be able to stop
	// the old command before any durable state changes. A moving replacement
	// publishes its new command directly after the atomic state commit, which
	// replaces the old Navigator setpoint without an intervening hold command.
	if (superseding && !movement_requested && _active.command_flags != 0
	    && !assignment_cancellation_ready(_active)) {
		send_ack(decided, MAVLINK_M_ACK_RECEIVED,
			 "acceptance blocked: cannot stop active navigation");
		publish_status();
		return;
	}

	const Assignment active_before = _active;
	const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
	const Assignment terminal_evicted = _terminal[TerminalCapacity - 1];
	const InterceptTracking intercept_before = intercept_tracking();
	Assignment superseded{};

	if (superseding) {
		superseded = active_before;

		if ((superseded.command_flags & CommandNav) != 0) {
			// This bit is the durable obligation to replace or stop the old
			// navigation. It is cleared only in the same durable commit that
			// confirms the replacement, or after a confirmed current-position
			// hold. A module restart can therefore never lose the stop.
			superseded.command_flags |= CommandStopPending;
		}

		++superseded.status_sequence;
		remember_terminal(superseded, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);
	}

	_active = candidate;
	// Movement promotion is a durable intermediate state until navigation is
	// actually published. Persist RECEIVED so a restart or duplicate replay can
	// never claim acceptance for a command that was not issued.
	_active.last_ack_result = movement_requested ? MAVLINK_M_ACK_RECEIVED : MAVLINK_M_ACK_ACCEPTED;
	++_active.status_sequence;
	remove_inbox(pending);

	if (!save_state()) {
		_active = active_before;
		_inbox[0] = inbox_before[0];
		_inbox[1] = inbox_before[1];

		if (superseding) {
			undo_remember_terminal(terminal_evicted);
		}

		restore_intercept_tracking(intercept_before);
		send_ack(decided, MAVLINK_M_ACK_RECEIVED,
			 superseding && movement_requested
			 ? "movement blocked: storage failed; active retained"
			 : "acceptance blocked: storage failed; cue pending");
		return;
	}

	// These snapshots are the last known durable intermediate state. Movement
	// remains RECEIVED until its command publishes and command flags persist.
	const Assignment staged_active = _active;
	const Assignment staged_inbox[2] {_inbox[0], _inbox[1]};
	bool execution_applied = true;
	CommandApplicationResult command_result = CommandApplicationResult::Applied;

	if (movement_requested) {
		command_result = command_active_assignment();
		execution_applied = command_result == CommandApplicationResult::Applied;

	} else if (superseding && active_before.command_flags != 0) {
		execution_applied = cancel_assignment_commands(active_before);

		if (execution_applied) {
			clear_intercept_tracking();
		}

	} else if (superseding) {
		clear_intercept_tracking();
	}

	if (execution_applied && superseding && !movement_requested
	    && (active_before.command_flags & CommandNav) != 0) {
		// The current-position hold has released the predecessor. Persist that
		// release separately from the already durable nonmoving acceptance. A
		// storage failure leaves the stop marker intact so update() can retry it.
		Assignment *stop_marker = find_navigation_stop_marker();

		if (stop_marker != nullptr) {
			const uint8_t command_flags_before = stop_marker->command_flags;
			stop_marker->command_flags &= static_cast<uint8_t>(~CommandStopPending);

			if (!save_state()) {
				stop_marker->command_flags = command_flags_before;
				_deferred_navigation_stop = *stop_marker;
				PX4_WARN("superseded navigation stopped but release marker storage failed");
			}
		}
	}

	const bool persistence_failed_after_publication =
		command_result == CommandApplicationResult::PersistenceFailedAfterPublicationStopped
		|| command_result == CommandApplicationResult::PersistenceFailedAfterPublicationStopUnconfirmed;

	if (persistence_failed_after_publication) {
		// The replacement command reached uORB, but its command flags could not be
		// committed. When its stop command was published, keep RAM aligned with the
		// last durable Active/Received stage. If that stop could not be published,
		// retain the in-memory command flags and tracking so the UI cannot claim the
		// navigation stopped and an explicit Abort can try again. In neither case may
		// the old task be restored as Active after its navigation was replaced.
		const bool navigation_stopped =
			command_result == CommandApplicationResult::PersistenceFailedAfterPublicationStopped;

		if (navigation_stopped) {
			_active = staged_active;
			_inbox[0] = staged_inbox[0];
			_inbox[1] = staged_inbox[1];
			clear_intercept_tracking();

		} else {
			_active.last_ack_result = MAVLINK_M_ACK_RECEIVED;
		}

		if (superseding) {
			if (Assignment *stop_marker = find_navigation_stop_marker()) {
				_deferred_navigation_stop = *stop_marker;
			}

			send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED,
				 "superseded; replacement storage failed");
		}

		send_ack(_active, MAVLINK_M_ACK_RECEIVED,
			 navigation_stopped
			 ? "movement uncommitted: navigation stopped; abort"
			 : "movement uncommitted: stop unconfirmed; abort");
		publish_status();
		return;
	}

	if (!execution_applied) {
		// Readiness is rechecked after the durable decision write. If the state
		// changed during that write, a replacement command failed, or an old
		// navigation task could not be stopped, undo the promotion and require a
		// fresh operator decision. Never emit ACCEPTED for an unapplied takeover.
		_active = active_before;
		_inbox[0] = inbox_before[0];
		_inbox[1] = inbox_before[1];

		if (superseding) {
			undo_remember_terminal(terminal_evicted);
		}

		restore_intercept_tracking(intercept_before);

		if (!save_state()) {
			PX4_ERR("MAVLink-M acceptance rollback was not persisted");
			// Match RAM to the only known durable intermediate state. If this was a
			// supersession, recreate its terminal-ring insertion after undoing the
			// failed rollback and make a best effort to stop the old navigation.
			_active = staged_active;
			_inbox[0] = staged_inbox[0];
			_inbox[1] = staged_inbox[1];

			if (superseding) {
				remember_terminal(superseded, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);
				clear_intercept_tracking();

				if (active_before.command_flags != 0) {
					if (!assignment_cancellation_ready(active_before)
					    || !cancel_assignment_commands(active_before)) {
						_deferred_navigation_stop = active_before;
					}
				}

				send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED,
					 "superseded; replacement storage failed");
			}

			send_ack(_active, MAVLINK_M_ACK_RECEIVED,
				 _deferred_navigation_stop.state != AssignmentState::Empty
				 ? "movement uncommitted: old stop pending"
				 : "storage failed: movement uncommitted; abort");

		} else {
			send_ack(decided, MAVLINK_M_ACK_RECEIVED,
				 movement_requested
				 ? "movement blocked: command failed; active retained"
				 : "acceptance blocked: active navigation retained");
		}

		publish_status();
		return;
	}

	Assignment acknowledged = _active;
	acknowledged.last_ack_result = MAVLINK_M_ACK_ACCEPTED;

	if (superseding) {
		send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "superseded by accepted cue");
		send_ack(acknowledged, MAVLINK_M_ACK_ACCEPTED, "accepted; previous cue superseded");

	} else {
		send_ack(acknowledged, MAVLINK_M_ACK_ACCEPTED, "local operator accepted active target");
	}

	publish_status();
}

void MavlinkMHandler::reject_pending_or_abort_active(uint32_t message_id, uint32_t instance_id)
{
	Assignment *pending = find_pending(message_id, instance_id);

	if (pending != nullptr) {
		Assignment decided = *pending;

		if (!stop_deferred_navigation()) {
			send_ack(decided, MAVLINK_M_ACK_RECEIVED,
				 "rejection blocked: previous navigation stop pending");
			publish_status();
			return;
		}

		++decided.status_sequence;
		const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
		const Assignment terminal_evicted = _terminal[TerminalCapacity - 1];
		remove_inbox(pending);
		remember_terminal(decided, AssignmentState::Rejected, MAVLINK_M_ACK_REJECTED);

		if (!save_state()) {
			_inbox[0] = inbox_before[0];
			_inbox[1] = inbox_before[1];
			undo_remember_terminal(terminal_evicted);
			send_ack(decided, MAVLINK_M_ACK_RECEIVED,
				 "rejection blocked: decision storage failed; cue remains pending");
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
		const bool navigation_issued = (_active.command_flags & CommandNav) != 0;

		if (!stop_deferred_navigation()) {
			send_ack(_active, _active.last_ack_result,
				 "abort blocked: previous navigation stop pending");
			publish_status();
			return;
		}

		// Never make an Aborted state durable while its issued navigation may still
		// be running. Publish the current-position stop first. If it cannot be
		// confirmed, retain the Active task and its command flags so the operator can
		// retry and the UI cannot falsely report a completed abort.
		if (navigation_issued
		    && (!assignment_cancellation_ready(_active) || !cancel_assignment_commands(_active))) {
			send_ack(_active, _active.last_ack_result,
				 "abort blocked: navigation stop unconfirmed");
			publish_status();
			return;
		}

		const Assignment active_before = _active;
		const Assignment inbox_before[2] {_inbox[0], _inbox[1]};
		const Assignment terminal_evicted = _terminal[TerminalCapacity - 1];
		const InterceptTracking intercept_before = intercept_tracking();
		const bool aborting_intercept = intercept_assignment_matches(_active);

		_active = Assignment{};
		remember_terminal(decided, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);

		if (!save_state()) {
			_active = active_before;
			_inbox[0] = inbox_before[0];
			_inbox[1] = inbox_before[1];
			undo_remember_terminal(terminal_evicted);
			restore_intercept_tracking(intercept_before);
			send_ack(decided, active_before.last_ack_result,
				 navigation_issued
				 ? "abort unsaved: navigation stopped; task active"
				 : "abort blocked: storage failed; task remains active");
			return;
		}

		if (aborting_intercept) {
			abort_intercept("task aborted");
		}

		send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "local operator aborted active target");

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
			accept_pending(decision.task_msgid, decision.task_instance, decision.requested_effect);

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

bool MavlinkMHandler::assignment_requests_movement(const Assignment &assignment) const
{
	return assignment.message_id == MAVLINK_MSG_ID_TARGET_CUE
	       && assignment.cue_type == MAVLINK_M_CUE_TYPE_INVESTIGATE
	       && (assignment.execution_effect == ActionRepositionCurrentAltitude
		   || assignment.execution_effect == ActionInterceptCueAltitude);
}

bool MavlinkMHandler::movement_acceptance_ready(const Assignment &assignment, const char **reason)
{
	const bool guarded_intercept = assignment.execution_effect == ActionInterceptCueAltitude
				       && PX4_ISFINITE(assignment.alt);

	if (!source_recent(assignment.source_system)) {
		*reason = "movement blocked: cue source is stale";
		return false;
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_global_position_sub.update(&_global_position);
	_local_position_sub.update(&_local_position);

	if (!vehicle_ready_for_reposition() || !vehicle_airborne_for_reposition()) {
		*reason = "movement blocked: vehicle must be armed, airborne, safe, and in Hold";
		return false;
	}

	if (_global_position.timestamp == 0
	    || hrt_elapsed_time(&_global_position.timestamp) >= 2'000'000
	    || !PX4_ISFINITE(_global_position.lat)
	    || !PX4_ISFINITE(_global_position.lon)
	    || !PX4_ISFINITE(_global_position.alt)) {
		*reason = "movement blocked: fresh aircraft AMSL position unavailable";
		return false;
	}

	if (guarded_intercept
	    && fabsf(assignment.alt - _global_position.alt) > _intercept_delta_z_m) {
		*reason = "movement blocked: cue altitude exceeds MAV_M_INT_DZ";
		return false;
	}

	if (guarded_intercept && !intercept_terrain_clearance_valid(assignment.alt, reason)) {
		return false;
	}

	return true;
}

bool MavlinkMHandler::intercept_terrain_clearance_valid(float candidate_altitude_m, const char **reason) const
{
	if (_intercept_clearance_m < 0.f) {
		return true;
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool global_terrain_fresh = _global_position.timestamp != 0
					  && now >= _global_position.timestamp
					  && now - _global_position.timestamp < 2'000'000
					  && _global_position.terrain_alt_valid
					  && PX4_ISFINITE(_global_position.alt)
					  && PX4_ISFINITE(_global_position.terrain_alt);
	const bool hagl_fresh = _local_position.timestamp != 0
				&& now >= _local_position.timestamp
				&& now - _local_position.timestamp < 2'000'000
				&& _local_position.dist_bottom_valid
				&& PX4_ISFINITE(_local_position.dist_bottom);

	if (!global_terrain_fresh || !hagl_fresh) {
		*reason = "movement blocked: MAV_M_INT_CLR requires fresh terrain and HAGL";
		return false;
	}

	const float actual_terrain_clearance_m = _global_position.alt - _global_position.terrain_alt;
	const float candidate_terrain_clearance_m = candidate_altitude_m - _global_position.terrain_alt;

	if (!PX4_ISFINITE(candidate_altitude_m)
	    || !PX4_ISFINITE(actual_terrain_clearance_m)
	    || !PX4_ISFINITE(candidate_terrain_clearance_m)
	    || actual_terrain_clearance_m < _intercept_clearance_m
	    || _local_position.dist_bottom < _intercept_clearance_m
	    || candidate_terrain_clearance_m < _intercept_clearance_m) {
		*reason = "movement blocked: MAV_M_INT_CLR terrain clearance breached";
		return false;
	}

	return true;
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

bool MavlinkMHandler::publish_internal_fly_through(const Assignment &assignment, float altitude_m,
		float recovery_altitude_m, float minimum_clearance_m, uint32_t token)
{
	if (_mavlink == nullptr || !PX4_ISFINITE(altitude_m) || !PX4_ISFINITE(recovery_altitude_m)
	    || !approximately_equal(sanitize_intercept_clearance(minimum_clearance_m), minimum_clearance_m)
	    || token == 0 || token > InterceptTokenMaximum) {
		return false;
	}

	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_PX4_MAVLINK_M_FLY_THROUGH;
	// Binary32 represents every 16-bit integer exactly. Splitting the runtime
	// token prevents a stale or unrelated local navigation event from proving
	// completion of this accepted cue.
	vehicle_command.param1 = static_cast<float>(token & UINT16_MAX);
	vehicle_command.param2 = static_cast<float>((token >> 16) & UINT16_MAX);
	vehicle_command.param3 = recovery_altitude_m;
	vehicle_command.param4 = minimum_clearance_m;
	vehicle_command.param5 = assignment.lat * 1e-7;
	vehicle_command.param6 = assignment.lon * 1e-7;
	vehicle_command.param7 = altitude_m;
	vehicle_command.target_system = static_cast<uint8_t>(_mavlink->get_system_id());
	vehicle_command.target_component = static_cast<uint8_t>(_mavlink->get_component_id());
	vehicle_command.source_system = vehicle_command.target_system;
	vehicle_command.source_component = vehicle_command.target_component;
	vehicle_command.from_external = false;

	return _vehicle_command_pub.publish(vehicle_command);
}

MavlinkMHandler::CommandApplicationResult MavlinkMHandler::command_active_assignment()
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
	    || (_active.execution_effect != ActionRepositionCurrentAltitude
		&& _active.execution_effect != ActionInterceptCueAltitude)
	    || _active.execution_effect > sanitize_action(_action_mode)
	    || (_active.command_flags & CommandNav) != 0) {
		return CommandApplicationResult::FailedBeforePublication;
	}

	clear_intercept_tracking();
	const bool guarded_intercept = _active.execution_effect == ActionInterceptCueAltitude
				       && PX4_ISFINITE(_active.alt);
	const char *freshness_reason = nullptr;
	uint8_t freshness_result = MAVLINK_M_ACK_FAILED;

	// The durable decision write can include an fsync/rename. Recheck immediately
	// before publication so a cue crossing its deadline during that commit is
	// rolled back to Pending and can never receive a false ACCEPTED ACK.
	if (!validate_common(_active, &freshness_reason, &freshness_result)) {
		PX4_WARN("MAVLink-M reposition suppressed: %s",
			 freshness_reason != nullptr ? freshness_reason : "cue no longer valid");

		if (guarded_intercept) {
			abort_intercept(freshness_reason != nullptr ? freshness_reason : "cue no longer valid");
		}

		return CommandApplicationResult::FailedBeforePublication;
	}

	// Acceptance persistence can also span a vehicle-state transition. Pull the
	// newest sample after the commit and immediately before the flight-state
	// gate rather than relying on update()'s pre-decision cache.
	const char *movement_reason = nullptr;

	if (!movement_acceptance_ready(_active, &movement_reason)) {
		if (_last_reposition_block_log == 0
		    || hrt_elapsed_time(&_last_reposition_block_log) > 2'000'000) {
			PX4_WARN("MAVLink-M reposition not issued: %s",
				 movement_reason != nullptr ? movement_reason : "navigation unavailable");
			_last_reposition_block_log = hrt_absolute_time();
		}

		if (guarded_intercept) {
			abort_intercept(movement_reason != nullptr ? movement_reason : "vehicle not safe for intercept");
		}

		return CommandApplicationResult::FailedBeforePublication;
	}

	const double target_lat = _active.lat * 1e-7;
	const double target_lon = _active.lon * 1e-7;
	// Level travel keeps the acceptance-time AMSL altitude. Guarded intercept
	// carries the exact cue altitude so Navigator can build a performance-bound
	// approach before crossing the target coordinate.
	const float acceptance_alt = _global_position.alt;
	const float target_alt = guarded_intercept ? _active.alt : acceptance_alt;

	uint32_t intercept_token = 0;

	if (guarded_intercept) {
		if (_intercept_token_counter == 0) {
			_intercept_token_counter = static_cast<uint32_t>(hrt_absolute_time()) & InterceptTokenMaximum;
		}

		_intercept_token_counter = (_intercept_token_counter % InterceptTokenMaximum) + 1;
		intercept_token = _intercept_token_counter;
	}

	const bool command_published = guarded_intercept
				       ? publish_internal_fly_through(_active, target_alt, acceptance_alt,
						       _intercept_clearance_m, intercept_token)
				       : publish_vehicle_command(_active, vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
						       NAN, RepositionChangeModeFlag, NAN, NAN,
						       target_lat, target_lon, target_alt);

	if (command_published) {
		_active.command_flags |= CommandNav;
		_active.last_ack_result = MAVLINK_M_ACK_ACCEPTED;

		if (guarded_intercept) {
			_active.command_flags |= CommandInterceptAltitude;
			begin_intercept_tracking(acceptance_alt, target_alt, intercept_token);

			if (_intercept_clearance_m < 0.f) {
				PX4_WARN("MAV_M_INT_CLR=-1: externally surveyed corridor override accepted locally for this effect-2 cue");
			}
		}

		// Publishing the replacement command releases the predecessor. Clear its
		// durable stop marker in the same commit as the new command flags. If that
		// commit fails, restore the marker before stopping the uncommitted command.
		Assignment *stop_marker = find_navigation_stop_marker();
		const uint8_t stop_marker_flags = stop_marker != nullptr ? stop_marker->command_flags : 0;

		if (stop_marker != nullptr) {
			stop_marker->command_flags &= static_cast<uint8_t>(~CommandStopPending);
		}

		if (!save_state()) {
			if (stop_marker != nullptr) {
				stop_marker->command_flags = stop_marker_flags;
			}

			PX4_ERR("MAVLink-M command state was not persisted; stopping navigation");
			const bool navigation_stopped = cancel_assignment_commands(_active);

			if (navigation_stopped) {
				clear_intercept_tracking();
			}

			return navigation_stopped
			       ? CommandApplicationResult::PersistenceFailedAfterPublicationStopped
			       : CommandApplicationResult::PersistenceFailedAfterPublicationStopUnconfirmed;
		}

		publish_status();
		return CommandApplicationResult::Applied;
	}

	return CommandApplicationResult::FailedBeforePublication;
}

void MavlinkMHandler::clear_intercept_tracking()
{
	_intercept_phase = InterceptPhase::None;
	_intercept_message_id = 0;
	_intercept_instance_id = 0;
	_intercept_source_system = 0;
	_intercept_source_component = 0;
	_intercept_acceptance_altitude_m = NAN;
	_intercept_expected_altitude_m = NAN;
	_intercept_command_time = 0;
	_intercept_completion_ack_time = 0;
	_intercept_completion_wait_started = 0;
	_intercept_dwell_started = 0;
	_intercept_token = 0;
	_intercept_setpoint_seen = false;
	_intercept_navigator_started = false;
	_intercept_navigator_completed = false;
	_intercept_navigator_missed = false;
	_intercept_navigator_failed = false;
}

MavlinkMHandler::InterceptTracking MavlinkMHandler::intercept_tracking() const
{
	InterceptTracking tracking{};
	tracking.phase = _intercept_phase;
	tracking.command_time = _intercept_command_time;
	tracking.completion_ack_time = _intercept_completion_ack_time;
	tracking.completion_wait_started = _intercept_completion_wait_started;
	tracking.dwell_started = _intercept_dwell_started;
	tracking.message_id = _intercept_message_id;
	tracking.instance_id = _intercept_instance_id;
	tracking.token = _intercept_token;
	tracking.source_system = _intercept_source_system;
	tracking.source_component = _intercept_source_component;
	tracking.acceptance_altitude_m = _intercept_acceptance_altitude_m;
	tracking.expected_altitude_m = _intercept_expected_altitude_m;
	tracking.setpoint_seen = _intercept_setpoint_seen;
	tracking.navigator_started = _intercept_navigator_started;
	tracking.navigator_completed = _intercept_navigator_completed;
	tracking.navigator_missed = _intercept_navigator_missed;
	tracking.navigator_failed = _intercept_navigator_failed;
	return tracking;
}

void MavlinkMHandler::restore_intercept_tracking(const InterceptTracking &tracking)
{
	_intercept_phase = tracking.phase;
	_intercept_command_time = tracking.command_time;
	_intercept_completion_ack_time = tracking.completion_ack_time;
	_intercept_completion_wait_started = tracking.completion_wait_started;
	_intercept_dwell_started = tracking.dwell_started;
	_intercept_message_id = tracking.message_id;
	_intercept_instance_id = tracking.instance_id;
	_intercept_token = tracking.token;
	_intercept_source_system = tracking.source_system;
	_intercept_source_component = tracking.source_component;
	_intercept_acceptance_altitude_m = tracking.acceptance_altitude_m;
	_intercept_expected_altitude_m = tracking.expected_altitude_m;
	_intercept_setpoint_seen = tracking.setpoint_seen;
	_intercept_navigator_started = tracking.navigator_started;
	_intercept_navigator_completed = tracking.navigator_completed;
	_intercept_navigator_missed = tracking.navigator_missed;
	_intercept_navigator_failed = tracking.navigator_failed;
}

void MavlinkMHandler::begin_intercept_tracking(float acceptance_altitude_m, float target_altitude_m, uint32_t token)
{
	clear_intercept_tracking();
	_intercept_message_id = _active.message_id;
	_intercept_instance_id = _active.instance_id;
	_intercept_source_system = _active.source_system;
	_intercept_source_component = _active.source_component;
	_intercept_acceptance_altitude_m = acceptance_altitude_m;
	_intercept_expected_altitude_m = target_altitude_m;
	_intercept_token = token;
	_intercept_command_time = hrt_absolute_time();
	_intercept_phase = InterceptPhase::Transit;
}

bool MavlinkMHandler::intercept_assignment_matches(const Assignment &assignment) const
{
	return assignment.state != AssignmentState::Empty
	       && assignment.message_id == _intercept_message_id
	       && assignment.instance_id == _intercept_instance_id
	       && assignment.source_system == _intercept_source_system
	       && assignment.source_component == _intercept_source_component;
}

void MavlinkMHandler::abort_intercept(const char *reason, bool stop_navigation)
{
	if (_intercept_phase == InterceptPhase::Aborted) {
		return;
	}

	// A safety-gate loss must stop every phase of the owned intercept, including
	// its initial POSITION fly-through and target-centered dwell. Explicit task
	// abort and expiry leave this false because their durable decision paths
	// cancel the assignment exactly once after saving the terminal state.
	const bool navigation_active = stop_navigation
				       && _intercept_phase != InterceptPhase::None
				       && _active.state == AssignmentState::Active
				       && intercept_assignment_matches(_active);

	if (_intercept_phase == InterceptPhase::None
	    && _active.state == AssignmentState::Active
	    && _active.message_id == MAVLINK_MSG_ID_TARGET_CUE
	    && _active.cue_type == MAVLINK_M_CUE_TYPE_INVESTIGATE
	    && _active.execution_effect == ActionInterceptCueAltitude
	    && PX4_ISFINITE(_active.alt)) {
		_intercept_message_id = _active.message_id;
		_intercept_instance_id = _active.instance_id;
		_intercept_source_system = _active.source_system;
		_intercept_source_component = _active.source_component;
	}

	if (_intercept_message_id == 0 || _intercept_instance_id == 0) {
		return;
	}

	_intercept_phase = InterceptPhase::Aborted;
	_intercept_dwell_started = 0;
	_intercept_setpoint_seen = false;
	PX4_WARN("MAVLink-M intercept aborted: %s", reason != nullptr ? reason : "safety gate");

	if (navigation_active) {
		(void)cancel_assignment_commands(_active);
	}

	publish_status();
}

bool MavlinkMHandler::intercept_transit_setpoint_matches(float expected_altitude_m) const
{
	const position_setpoint_s &setpoint = _position_setpoint_triplet.current;
	const position_setpoint_s &next = _position_setpoint_triplet.next;

	if (!setpoint.valid || setpoint.type != position_setpoint_s::SETPOINT_TYPE_POSITION
	    || !next.valid
	    || !PX4_ISFINITE(setpoint.lat) || !PX4_ISFINITE(setpoint.lon)
	    || !PX4_ISFINITE(setpoint.alt) || !PX4_ISFINITE(next.lat) || !PX4_ISFINITE(next.lon)
	    || !PX4_ISFINITE(next.alt) || !PX4_ISFINITE(expected_altitude_m)) {
		return false;
	}

	const double target_lat = _active.lat * 1e-7;
	const double target_lon = _active.lon * 1e-7;
	const float setpoint_offset_m = get_distance_to_next_waypoint(
						setpoint.lat, setpoint.lon, target_lat, target_lon);
	const float next_offset_m = get_distance_to_next_waypoint(
					    next.lat, next.lon, target_lat, target_lon);
	const bool target_leg = next.type == position_setpoint_s::SETPOINT_TYPE_LOITER
				&& setpoint.mavlink_m_exact_altitude
				&& PX4_ISFINITE(setpoint_offset_m)
				&& setpoint_offset_m <= SetpointHorizontalToleranceM
				&& PX4_ISFINITE(next_offset_m)
				&& next_offset_m <= SetpointHorizontalToleranceM
				&& fabsf(setpoint.alt - expected_altitude_m) <= SetpointAltitudeToleranceM
				&& fabsf(next.alt - expected_altitude_m) <= SetpointAltitudeToleranceM;
	const bool approach_leg = next.type == position_setpoint_s::SETPOINT_TYPE_POSITION
				  && next.mavlink_m_exact_altitude
				  && PX4_ISFINITE(next_offset_m)
				  && next_offset_m <= SetpointHorizontalToleranceM
				  && fabsf(next.alt - expected_altitude_m) <= SetpointAltitudeToleranceM
				  && PX4_ISFINITE(_intercept_acceptance_altitude_m)
				  && fabsf(setpoint.alt - _intercept_acceptance_altitude_m) <= SetpointAltitudeToleranceM;
	return target_leg || approach_leg;
}

bool MavlinkMHandler::intercept_loiter_setpoint_matches(float expected_altitude_m) const
{
	const position_setpoint_s &setpoint = _position_setpoint_triplet.current;

	if (!setpoint.valid || setpoint.type != position_setpoint_s::SETPOINT_TYPE_LOITER
	    || !PX4_ISFINITE(setpoint.lat) || !PX4_ISFINITE(setpoint.lon)
	    || !PX4_ISFINITE(setpoint.alt) || !PX4_ISFINITE(expected_altitude_m)) {
		return false;
	}

	const double target_lat = _active.lat * 1e-7;
	const double target_lon = _active.lon * 1e-7;
	const float setpoint_offset_m = get_distance_to_next_waypoint(
						setpoint.lat, setpoint.lon, target_lat, target_lon);
	return PX4_ISFINITE(setpoint_offset_m)
	       && setpoint_offset_m <= SetpointHorizontalToleranceM
	       && fabsf(setpoint.alt - expected_altitude_m) <= SetpointAltitudeToleranceM;
}

bool MavlinkMHandler::intercept_recovery_setpoint_matches(float recovery_altitude_m) const
{
	const position_setpoint_s &setpoint = _position_setpoint_triplet.current;
	const position_setpoint_s &next = _position_setpoint_triplet.next;

	if (!setpoint.valid || setpoint.type != position_setpoint_s::SETPOINT_TYPE_POSITION
	    || !PX4_ISFINITE(setpoint.lat) || !PX4_ISFINITE(setpoint.lon)
	    || !PX4_ISFINITE(setpoint.alt) || !PX4_ISFINITE(recovery_altitude_m)
	    || next.valid) {
		return false;
	}

	return fabsf(setpoint.alt - recovery_altitude_m) <= SetpointAltitudeToleranceM
	       && !setpoint.mavlink_m_exact_altitude;
}

bool MavlinkMHandler::intercept_safety_gates_valid(const char **reason) const
{
	if (!intercept_assignment_matches(_active)
	    || _active.state != AssignmentState::Active
	    || _active.message_id != MAVLINK_MSG_ID_TARGET_CUE
	    || _active.cue_type != MAVLINK_M_CUE_TYPE_INVESTIGATE
	    || _active.execution_effect != ActionInterceptCueAltitude
	    || _active.execution_effect > sanitize_action(_action_mode)
	    || !PX4_ISFINITE(_active.alt)
	    || (_active.command_flags & CommandNav) == 0) {
		*reason = "active cue or action changed";
		return false;
	}

	const char *assignment_reason = nullptr;
	uint8_t assignment_result = MAVLINK_M_ACK_FAILED;

	if (!validate_common(_active, &assignment_reason, &assignment_result)) {
		*reason = assignment_reason != nullptr ? assignment_reason : "cue invalid";
		return false;
	}

	if (!source_recent(_active.source_system)) {
		*reason = "cue source stale";
		return false;
	}

	if (!vehicle_ready_for_reposition() || !vehicle_airborne_for_reposition()) {
		*reason = "vehicle left armed airborne Hold";
		return false;
	}

	if (_global_position.timestamp == 0
	    || hrt_elapsed_time(&_global_position.timestamp) >= 2'000'000
	    || !PX4_ISFINITE(_global_position.lat)
	    || !PX4_ISFINITE(_global_position.lon)
	    || !PX4_ISFINITE(_global_position.alt)) {
		*reason = "aircraft position stale";
		return false;
	}

	if (!PX4_ISFINITE(_intercept_acceptance_altitude_m)
	    || fabsf(_active.alt - _intercept_acceptance_altitude_m) > _intercept_delta_z_m) {
		*reason = "cue altitude exceeds MAV_M_INT_DZ";
		return false;
	}

	const bool recovery_navigation_owned =
		intercept_recovery_setpoint_matches(_intercept_acceptance_altitude_m)
		|| intercept_loiter_setpoint_matches(_intercept_acceptance_altitude_m);

	if (!intercept_terrain_clearance_valid(_intercept_acceptance_altitude_m, reason)
	    || (!recovery_navigation_owned
		&& !intercept_terrain_clearance_valid(_active.alt, reason))) {
		return false;
	}

	return true;
}

float MavlinkMHandler::intercept_arrival_radius() const
{
	const bool fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	return effective_intercept_radius(_intercept_radius_m, fixed_wing, _nav_loiter_radius_m);
}

void MavlinkMHandler::update_intercept_command_ack()
{
	for (unsigned i = 0; i < vehicle_command_ack_s::ORB_QUEUE_LENGTH; ++i) {
		vehicle_command_ack_s ack{};

		if (!_vehicle_command_ack_sub.update(&ack)) {
			return;
		}

		if (_intercept_phase == InterceptPhase::None
		    || _intercept_phase == InterceptPhase::Aborted
		    || ack.command != vehicle_command_s::VEHICLE_CMD_PX4_MAVLINK_M_FLY_THROUGH
		    || ack.from_external || ack.result_param2 <= 0
		    || static_cast<uint32_t>(ack.result_param2) != _intercept_token
		    || _mavlink == nullptr
		    || ack.target_system != static_cast<uint8_t>(_mavlink->get_system_id())
		    || ack.target_component != static_cast<uint8_t>(_mavlink->get_component_id())) {
			continue;
		}

		switch (ack.result) {
		case vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS:
			_intercept_navigator_started = true;
			break;

		case vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED:
			_intercept_navigator_started = true;
			_intercept_navigator_completed = true;
			_intercept_completion_ack_time = hrt_absolute_time();
			break;

		case vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED:
			_intercept_completion_ack_time = hrt_absolute_time();

			if (ack.result_param1 == 1) {
				_intercept_navigator_missed = true;

			} else {
				_intercept_navigator_failed = true;
			}

			break;

		case vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		case vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED:
		case vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED:
			_intercept_navigator_failed = true;
			break;

		default:
			break;
		}
	}
}

void MavlinkMHandler::update_intercept()
{
	if (_intercept_phase == InterceptPhase::None
	    || _intercept_phase == InterceptPhase::Aborted) {
		return;
	}

	const char *safety_reason = nullptr;

	if (!intercept_safety_gates_valid(&safety_reason)) {
		abort_intercept(safety_reason, true);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool transit_setpoint_owned = intercept_transit_setpoint_matches(_intercept_expected_altitude_m);
	const bool recovery_setpoint_owned = intercept_recovery_setpoint_matches(_intercept_acceptance_altitude_m);
	const bool recovery_loiter_owned = intercept_loiter_setpoint_matches(_intercept_acceptance_altitude_m);
	const bool recovery_altitude_reached = PX4_ISFINITE(_global_position.alt)
					       && PX4_ISFINITE(_intercept_acceptance_altitude_m)
					       && fabsf(_global_position.alt - _intercept_acceptance_altitude_m)
					       <= RecoveryAltitudeToleranceM;

	if (_intercept_navigator_missed) {
		// A miss remains an aborted acceptance, but Navigator first completes the
		// same acceptance-altitude recovery used after a hit. Never leave a missed
		// aircraft loitering or descending at cue altitude.
		if (recovery_loiter_owned && recovery_altitude_reached) {
			abort_intercept("exact target missed; acceptance-altitude target loiter retained");

		} else if (_intercept_completion_ack_time != 0
			   && now - _intercept_completion_ack_time >= CompletionAckTimeout) {
			abort_intercept("exact target missed without owned recovery loiter", true);
		}

		return;
	}

	if (_intercept_navigator_failed) {
		abort_intercept("Navigator rejected, cancelled, or missed the exact target", true);
		return;
	}

	if (_intercept_phase == InterceptPhase::Transit) {
		if (transit_setpoint_owned || recovery_setpoint_owned) {
			// Seeing the private token-bound approach or target triplet binds this
			// state machine to Navigator's exact fly-through path. Merely entering
			// the future loiter radius can never satisfy this gate.
			_intercept_setpoint_seen = true;

			if (_intercept_navigator_completed && _intercept_completion_ack_time != 0
			    && now - _intercept_completion_ack_time >= CompletionAckTimeout) {
				abort_intercept("Navigator completion did not promote the target loiter", true);
			}

			return;
		}

		if (recovery_loiter_owned && _intercept_setpoint_seen) {
			if (_intercept_navigator_completed) {
				// The target loiter transition is accepted only with Navigator's
				// token-matched completion ACK. An ordinary reposition to the same
				// coordinate cannot prove that the target plane was crossed.
				_intercept_phase = InterceptPhase::Dwell;
				_intercept_completion_wait_started = 0;
				_intercept_dwell_started = 0;
				publish_status();

			} else {
				if (_intercept_completion_wait_started == 0) {
					_intercept_completion_wait_started = now;
				}

				if (now - _intercept_completion_wait_started >= CompletionAckTimeout) {
					abort_intercept("target loiter has no matching completion ACK", true);
				}

				return;
			}

		} else if (_intercept_setpoint_seen
			   || _intercept_command_time == 0
			   || now - _intercept_command_time >= SetpointApplyTimeout) {
			abort_intercept("navigation setpoint overridden");
			return;

		} else {
			return;
		}
	}

	if (_intercept_phase == InterceptPhase::Complete) {
		if (recovery_loiter_owned && recovery_altitude_reached) {
			_intercept_setpoint_seen = true;
			return;
		}

		if (_intercept_setpoint_seen || _intercept_command_time == 0
		    || now - _intercept_command_time >= SetpointApplyTimeout) {
			abort_intercept("navigation setpoint overridden", true);
		}

		return;
	}

	if (_intercept_phase != InterceptPhase::Dwell || !recovery_loiter_owned
	    || !_intercept_setpoint_seen) {
		abort_intercept("fly-through completion was not retained");
		return;
	}

	const double target_lat = _active.lat * 1e-7;
	const double target_lon = _active.lon * 1e-7;
	const float distance_m = get_distance_to_next_waypoint(
					 _global_position.lat, _global_position.lon, target_lat, target_lon);
	const float arrival_radius_m = intercept_arrival_radius();
	const bool inside_radius = PX4_ISFINITE(distance_m) && distance_m <= arrival_radius_m;
	const bool recovery_dwell_eligible = inside_radius && recovery_altitude_reached;

	if (!recovery_dwell_eligible) {
		if (_intercept_dwell_started != 0) {
			_intercept_dwell_started = 0;
			publish_status();
		}

		return;
	}

	if (_intercept_dwell_started == 0) {
		_intercept_dwell_started = now;
		publish_status();
	}

	const hrt_abstime required_dwell = static_cast<hrt_abstime>(_intercept_dwell_s * 1'000'000.f);
	const bool dwell_complete = _intercept_dwell_started != 0
				    && now - _intercept_dwell_started >= required_dwell;

	if (!dwell_complete) {
		return;
	}

	// Re-evaluate every gate immediately before latching completion. The cue
	// altitude was already part of the guarded exact approach, so no second
	// reposition command is emitted here.
	safety_reason = nullptr;
	const bool task_and_vehicle_safe = intercept_safety_gates_valid(&safety_reason);
	const bool setpoint_still_owned = intercept_loiter_setpoint_matches(_intercept_acceptance_altitude_m);
	const bool fly_through_complete = _intercept_phase == InterceptPhase::Dwell
					  && _intercept_setpoint_seen;
	const bool recovery_altitude_within_tolerance = PX4_ISFINITE(_global_position.alt)
			&& PX4_ISFINITE(_intercept_acceptance_altitude_m)
			&& fabsf(_global_position.alt - _intercept_acceptance_altitude_m)
			<= RecoveryAltitudeToleranceM;

	if (!intercept_completion_allowed(task_and_vehicle_safe, true, true,
					  setpoint_still_owned, fly_through_complete && inside_radius, dwell_complete,
					  recovery_altitude_within_tolerance)) {
		abort_intercept(safety_reason != nullptr ? safety_reason : "completion gate changed");
		return;
	}

	_intercept_phase = InterceptPhase::Complete;
	_intercept_dwell_started = 0;

	if (!save_state()) {
		PX4_WARN("MAVLink-M intercept completion was not persisted");
	}

	publish_status();
}

bool MavlinkMHandler::cancel_assignment_commands(const Assignment &assignment)
{
	if (assignment.command_flags == 0) {
		return false;
	}

	// Revoking future action authority must not revoke the ability to stop a
	// reposition that this endpoint already issued and durably marked.
	if (!assignment_cancellation_ready(assignment)) {
		return false;
	}

	return publish_vehicle_command(assignment, vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,
				       NAN, RepositionChangeModeFlag, NAN, NAN,
				       _global_position.lat, _global_position.lon,
				       _global_position.alt);
}

bool MavlinkMHandler::assignment_cancellation_ready(const Assignment &assignment)
{
	if (assignment.command_flags == 0) {
		return true;
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_global_position_sub.update(&_global_position);

	return (assignment.command_flags & CommandNav) != 0
	       && vehicle_ready_for_reposition()
	       && _global_position.timestamp != 0 && hrt_elapsed_time(&_global_position.timestamp) < 2'000'000
	       && PX4_ISFINITE(_global_position.lat) && PX4_ISFINITE(_global_position.lon)
	       && PX4_ISFINITE(_global_position.alt);
}

MavlinkMHandler::Assignment *MavlinkMHandler::find_navigation_stop_marker()
{
	for (Assignment &assignment : _terminal) {
		if ((assignment.state == AssignmentState::Aborted
		     || assignment.state == AssignmentState::Expired)
		    && (assignment.command_flags & CommandNav) != 0
		    && (assignment.command_flags & CommandStopPending) != 0) {
			return &assignment;
		}
	}

	return nullptr;
}

bool MavlinkMHandler::stop_deferred_navigation()
{
	Assignment *marker = find_navigation_stop_marker();

	if (_deferred_navigation_stop.state == AssignmentState::Empty && marker != nullptr) {
		_deferred_navigation_stop = *marker;
	}

	if (_deferred_navigation_stop.state == AssignmentState::Empty
	    || (_deferred_navigation_stop.command_flags & CommandNav) == 0) {
		_deferred_navigation_stop = Assignment{};
		return marker == nullptr;
	}

	if (!assignment_cancellation_ready(_deferred_navigation_stop)
	    || !cancel_assignment_commands(_deferred_navigation_stop)) {
		return false;
	}

	// The hold is published, but the obligation is not complete until clearing
	// its marker is durable. If storage fails, retain the marker and retry. A
	// repeated hold is safe; forgetting the obligation across a restart is not.
	marker = find_navigation_stop_marker();
	const bool stopped_uncommitted_active = _active.state == AssignmentState::Active
						&& _active.last_ack_result == MAVLINK_M_ACK_RECEIVED
						&& (_active.command_flags & CommandNav) != 0;

	if (stopped_uncommitted_active) {
		// The failed final commit left these execution flags only in RAM. The
		// durable stage already has RECEIVED with no command. The hold just stopped
		// that uncommitted navigation, so never persist or display the stale flags.
		_active.command_flags &= static_cast<uint8_t>(~CommandExecutionAll);
		clear_intercept_tracking();
	}

	if (marker != nullptr) {
		const uint8_t command_flags_before = marker->command_flags;
		marker->command_flags &= static_cast<uint8_t>(~CommandStopPending);

		if (!save_state()) {
			marker->command_flags = command_flags_before;
			PX4_WARN("deferred MAVLink-M navigation stopped but marker storage failed");
			return false;
		}

	} else if (stopped_uncommitted_active && !save_state()) {
		PX4_WARN("uncommitted MAVLink-M navigation stopped but state storage failed");
		return false;
	}

	PX4_WARN("stopped and cleared deferred MAVLink-M navigation");
	_deferred_navigation_stop = Assignment{};
	return true;
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
	const InterceptTracking intercept_before = intercept_tracking();
	bool active_intercept_expired = false;

	if (_active.state == AssignmentState::Active
	    && assignment_expired_at(_active, now_usec)) {
		Assignment expired = _active;

		if ((expired.command_flags & CommandNav) != 0) {
			expired.command_flags |= CommandStopPending;
		}

		++expired.status_sequence;

		active_intercept_expired = intercept_assignment_matches(_active);

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

			restore_intercept_tracking(intercept_before);

		} else {
			if (active_intercept_expired) {
				abort_intercept("task expired");
			}

			for (unsigned i = 0; i < expired_count; ++i) {
				send_ack(expired_assignments[i], MAVLINK_M_ACK_EXPIRED, "assignment expired");
			}

			if (Assignment *stop_marker = find_navigation_stop_marker()) {
				_deferred_navigation_stop = *stop_marker;
				(void)stop_deferred_navigation();
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

	_mavlink->set_protocol_version(2);

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
	_mavlink->set_protocol_version(2);

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
	if (!control_enabled() && !enabled()) {
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

	_mavlink->set_protocol_version(2);
	// These receiver-authored fields let every fleet observer render the same
	// cue state without mistaking visibility for decision authority. Zero means
	// network decisions are disabled; minus one is the documented any-system
	// selector. PX4 still enforces the selector on every command.
	send_int("AAGS_CSYS", _control_configuration_valid ? _control_system : 0);
	send_int("AAGS_CCMP", _control_configuration_valid ? _control_component : 0);

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
		send_int("AAGS_IPHS", static_cast<int32_t>(_control_status.intercept_phase));
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
	status.intercept_phase = mavlink_m_target_status_s::INTERCEPT_PHASE_NONE;

	for (const Assignment &assignment : _inbox) {
		if (assignment.state != AssignmentState::Empty) {
			++status.queue_depth;
		}
	}

	if (display != nullptr) {
		status.source_fresh = source_recent(display->source_system);
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
		status.action_mode = display->state == AssignmentState::Pending
				     ? sanitize_action(_action_mode) : display->execution_effect;
		status.command_flags = display->command_flags;

		if (intercept_assignment_matches(*display)) {
			status.intercept_phase = static_cast<uint8_t>(_intercept_phase);
		}

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
	const hrt_abstime now = hrt_absolute_time();
	const bool route_selected = cue_route_selected() || control_route_selected();

	if (route_selected) {
		// Signed MAVLink-M traffic cannot be represented in MAVLink 1. Select
		// MAVLink 2 even while a physical route is waiting for UTC or its key.
		_mavlink->set_protocol_version(2);
	}

	if (route_selected && signing_required() && !signing_active()
	    && (_last_signing_retry == 0 || now - _last_signing_retry >= 1'000'000)) {
		_last_signing_retry = now;
		(void)configure_signing();
	}

	if (_udp_peer_configuration_dirty) {
		apply_udp_peer_configuration();
	}

	_global_position_sub.update(&_global_position);
	_local_position_sub.update(&_local_position);
	_attitude_sub.update(&_attitude);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_status_sub.update(&_vehicle_status);
	_position_setpoint_triplet_sub.update(&_position_setpoint_triplet);

	if (_deferred_navigation_stop.state != AssignmentState::Empty) {
		(void)stop_deferred_navigation();
	}

	const bool cue_receiver_enabled = enabled();
	const bool owner_control_enabled = control_enabled();

	if (!cue_receiver_enabled && !owner_control_enabled) {
		return;
	}

	if (cue_receiver_enabled) {
		if (!_state_loaded) {
			(void)load_state();
			_state_loaded = true;
		}

		// Do not let expiry, RC decisions, or another acceptance mutate the
		// terminal ring while its persisted predecessor-stop marker is unresolved.
		if (!stop_deferred_navigation()) {
			publish_status();
			return;
		}

		const uint64_t now_usec = utc_now_usec();
		expire_assignments(now_usec);
		update_rc();
		update_local_decision();
		update_intercept_command_ack();
		update_intercept();

		if (now - _last_osd_send >= 200'000) {
			send_osd_vector();
			_last_osd_send = now;
		}

		if (now - _last_status_publish >= 200'000) {
			publish_status();
		}
	}

	// Publish receiver-confirmed cue state on both the owner route and the cue
	// route. Separate-endpoint observers receive state without gaining control,
	// because command authorization remains bound to MAV_M_CTL_INST and its
	// independent identity, endpoint, and signing-link checks.
	if ((owner_control_enabled || cue_receiver_enabled)
	    && now - _last_control_status_send >= 200'000) {
		send_control_status();
	}
}

bool MavlinkMHandler::configure_signing()
{
	if (_mavlink == nullptr) {
		return false;
	}

	mavlink_status_t *tx_status = _mavlink->get_status();
	const bool selected_cue_link = _mode == 2 && cue_route_selected();
	const bool selected_control_link = _mode == 2 && control_route_selected();
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
		_signing_failure_warned = false;
		_last_signing_retry = 0;
		return _mode != 2;
	}

	if (signing_active()) {
		return true;
	}

	_signing_ready = false;
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

		if (!_signing_failure_warned) {
			PX4_ERR("MAVLink-M signed mode locked: key/time invalid");
			_signing_failure_warned = true;
		}

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
	tx_status->signing = &_signing;
	tx_status->signing_streams = &_signing_streams;

	if (_receiver_status != nullptr) {
		_receiver_status->signing = &_signing;
		_receiver_status->signing_streams = &_signing_streams;
	}

	// The route may unlock only after the parser has the validation key and
	// replay-stream state, not merely after the key file was read.
	_signing_ready = _receiver_status != nullptr
			 && _receiver_status->signing == &_signing
			 && _receiver_status->signing_streams == &_signing_streams;

	if (_signing_ready) {
		_signing_failure_warned = false;
		PX4_INFO("MAVLink-M signed physical link active");
	}

	return _signing_ready;
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
	const auto *bytes = reinterpret_cast<const uint8_t *>(&state);
	constexpr size_t crc_offset = offsetof(PersistedState, crc);
	const uint32_t zero_crc = 0;
	uint32_t crc = crc32_signature(0, crc_offset, bytes);
	crc = crc32_signature(crc, sizeof(zero_crc), reinterpret_cast<const uint8_t *>(&zero_crc));
	const size_t suffix_offset = crc_offset + sizeof(state.crc);
	return crc32_signature(crc, sizeof(state) - suffix_offset, bytes + suffix_offset);
}

bool MavlinkMHandler::load_state()
{
#ifdef PX4_STORAGEDIR
	const int fd = ::open(StatePath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	_persistence_state = PersistedState{};
	PersistedState &state = _persistence_state;
	const ssize_t bytes = ::read(fd, &state, sizeof(state));
	::close(fd);

	if (bytes != static_cast<ssize_t>(sizeof(state)) || state.magic != PersistenceMagic
	    || (state.version != PersistenceVersion && state.version != PreviousPersistenceVersion)
	    || state.size != sizeof(state)
	    || state.crc != state_crc(state)) {
		PX4_WARN("ignoring invalid MAVLink-M persisted state");
		return false;
	}

	if (state.mavlink_instance != static_cast<uint8_t>(_instance)
	    || state.source_system != persisted_system_selector(_source_system)
	    || state.source_component != static_cast<uint8_t>(_source_component)
	    || strncmp(state.profile_hash, AAGS_MAVLINK_M_CORE_XML_SHA256, sizeof(state.profile_hash)) != 0) {
		PX4_WARN("ignoring MAVLink-M state for different endpoint/profile");
		Assignment *stale_navigation = nullptr;

		for (Assignment &assignment : state.terminal) {
			if ((assignment.command_flags & CommandNav) != 0
			    && (assignment.command_flags & CommandStopPending) != 0) {
				stale_navigation = &assignment;
				break;
			}
		}

		if (stale_navigation == nullptr
		    && state.active.state == AssignmentState::Active
		    && (state.active.command_flags & CommandNav) != 0) {
			state.active.state = AssignmentState::Aborted;
			state.active.last_ack_result = MAVLINK_M_ACK_REJECTED;
			state.active.command_flags |= CommandStopPending;
			++state.active.status_sequence;
			stale_navigation = &state.active;
		}

		if (stale_navigation != nullptr) {
			// Quarantine only the cancellation authority under the new endpoint
			// header. The old task is never restored as Active, but its stop survives
			// repeated MAVLink module restarts until a hold is confirmed and saved.
			_active = Assignment{};
			_deferred_navigation_stop = *stale_navigation;

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

			remember_terminal(*stale_navigation, stale_navigation->state,
					  stale_navigation->last_ack_result);

			if (!save_state()) {
				PX4_WARN("stale MAVLink-M navigation quarantine was not persisted");
			}

			return true;
		}

		if (!invalidate_persisted_state()) {
			PX4_ERR("stale MAVLink-M state could not be invalidated");
		}

		return false;
	}

	_active = state.active;
	memcpy(_inbox, state.inbox, sizeof(_inbox));
	memcpy(_terminal, state.terminal, sizeof(_terminal));
	memcpy(_controls, state.controls, sizeof(_controls));

	const bool migrated_previous_version = state.version == PreviousPersistenceVersion;

	if (migrated_previous_version) {
		// Version 6 had no per-cue execution effect. Preserve its audit state,
		// but restore every assignment as receipt-only so an old padding byte
		// can never become movement authority.
		_active.execution_effect = mavlink_m_task_decision_s::EFFECT_DEFAULT;

		for (Assignment &assignment : _inbox) {
			assignment.execution_effect = mavlink_m_task_decision_s::EFFECT_DEFAULT;
		}

		for (Assignment &assignment : _terminal) {
			assignment.execution_effect = mavlink_m_task_decision_s::EFFECT_DEFAULT;
		}
	}

	if (Assignment *stop_marker = find_navigation_stop_marker()) {
		// A distinct persisted bit, rather than historical CommandNav, records
		// that a superseded command has not yet been durably replaced or stopped.
		// Scan the full terminal ring so rejection or expiry of a later pending cue
		// cannot hide this obligation across repeated MAVLink module restarts.
		_deferred_navigation_stop = *stop_marker;
		PX4_WARN("restored superseded MAVLink-M navigation stop");
	}

	bool recovered_uncommitted_movement = false;

	if (_active.state == AssignmentState::Active
	    && _active.last_ack_result == MAVLINK_M_ACK_RECEIVED
	    && _active.command_flags == 0
	    && assignment_requests_movement(_active)) {
		// This is the durable stage between local acceptance and publication of
		// its navigation command. A restart in that narrow window proves that no
		// committed command exists. Return the cue to Pending so it requires a new
		// explicit decision instead of restoring a commandless Active task.
		_active.state = AssignmentState::Pending;
		_active.last_ack_result = MAVLINK_M_ACK_RECEIVED;
		_active.restored = 1;
		++_active.status_sequence;
		Assignment *slot = find_free_inbox();

		if (slot != nullptr) {
			*slot = _active;
			_active = Assignment{};
			PX4_WARN("restored uncommitted MAVLink-M movement as pending");

		} else {
			// The normal staging transition always frees the candidate's inbox slot.
			// If a CRC-valid but inconsistent state has no slot, fail the commandless
			// task closed rather than making it impossible to decide or abort.
			remember_terminal(_active, AssignmentState::Aborted, MAVLINK_M_ACK_FAILED);
			_active = Assignment{};
			PX4_WARN("aborted inconsistent uncommitted MAVLink-M movement");
		}

		recovered_uncommitted_movement = true;
	}

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

	if (_active.state == AssignmentState::Active
	    && _active.message_id == MAVLINK_MSG_ID_TARGET_CUE
	    && _active.cue_type == MAVLINK_M_CUE_TYPE_INVESTIGATE
	    && _active.execution_effect == ActionInterceptCueAltitude
	    && PX4_ISFINITE(_active.alt)
	    && (_active.command_flags & CommandNav) != 0) {
		abort_intercept("restart requires fresh acceptance");
	}

	for (Assignment &assignment : _terminal) {
		assignment.name[sizeof(assignment.name) - 1] = '\0';
	}

	if ((migrated_previous_version || migrated_legacy_queue || recovered_uncommitted_movement)
	    && !save_state()) {
		PX4_WARN("failed to persist MAVLink-M state migration");
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
	_persistence_state = PersistedState {};
	PersistedState &state = _persistence_state;
	state.magic = PersistenceMagic;
	state.version = PersistenceVersion;
	state.size = static_cast<uint16_t>(sizeof(state));
	state.mavlink_instance = static_cast<uint8_t>(_instance);
	state.source_system = persisted_system_selector(_source_system);
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

	if (!sync_persistence_directory("state commit")) {
		return false;
	}

	return true;
#else
	PX4_ERR("MAVLink-M durable storage unavailable");
	return false;
#endif
}
