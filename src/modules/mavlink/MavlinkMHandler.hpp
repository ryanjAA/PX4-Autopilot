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

#pragma once

#include "mavlink_bridge_header.h"

#include <drivers/drv_hrt.h>
#include <math.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/mavlink_m_target_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>

class Mavlink;

/**
 * Safe endpoint for the provisional AAGS 54xxx MAVLink-M profile.
 *
 * The supplied profile has no recipient fields. Consequently this handler is
 * disabled by default and can only be enabled as an exact-source,
 * exact-MAVLink-instance compatibility endpoint. It never publishes a
 * navigation setpoint, vehicle command, arming command, or actuator command.
 */
class MavlinkMHandler
{
public:
	explicit MavlinkMHandler(Mavlink *mavlink);

	void handle_message(const mavlink_message_t &message);
	void update();
	void update_parameters();

private:
	enum class AssignmentState : uint8_t {
		Empty = 0,
		Pending,
		Active,
		Queued,
		Rejected,
		Aborted,
		Expired,
	};

	enum class RcPosition : uint8_t {
		Unknown = 0,
		Reject,
		Center,
		Accept,
	};

	struct Assignment {
		uint64_t time_usec{0};
		uint64_t valid_until_usec{0};
		uint64_t track_identity_time_usec{0};
		uint32_t message_id{0};
		uint32_t instance_id{0};
		uint32_t target_set_id{0};
		uint32_t payload_crc{0};
		int32_t lat{0};
		int32_t lon{0};
		float alt{NAN};
		float vx{NAN};
		float vy{NAN};
		float vz{NAN};
		float confidence{NAN};
		uint8_t source_system{0};
		uint8_t source_component{0};
		uint8_t origin_system{0};
		uint8_t target_class{0};
		uint8_t target_force{0};
		uint8_t cue_type{0};
		uint8_t track_uid[16]{};
		char name[51]{};
		AssignmentState state{AssignmentState::Empty};
		uint8_t last_ack_result{0}; // MAVLINK_M_ACK_RECEIVED
		uint8_t restored{0};
	};

	struct TrackIdentity {
		uint64_t time_usec{0};
		uint64_t first_detected_usec{0};
		uint32_t target_set_id{0};
		float confidence{NAN};
		uint8_t source_system{0};
		uint8_t source_component{0};
		uint8_t origin_system{0};
		uint8_t track_uid[16]{};
	};

	struct PersistedState {
		uint32_t magic{0};
		uint16_t version{0};
		uint16_t size{0};
		uint8_t mavlink_instance{0};
		uint8_t source_system{0};
		uint8_t source_component{0};
		uint8_t reserved{0};
		char profile_hash[65]{};
		Assignment active{};
		Assignment inbox[2]{};
		Assignment terminal[8]{};
		uint32_t crc{0};
	};

	static constexpr uint32_t PersistenceMagic = 0x4d534741; // "AGSM"
	static constexpr uint16_t PersistenceVersion = 3;
	static constexpr unsigned InboxCapacity = 2;
	static constexpr unsigned TerminalCapacity = 8;
	static constexpr unsigned TrackIdentityCapacity = 4;
	static constexpr hrt_abstime SourceFreshTimeout = 15'000'000;

	bool enabled() const;
	bool source_matches(const mavlink_message_t &message) const;
	void handle_track_identity(const mavlink_message_t &message);
	void handle_target_cue(const mavlink_message_t &message);
	void handle_target_handover(const mavlink_message_t &message);
	void store_assignment(const Assignment &assignment);

	bool validate_common(const Assignment &assignment, const char **reason, uint8_t *ack_result) const;
	bool validate_time(uint64_t time_usec, uint64_t valid_until_usec, const char **reason,
			   uint8_t *ack_result) const;

	Assignment *find_duplicate(const Assignment &candidate);
	const TrackIdentity *find_track_identity(const Assignment &assignment) const;
	void apply_track_identity(Assignment &assignment, const TrackIdentity &identity);
	Assignment *find_pending();
	Assignment *find_free_inbox();
	void remove_inbox(Assignment *assignment);
	void remember_terminal(const Assignment &assignment, AssignmentState state, uint8_t result);
	void undo_remember_terminal(const Assignment &evicted);
	void promote_queued();
	void expire_assignments(uint64_t now_usec);

	void update_rc();
	RcPosition classify_rc(uint16_t pwm) const;
	void accept_pending();
	void reject_pending_or_abort_active();

	void send_ack(const Assignment &assignment, uint8_t result, const char *reason);
	void send_osd_vector();
	void publish_status();

	bool load_state();
	bool save_state();
	uint32_t state_crc(const PersistedState &state) const;
	static uint64_t utc_now_usec();
	static float wrap_180(float angle_deg);

	Mavlink *_mavlink{nullptr};

	Assignment _active{};
	Assignment _inbox[InboxCapacity]{};
	Assignment _terminal[TerminalCapacity]{};
	TrackIdentity _track_identities[TrackIdentityCapacity]{};
	RcPosition _last_rc_position{RcPosition::Unknown};
	bool _rc_center_latched{false};
	bool _state_loaded{false};
	bool _warned_unaddressed{false};

	hrt_abstime _source_last_seen{0};
	hrt_abstime _last_status_publish{0};
	hrt_abstime _last_osd_send{0};

	vehicle_global_position_s _global_position{};
	vehicle_attitude_s _attitude{};

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<mavlink_m_target_status_s> _status_pub{ORB_ID(mavlink_m_target_status)};

	param_t _param_mode{PARAM_INVALID};
	param_t _param_instance{PARAM_INVALID};
	param_t _param_source_system{PARAM_INVALID};
	param_t _param_source_component{PARAM_INVALID};
	param_t _param_rc_channel{PARAM_INVALID};
	param_t _param_rc_reject{PARAM_INVALID};
	param_t _param_rc_accept{PARAM_INVALID};
	param_t _param_max_age{PARAM_INVALID};

	int32_t _mode{0};
	int32_t _instance{0};
	int32_t _source_system{255};
	int32_t _source_component{190};
	int32_t _rc_channel{0};
	int32_t _rc_reject{1300};
	int32_t _rc_accept{1700};
	int32_t _max_age_s{30};
};
