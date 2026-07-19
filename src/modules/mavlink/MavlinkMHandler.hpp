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
#include <uORB/topics/mavlink_m_task_decision.h>
#include <uORB/topics/mavlink_m_target_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

class Mavlink;

/**
 * Endpoint for the finalized Dronecode MAVLink-M dialect.
 *
 * Cue and handover packets are route-selected by MAVLink instance, bound to an
 * exact source endpoint, and durable before receipt is acknowledged. Optional
 * non-kinetic navigation authority is separately parameter-gated and runs only
 * after a local operator accepts an INVESTIGATE cue. Normal reposition keeps
 * the aircraft's acceptance-time AMSL altitude; a separate local intercept
 * mode is the only path that may use TARGET_CUE.alt.
 */
class MavlinkMHandler
{
public:
	explicit MavlinkMHandler(Mavlink *mavlink);

	void configure_receiver_status(mavlink_status_t *status);
	/// Returns true when this endpoint consumed the frame and the generic
	/// MAVLink command path must not process it a second time.
	bool handle_message(const mavlink_message_t &message);
	void update();
	void update_parameters();

private:
	enum class AssignmentState : uint8_t {
		Empty = 0,
		Pending,
		Active,
		Queued, // legacy persisted state; restored as Pending
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
		uint32_t status_sequence{0};
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
		uint8_t command_flags{0};
	};

	struct ControlRecord {
		uint32_t control_id{0};
		uint32_t task_msgid{0};
		uint32_t task_instance{0};
		uint32_t replacement_instance{0};
		uint32_t payload_crc{0};
		uint8_t source_system{0};
		uint8_t source_component{0};
		uint8_t action{0};
		uint8_t ack_result{0};
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
		ControlRecord controls[8]{};
		uint32_t crc{0};
	};

	static constexpr uint32_t PersistenceMagic = 0x4d534741; // "AGSM"
	static constexpr uint16_t PersistenceVersion = 6;
	static constexpr unsigned InboxCapacity = 2;
	static constexpr unsigned TerminalCapacity = 8;
	static constexpr unsigned TrackIdentityCapacity = 4;
	static constexpr unsigned ControlCapacity = 8;
	static constexpr hrt_abstime SourceFreshTimeout = 15'000'000;
	static constexpr uint8_t CommandNav = 1 << 0;
	static constexpr uint8_t CommandAll = CommandNav;
	static constexpr uint8_t CommandTargetLocalComponent = 255;
	static_assert(CommandAll == 1, "MAVLink-M must expose reposition authority only");

	bool enabled() const;
	bool control_enabled() const;
	bool signing_required() const;
	bool source_matches(const mavlink_message_t &message) const;
	bool control_source_matches(const mavlink_message_t &message) const;
	bool task_message_allowed(const mavlink_message_t &message) const;
	bool handle_control_command(const mavlink_message_t &message);
	void handle_track_identity(const mavlink_message_t &message);
	void handle_target_cue(const mavlink_message_t &message);
	void handle_target_handover(const mavlink_message_t &message);
	void store_assignment(const Assignment &assignment);

	bool validate_common(const Assignment &assignment, const char **reason, uint8_t *ack_result) const;
	bool validate_time(uint64_t time_usec, uint64_t valid_until_usec, const char **reason,
			   uint8_t *ack_result) const;

	Assignment *find_duplicate(const Assignment &candidate);
	Assignment *find_task(uint32_t message_id, uint32_t instance_id);
	ControlRecord *find_control(uint32_t control_id);
	bool track_identity_fresh(const TrackIdentity &identity, uint64_t reference_time_usec) const;
	bool track_identity_usable(const Assignment &assignment, const TrackIdentity &identity) const;
	const TrackIdentity *find_track_identity(const Assignment &assignment) const;
	void apply_track_identity(Assignment &assignment, const TrackIdentity &identity);
	Assignment *find_pending(uint32_t message_id = 0, uint32_t instance_id = 0);
	Assignment *find_free_inbox();
	void remove_inbox(Assignment *assignment);
	void remember_terminal(const Assignment &assignment, AssignmentState state, uint8_t result);
	void undo_remember_terminal(const Assignment &evicted);
	bool assignment_expired_at(const Assignment &assignment, uint64_t now_usec) const;
	void expire_assignments(uint64_t now_usec);

	void update_rc();
	void update_local_decision();
	RcPosition classify_rc(uint16_t pwm) const;
	void accept_pending(uint32_t message_id = 0, uint32_t instance_id = 0);
	void reject_pending_or_abort_active(uint32_t message_id = 0, uint32_t instance_id = 0);
	bool vehicle_ready_for_reposition() const;
	bool vehicle_airborne_for_reposition() const;
	bool publish_vehicle_command(const Assignment &assignment, uint32_t command, float param1 = NAN, float param2 = NAN,
				      float param3 = NAN, float param4 = NAN, double param5 = static_cast<double>(NAN),
				      double param6 = static_cast<double>(NAN),
				      float param7 = NAN, uint8_t target_component = CommandTargetLocalComponent);
	bool command_active_assignment();
	bool cancel_assignment_commands(const Assignment &assignment);

	void send_ack(const Assignment &assignment, uint8_t result, const char *reason);
	void send_control_command_ack(const mavlink_message_t &request, uint8_t result);
	void send_control_status();
	void send_osd_vector();
	void publish_status();

	bool load_state();
	bool save_state();
	bool configure_signing();
	bool load_signing_key(uint8_t key[32]) const;
	uint32_t state_crc(const PersistedState &state) const;
	static uint64_t utc_now_usec();
	static uint64_t signing_timestamp();
	static float wrap_180(float angle_deg);

	Mavlink *_mavlink{nullptr};

	Assignment _active{};
	Assignment _inbox[InboxCapacity]{};
	Assignment _terminal[TerminalCapacity]{};
	ControlRecord _controls[ControlCapacity]{};
	TrackIdentity _track_identities[TrackIdentityCapacity]{};
	RcPosition _last_rc_position{RcPosition::Unknown};
	bool _rc_center_latched{false};
	bool _state_loaded{false};
	bool _signing_ready{false};
	bool _endpoint_configuration_valid{false};
	bool _control_configuration_valid{false};
	mavlink_status_t *_receiver_status{nullptr};
	mavlink_signing_t _signing{};
	mavlink_signing_streams_t _signing_streams{};

	hrt_abstime _source_last_seen{0};
	hrt_abstime _last_status_publish{0};
	hrt_abstime _last_control_status_send{0};
	hrt_abstime _last_osd_send{0};
	hrt_abstime _last_reposition_block_log{0};

	vehicle_global_position_s _global_position{};
	vehicle_attitude_s _attitude{};
	vehicle_land_detected_s _vehicle_land_detected{};
	vehicle_status_s _vehicle_status{};

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _task_decision_sub{ORB_ID(mavlink_m_task_decision)};
	uORB::Subscription _target_status_sub{ORB_ID(mavlink_m_target_status)};
	uORB::Subscription _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<mavlink_m_target_status_s> _status_pub{ORB_ID(mavlink_m_target_status)};
	uORB::Publication<mavlink_m_task_decision_s> _task_decision_pub{ORB_ID(mavlink_m_task_decision)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	param_t _param_mode{PARAM_INVALID};
	param_t _param_link_id{PARAM_INVALID};
	param_t _param_instance{PARAM_INVALID};
	param_t _param_source_system{PARAM_INVALID};
	param_t _param_source_component{PARAM_INVALID};
	param_t _param_control_instance{PARAM_INVALID};
	param_t _param_control_system{PARAM_INVALID};
	param_t _param_control_component{PARAM_INVALID};
	param_t _param_control_link_id{PARAM_INVALID};
	param_t _param_same_endpoint{PARAM_INVALID};
	param_t _param_rc_channel{PARAM_INVALID};
	param_t _param_rc_reject{PARAM_INVALID};
	param_t _param_rc_accept{PARAM_INVALID};
	param_t _param_max_age{PARAM_INVALID};
	param_t _param_action{PARAM_INVALID};
	hrt_abstime _last_task_decision{0};

	int32_t _mode{0};
	int32_t _link_id{0};
	int32_t _instance{0};
	int32_t _source_system{255};
	int32_t _source_component{190};
	int32_t _control_instance{-1};
	int32_t _control_system{0};
	int32_t _control_component{190};
	int32_t _control_link_id{1};
	int32_t _same_endpoint{0};
	int32_t _rc_channel{0};
	int32_t _rc_reject{1300};
	int32_t _rc_accept{1700};
	int32_t _max_age_s{30};
	int32_t _action_mode{0};
	mavlink_m_target_status_s _control_status{};
};
