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
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

class Mavlink;

/**
 * Endpoint for the finalized Dronecode MAVLink-M dialect.
 *
 * Cue and handover packets are route-selected by MAVLink instance, bound to a
 * configured source selector and exact component, and durable before receipt
 * is acknowledged. Optional
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
	bool handle_message(const mavlink_message_t &message, bool udp_endpoint_authorized);
	void update();
	void update_parameters();

	/// A fleet peer is learned only from a fully decoded GCS heartbeat on the
	/// configured MAVLink-M cue instance. The component and signing link must
	/// match either the cue-source role or the valid ESAD owner-control role.
	bool accepts_udp_peer_heartbeat(const mavlink_message_t &message) const;
	/// A selected physical-mode route remains a route-wide deny boundary until
	/// both transmit signing and receiver signature validation are active.
	bool ingress_locked() const;
	/// Once a MAVLink-M route is selected, ordinary PX4 ingress belongs only to
	/// the configured owner-control identity on its exact route. Learned peers
	/// remain read-only observers unless an explicit wildcard grants authority.
	bool generic_ingress_allowed(const mavlink_message_t &message,
				     bool udp_endpoint_authorized) const;
	unsigned udp_peer_limit() const { return static_cast<unsigned>(_peer_limit); }
	hrt_abstime udp_peer_timeout() const
	{
		return static_cast<hrt_abstime>(_peer_timeout_s) * 1'000'000ULL;
	}

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

	enum class InterceptPhase : uint8_t {
		None = 0,
		Transit,
		Dwell,
		Complete,
		Aborted,
	};

	enum class CommandApplicationResult : uint8_t {
		FailedBeforePublication = 0,
		Applied,
		PersistenceFailedAfterPublicationStopped,
		PersistenceFailedAfterPublicationStopUnconfirmed,
	};

	struct InterceptTracking {
		InterceptPhase phase{InterceptPhase::None};
		hrt_abstime command_time{0};
		hrt_abstime completion_ack_time{0};
		hrt_abstime completion_wait_started{0};
		hrt_abstime dwell_started{0};
		uint32_t message_id{0};
		uint32_t instance_id{0};
		uint32_t token{0};
		uint8_t source_system{0};
		uint8_t source_component{0};
		float acceptance_altitude_m{NAN};
		float expected_altitude_m{NAN};
		bool setpoint_seen{false};
		bool navigator_started{false};
		bool navigator_completed{false};
		bool navigator_missed{false};
		bool navigator_failed{false};
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
		uint8_t execution_effect{0};
	};
	static_assert(sizeof(Assignment) == 152,
		      "per-cue effect must remain inside the version 6 assignment padding");

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
	static constexpr uint16_t PersistenceVersion = 7;
	static constexpr uint16_t PreviousPersistenceVersion = 6;
	static constexpr unsigned InboxCapacity = 2;
	static constexpr unsigned TerminalCapacity = 8;
	static constexpr unsigned TrackIdentityCapacity = 4;
	static constexpr unsigned ControlCapacity = 8;
	static constexpr hrt_abstime SourceFreshTimeout = 15'000'000;
	static constexpr uint8_t CommandNav = 1 << 0;
	static constexpr uint8_t CommandInterceptAltitude = 1 << 1;
	// Persisted only on a superseded terminal assignment until PX4 has either
	// replaced or stopped that assignment's navigation. This is deliberately
	// distinct from CommandNav, which remains useful historical evidence after
	// an ordinary confirmed Abort.
	static constexpr uint8_t CommandStopPending = 1 << 2;
	static constexpr uint8_t CommandExecutionAll = CommandNav | CommandInterceptAltitude;
	static constexpr uint8_t CommandAll = CommandExecutionAll | CommandStopPending;
	static constexpr uint8_t CommandTargetLocalComponent = 255;
	static_assert(CommandExecutionAll == 3 && CommandAll == 7,
		      "MAVLink-M command and stop-state flags must remain stable");

	bool cue_physical_route_selected() const;
	bool control_physical_route_selected() const;
	bool cue_route_selected() const;
	bool control_route_selected() const;
	bool signing_active() const;
	bool enabled() const;
	bool control_enabled() const;
	bool signing_required() const;
	bool signing_link_matches(const mavlink_message_t &message, int32_t expected_link_id) const;
	bool source_matches(const mavlink_message_t &message) const;
	bool control_source_matches(const mavlink_message_t &message) const;
	bool source_recent(uint8_t source_system) const;
	bool task_message_allowed(const mavlink_message_t &message, bool udp_endpoint_authorized) const;
	bool esad_control_allowed(const mavlink_message_t &message, bool udp_endpoint_authorized) const;
	bool handle_control_command(const mavlink_message_t &message, bool udp_endpoint_authorized);
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
	void accept_pending(uint32_t message_id = 0, uint32_t instance_id = 0,
			    uint8_t requested_effect = mavlink_m_task_decision_s::EFFECT_DEFAULT);
	void reject_pending_or_abort_active(uint32_t message_id = 0, uint32_t instance_id = 0);
	bool assignment_requests_movement(const Assignment &assignment) const;
	bool movement_acceptance_ready(const Assignment &assignment, const char **reason);
	bool vehicle_ready_for_reposition() const;
	bool vehicle_airborne_for_reposition() const;
	bool publish_vehicle_command(const Assignment &assignment, uint32_t command, float param1 = NAN, float param2 = NAN,
				      float param3 = NAN, float param4 = NAN, double param5 = static_cast<double>(NAN),
				      double param6 = static_cast<double>(NAN),
				      float param7 = NAN, uint8_t target_component = CommandTargetLocalComponent);
	bool publish_internal_fly_through(const Assignment &assignment, float altitude_m,
					  float recovery_altitude_m, float minimum_clearance_m, uint32_t token);
	CommandApplicationResult command_active_assignment();
	bool assignment_cancellation_ready(const Assignment &assignment);
	bool cancel_assignment_commands(const Assignment &assignment);
	Assignment *find_navigation_stop_marker();
	bool stop_deferred_navigation();
	void update_intercept_command_ack();
	void update_intercept();
	void begin_intercept_tracking(float acceptance_altitude_m, float target_altitude_m, uint32_t token);
	void abort_intercept(const char *reason, bool stop_navigation = false);
	void clear_intercept_tracking();
	InterceptTracking intercept_tracking() const;
	void restore_intercept_tracking(const InterceptTracking &tracking);
	bool intercept_assignment_matches(const Assignment &assignment) const;
	bool intercept_transit_setpoint_matches(float expected_altitude_m) const;
	bool intercept_recovery_setpoint_matches(float recovery_altitude_m) const;
	bool intercept_loiter_setpoint_matches(float expected_altitude_m) const;
	bool intercept_safety_gates_valid(const char **reason) const;
	bool intercept_terrain_clearance_valid(float candidate_altitude_m, const char **reason) const;
	float intercept_arrival_radius() const;

	void send_ack(const Assignment &assignment, uint8_t result, const char *reason);
	void send_control_command_ack(const mavlink_message_t &request, uint8_t result);
	void send_control_status();
	void send_osd_vector();
	void publish_status();
	void apply_udp_peer_configuration();

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
	Assignment _deferred_navigation_stop{};
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

	hrt_abstime _source_last_seen[UINT8_MAX + 1]{};
	hrt_abstime _last_status_publish{0};
	hrt_abstime _last_control_status_send{0};
	hrt_abstime _last_osd_send{0};
	hrt_abstime _last_reposition_block_log{0};
	hrt_abstime _intercept_command_time{0};
	hrt_abstime _intercept_completion_ack_time{0};
	hrt_abstime _intercept_completion_wait_started{0};
	hrt_abstime _intercept_dwell_started{0};

	vehicle_global_position_s _global_position{};
	vehicle_local_position_s _local_position{};
	vehicle_attitude_s _attitude{};
	vehicle_land_detected_s _vehicle_land_detected{};
	vehicle_status_s _vehicle_status{};
	position_setpoint_triplet_s _position_setpoint_triplet{};

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _task_decision_sub{ORB_ID(mavlink_m_task_decision)};
	uORB::Subscription _target_status_sub{ORB_ID(mavlink_m_target_status)};
	uORB::Subscription _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
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
	param_t _param_peer_limit{PARAM_INVALID};
	param_t _param_peer_timeout{PARAM_INVALID};
	param_t _param_rc_channel{PARAM_INVALID};
	param_t _param_rc_reject{PARAM_INVALID};
	param_t _param_rc_accept{PARAM_INVALID};
	param_t _param_max_age{PARAM_INVALID};
	param_t _param_action{PARAM_INVALID};
	param_t _param_intercept_radius{PARAM_INVALID};
	param_t _param_intercept_dwell{PARAM_INVALID};
	param_t _param_intercept_delta_z{PARAM_INVALID};
	param_t _param_intercept_clearance{PARAM_INVALID};
	param_t _param_nav_loiter_radius{PARAM_INVALID};
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
	int32_t _peer_limit{4};
	int32_t _peer_timeout_s{30};
	int32_t _rc_channel{0};
	int32_t _rc_reject{1300};
	int32_t _rc_accept{1700};
	int32_t _max_age_s{30};
	int32_t _action_mode{0};
	float _intercept_radius_m{25.f};
	float _intercept_dwell_s{3.f};
	float _intercept_delta_z_m{100.f};
	float _intercept_clearance_m{30.f};
	float _nav_loiter_radius_m{80.f};
	InterceptPhase _intercept_phase{InterceptPhase::None};
	uint32_t _intercept_message_id{0};
	uint32_t _intercept_instance_id{0};
	uint32_t _intercept_token_counter{0};
	uint32_t _intercept_token{0};
	uint8_t _intercept_source_system{0};
	uint8_t _intercept_source_component{0};
	float _intercept_acceptance_altitude_m{NAN};
	float _intercept_expected_altitude_m{NAN};
	bool _intercept_setpoint_seen{false};
	bool _intercept_navigator_started{false};
	bool _intercept_navigator_completed{false};
	bool _intercept_navigator_missed{false};
	bool _intercept_navigator_failed{false};
	bool _udp_peer_configuration_dirty{true};
	bool _signing_failure_warned{false};
	hrt_abstime _last_signing_retry{0};
	mavlink_m_target_status_s _control_status{};
};
