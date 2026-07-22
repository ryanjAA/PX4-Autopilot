/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file loiter.cpp
 *
 * Helper class to loiter
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "loiter.h"
#include "navigator.h"

namespace
{
constexpr float MavlinkMMinimumApproachGroundspeedMps{3.f};
constexpr float MavlinkMApproachInboundAlignmentCosine{0.5f};
constexpr hrt_abstime MavlinkMApproachVelocityMaxAge{2_s};
}

Loiter::Loiter(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
Loiter::on_inactive()
{
	_loiter_pos_set = false;
	cancel_fly_through();
}

void
Loiter::prepare_fly_through(uint32_t token, hrt_abstime command_timestamp,
		double target_lat, double target_lon, float target_alt)
{
	cancel_fly_through();

	if (token == 0 || command_timestamp == 0 || !PX4_ISFINITE(target_lat)
	    || !PX4_ISFINITE(target_lon) || !PX4_ISFINITE(target_alt)) {
		return;
	}

	_fly_through_pending_token = token;
	_fly_through_pending_command_timestamp = command_timestamp;
	_fly_through_pending_target_lat = target_lat;
	_fly_through_pending_target_lon = target_lon;
	_fly_through_pending_target_alt = target_alt;
}

void
Loiter::clear_fly_through_state()
{
	_fly_through_pending_token = 0;
	_fly_through_pending_command_timestamp = 0;
	_fly_through_pending_target_lat = static_cast<double>(NAN);
	_fly_through_pending_target_lon = static_cast<double>(NAN);
	_fly_through_pending_target_alt = NAN;
	_fly_through_active = false;
	_fly_through_token = 0;
	_fly_through_started = 0;
	_fly_through_start_lat = static_cast<double>(NAN);
	_fly_through_start_lon = static_cast<double>(NAN);
	_fly_through_target_lat = static_cast<double>(NAN);
	_fly_through_target_lon = static_cast<double>(NAN);
	_fly_through_target_alt = NAN;
	_fly_through_approach_active = false;
	_fly_through_approach_lat = static_cast<double>(NAN);
	_fly_through_approach_lon = static_cast<double>(NAN);
	_fly_through_approach_alt = NAN;
	_fly_through_previous_target_north = NAN;
	_fly_through_previous_target_east = NAN;
	_fly_through_previous_alt = NAN;
	_fly_through_previous_sample_valid = false;
}

void
Loiter::cancel_fly_through()
{
	const uint32_t token = _fly_through_active ? _fly_through_token : _fly_through_pending_token;
	clear_fly_through_state();

	if (token != 0) {
		_navigator->publish_mavlink_m_fly_through_ack(token,
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED);
	}
}

void
Loiter::on_activation()
{
	if (_navigator->get_reposition_triplet()->current.valid
	    && hrt_elapsed_time(&_navigator->get_reposition_triplet()->current.timestamp) < 500_ms) {
		reposition();

	} else {
		// this is executed when the flight mode is switched to Hold manually, not through a reposition
		cancel_fly_through();
		set_loiter_position();
	}

	if (_fly_through_active && fly_through_endpoints_allowed()) {
		update_fly_through();

	} else if (_fly_through_active) {
		cancel_fly_through();
	}

	// reset cruising speed to default
	_navigator->reset_cruising_speed();
}

void
Loiter::on_active()
{
	if (_navigator->get_reposition_triplet()->current.valid
	    && hrt_elapsed_time(&_navigator->get_reposition_triplet()->current.timestamp) < 500_ms) {
		reposition();
	}

	if (_fly_through_active) {
		if (fly_through_endpoints_allowed()) {
			update_fly_through();

		} else {
			cancel_fly_through();
		}
	}

	// Reset the loiter position after the fly-through has been cancelled so a
	// disarm or other safety-state loss can never be evaluated as a target hit.
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		_loiter_pos_set = false;
	}
}

void
Loiter::set_loiter_position()
{
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED &&
	    _navigator->get_land_detected()->landed) {

		// Not setting loiter position if disarmed and landed, instead mark the current
		// setpoint as invalid and idle (both, just to be sure).

		_navigator->set_can_loiter_at_sp(false);
		_navigator->get_position_setpoint_triplet()->current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		_navigator->set_position_setpoint_triplet_updated();
		_loiter_pos_set = false;
		return;

	} else if (_loiter_pos_set) {
		// Already set, nothing to do.
		return;
	}

	_loiter_pos_set = true;

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	if (_navigator->get_land_detected()->landed) {
		_mission_item.nav_cmd = NAV_CMD_IDLE;

	} else {
		if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			setLoiterItemFromCurrentPositionWithBreaking(&_mission_item);

		} else {
			setLoiterItemFromCurrentPosition(&_mission_item);
		}

	}

	// convert mission item to current setpoint
	pos_sp_triplet->previous.valid = false;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);
	_navigator->set_position_setpoint_triplet_updated();
}

void
Loiter::reposition()
{
	// we can't reposition if we are not armed yet
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		cancel_fly_through();
		return;
	}

	struct position_setpoint_triplet_s *rep = _navigator->get_reposition_triplet();

	if (rep->current.valid) {
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		const bool direct_shape = rep->next.valid
					  && rep->next.type == position_setpoint_s::SETPOINT_TYPE_LOITER;
		const bool approach_shape = rep->next.valid
					    && rep->next.type == position_setpoint_s::SETPOINT_TYPE_POSITION
					    && rep->next.mavlink_m_exact_altitude;
		const position_setpoint_s &target_setpoint = approach_shape ? rep->next : rep->current;
		const bool fly_through_shape_valid = rep->previous.valid
						 && rep->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION
						 && (direct_shape || approach_shape)
						 && PX4_ISFINITE(rep->current.lat) && PX4_ISFINITE(rep->current.lon)
						 && PX4_ISFINITE(rep->current.alt)
						 && PX4_ISFINITE(target_setpoint.lat) && PX4_ISFINITE(target_setpoint.lon)
						 && PX4_ISFINITE(target_setpoint.alt)
						 && target_setpoint.mavlink_m_exact_altitude;
		const bool pending_target_matches = _fly_through_pending_token != 0
						    && rep->timestamp == _fly_through_pending_command_timestamp
						    && get_distance_to_next_waypoint(target_setpoint.lat, target_setpoint.lon,
							    _fly_through_pending_target_lat,
							    _fly_through_pending_target_lon) <= 1.f
						    && fabsf(target_setpoint.alt - _fly_through_pending_target_alt) <= 1.f;
		const bool fly_through_request = fly_through_shape_valid && pending_target_matches;

		if (fly_through_request) {
			const uint32_t token = _fly_through_pending_token;
			const double target_lat = _fly_through_pending_target_lat;
			const double target_lon = _fly_through_pending_target_lon;
			const float target_alt = _fly_through_pending_target_alt;
			memcpy(pos_sp_triplet, rep, sizeof(*rep));
			clear_fly_through_state();
			_fly_through_active = true;
			_fly_through_token = token;
			_fly_through_started = hrt_absolute_time();
			_fly_through_start_lat = rep->previous.lat;
			_fly_through_start_lon = rep->previous.lon;
			_fly_through_target_lat = target_lat;
			_fly_through_target_lon = target_lon;
			_fly_through_target_alt = target_alt;
			_fly_through_approach_active = approach_shape;
			_fly_through_approach_lat = approach_shape ? rep->current.lat : static_cast<double>(NAN);
			_fly_through_approach_lon = approach_shape ? rep->current.lon : static_cast<double>(NAN);
			_fly_through_approach_alt = approach_shape ? rep->current.alt : NAN;

			if (!approach_shape) {
				reset_fly_through_crossing_sample();
			}

			_loiter_pos_set = false;

		} else {
			cancel_fly_through();
			// Preserve the standard DO_REPOSITION behavior. It remains a direct
			// target-centered loiter and never enters the private fly-through path.
			pos_sp_triplet->previous.yaw = _navigator->get_local_position()->heading;
			pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
			pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
			memcpy(&pos_sp_triplet->current, &rep->current, sizeof(rep->current));
			pos_sp_triplet->next.valid = false;
		}

		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);
		_navigator->set_position_setpoint_triplet_updated();

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}
}

bool
Loiter::fly_through_endpoints_allowed()
{
	const bool target_allowed = _navigator->mavlink_m_fly_through_allowed(
					_fly_through_target_lat, _fly_through_target_lon, _fly_through_target_alt);
	const bool approach_allowed = !_fly_through_approach_active
				      || _navigator->mavlink_m_fly_through_allowed(
					      _fly_through_approach_lat, _fly_through_approach_lon,
					      _fly_through_approach_alt);

	return target_allowed && approach_allowed;
}

void
Loiter::update_fly_through()
{
	if (!_fly_through_active) {
		return;
	}

	// Publish the token-bound approach or target triplet before it can be
	// promoted. The receiver must observe ownership of that exact shape,
	// including for a cue created at the aircraft's current location.
	if (_fly_through_started == 0 || hrt_elapsed_time(&_fly_through_started) < 100_ms) {
		return;
	}

	position_setpoint_triplet_s *triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s &current = triplet->current;
	const position_setpoint_s &next = triplet->next;
	const bool approach_owned = _fly_through_approach_active
					    && current.valid && current.type == position_setpoint_s::SETPOINT_TYPE_POSITION
					    && next.valid && next.type == position_setpoint_s::SETPOINT_TYPE_POSITION
					    && next.mavlink_m_exact_altitude
					    && get_distance_to_next_waypoint(current.lat, current.lon,
						    _fly_through_approach_lat, _fly_through_approach_lon) <= 1.f
					    && fabsf(current.alt - _fly_through_approach_alt) <= 1.f
					    && get_distance_to_next_waypoint(next.lat, next.lon,
						    _fly_through_target_lat, _fly_through_target_lon) <= 1.f
					    && fabsf(next.alt - _fly_through_target_alt) <= 1.f;
	const bool target_owned = !_fly_through_approach_active
					  && current.valid && next.valid
					  && current.type == position_setpoint_s::SETPOINT_TYPE_POSITION
					  && current.mavlink_m_exact_altitude
					  && next.type == position_setpoint_s::SETPOINT_TYPE_LOITER
					  && get_distance_to_next_waypoint(current.lat, current.lon,
						  _fly_through_target_lat, _fly_through_target_lon) <= 1.f
					  && get_distance_to_next_waypoint(next.lat, next.lon,
						  _fly_through_target_lat, _fly_through_target_lon) <= 1.f
					  && fabsf(current.alt - _fly_through_target_alt) <= 1.f
					  && fabsf(next.alt - _fly_through_target_alt) <= 1.f;
	const bool setpoint_owned = approach_owned || target_owned;

	if (!setpoint_owned) {
		cancel_fly_through();
		return;
	}

	if (_fly_through_approach_active) {
		const float distance_to_approach = get_distance_to_next_waypoint(
						 _navigator->get_global_position()->lat,
						 _navigator->get_global_position()->lon,
						 _fly_through_approach_lat, _fly_through_approach_lon);
		const float approach_acceptance = PX4_ISFINITE(current.acceptance_radius)
						  ? math::max(current.acceptance_radius, Navigator::MavlinkMHitRadiusM)
						  : math::max(_navigator->get_acceptance_radius(), Navigator::MavlinkMHitRadiusM);
		const float altitude_error = fabsf(_navigator->get_global_position()->alt - _fly_through_approach_alt);
		const float nav_altitude_acceptance = _navigator->get_altitude_acceptance_radius();
		const float altitude_acceptance = PX4_ISFINITE(nav_altitude_acceptance)
						  ? math::max(nav_altitude_acceptance, Navigator::MavlinkMHitRadiusM)
						  : Navigator::MavlinkMHitRadiusM;
		bool inbound_aligned = true;

		if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			float approach_to_target_north = 0.f;
			float approach_to_target_east = 0.f;
			get_vector_to_next_waypoint_fast(_fly_through_approach_lat, _fly_through_approach_lon,
						 _fly_through_target_lat, _fly_through_target_lon,
						 &approach_to_target_north, &approach_to_target_east);
			const vehicle_local_position_s *local_position = _navigator->get_local_position();
			const float approach_leg_length = hypotf(approach_to_target_north, approach_to_target_east);
			const float ground_speed = hypotf(local_position->vx, local_position->vy);
			const hrt_abstime now = hrt_absolute_time();
			inbound_aligned = local_position->v_xy_valid
					  && local_position->timestamp != 0 && now >= local_position->timestamp
					  && now - local_position->timestamp <= MavlinkMApproachVelocityMaxAge
					  && PX4_ISFINITE(approach_to_target_north) && PX4_ISFINITE(approach_to_target_east)
					  && PX4_ISFINITE(local_position->vx) && PX4_ISFINITE(local_position->vy)
					  && PX4_ISFINITE(approach_leg_length) && approach_leg_length > 1.f
					  && PX4_ISFINITE(ground_speed) && ground_speed >= MavlinkMMinimumApproachGroundspeedMps;

			if (inbound_aligned) {
				const float alignment_cosine = (local_position->vx * approach_to_target_north
								+ local_position->vy * approach_to_target_east)
							       / (ground_speed * approach_leg_length);
				inbound_aligned = PX4_ISFINITE(alignment_cosine)
						  && alignment_cosine >= MavlinkMApproachInboundAlignmentCosine;
			}
		}

		if (PX4_ISFINITE(distance_to_approach) && distance_to_approach <= approach_acceptance
		    && PX4_ISFINITE(altitude_error) && altitude_error <= altitude_acceptance
		    && inbound_aligned) {
			promote_fly_through_approach();
		}

		return;
	}

	const float distance_to_target = get_distance_to_next_waypoint(
					 _navigator->get_global_position()->lat,
					 _navigator->get_global_position()->lon,
					 _fly_through_target_lat, _fly_through_target_lon);
	const float nav_altitude_radius = _navigator->get_altitude_acceptance_radius();
	const float altitude_hit_radius = PX4_ISFINITE(nav_altitude_radius) && nav_altitude_radius > 0.f
					  ? math::min(nav_altitude_radius, Navigator::MavlinkMHitRadiusM)
					  : Navigator::MavlinkMHitRadiusM;
	const float altitude_error = fabsf(_navigator->get_global_position()->alt - _fly_through_target_alt);
	float inbound_north = 0.f;
	float inbound_east = 0.f;
	get_vector_to_next_waypoint_fast(_fly_through_start_lat, _fly_through_start_lon,
					 _fly_through_target_lat, _fly_through_target_lon,
					 &inbound_north, &inbound_east);
	const float inbound_distance = sqrtf(inbound_north * inbound_north + inbound_east * inbound_east);

	// A cue created at the current coordinate has no meaningful target plane.
	// Hold the exact POSITION setpoint until both tight bounds are satisfied.
	// Failing immediately would reject a valid vertical arrival before a
	// multicopter has had time to climb or descend to the cue altitude.
	if (inbound_distance <= 1.f) {
		const bool target_hit = PX4_ISFINITE(distance_to_target)
					&& distance_to_target <= Navigator::MavlinkMHitRadiusM
					&& PX4_ISFINITE(altitude_error) && altitude_error <= altitude_hit_radius;

		if (target_hit) {
			complete_fly_through(true);
		}

		return;
	}

	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		float target_to_vehicle_north = 0.f;
		float target_to_vehicle_east = 0.f;
		get_vector_to_next_waypoint_fast(_fly_through_target_lat, _fly_through_target_lon,
						 _navigator->get_global_position()->lat,
						 _navigator->get_global_position()->lon,
						 &target_to_vehicle_north, &target_to_vehicle_east);
		const float unit_north = inbound_north / inbound_distance;
		const float unit_east = inbound_east / inbound_distance;
		const float current_along = target_to_vehicle_north * unit_north
					    + target_to_vehicle_east * unit_east;
		const float previous_along = _fly_through_previous_target_north * unit_north
					     + _fly_through_previous_target_east * unit_east;
		const bool crossing = _fly_through_previous_sample_valid
				      && PX4_ISFINITE(previous_along) && PX4_ISFINITE(current_along)
				      && previous_along < 0.f && current_along >= 0.f;

		if (crossing) {
			const float along_delta = current_along - previous_along;
			const float alpha = along_delta > FLT_EPSILON
					    ? math::constrain(-previous_along / along_delta, 0.f, 1.f) : 1.f;
			const float crossing_north = _fly_through_previous_target_north
						     + alpha * (target_to_vehicle_north - _fly_through_previous_target_north);
			const float crossing_east = _fly_through_previous_target_east
						    + alpha * (target_to_vehicle_east - _fly_through_previous_target_east);
			const float crossing_alt = _fly_through_previous_alt
						   + alpha * (_navigator->get_global_position()->alt - _fly_through_previous_alt);
			const float horizontal_miss = sqrtf(crossing_north * crossing_north + crossing_east * crossing_east);
			const float vertical_miss = fabsf(crossing_alt - _fly_through_target_alt);
			const bool target_hit = PX4_ISFINITE(horizontal_miss) && PX4_ISFINITE(vertical_miss)
						&& horizontal_miss <= Navigator::MavlinkMHitRadiusM
						&& vertical_miss <= altitude_hit_radius;
			complete_fly_through(target_hit, crossing_alt);
			return;
		}

		_fly_through_previous_target_north = target_to_vehicle_north;
		_fly_through_previous_target_east = target_to_vehicle_east;
		_fly_through_previous_alt = _navigator->get_global_position()->alt;
		_fly_through_previous_sample_valid = PX4_ISFINITE(target_to_vehicle_north)
							     && PX4_ISFINITE(target_to_vehicle_east)
							     && PX4_ISFINITE(_fly_through_previous_alt);

	} else if (PX4_ISFINITE(distance_to_target)
		   && distance_to_target <= Navigator::MavlinkMHitRadiusM
		   && PX4_ISFINITE(altitude_error) && altitude_error <= altitude_hit_radius) {
		complete_fly_through(true);
	}
}

void
Loiter::promote_fly_through_approach()
{
	position_setpoint_triplet_s *triplet = _navigator->get_position_setpoint_triplet();
	const hrt_abstime now = hrt_absolute_time();
	triplet->previous = triplet->current;
	triplet->previous.timestamp = now;
	triplet->current = triplet->next;
	triplet->current.timestamp = now;
	triplet->next = triplet->current;
	triplet->next.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	triplet->next.loiter_radius = _navigator->get_loiter_radius();
	triplet->next.mavlink_m_exact_altitude = false;
	triplet->next.timestamp = now;
	triplet->timestamp = now;
	_fly_through_start_lat = _fly_through_approach_lat;
	_fly_through_start_lon = _fly_through_approach_lon;
	_fly_through_approach_active = false;
	reset_fly_through_crossing_sample();
	_navigator->set_can_loiter_at_sp(false);
	_navigator->set_position_setpoint_triplet_updated();
}

void
Loiter::reset_fly_through_crossing_sample()
{
	get_vector_to_next_waypoint_fast(_fly_through_target_lat, _fly_through_target_lon,
					 _navigator->get_global_position()->lat,
					 _navigator->get_global_position()->lon,
					 &_fly_through_previous_target_north,
					 &_fly_through_previous_target_east);
	_fly_through_previous_alt = _navigator->get_global_position()->alt;
	_fly_through_previous_sample_valid = PX4_ISFINITE(_fly_through_previous_target_north)
					     && PX4_ISFINITE(_fly_through_previous_target_east)
					     && PX4_ISFINITE(_fly_through_previous_alt);
}

void
Loiter::complete_fly_through(bool target_hit, float safe_loiter_altitude)
{
	position_setpoint_triplet_s *triplet = _navigator->get_position_setpoint_triplet();
	const hrt_abstime now = hrt_absolute_time();
	const uint32_t token = _fly_through_token;
	triplet->previous = triplet->current;
	triplet->previous.timestamp = now;
	triplet->current = triplet->next;
	triplet->current.timestamp = now;

	if (!target_hit) {
		const float current_altitude = _navigator->get_global_position()->alt;
		const float hold_altitude = PX4_ISFINITE(safe_loiter_altitude)
					    ? safe_loiter_altitude : current_altitude;

		if (PX4_ISFINITE(hold_altitude)) {
			triplet->previous.alt = hold_altitude;
			triplet->current.alt = hold_altitude;
		}

		triplet->previous.mavlink_m_exact_altitude = false;
		triplet->current.mavlink_m_exact_altitude = false;
	}

	triplet->next = position_setpoint_s{};
	triplet->timestamp = now;
	clear_fly_through_state();
	_loiter_pos_set = true;
	_navigator->set_can_loiter_at_sp(true);
	_navigator->set_position_setpoint_triplet_updated();
	_navigator->publish_mavlink_m_fly_through_ack(token,
			target_hit ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
			: vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED,
			target_hit ? 100 : 1);

	if (!target_hit) {
		PX4_WARN("MAVLink-M exact target was missed; holding at target-centered loiter");
	}
}
