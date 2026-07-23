#!/usr/bin/env python3
"""Focused source and policy checks for guarded MAVLink-M intercept."""

from __future__ import annotations

import math
import re
import unittest
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
HANDLER = (ROOT / "src/modules/mavlink/MavlinkMHandler.cpp").read_text()
HEADER = (ROOT / "src/modules/mavlink/MavlinkMHandler.hpp").read_text()
PARAMETERS = (ROOT / "src/modules/mavlink/mavlink_params.c").read_text()
STATUS = (ROOT / "msg/MavlinkMTargetStatus.msg").read_text()
DECISION = (ROOT / "msg/MavlinkMTaskDecision.msg").read_text()
VEHICLE_COMMAND = (ROOT / "msg/VehicleCommand.msg").read_text()
NAVIGATOR = (ROOT / "src/modules/navigator/navigator_main.cpp").read_text()
LOITER = (ROOT / "src/modules/navigator/loiter.cpp").read_text()
NAVIGATOR_HEADER = (ROOT / "src/modules/navigator/navigator.h").read_text()
FW_CONTROL = (ROOT / "src/modules/fw_pos_control/FixedwingPositionControl.cpp").read_text()
POSITION_SETPOINT = (ROOT / "msg/PositionSetpoint.msg").read_text()
COMMANDER = (ROOT / "src/modules/commander/Commander.cpp").read_text()
MAVLINK_MAIN = (ROOT / "src/modules/mavlink/mavlink_main.cpp").read_text()
MAVLINK_HEADER = (ROOT / "src/modules/mavlink/mavlink_main.h").read_text()
SITL_RUNNER = (ROOT / "Tools/aags_mavlink_m/run_sitl_acceptance.py").read_text()
DUAL_RUNNER_PATH = ROOT / "Tools/aags_mavlink_m/run_dual_gazebo.sh"
DUAL_RUNNER = DUAL_RUNNER_PATH.read_text() if DUAL_RUNNER_PATH.exists() else ""
ENDPOINT_TOOL = (ROOT / "Tools/aags_mavlink_m/endpoint_tool.py").read_text()
MAVLINK_TIMESYNC = (ROOT / "src/modules/mavlink/mavlink_timesync.cpp").read_text()


class Phase(IntEnum):
    NONE = 0
    TRANSIT = 1
    DWELL = 2
    COMPLETE = 3
    ABORTED = 4


@dataclass
class PolicyModel:
    phase: Phase = Phase.TRANSIT
    dwell_started: float | None = None
    completions: int = 0
    navigation_cancels: int = 0

    def update(
        self,
        now: float,
        *,
        gates_valid: bool = True,
        setpoint_owned: bool = True,
        fly_through_complete: bool = False,
        completion_token_matches: bool = True,
        inside_radius: bool = False,
        dwell_s: float = 3.0,
    ) -> None:
        if self.phase in (Phase.NONE, Phase.ABORTED):
            return
        if not gates_valid or not setpoint_owned:
            self.navigation_cancels += 1
            self.phase = Phase.ABORTED
            self.dwell_started = None
            return
        if self.phase == Phase.COMPLETE:
            return
        if fly_through_complete and not completion_token_matches:
            return
        if self.phase == Phase.TRANSIT and not fly_through_complete:
            return
        if self.phase == Phase.TRANSIT:
            self.phase = Phase.DWELL
        if not inside_radius:
            self.dwell_started = None
            return
        if self.phase == Phase.DWELL and self.dwell_started is None:
            self.dwell_started = now
        if self.dwell_started is not None and now - self.dwell_started >= dwell_s:
            self.completions += 1
            self.phase = Phase.COMPLETE


def ground_speed_bounds(
    maximum_airspeed_m_s: float,
    wind_north_m_s: float,
    wind_east_m_s: float,
    variance_north: float,
    variance_east: float,
    sample_age_s: float,
) -> tuple[float, float] | None:
    values = (
        maximum_airspeed_m_s,
        wind_north_m_s,
        wind_east_m_s,
        variance_north,
        variance_east,
        sample_age_s,
    )
    if (
        not all(math.isfinite(value) for value in values)
        or maximum_airspeed_m_s <= 1.0
        or variance_north < 0.0
        or variance_east < 0.0
        or sample_age_s < 0.0
        or sample_age_s > 2.0
    ):
        return None
    tas_bound = 2.0 * maximum_airspeed_m_s
    wind_bound = math.hypot(wind_north_m_s, wind_east_m_s) + 3.0 * math.sqrt(
        max(0.0, variance_north) + max(0.0, variance_east)
    )
    ground_bound = tas_bound + wind_bound
    if not all(math.isfinite(value) for value in (tas_bound, wind_bound, ground_bound)):
        return None
    return tas_bound, ground_bound


def approach_distance(
    altitude_change_m: float,
    pitch_limit_deg: float,
    vertical_rate_m_s: float,
    true_airspeed_bound_m_s: float,
    ground_speed_bound_m_s: float,
) -> float:
    pitch_ground_gradient = (
        math.tan(math.radians(pitch_limit_deg))
        * true_airspeed_bound_m_s
        / ground_speed_bound_m_s
    )
    rate_gradient = vertical_rate_m_s / ground_speed_bound_m_s
    return abs(altitude_change_m) / (
        min(pitch_ground_gradient, rate_gradient) * 0.8
    )


def approach_inbound_aligned(
    velocity_north_m_s: float,
    velocity_east_m_s: float,
    approach_to_target_north_m: float,
    approach_to_target_east_m: float,
    velocity_age_s: float,
    *,
    fixed_wing: bool = True,
) -> bool:
    if not fixed_wing:
        return True
    values = (
        velocity_north_m_s,
        velocity_east_m_s,
        approach_to_target_north_m,
        approach_to_target_east_m,
        velocity_age_s,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    ground_speed = math.hypot(velocity_north_m_s, velocity_east_m_s)
    approach_leg_length = math.hypot(
        approach_to_target_north_m, approach_to_target_east_m
    )
    if velocity_age_s < 0.0 or velocity_age_s > 2.0:
        return False
    if ground_speed < 3.0 or approach_leg_length <= 1.0:
        return False
    alignment_cosine = (
        velocity_north_m_s * approach_to_target_north_m
        + velocity_east_m_s * approach_to_target_east_m
    ) / (ground_speed * approach_leg_length)
    return math.isfinite(alignment_cosine) and alignment_cosine >= 0.5


def restrictive_geofence_blocks_intercept(
    action: int,
    polygon_or_circle_count: int,
    max_horizontal_distance_m: float,
    max_vertical_distance_m: float,
) -> bool:
    configured = (
        polygon_or_circle_count > 0
        or max_horizontal_distance_m > 1.0e-7
        or max_vertical_distance_m > 1.0e-7
    )
    return configured and action not in (0, 1)


def effective_radius(configured_m: float, fixed_wing: bool, loiter_m: float) -> float:
    return max(configured_m, abs(loiter_m) + 10.0) if fixed_wing else configured_m


def terrain_clearance_allowed(
    minimum_m: float,
    *,
    terrain_fresh: bool,
    hagl_fresh: bool,
    aircraft_alt_msl_m: float,
    terrain_alt_msl_m: float,
    hagl_m: float,
    candidate_alt_msl_m: float,
) -> bool:
    if minimum_m == -1.0:
        return True
    values = (
        minimum_m,
        aircraft_alt_msl_m,
        terrain_alt_msl_m,
        hagl_m,
        candidate_alt_msl_m,
    )
    return (
        all(math.isfinite(value) for value in values)
        and 0.0 <= minimum_m <= 1000.0
        and terrain_fresh
        and hagl_fresh
        and aircraft_alt_msl_m - terrain_alt_msl_m >= minimum_m
        and hagl_m >= minimum_m
        and candidate_alt_msl_m - terrain_alt_msl_m >= minimum_m
    )


def interpolated_target_plane_miss(
    start: tuple[float, float],
    target: tuple[float, float],
    previous_vehicle: tuple[float, float],
    current_vehicle: tuple[float, float],
    previous_alt: float,
    current_alt: float,
    target_alt: float,
) -> tuple[float, float] | None:
    inbound = (target[0] - start[0], target[1] - start[1])
    inbound_length = math.hypot(*inbound)
    unit = (inbound[0] / inbound_length, inbound[1] / inbound_length)
    previous_relative = (
        previous_vehicle[0] - target[0],
        previous_vehicle[1] - target[1],
    )
    current_relative = (
        current_vehicle[0] - target[0],
        current_vehicle[1] - target[1],
    )
    previous_along = previous_relative[0] * unit[0] + previous_relative[1] * unit[1]
    current_along = current_relative[0] * unit[0] + current_relative[1] * unit[1]
    if previous_along >= 0.0 or current_along < 0.0:
        return None
    alpha = -previous_along / (current_along - previous_along)
    crossing = (
        previous_relative[0] + alpha * (current_relative[0] - previous_relative[0]),
        previous_relative[1] + alpha * (current_relative[1] - previous_relative[1]),
    )
    crossing_alt = previous_alt + alpha * (current_alt - previous_alt)
    return math.hypot(*crossing), abs(crossing_alt - target_alt)


class InterceptPolicyTest(unittest.TestCase):
    def test_continuous_dwell_resets_after_radius_exit(self) -> None:
        policy = PolicyModel()
        policy.update(0.0, fly_through_complete=True, inside_radius=True)
        policy.update(2.9, fly_through_complete=True, inside_radius=True)
        self.assertEqual(policy.completions, 0)
        policy.update(3.0, fly_through_complete=True, inside_radius=False)
        policy.update(10.0, fly_through_complete=True, inside_radius=True)
        policy.update(12.9, fly_through_complete=True, inside_radius=True)
        self.assertEqual(policy.completions, 0)
        policy.update(13.0, fly_through_complete=True, inside_radius=True)
        self.assertEqual(policy.completions, 1)

    def test_completion_is_exactly_once(self) -> None:
        policy = PolicyModel()
        policy.update(0.0, fly_through_complete=True, inside_radius=True)
        policy.update(3.0, fly_through_complete=True, inside_radius=True)
        policy.update(4.0, fly_through_complete=True, inside_radius=True)
        self.assertEqual(policy.phase, Phase.COMPLETE)
        self.assertEqual(policy.completions, 1)

    def test_radius_entry_cannot_replace_target_plane_crossing(self) -> None:
        policy = PolicyModel()
        policy.update(0.0, inside_radius=True)
        policy.update(30.0, inside_radius=True)
        self.assertEqual(policy.phase, Phase.TRANSIT)
        self.assertEqual(policy.completions, 0)
        policy.update(31.0, fly_through_complete=True, inside_radius=True)
        policy.update(34.0, fly_through_complete=True, inside_radius=True)
        self.assertEqual(policy.completions, 1)

    def test_matching_completion_token_is_mandatory(self) -> None:
        policy = PolicyModel()
        policy.update(
            0.0,
            fly_through_complete=True,
            completion_token_matches=False,
            inside_radius=True,
        )
        policy.update(
            30.0,
            fly_through_complete=True,
            completion_token_matches=False,
            inside_radius=True,
        )
        self.assertEqual(policy.phase, Phase.TRANSIT)
        self.assertEqual(policy.completions, 0)

    def test_fixed_wing_target_plane_geometry_is_bounded_and_one_shot(self) -> None:
        start = (0.0, 0.0)
        target = (100.0, 0.0)
        hit = interpolated_target_plane_miss(
            start, target, (99.0, 4.0), (101.0, 4.0), 500.0, 500.0, 500.0
        )
        lateral_miss = interpolated_target_plane_miss(
            start, target, (99.0, 100.0), (101.0, 100.0), 500.0, 500.0, 500.0
        )
        altitude_miss = interpolated_target_plane_miss(
            start, target, (99.0, 0.0), (101.0, 0.0), 520.0, 520.0, 500.0
        )
        self.assertLessEqual(hit[0], 5.0)
        self.assertGreater(lateral_miss[0], 5.0)
        self.assertGreater(altitude_miss[1], 5.0)

    def test_descent_distance_uses_rate_limit_when_it_is_shallower(self) -> None:
        tas_bound, ground_bound = ground_speed_bounds(20.0, 0.0, 0.0, 0.0, 0.0, 0.5)
        distance = approach_distance(50.0, 15.0, 2.7, tas_bound, ground_bound)
        self.assertAlmostEqual(distance, 50.0 / ((2.7 / 40.0) * 0.8), places=4)

    def test_descent_distance_uses_pitch_limit_when_it_is_shallower(self) -> None:
        tas_bound, ground_bound = ground_speed_bounds(20.0, 0.0, 0.0, 0.0, 0.0, 0.5)
        distance = approach_distance(50.0, 15.0, 15.0, tas_bound, ground_bound)
        self.assertAlmostEqual(
            distance, 50.0 / (math.tan(math.radians(15.0)) * 0.8), places=4
        )

    def test_ground_bound_includes_doubled_tas_wind_and_three_sigma(self) -> None:
        tas_bound, ground_bound = ground_speed_bounds(20.0, 3.0, 4.0, 4.0, 5.0, 2.0)
        self.assertEqual(tas_bound, 40.0)
        self.assertEqual(ground_bound, 54.0)

    def test_ground_bound_rejects_stale_invalid_or_negative_wind(self) -> None:
        self.assertIsNone(ground_speed_bounds(20.0, 0.0, 0.0, 0.0, 0.0, 2.001))
        self.assertIsNone(ground_speed_bounds(20.0, math.nan, 0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(ground_speed_bounds(20.0, 0.0, 0.0, -0.01, 0.0, 0.0))
        self.assertIsNone(ground_speed_bounds(20.0, 0.0, 0.0, 0.0, math.inf, 0.0))

    def test_tailwind_and_uncertainty_lengthen_the_approach(self) -> None:
        calm_tas, calm_ground = ground_speed_bounds(20.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        wind_tas, wind_ground = ground_speed_bounds(20.0, 20.0, 0.0, 9.0, 16.0, 0.1)
        calm_distance = approach_distance(50.0, 15.0, 2.7, calm_tas, calm_ground)
        wind_distance = approach_distance(50.0, 15.0, 2.7, wind_tas, wind_ground)
        self.assertGreater(wind_ground, calm_ground)
        self.assertGreater(wind_distance, calm_distance)

    def test_close_target_places_entry_behind_aircraft(self) -> None:
        start_x = 0.0
        target_x = 100.0
        required_slope = 300.0
        transition_margin = 50.0
        approach_x = target_x - required_slope - transition_margin
        self.assertLess(approach_x, start_x)
        self.assertEqual(target_x - approach_x, 350.0)

    def test_approach_promotion_holds_outbound_and_promotes_inbound(self) -> None:
        self.assertFalse(approach_inbound_aligned(-20.0, 0.0, 100.0, 0.0, 0.1))
        self.assertFalse(approach_inbound_aligned(0.0, 20.0, 100.0, 0.0, 0.1))
        self.assertTrue(approach_inbound_aligned(20.0, 0.0, 100.0, 0.0, 0.1))
        self.assertFalse(approach_inbound_aligned(2.9, 0.0, 100.0, 0.0, 0.1))
        self.assertFalse(approach_inbound_aligned(20.0, 0.0, 100.0, 0.0, 2.1))
        self.assertTrue(
            approach_inbound_aligned(
                -20.0, 0.0, 100.0, 0.0, 10.0, fixed_wing=False
            )
        )
        for contract in (
            "MavlinkMMinimumApproachGroundspeedMps{3.f}",
            "MavlinkMApproachInboundAlignmentCosine{0.5f}",
            "MavlinkMApproachVelocityMaxAge{2_s}",
            "local_position->v_xy_valid",
            "get_vector_to_next_waypoint_fast(_fly_through_approach_lat",
            "local_position->vx * approach_to_target_north",
            "local_position->vy * approach_to_target_east",
            "alignment_cosine >= MavlinkMApproachInboundAlignmentCosine",
            "&& inbound_aligned",
        ):
            self.assertIn(contract, LOITER)

    def test_gate_loss_is_permanent_for_acceptance(self) -> None:
        policy = PolicyModel()
        policy.update(0.0, fly_through_complete=True, inside_radius=True)
        policy.update(
            1.0, gates_valid=False, fly_through_complete=True, inside_radius=True
        )
        policy.update(
            10.0, gates_valid=True, fly_through_complete=True, inside_radius=True
        )
        self.assertEqual(policy.phase, Phase.ABORTED)
        self.assertEqual(policy.completions, 0)

    def test_source_stale_during_transit_stops_navigation(self) -> None:
        policy = PolicyModel()
        policy.update(1.0, gates_valid=False)
        self.assertEqual(policy.phase, Phase.ABORTED)
        self.assertEqual(policy.navigation_cancels, 1)
        self.assertIn('*reason = "cue source stale";', HANDLER)
        self.assertIn("abort_intercept(safety_reason, true);", HANDLER)
        self.assertIn(
            "const bool navigation_active = stop_navigation\n"
            "\t\t\t\t       && _intercept_phase != InterceptPhase::None",
            HANDLER,
        )
        self.assertIn("if (navigation_active) {", HANDLER)
        self.assertIn("(void)cancel_assignment_commands(_active);", HANDLER)

    def test_explicit_task_abort_confirms_stop_before_durable_terminal_state(self) -> None:
        active_abort = re.search(
            r"if \(active_matches\) \{(?P<body>.*?)\n\t\} else \{", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(active_abort)
        body = active_abort.group("body")
        self.assertIn('abort_intercept("task aborted");', body)
        self.assertNotIn('abort_intercept("task aborted", true);', body)
        self.assertEqual(body.count("cancel_assignment_commands(_active)"), 1)
        self.assertLess(
            body.index("cancel_assignment_commands(_active)"),
            body.index("if (!save_state())"),
        )
        self.assertIn("abort blocked: navigation stop unconfirmed", body)
        self.assertIn("const InterceptTracking intercept_before = intercept_tracking();", body)
        self.assertIn("restore_intercept_tracking(intercept_before);", body)
        self.assertLess(
            body.index("if (!save_state())"),
            body.index('abort_intercept("task aborted");'),
        )

        expiry = re.search(
            r"void MavlinkMHandler::expire_assignments.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(expiry)
        expiry_body = expiry.group(0)
        self.assertIn("const InterceptTracking intercept_before = intercept_tracking();", expiry_body)
        self.assertIn("restore_intercept_tracking(intercept_before);", expiry_body)
        self.assertLess(
            expiry_body.index("if (!save_state())"),
            expiry_body.index('abort_intercept("task expired");'),
        )

    def test_blocked_movement_never_becomes_active_or_accepted(self) -> None:
        accept = re.search(
            r"void MavlinkMHandler::accept_pending.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(accept)
        body = accept.group(0)
        self.assertIn("assignment_requests_movement(candidate)", body)
        self.assertIn("movement_acceptance_ready(candidate, &movement_reason)", body)
        self.assertLess(
            body.index("movement_acceptance_ready(candidate, &movement_reason)"),
            body.index("_active = candidate;"),
        )
        self.assertIn("MAVLINK_M_ACK_RECEIVED", body)
        self.assertNotIn("send_ack(decided, MAVLINK_M_ACK_FAILED", body)
        self.assertIn("movement blocked: command failed; active retained", body)
        self.assertIn("_inbox[0] = inbox_before[0];", body)
        self.assertIn("_inbox[1] = inbox_before[1];", body)
        self.assertLess(
            body.index("command_active_assignment()"),
            body.index("Assignment acknowledged = _active;"),
        )

        movement = re.search(
            r"bool MavlinkMHandler::assignment_requests_movement.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(movement)
        self.assertIn("MAVLINK_M_CUE_TYPE_INVESTIGATE", movement.group(0))
        self.assertNotIn("MAVLINK_M_CUE_TYPE_OBSERVE", movement.group(0))
        self.assertNotIn("MAVLINK_M_CUE_TYPE_MARK", movement.group(0))

        readiness = re.search(
            r"bool MavlinkMHandler::movement_acceptance_ready.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(readiness)
        for gate in (
            "source_recent(assignment.source_system)",
            "vehicle_ready_for_reposition()",
            "vehicle_airborne_for_reposition()",
            "_global_position.timestamp",
            "_intercept_delta_z_m",
        ):
            self.assertIn(gate, readiness.group(0))

        self.assertIn('"instance_id: 735",\n                "state: 1"', SITL_RUNNER)
        self.assertIn('["instance_id: 754", "state: 2", "command_flags: 0"]', SITL_RUNNER)

    def test_every_recoverable_effect_rejection_has_stable_movement_prefix(self) -> None:
        accept = re.search(
            r"void MavlinkMHandler::accept_pending.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(accept)
        body = accept.group(0)
        for reason in (
            "movement blocked: invalid requested effect",
            "movement blocked: requested effect requires INVESTIGATE cue",
            "movement blocked: intercept requires finite cue altitude",
            "movement blocked: requested effect exceeds MAV_M_ACTION",
        ):
            self.assertIn(reason, body)

    def test_source_freshness_gates_level_and_intercept_movement(self) -> None:
        readiness = re.search(
            r"bool MavlinkMHandler::movement_acceptance_ready.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(readiness)
        body = readiness.group(0)
        self.assertIn("if (!source_recent(assignment.source_system))", body)
        source_gate = body.index("if (!source_recent(assignment.source_system))")
        vehicle_gate = body.index("if (!vehicle_ready_for_reposition()")
        self.assertLess(source_gate, vehicle_gate)
        source_recent = re.search(
            r"bool MavlinkMHandler::source_recent.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(source_recent)
        self.assertIn("_source_last_seen[source_system]", source_recent.group(0))
        self.assertIn(
            "_source_last_seen[message.sysid] = hrt_absolute_time();", HANDLER
        )
        self.assertRegex(
            HEADER,
            r"hrt_abstime\s+_source_last_seen\[UINT8_MAX \+ 1\]\s*\{\};",
        )
        self.assertIn("level_movement_requires_fresh_cue_source", SITL_RUNNER)
        self.assertIn("movement blocked: cue source is stale", SITL_RUNNER)
        stale_fixture = re.search(
            r"establish_udp_peer\(endpoint\)\s+"
            r"stale_source_cue = make_cue\(\s*endpoint,\s*758,.*?"
            r"endpoint\.send_frozen\(stale_source_cue\)",
            SITL_RUNNER,
            re.DOTALL,
        )
        self.assertIsNotNone(stale_fixture)

    def test_accepting_pending_explicitly_supersedes_active(self) -> None:
        accept = re.search(
            r"void MavlinkMHandler::accept_pending.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(accept)
        body = accept.group(0)
        for contract in (
            "const bool superseding = _active.state == AssignmentState::Active;",
            "const Assignment active_before = _active;",
            "const InterceptTracking intercept_before = intercept_tracking();",
            "remember_terminal(superseded, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);",
            "_active = candidate;",
            "command_result = command_active_assignment();",
            'send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "superseded by accepted cue");',
            'send_ack(acknowledged, MAVLINK_M_ACK_ACCEPTED, "accepted; previous cue superseded");',
        ):
            self.assertIn(contract, body)
        self.assertLess(
            body.index("movement_acceptance_ready(candidate, &movement_reason)"),
            body.index("remember_terminal(superseded"),
        )
        self.assertLess(body.index("remember_terminal(superseded"), body.index("_active = candidate;"))
        self.assertLess(body.index("if (!save_state())"), body.index("command_active_assignment()"))
        self.assertLess(
            body.index('send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED, "superseded by accepted cue");'),
            body.index('send_ack(acknowledged, MAVLINK_M_ACK_ACCEPTED, "accepted; previous cue superseded");'),
        )
        self.assertNotIn("another task is active", body)
        self.assertIn("accepted_cue_explicitly_supersedes_active", SITL_RUNNER)
        self.assertIn("moving_cue_supersession_replaces_navigation", SITL_RUNNER)
        self.assertIn("moving_to_nonmoving_supersession_stops_navigation", SITL_RUNNER)

    def test_command_persistence_failure_never_falsely_restores_old_navigation(self) -> None:
        accept = re.search(
            r"void MavlinkMHandler::accept_pending.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(accept)
        body = accept.group(0)
        for contract in (
            "movement_requested ? MAVLINK_M_ACK_RECEIVED : MAVLINK_M_ACK_ACCEPTED",
            "const Assignment staged_active = _active;",
            "const Assignment staged_inbox[2] {_inbox[0], _inbox[1]};",
            "_active = staged_active;",
            "_inbox[0] = staged_inbox[0];",
            "_inbox[1] = staged_inbox[1];",
            "restore_intercept_tracking(intercept_before);",
            "remember_terminal(superseded, AssignmentState::Aborted, MAVLINK_M_ACK_REJECTED);",
            "send_ack(_terminal[0], MAVLINK_M_ACK_REJECTED",
            "CommandApplicationResult::PersistenceFailedAfterPublicationStopped",
            "CommandApplicationResult::PersistenceFailedAfterPublicationStopUnconfirmed",
            "movement uncommitted: navigation stopped; abort",
            "movement uncommitted: stop unconfirmed; abort",
            "_deferred_navigation_stop = active_before;",
        ):
            self.assertIn(contract, body)
        self.assertLess(
            body.index("if (persistence_failed_after_publication)"),
            body.index("if (!execution_applied)"),
        )
        self.assertNotIn("send_ack(_active, MAVLINK_M_ACK_FAILED", body)

        command = re.search(
            r"MavlinkMHandler::CommandApplicationResult "
            r"MavlinkMHandler::command_active_assignment.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(command)
        command_body = command.group(0)
        self.assertIn("if (!save_state())", command_body)
        self.assertIn("const bool navigation_stopped = cancel_assignment_commands(_active);", command_body)
        self.assertIn("if (navigation_stopped)", command_body)
        self.assertIn("PersistenceFailedAfterPublicationStopped", command_body)
        self.assertIn("PersistenceFailedAfterPublicationStopUnconfirmed", command_body)
        self.assertIn("Assignment *stop_marker = find_navigation_stop_marker();", command_body)
        self.assertIn("~CommandStopPending", command_body)
        self.assertLess(
            command_body.index("~CommandStopPending"),
            command_body.index("if (!save_state())"),
        )

        load_state = re.search(
            r"bool MavlinkMHandler::load_state\(\).*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(load_state)
        self.assertNotIn("if (command_active_assignment())", load_state.group(0))
        self.assertNotIn("(void)command_active_assignment()", load_state.group(0))
        for recovery_contract in (
            "if (Assignment *stop_marker = find_navigation_stop_marker())",
            "_deferred_navigation_stop = *stop_marker;",
            "_active.last_ack_result == MAVLINK_M_ACK_RECEIVED",
            "_active.command_flags == 0",
            "assignment_requests_movement(_active)",
            "_active.state = AssignmentState::Pending;",
            "_active.last_ack_result = MAVLINK_M_ACK_RECEIVED;",
            "*slot = _active;",
            "_active = Assignment{};",
            "recovered_uncommitted_movement",
        ):
            self.assertIn(recovery_contract, load_state.group(0))

        self.assertIn("CommandStopPending = 1 << 2", HEADER)
        marker = re.search(
            r"MavlinkMHandler::Assignment \*MavlinkMHandler::find_navigation_stop_marker.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(marker)
        self.assertIn("for (Assignment &assignment : _terminal)", marker.group(0))
        self.assertIn("assignment.state == AssignmentState::Aborted", marker.group(0))
        self.assertIn("assignment.state == AssignmentState::Expired", marker.group(0))
        self.assertIn("assignment.command_flags & CommandStopPending", marker.group(0))

        stop = re.search(
            r"bool MavlinkMHandler::stop_deferred_navigation.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(stop)
        self.assertIn("marker->command_flags", stop.group(0))
        self.assertIn("if (!save_state())", stop.group(0))
        self.assertIn("marker->command_flags = command_flags_before;", stop.group(0))
        self.assertIn("const bool stopped_uncommitted_active", stop.group(0))
        self.assertIn("~CommandExecutionAll", stop.group(0))
        self.assertIn("clear_intercept_tracking();", stop.group(0))
        self.assertLess(
            stop.group(0).index("cancel_assignment_commands"),
            stop.group(0).index("~CommandStopPending"),
        )

        self.assertIn("acceptance blocked: previous navigation stop pending", body)

        expire = re.search(
            r"void MavlinkMHandler::expire_assignments.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(expire)
        self.assertIn("expired.command_flags |= CommandStopPending;", expire.group(0))
        self.assertIn("_deferred_navigation_stop = *stop_marker;", expire.group(0))

    def test_persistence_restore_avoids_redundant_large_stack_copies(self) -> None:
        state_crc = re.search(
            r"uint32_t MavlinkMHandler::state_crc.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(state_crc)
        self.assertNotIn("PersistedState copy", state_crc.group(0))
        for contract in (
            "offsetof(PersistedState, crc)",
            "const uint32_t zero_crc = 0;",
            "crc32_signature(crc, sizeof(zero_crc)",
            "sizeof(state) - suffix_offset",
        ):
            self.assertIn(contract, state_crc.group(0))

        load_state = re.search(
            r"bool MavlinkMHandler::load_state\(\).*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(load_state)
        self.assertNotIn("Assignment stale_navigation{};", load_state.group(0))
        self.assertNotIn("Assignment retry = _active;", load_state.group(0))

    def test_endpoint_selector_change_stops_navigation_and_invalidates_state(self) -> None:
        update_parameters = re.search(
            r"void MavlinkMHandler::update_parameters.*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(update_parameters)
        body = update_parameters.group(0)
        for contract in (
            "const bool endpoint_identity_changed",
            "deferred_navigation_stopped_before_parameter_read",
            "assignment_cancellation_ready(_active)",
            "cancel_assignment_commands(_active)",
            "invalidate_persisted_state()",
            "param_set(_param_instance, &_instance)",
            "param_set(_param_source_system, &_source_system)",
            "param_set(_param_source_component, &_source_component)",
            "_active = Assignment{};",
            "memset(_source_last_seen, 0, sizeof(_source_last_seen));",
        ):
            self.assertIn(contract, body)
        self.assertLess(
            body.index("stop_deferred_navigation()"),
            body.index("get_parameter(_param_instance, _instance)"),
        )
        self.assertLess(
            body.index("cancel_assignment_commands(_active)"),
            body.index("_active = Assignment{};"),
        )
        self.assertLess(
            body.index("invalidate_persisted_state()"),
            body.index("_active = Assignment{};"),
        )

        load_state = re.search(
            r"bool MavlinkMHandler::load_state\(\).*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(load_state)
        mismatch = load_state.group(0).index(
            'PX4_WARN("ignoring MAVLink-M state for different endpoint/profile")'
        )
        mismatch_body = load_state.group(0)[mismatch:]
        for contract in (
            "Assignment *stale_navigation = nullptr;",
            "state.active.command_flags & CommandNav",
            "state.active.command_flags |= CommandStopPending;",
            "stale_navigation = &state.active;",
            "_deferred_navigation_stop = *stale_navigation;",
            "remember_terminal(*stale_navigation",
            "stale MAVLink-M navigation quarantine was not persisted",
        ):
            self.assertIn(contract, mismatch_body)
        self.assertGreater(
            load_state.group(0).index("invalidate_persisted_state()", mismatch),
            mismatch,
        )
        self.assertIn("stop_deferred_navigation();", HANDLER)

    def test_system_wildcards_preserve_route_component_and_signing_boundaries(self) -> None:
        for contract in (
            "selector == -1 || (selector >= 1 && selector <= UINT8_MAX)",
            "system_id != 0 && (selector == -1 || selector == system_id)",
            "persisted_system_selector(-1) == 0",
            "system_selector_matches(_source_system, message.sysid)",
            "message.compid == static_cast<uint8_t>(_source_component)",
            "system_selector_matches(_control_system, message.sysid)",
            "message.compid == static_cast<uint8_t>(_control_component)",
            "_mavlink->get_instance_id() == _instance",
            "_mavlink->get_instance_id() == _control_instance",
            "(!signing_required() || signing_active())",
            "bool MavlinkMHandler::ingress_locked() const",
            "assignment instance ambiguous across sources",
        ):
            self.assertIn(contract, HANDLER)

        def parameter_block(name: str) -> str:
            match = re.search(
                rf"/\*\*(?P<body>(?:(?!/\*\*).)*?)\*/\s*"
                rf"PARAM_DEFINE_INT32\({name},",
                PARAMETERS,
                re.DOTALL,
            )
            self.assertIsNotNone(match, name)
            return match.group("body")

        source = parameter_block("MAV_M_SRC_SYS")
        self.assertIn("@min -1", source)
        self.assertIn("Any nonzero system on MAV_M_INST", source)
        self.assertIn("exact MAV_M_SRC_CMP check", source)

        control_instance = parameter_block("MAV_M_CTL_INST")
        self.assertIn("MAV_M_CTL_SYS=-1", control_instance)
        self.assertIn("only on this selected instance", control_instance)

        control_system = parameter_block("MAV_M_CTL_SYS")
        self.assertIn("@min -1", control_system)
        self.assertIn("Any nonzero system on MAV_M_CTL_INST", control_system)
        self.assertIn("MAV_M_CTL_CMP remains an exact check", control_system)

        control_component = parameter_block("MAV_M_CTL_CMP")
        self.assertIn("always applies", control_component)
        self.assertIn("MAV_M_CTL_INST", control_component)

        control_link = parameter_block("MAV_M_CTL_LNK")
        self.assertRegex(control_link, r"does not weaken\s+\* signing")
        self.assertIn("other MAVLink instances", control_link)

        self.assertIn("system_wildcards_preserve_component_and_route", SITL_RUNNER)
        self.assertIn('px4.command("param set MAV_M_SRC_SYS -1")', SITL_RUNNER)
        self.assertIn('px4.command("param set MAV_M_CTL_SYS -1")', SITL_RUNNER)
        self.assertIn('px4.command("param set MAV_M_ACTION 1")', SITL_RUNNER)
        self.assertIn(
            'f"source_system: {WILDCARD_SOURCE_SYSTEM}"', SITL_RUNNER
        )
        self.assertIn(
            'rf"\\bsource_system:\\s*{WILDCARD_SOURCE_SYSTEM}\\b"',
            SITL_RUNNER,
        )

    def test_fixed_wing_radius_includes_loiter_margin(self) -> None:
        self.assertEqual(effective_radius(25.0, False, 80.0), 25.0)
        self.assertEqual(effective_radius(25.0, True, 80.0), 90.0)
        self.assertEqual(effective_radius(120.0, True, 80.0), 120.0)

    def test_nan_altitude_remains_one_phase_in_source(self) -> None:
        self.assertIn(
            "guarded_intercept = _active.execution_effect == ActionInterceptCueAltitude\n"
            "\t\t\t\t       && PX4_ISFINITE(_active.alt);",
            HANDLER,
        )
        self.assertIn("const float acceptance_alt = _global_position.alt;", HANDLER)
        self.assertIn(
            "const float target_alt = guarded_intercept ? _active.alt : acceptance_alt;",
            HANDLER,
        )
        self.assertIn(
            "begin_intercept_tracking(acceptance_alt, target_alt, intercept_token);",
            HANDLER,
        )

    def test_intercept_uses_internal_exact_fly_through_only(self) -> None:
        self.assertIn(
            "VEHICLE_CMD_PX4_MAVLINK_M_FLY_THROUGH = 100001", VEHICLE_COMMAND
        )
        self.assertIn(
            "VEHICLE_CMD_PX4_MAVLINK_M_FLY_THROUGH > UINT16_MAX", HANDLER
        )
        self.assertIn(
            "? publish_internal_fly_through(_active, target_alt, acceptance_alt",
            HANDLER,
        )
        self.assertIn(
            ": publish_vehicle_command(_active, vehicle_command_s::VEHICLE_CMD_DO_REPOSITION",
            HANDLER,
        )
        self.assertIn(
            "cmd.command == vehicle_command_s::VEHICLE_CMD_PX4_MAVLINK_M_FLY_THROUGH",
            NAVIGATOR,
        )
        self.assertIn(
            "rep->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION",
            NAVIGATOR,
        )
        self.assertIn(
            "rep->next.type = approach_required", NAVIGATOR
        )
        self.assertIn("rep->next.mavlink_m_exact_altitude = approach_required;", NAVIGATOR)
        self.assertIn("previous_along < 0.f && current_along >= 0.f", LOITER)
        self.assertIn("const float alpha", LOITER)
        self.assertIn("horizontal_miss <= Navigator::MavlinkMHitRadiusM", LOITER)
        self.assertIn("complete_fly_through(target_hit);", LOITER)
        self.assertIn(
            "VEHICLE_CMD_PX4_MAVLINK_M_FLY_THROUGH:", COMMANDER
        )

    def test_completion_requires_observed_fly_through_promotion(self) -> None:
        self.assertIn("intercept_transit_setpoint_matches", HANDLER)
        self.assertIn("intercept_recovery_setpoint_matches", HANDLER)
        self.assertIn("intercept_loiter_setpoint_matches", HANDLER)
        self.assertIn(
            "recovery_loiter_owned && _intercept_setpoint_seen", HANDLER
        )
        self.assertIn(
            "token-matched completion ACK", HANDLER
        )
        self.assertIn("_intercept_navigator_completed", HANDLER)
        self.assertIn("ack.result_param2", HANDLER)
        self.assertIn(
            "fly_through_complete && inside_radius", HANDLER
        )
        self.assertIn("_intercept_phase = InterceptPhase::Complete;", HANDLER)

    def test_dwell_radius_is_not_target_hit_radius(self) -> None:
        internal_publish = re.search(
            r"bool MavlinkMHandler::publish_internal_fly_through.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(internal_publish)
        self.assertNotIn("_intercept_radius_m", internal_publish.group(0))
        self.assertIn("token & UINT16_MAX", internal_publish.group(0))
        navigator_header = (ROOT / "src/modules/navigator/navigator.h").read_text()
        self.assertIn("static constexpr float MavlinkMHitRadiusM{5.f};", navigator_header)
        self.assertIn("? math::max(get_acceptance_radius(), MavlinkMHitRadiusM)", NAVIGATOR)
        self.assertIn("intercept_arrival_radius()", HANDLER)

    def test_private_triplet_and_completion_are_token_bound(self) -> None:
        self.assertIn("Token low 16 bits", VEHICLE_COMMAND)
        self.assertIn("Token high 16 bits", VEHICLE_COMMAND)
        self.assertIn("exact_token_half(cmd.param1", NAVIGATOR)
        self.assertIn("exact_token_half(cmd.param2", NAVIGATOR)
        self.assertIn("rep->timestamp == _fly_through_pending_command_timestamp", LOITER)
        self.assertIn("VEHICLE_CMD_RESULT_IN_PROGRESS", NAVIGATOR)
        self.assertIn("VEHICLE_CMD_RESULT_ACCEPTED", LOITER)
        self.assertIn("VEHICLE_CMD_RESULT_FAILED", LOITER)
        self.assertIn("VEHICLE_CMD_RESULT_CANCELLED", LOITER)
        self.assertIn("recovery_loiter_owned && _intercept_setpoint_seen", HANDLER)
        self.assertIn("if (_intercept_navigator_completed)", HANDLER)
        self.assertIn(
            "command_ack.command < vehicle_command_s::VEHICLE_CMD_PX4_INTERNAL_START",
            MAVLINK_MAIN,
        )

    def test_bounded_miss_retains_target_loiter_but_blocks_completion(self) -> None:
        self.assertIn("target_hit ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED", LOITER)
        self.assertIn("target_hit ? 100 : 1", LOITER)
        self.assertIn("if (ack.result_param1 == 1)", HANDLER)
        self.assertIn("_intercept_navigator_missed = true;", HANDLER)
        miss_gate = re.search(
            r"if \(_intercept_navigator_missed\) \{(?P<body>.*?)\n\t\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(miss_gate)
        self.assertIn("acceptance-altitude target loiter retained", miss_gate.group("body"))
        self.assertIn("recovery_loiter_owned && recovery_altitude_reached", miss_gate.group("body"))

    def test_navigator_rechecks_state_age_and_geofence_fail_closed(self) -> None:
        for contract in (
            "MavlinkMInternalCommandMaxAge",
            "now - cmd.timestamp <= MavlinkMInternalCommandMaxAge",
            "_vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER",
            "!_vstatus.failsafe",
            "failure_detector_status == vehicle_status_s::FAILURE_NONE",
            "!_land_detected.freefall",
            "!_land_detected.ground_contact",
            "!_land_detected.maybe_landed",
            "!_land_detected.landed",
            "!_global_pos.dead_reckoning",
            "if (restrictive_geofence)",
            "return false;",
        ):
            self.assertIn(contract, NAVIGATOR)
        fly_through_policy = re.search(
            r"bool Navigator::mavlink_m_fly_through_allowed.*?\n\}",
            NAVIGATOR,
            re.DOTALL,
        )
        self.assertIsNotNone(fly_through_policy)
        self.assertNotIn("geofence_allows_position", fly_through_policy.group(0))
        self.assertNotIn("isInsidePolygonOrCircle", fly_through_policy.group(0))

    def test_default_geofence_action_only_blocks_when_a_fence_is_configured(self) -> None:
        self.assertFalse(restrictive_geofence_blocks_intercept(2, 0, 0.0, 0.0))
        self.assertTrue(restrictive_geofence_blocks_intercept(2, 1, 0.0, 0.0))
        self.assertTrue(restrictive_geofence_blocks_intercept(2, 0, 1000.0, 0.0))
        self.assertTrue(restrictive_geofence_blocks_intercept(2, 0, 0.0, 1000.0))
        self.assertFalse(restrictive_geofence_blocks_intercept(1, 1, 1000.0, 1000.0))
        for contract in (
            "const bool geofence_configured = !_geofence.isEmpty()",
            "_geofence.getMaxHorDistanceHome() > FLT_EPSILON",
            "_geofence.getMaxVerDistanceHome() > FLT_EPSILON",
            "const bool restrictive_geofence = geofence_configured",
            "MAVLink-M intercept blocked by configured restrictive geofence",
        ):
            self.assertIn(contract, NAVIGATOR)

    def test_loiter_rechecks_target_and_active_approach_endpoint(self) -> None:
        endpoint_policy = re.search(
            r"bool\nLoiter::fly_through_endpoints_allowed\(\).*?\n\}",
            LOITER,
            re.DOTALL,
        )
        self.assertIsNotNone(endpoint_policy)
        body = endpoint_policy.group(0)
        self.assertEqual(body.count("mavlink_m_fly_through_allowed"), 4)
        self.assertIn("_fly_through_target_lat", body)
        self.assertIn("_fly_through_approach_active", body)
        self.assertIn("_fly_through_approach_lat", body)
        self.assertIn("_fly_through_recovery_lat", body)
        self.assertIn("_fly_through_recovery_alt", body)
        self.assertIn("_fly_through_minimum_clearance", body)
        self.assertGreaterEqual(LOITER.count("fly_through_endpoints_allowed()"), 3)

    def test_fixed_wing_approach_uses_airframe_limits_and_safe_entry(self) -> None:
        for contract in (
            "get_fw_pitch_limit_min()",
            "get_fw_pitch_limit_max()",
            "get_fw_max_sink_rate()",
            "get_fw_max_climb_rate()",
            "get_fw_max_airspeed()",
            "mavlink_m_fixed_wing_ground_speed_bounds",
            "2.f * maximum_airspeed",
            "hypotf(wind.windspeed_north, wind.windspeed_east)",
            "3.f * wind_sigma",
            "wind.variance_north < 0.f",
            "now - wind.timestamp > MavlinkMStateMaxAge",
            "now - wind.timestamp_sample > MavlinkMStateMaxAge",
            "const float pitch_ground_gradient",
            "vertical_rate_limit / ground_speed_bound",
            "const float usable_gradient = math::min(pitch_ground_gradient, rate_gradient)",
            "*distance_m <= MavlinkMMaximumApproachDistanceM",
            "required_slope_distance_m + transition_margin_m",
            "create_waypoint_from_line_and_dist(cmd.param5, cmd.param6",
            "get_local_position()->heading + M_PI_F",
            "mavlink_m_fly_through_allowed(approach_lat, approach_lon",
        ):
            self.assertIn(contract, NAVIGATOR)
        self.assertIn("promote_fly_through_approach();", LOITER)
        self.assertIn("next.type == position_setpoint_s::SETPOINT_TYPE_POSITION", LOITER)
        self.assertIn("SubscriptionData<wind_s>", NAVIGATOR_HEADER)
        self.assertIn("_wind_sub.update();", NAVIGATOR)

    def test_exact_altitude_path_does_not_finish_early(self) -> None:
        self.assertIn("bool mavlink_m_exact_altitude", POSITION_SETPOINT)
        self.assertIn("!pos_sp_curr.mavlink_m_exact_altitude", FW_CONTROL)
        self.assertRegex(
            FW_CONTROL,
            r"pos_sp_curr\.mavlink_m_exact_altitude\s*\?\s*0\.f",
        )
        self.assertIn("? _param_fw_t_sink_max.get()", FW_CONTROL)
        self.assertIn("? _param_fw_t_clmb_max.get()", FW_CONTROL)

    def test_same_coordinate_waits_for_vertical_arrival(self) -> None:
        degenerate_leg = re.search(
            r"if \(inbound_distance <= 1\.f\) \{(?P<body>.*?)\n\t\}",
            LOITER,
            re.DOTALL,
        )
        self.assertIsNotNone(degenerate_leg)
        self.assertIn("if (target_hit)", degenerate_leg.group("body"))
        self.assertIn("complete_fly_through(true);", degenerate_leg.group("body"))
        self.assertNotIn("complete_fly_through(target_hit);", degenerate_leg.group("body"))

    def test_failed_crossing_recovers_to_acceptance_altitude(self) -> None:
        completion = re.search(
            r"Loiter::complete_fly_through\(bool target_hit\)"
            r"(?P<body>.*?)\n\}",
            LOITER,
            re.DOTALL,
        )
        self.assertIsNotNone(completion)
        body = completion.group("body")
        self.assertIn("triplet->current.alt = _fly_through_recovery_alt;", body)
        self.assertIn("triplet->next = position_setpoint_s{};", body)
        self.assertIn("fixed-wing path smoothing cannot", body)
        self.assertIn("_fly_through_recovery_active = true;", body)
        self.assertIn("triplet->current.mavlink_m_exact_altitude = false;", body)
        self.assertIn("complete_fly_through(target_hit);", LOITER)

    def test_intercept_has_no_second_reposition_command(self) -> None:
        update = re.search(
            r"void MavlinkMHandler::update_intercept\(\).*?\n\}", HANDLER, re.DOTALL
        )
        self.assertIsNotNone(update)
        self.assertNotIn("VEHICLE_CMD_DO_REPOSITION", update.group(0))
        self.assertIn("InterceptPhase::Complete", update.group(0))

    def test_cpp_contains_every_permanent_abort_gate(self) -> None:
        required = (
            "task aborted",
            "task expired",
            "vehicle left armed airborne Hold",
            "cue source stale",
            "navigation setpoint overridden",
            "navigation policy changed",
            "restart requires fresh acceptance",
            "cue altitude exceeds MAV_M_INT_DZ",
            "MAV_M_INT_CLR requires fresh terrain and HAGL",
            "MAV_M_INT_CLR terrain clearance breached",
        )
        for gate in required:
            self.assertIn(gate, HANDLER)
        self.assertIn("_active.command_flags |= CommandInterceptAltitude;", HANDLER)
        self.assertIn("now - _intercept_dwell_started >= required_dwell", HANDLER)

    def test_parameter_and_status_contract(self) -> None:
        expected = {
            "MAV_M_INT_RAD": ("25.0f", "1", "500"),
            "MAV_M_INT_DWL": ("3.0f", "0", "60"),
            "MAV_M_INT_DZ": ("100.0f", "0", "1000"),
            "MAV_M_INT_CLR": ("30.0f", "-1", "1000"),
        }
        for name, (default, minimum, maximum) in expected.items():
            block = re.search(
                rf"/\*\*.*?@min {minimum}.*?@max {maximum}.*?"
                rf"PARAM_DEFINE_FLOAT\({name}, {re.escape(default)}\);",
                PARAMETERS,
                re.DOTALL,
            )
            self.assertIsNotNone(block, name)
        self.assertIn("uint8 INTERCEPT_PHASE_TRANSIT = 1", STATUS)
        self.assertIn("uint8 INTERCEPT_PHASE_COMPLETE = 3", STATUS)
        self.assertIn("uint8 INTERCEPT_PHASE_ABORTED = 4", STATUS)
        self.assertIn('send_int("AAGS_IPHS"', HANDLER)
        self.assertIn('"intercept_phase"', SITL_RUNNER)
        self.assertIn("position_setpoint_triplet", HEADER)
        self.assertIn('"param set MAV_M_INT_CLR -1"', SITL_RUNNER)
        self.assertIn('"MAV_M_INT_CLR": -1', SITL_RUNNER)
        self.assertIn(
            "intercept_terrain_gate_requires_fresh_decision_after_override",
            SITL_RUNNER,
        )

    def test_owner_decision_binds_per_cue_effect_under_permission_ceiling(self) -> None:
        self.assertIn("uint8 EFFECT_DEFAULT = 0", DECISION)
        self.assertIn("uint8 EFFECT_LEVEL_TRAVEL = 1", DECISION)
        self.assertIn("uint8 EFFECT_INTERCEPT = 2", DECISION)
        self.assertIn("exact_unsigned_word(command.param5, &requested_effect)", HANDLER)
        self.assertIn("requested_effect > sanitize_action(_action_mode)", HANDLER)
        self.assertIn("candidate.execution_effect = execution_effect;", HANDLER)
        self.assertIn(
            "_active.execution_effect > sanitize_action(_action_mode)", HANDLER
        )
        self.assertIn(
            "accept_pending(decision.task_msgid, decision.task_instance, "
            "decision.requested_effect);",
            HANDLER,
        )
        self.assertIn("PreviousPersistenceVersion = 6", HEADER)
        self.assertIn("sizeof(Assignment) == 152", HEADER)
        self.assertIn("migrated_previous_version", HANDLER)
        self.assertIn("requested_effect: int = 0", SITL_RUNNER)

    def test_explicit_intercept_requires_finite_exact_cue_altitude(self) -> None:
        self.assertIn(
            "requested_effect == mavlink_m_task_decision_s::EFFECT_INTERCEPT\n"
            "\t\t    && !PX4_ISFINITE(_control_status.alt_msl_m)",
            HANDLER,
        )
        self.assertIn(
            "requested_effect == mavlink_m_task_decision_s::EFFECT_INTERCEPT\n"
            "\t    && !PX4_ISFINITE(decided.alt)",
            HANDLER,
        )
        self.assertIn("intercept requires finite cue altitude", HANDLER)
        self.assertIn(
            "requested_effect == mavlink_m_task_decision_s::EFFECT_DEFAULT\n"
            "\t\t? permission_ceiling : requested_effect",
            HANDLER,
        )

    def test_live_owner_effect_and_legacy_acceptance_paths_are_covered(self) -> None:
        for result_name in (
            "owner_explicit_intercept_requires_altitude",
            "owner_explicit_level_under_intercept_ceiling",
            "owner_explicit_intercept_guarded_approach",
        ):
            self.assertIn(result_name, SITL_RUNNER)
        self.assertIn(
            "owner_effect_endpoint, 756, True, requested_effect=1",
            SITL_RUNNER,
        )
        self.assertIn(
            "owner_effect_endpoint, 757, True, requested_effect=2",
            SITL_RUNNER,
        )
        self.assertIn(
            "owner_effect_endpoint, 755, True, requested_effect=2",
            SITL_RUNNER,
        )
        self.assertIn('"action_mode: 1"', SITL_RUNNER)
        self.assertIn('"intercept_phase: 3"', SITL_RUNNER)
        self.assertIn('px4.command("mavlink task accept 736 53001")', SITL_RUNNER)
        self.assertIn('px4.command("mavlink task accept 746 53001")', SITL_RUNNER)
        self.assertIn('px4.command("mavlink task accept 747 53001")', SITL_RUNNER)

    def test_bench_endpoint_self_registers_before_direct_fleet_tasks(self) -> None:
        advertise = ENDPOINT_TOOL[
            ENDPOINT_TOOL.index("def advertise_gcs") :
            ENDPOINT_TOOL.index("def print_message")
        ]
        for contract in (
            "MAV_TYPE_GCS",
            "MAV_AUTOPILOT_INVALID",
            "time.sleep(registration_delay)",
        ):
            self.assertIn(contract, advertise)
        dispatch_start = ENDPOINT_TOOL.index(
            'if args.command in ("cue", "track", "handover")'
        )
        dispatch = ENDPOINT_TOOL[
            dispatch_start : ENDPOINT_TOOL.index(
                'if args.command == "cue"', dispatch_start
            )
        ]
        self.assertIn(
            "advertise_gcs(endpoint, args.peer_registration_delay)",
            dispatch,
        )
        self.assertIn("advertise_gcs(endpoint, 0.0)", SITL_RUNNER)

    def test_sitl_data_tree_precedes_px4_options(self) -> None:
        self.assertRegex(
            SITL_RUNNER,
            r"str\(self\.binary\),\s*\"-i\"[\s\S]*?str\(self\.etc\)",
        )
        if DUAL_RUNNER:
            self.assertRegex(
                DUAL_RUNNER,
                r'"\$command"\s+"\$PX4"\s+"\$BUILD/etc"\s+"\$instance"',
            )

    def test_esad_output_selection_never_uses_stale_parameter_cache(self) -> None:
        self.assertIn(
            "get_esad_arming_forwarding_instance() const;",
            MAVLINK_HEADER,
        )
        getter = re.search(
            r"int\s+Mavlink::get_esad_arming_forwarding_instance\(\) const"
            r"(?P<body>.*?)\n\}",
            MAVLINK_MAIN,
            re.DOTALL,
        )
        self.assertIsNotNone(getter)
        body = getter.group("body")
        self.assertIn("int32_t selected_instance = -2;", body)
        self.assertIn(
            "param_get(_param_mav_m_esad_i.handle(), &selected_instance)",
            body,
        )
        self.assertIn("return selected_instance;", body)
        self.assertNotIn("_param_mav_m_esad_i.get()", body)
        self.assertIn('"self-output ESAD forwarding"', SITL_RUNNER)

    def test_terrain_clearance_model_fails_closed_and_override_is_exact(self) -> None:
        safe = dict(
            terrain_fresh=True,
            hagl_fresh=True,
            aircraft_alt_msl_m=500.0,
            terrain_alt_msl_m=450.0,
            hagl_m=50.0,
            candidate_alt_msl_m=485.0,
        )
        self.assertTrue(terrain_clearance_allowed(30.0, **safe))
        self.assertFalse(
            terrain_clearance_allowed(30.0, **{**safe, "terrain_fresh": False})
        )
        self.assertFalse(
            terrain_clearance_allowed(30.0, **{**safe, "hagl_fresh": False})
        )
        self.assertFalse(
            terrain_clearance_allowed(
                30.0, **{**safe, "aircraft_alt_msl_m": 479.9}
            )
        )
        self.assertFalse(
            terrain_clearance_allowed(30.0, **{**safe, "hagl_m": 29.9})
        )
        self.assertFalse(
            terrain_clearance_allowed(
                30.0, **{**safe, "candidate_alt_msl_m": 479.9}
            )
        )
        self.assertTrue(
            terrain_clearance_allowed(
                -1.0,
                terrain_fresh=False,
                hagl_fresh=False,
                aircraft_alt_msl_m=math.nan,
                terrain_alt_msl_m=math.nan,
                hagl_m=math.nan,
                candidate_alt_msl_m=math.nan,
            )
        )
        self.assertFalse(terrain_clearance_allowed(-0.5, **safe))

    def test_handler_requires_fresh_terrain_hagl_and_candidate_clearance(self) -> None:
        clearance = re.search(
            r"bool MavlinkMHandler::intercept_terrain_clearance_valid.*?\n\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(clearance)
        body = clearance.group(0)
        for contract in (
            "_intercept_clearance_m < 0.f",
            "_global_position.terrain_alt_valid",
            "_local_position.dist_bottom_valid",
            "now - _global_position.timestamp < 2'000'000",
            "now - _local_position.timestamp < 2'000'000",
            "_global_position.alt - _global_position.terrain_alt",
            "candidate_altitude_m - _global_position.terrain_alt",
            "_local_position.dist_bottom < _intercept_clearance_m",
            "MAV_M_INT_CLR requires fresh terrain and HAGL",
            "MAV_M_INT_CLR terrain clearance breached",
        ):
            self.assertIn(contract, body)
        self.assertIn(
            "intercept_terrain_clearance_valid(assignment.alt, reason)", HANDLER
        )
        self.assertIn(
            "intercept_terrain_clearance_valid(_active.alt, reason)", HANDLER
        )
        self.assertIn(
            "intercept_terrain_clearance_valid(_intercept_acceptance_altitude_m, reason)",
            HANDLER,
        )
        self.assertIn("const bool recovery_navigation_owned", HANDLER)
        self.assertIn("!recovery_navigation_owned", HANDLER)
        self.assertIn("ORB_ID(vehicle_local_position)", HEADER)

    def test_navigator_rechecks_actual_and_candidate_clearance(self) -> None:
        policy = re.search(
            r"bool Navigator::mavlink_m_fly_through_allowed.*?\n\}",
            NAVIGATOR,
            re.DOTALL,
        )
        self.assertIsNotNone(policy)
        body = policy.group(0)
        for contract in (
            "minimum_clearance_m >= 0.f",
            "_global_pos.terrain_alt_valid",
            "_local_pos.dist_bottom_valid",
            "now - _local_pos.timestamp <= MavlinkMStateMaxAge",
            "_global_pos.alt - _global_pos.terrain_alt",
            "alt - _global_pos.terrain_alt",
            "_local_pos.dist_bottom < minimum_clearance_m",
        ):
            self.assertIn(contract, body)
        endpoint_policy = re.search(
            r"bool\nLoiter::fly_through_endpoints_allowed\(\).*?\n\}",
            LOITER,
            re.DOTALL,
        )
        self.assertIsNotNone(endpoint_policy)
        self.assertIn("_fly_through_minimum_clearance", endpoint_policy.group(0))
        self.assertIn("_fly_through_recovery_lat", endpoint_policy.group(0))

    def test_surveyed_corridor_override_is_explicit_and_still_local(self) -> None:
        self.assertIn("sanitize_intercept_clearance(-1.f)", HANDLER)
        self.assertIn(
            "MAV_M_INT_CLR=-1: externally surveyed corridor override accepted locally "
            "for this effect-2 cue",
            HANDLER,
        )
        self.assertIn(
            "exact Intercept using externally surveyed MAV_M_INT_CLR=-1 corridor",
            NAVIGATOR,
        )
        self.assertIn(
            "requested_effect == mavlink_m_task_decision_s::EFFECT_INTERCEPT",
            HANDLER,
        )
        parameter = re.search(
            r"/\*\*(?P<body>(?:(?!/\*\*).)*?)\*/\s*"
            r"PARAM_DEFINE_FLOAT\(MAV_M_INT_CLR,",
            PARAMETERS,
            re.DOTALL,
        )
        self.assertIsNotNone(parameter)
        self.assertIn("Every", parameter.group("body"))
        self.assertIn("local effect-2 acceptance", parameter.group("body"))

    def test_hit_and_miss_recover_before_target_centered_loiter(self) -> None:
        for contract in (
            "required_recovery_distance_m",
            "-recovery_distance_from_target_m",
            "recovery_lat, recovery_lon, recovery_required",
        ):
            self.assertIn(contract, NAVIGATOR)
        self.assertRegex(
            NAVIGATOR,
            r"mavlink_m_fixed_wing_approach_distance\(\s*"
            r"cmd\.param7, cmd\.param3,",
        )
        for contract in (
            "_fly_through_target_hit = target_hit;",
            "_fly_through_recovery_active = true;",
            "promote_fly_through_recovery();",
            "triplet->current.lat = _fly_through_target_lat;",
            "triplet->current.alt = _fly_through_recovery_alt;",
            "triplet->next = position_setpoint_s{};",
            "target_hit ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED",
            "target_hit ? 100 : 1",
        ):
            self.assertIn(contract, LOITER)

    def test_completion_waits_for_recovery_loiter_and_actual_altitude(self) -> None:
        self.assertIn(
            "const bool recovery_loiter_owned = "
            "intercept_loiter_setpoint_matches(_intercept_acceptance_altitude_m);",
            HANDLER,
        )
        self.assertIn("RecoveryAltitudeToleranceM = 5.f", HANDLER)
        self.assertIn(
            "fabsf(_global_position.alt - _intercept_acceptance_altitude_m)",
            HANDLER,
        )
        self.assertIn(
            "const bool recovery_dwell_eligible = inside_radius && recovery_altitude_reached;",
            HANDLER,
        )
        completion = re.search(
            r"if \(!intercept_completion_allowed.*?\n\t\}",
            HANDLER,
            re.DOTALL,
        )
        self.assertIsNotNone(completion)
        self.assertIn("recovery_altitude_within_tolerance", completion.group(0))
        self.assertNotIn(
            "intercept_loiter_setpoint_matches(_intercept_expected_altitude_m)",
            completion.group(0),
        )

    def test_vertical_delta_model_is_bounded(self) -> None:
        acceptance_alt = 500.0
        cue_alt = 575.0
        self.assertTrue(math.isfinite(cue_alt))
        self.assertLessEqual(abs(cue_alt - acceptance_alt), 100.0)
        self.assertGreater(abs(650.1 - acceptance_alt), 100.0)

    def test_timesync_response_is_initialized_and_addressed_to_requester(self) -> None:
        response = re.search(
            r"if \(tsync\.tc1 == 0\).*?"
            r"mavlink_msg_timesync_send_struct\(_mavlink->get_channel\(\), &rsync\);",
            MAVLINK_TIMESYNC,
            re.DOTALL,
        )
        self.assertIsNotNone(response)
        body = response.group(0)
        self.assertIn("mavlink_timesync_t rsync{};", body)
        self.assertIn("rsync.target_system = msg->sysid;", body)
        self.assertIn("rsync.target_component = msg->compid;", body)
        self.assertLess(
            body.index("rsync.target_system = msg->sysid;"),
            body.index("mavlink_msg_timesync_send_struct"),
        )
        self.assertLess(
            body.index("rsync.target_component = msg->compid;"),
            body.index("mavlink_msg_timesync_send_struct"),
        )


if __name__ == "__main__":
    unittest.main()
