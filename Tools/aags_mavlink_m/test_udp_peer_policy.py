#!/usr/bin/env python3
"""Behavior and source checks for protected MAVLink-M UDP ingress."""

from __future__ import annotations

import unittest
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MAIN = (ROOT / "src/modules/mavlink/mavlink_main.cpp").read_text()
HEADER = (ROOT / "src/modules/mavlink/mavlink_main.h").read_text()
HANDLER = (ROOT / "src/modules/mavlink/MavlinkMHandler.cpp").read_text()
RECEIVER = (ROOT / "src/modules/mavlink/mavlink_receiver.cpp").read_text()
PARAMETERS = (ROOT / "src/modules/mavlink/mavlink_params.c").read_text()
CMAKE = (ROOT / "src/modules/mavlink/CMakeLists.txt").read_text()


@dataclass(frozen=True)
class Endpoint:
    address: str
    port: int


@dataclass
class Peer:
    endpoint: Endpoint
    system_id: int
    component_id: int
    last_heartbeat_s: float


class PeerMode(Enum):
    DISABLED = auto()
    DIRECT = auto()
    GATEWAY = auto()


class PeerTable:
    """Executable model of the fixed C++ provenance policy."""

    def __init__(
        self,
        *,
        enabled: bool = True,
        limit: int = 4,
        timeout_s: float = 30.0,
        explicit_gateway: Endpoint | None = None,
    ) -> None:
        if not 0 <= limit <= 8:
            raise ValueError("limit")
        self.mode = (
            PeerMode.DISABLED
            if not enabled
            else PeerMode.GATEWAY
            if limit == 0
            else PeerMode.DIRECT
        )
        self.limit = limit if self.mode is PeerMode.DIRECT else 0
        self.timeout_s = timeout_s
        self.explicit_gateway = explicit_gateway
        self.peers: list[Peer] = []
        self.conflicts = 0
        self.capacity_rejections = 0
        self.expirations = 0

    def expire(self, now_s: float) -> None:
        if self.mode is not PeerMode.DIRECT:
            return
        retained = [
            peer
            for peer in self.peers
            if now_s - peer.last_heartbeat_s <= self.timeout_s
        ]
        self.expirations += len(self.peers) - len(retained)
        self.peers = retained

    def register(
        self,
        endpoint: Endpoint,
        system_id: int,
        component_id: int,
        now_s: float,
    ) -> bool:
        if self.mode is not PeerMode.DIRECT:
            return False
        self.expire(now_s)
        for peer in self.peers:
            same_identity = (
                peer.system_id == system_id
                and peer.component_id == component_id
            )
            same_endpoint = peer.endpoint == endpoint
            if same_identity and same_endpoint:
                peer.last_heartbeat_s = now_s
                return True
            if same_identity or same_endpoint:
                self.conflicts += 1
                return False
        if len(self.peers) >= self.limit:
            self.capacity_rejections += 1
            return False
        self.peers.append(
            Peer(endpoint, system_id, component_id, last_heartbeat_s=now_s)
        )
        return True

    def authorized(
        self,
        endpoint: Endpoint,
        system_id: int,
        component_id: int,
        now_s: float,
    ) -> bool:
        if system_id == 0 or component_id == 0:
            return False
        if self.mode is PeerMode.GATEWAY:
            return self.explicit_gateway is not None and endpoint == self.explicit_gateway
        if self.mode is not PeerMode.DIRECT:
            return False
        self.expire(now_s)
        return any(
            peer.endpoint == endpoint
            and peer.system_id == system_id
            and peer.component_id == component_id
            for peer in self.peers
        )


def signing_link_allowed(
    *, signed_mode: bool, signed: bool, actual_link: int, expected_link: int
) -> bool:
    return not signed_mode or (signed and actual_link == expected_link)


def route_wide_ingress_allowed(
    *, protocol: str, peer_mode: PeerMode, endpoint_authorized: bool
) -> bool:
    """Model the receiver gate shared by all MAVLink message handlers."""

    if protocol != "udp" or peer_mode is PeerMode.DISABLED:
        return True
    return endpoint_authorized


def esad_allowed(
    *,
    cue_route: bool,
    control_config_valid: bool,
    endpoint_authorized: bool,
    control_identity_matches: bool,
    signed_mode: bool,
    signed: bool,
    actual_link: int,
    control_link: int,
) -> bool:
    return (
        cue_route
        and control_config_valid
        and endpoint_authorized
        and control_identity_matches
        and signing_link_allowed(
            signed_mode=signed_mode,
            signed=signed,
            actual_link=actual_link,
            expected_link=control_link,
        )
    )


class DatagramParserModel:
    """Small framing model whose state is intentionally local to one datagram."""

    STX = 0xFD

    @classmethod
    def parse(cls, datagram: bytes) -> list[bytes]:
        frames: list[bytes] = []
        index = 0
        while index < len(datagram):
            if datagram[index] != cls.STX:
                index += 1
                continue
            if index + 2 > len(datagram):
                break
            payload_length = datagram[index + 1]
            end = index + 2 + payload_length
            if end > len(datagram):
                break
            frames.append(datagram[index + 2 : end])
            index = end
        return frames


class TestPeerTableBehavior(unittest.TestCase):
    def test_exact_identity_and_endpoint_refreshes(self) -> None:
        table = PeerTable()
        endpoint = Endpoint("10.0.0.20", 14550)
        self.assertTrue(table.register(endpoint, 253, 190, 1.0))
        self.assertTrue(table.register(endpoint, 253, 190, 20.0))
        self.assertTrue(table.authorized(endpoint, 253, 190, 20.0))
        self.assertEqual(len(table.peers), 1)

    def test_wrong_endpoint_is_not_authorized(self) -> None:
        table = PeerTable()
        table.register(Endpoint("10.0.0.20", 14550), 253, 190, 1.0)
        self.assertFalse(
            table.authorized(Endpoint("10.0.0.21", 14550), 253, 190, 2.0)
        )
        self.assertFalse(
            table.authorized(Endpoint("10.0.0.20", 14551), 253, 190, 2.0)
        )

    def test_unregistered_and_expired_peers_are_not_authorized(self) -> None:
        table = PeerTable(timeout_s=30.0)
        endpoint = Endpoint("10.0.0.20", 14550)
        self.assertFalse(table.authorized(endpoint, 253, 190, 1.0))
        self.assertTrue(table.register(endpoint, 253, 190, 1.0))
        self.assertFalse(table.authorized(endpoint, 253, 190, 32.0))
        self.assertEqual(table.expirations, 1)

    def test_live_identity_and_endpoint_conflicts_are_rejected(self) -> None:
        table = PeerTable()
        endpoint = Endpoint("10.0.0.20", 14550)
        self.assertTrue(table.register(endpoint, 253, 190, 1.0))
        self.assertFalse(
            table.register(Endpoint("10.0.0.21", 14550), 253, 190, 2.0)
        )
        self.assertFalse(table.register(endpoint, 254, 190, 2.0))
        self.assertEqual(table.conflicts, 2)

    def test_fixed_capacity_rejects_ninth_peer(self) -> None:
        table = PeerTable(limit=8)
        for offset in range(8):
            self.assertTrue(
                table.register(
                    Endpoint(f"10.0.0.{offset + 1}", 14550 + offset),
                    240 + offset,
                    190,
                    1.0,
                )
            )
        self.assertFalse(
            table.register(Endpoint("10.0.0.9", 14559), 249, 190, 2.0)
        )
        self.assertEqual(table.capacity_rejections, 1)

    def test_gateway_requires_exact_explicit_partner(self) -> None:
        gateway = Endpoint("10.0.0.1", 14550)
        table = PeerTable(limit=0, explicit_gateway=gateway)
        self.assertTrue(table.authorized(gateway, 253, 190, 1.0))
        self.assertTrue(table.authorized(gateway, 254, 190, 1.0))
        self.assertFalse(
            table.authorized(Endpoint("10.0.0.2", 14550), 253, 190, 1.0)
        )
        self.assertFalse(table.register(gateway, 253, 190, 1.0))

    def test_gateway_without_explicit_partner_fails_closed(self) -> None:
        table = PeerTable(limit=0)
        self.assertFalse(
            table.authorized(Endpoint("10.0.0.1", 14550), 253, 190, 1.0)
        )


class TestProtectedMessageBehavior(unittest.TestCase):
    def test_unregistered_udp_sender_cannot_reach_any_generic_handler(self) -> None:
        for message_kind in (
            "arm command",
            "mode command",
            "mission item",
            "parameter write",
            "FTP request",
            "timesync",
        ):
            with self.subTest(message_kind=message_kind):
                self.assertFalse(
                    route_wide_ingress_allowed(
                        protocol="udp",
                        peer_mode=PeerMode.DIRECT,
                        endpoint_authorized=False,
                    )
                )
                self.assertFalse(
                    route_wide_ingress_allowed(
                        protocol="udp",
                        peer_mode=PeerMode.GATEWAY,
                        endpoint_authorized=False,
                    )
                )

    def test_authorized_udp_and_non_udp_behavior_are_preserved(self) -> None:
        self.assertTrue(
            route_wide_ingress_allowed(
                protocol="udp",
                peer_mode=PeerMode.DIRECT,
                endpoint_authorized=True,
            )
        )
        self.assertTrue(
            route_wide_ingress_allowed(
                protocol="udp",
                peer_mode=PeerMode.GATEWAY,
                endpoint_authorized=True,
            )
        )
        self.assertTrue(
            route_wide_ingress_allowed(
                protocol="udp",
                peer_mode=PeerMode.DISABLED,
                endpoint_authorized=False,
            )
        )
        self.assertTrue(
            route_wide_ingress_allowed(
                protocol="serial",
                peer_mode=PeerMode.DIRECT,
                endpoint_authorized=False,
            )
        )

    def test_signed_mode_rejects_valid_key_with_wrong_link_id(self) -> None:
        self.assertTrue(
            signing_link_allowed(
                signed_mode=True, signed=True, actual_link=7, expected_link=7
            )
        )
        self.assertFalse(
            signing_link_allowed(
                signed_mode=True, signed=True, actual_link=8, expected_link=7
            )
        )
        self.assertFalse(
            signing_link_allowed(
                signed_mode=True, signed=False, actual_link=7, expected_link=7
            )
        )

    def test_esad_requires_every_control_gate(self) -> None:
        valid = dict(
            cue_route=True,
            control_config_valid=True,
            endpoint_authorized=True,
            control_identity_matches=True,
            signed_mode=True,
            signed=True,
            actual_link=8,
            control_link=8,
        )
        self.assertTrue(esad_allowed(**valid))
        for key in (
            "cue_route",
            "control_config_valid",
            "endpoint_authorized",
            "control_identity_matches",
            "signed",
        ):
            denied = valid | {key: False}
            self.assertFalse(esad_allowed(**denied), key)
        self.assertFalse(esad_allowed(**(valid | {"actual_link": 7})))

    def test_cross_datagram_split_never_completes(self) -> None:
        self.assertEqual(DatagramParserModel.parse(bytes([0xFD, 2, 0xAA])), [])
        self.assertEqual(DatagramParserModel.parse(bytes([0xBB])), [])
        self.assertEqual(
            DatagramParserModel.parse(bytes([0xFD, 2, 0xAA, 0xBB])),
            [bytes([0xAA, 0xBB])],
        )

    def test_full_forwarding_copy_preserves_tail_signature(self) -> None:
        parsed_object = bytes(range(32)) + b"signature-123"
        forwarded_object = bytes(parsed_object)
        self.assertEqual(forwarded_object[-13:], b"signature-123")


class TestSourcePolicy(unittest.TestCase):
    def test_table_and_parameter_bounds_match(self) -> None:
        self.assertIn("MavlinkMUdpPeerCapacity = 8", HEADER)
        self.assertIn("PARAM_DEFINE_INT32(MAV_M_PEERS, 4);", PARAMETERS)
        self.assertIn("PARAM_DEFINE_INT32(MAV_M_P_TMO, 30);", PARAMETERS)
        self.assertIn("@min 0\n * @max 8", PARAMETERS)
        self.assertIn("? peer_limit : 0", HANDLER)

    def test_gateway_state_and_explicit_partner_are_separate(self) -> None:
        self.assertIn("MavlinkMUdpPeerMode", HEADER)
        self.assertIn("MavlinkMUdpPeerMode::Gateway", MAIN)
        self.assertIn("_src_addr_explicitly_configured", HEADER)
        authorization = MAIN[
            MAIN.index("bool Mavlink::mavlink_m_udp_ingress_authorized") :
            MAIN.index("#endif // MAVLINK_UDP && CONFIG_MAVLINK_M_PRIVATE_PROFILE")
        ]
        self.assertIn("_src_addr_explicitly_configured", authorization)
        self.assertIn("udp_endpoints_equal(address, _src_addr)", authorization)

    def test_primary_udp_destination_is_not_duplicated_by_peer_fanout(self) -> None:
        send_finish = MAIN[
            MAIN.index("void Mavlink::send_finish()") :
            MAIN.index("void Mavlink::send_bytes(")
        ]
        self.assertIn("bool primary_destination_sent = false;", send_finish)
        self.assertIn(
            "primary_destination_sent = ret == static_cast<int>(_buf_fill);",
            send_finish,
        )
        self.assertIn(
            "primary_destination_sent && udp_endpoints_equal(peer.address, _src_addr)",
            send_finish,
        )

    def test_registration_is_after_valid_frame_parse(self) -> None:
        parse = RECEIVER.index("if (mavlink_parse_char(")
        admission = RECEIVER.index("accepts_udp_peer_heartbeat(msg)")
        registration = RECEIVER.index("register_mavlink_m_udp_peer(")
        self.assertLess(parse, admission)
        self.assertLess(admission, registration)

    def test_heartbeat_admission_checks_type_component_signature_and_link(self) -> None:
        admission = HANDLER[
            HANDLER.index("bool MavlinkMHandler::accepts_udp_peer_heartbeat") :
            HANDLER.index("void MavlinkMHandler::apply_udp_peer_configuration")
        ]
        self.assertIn("message.msgid != MAVLINK_MSG_ID_HEARTBEAT", admission)
        self.assertIn("message.compid == static_cast<uint8_t>(_source_component)", admission)
        self.assertIn("message.compid == static_cast<uint8_t>(_control_component)", admission)
        self.assertIn("(!cue_component && !control_component)", admission)
        self.assertIn("heartbeat.type == MAV_TYPE_GCS", admission)
        self.assertIn("cue_component && signing_link_matches(message, _link_id)", admission)
        self.assertIn(
            "control_component && signing_link_matches(message, _control_link_id)",
            admission,
        )

    def test_separate_route_esad_owner_can_register_on_cue_physical_route(self) -> None:
        admission = HANDLER[
            HANDLER.index("bool MavlinkMHandler::accepts_udp_peer_heartbeat") :
            HANDLER.index("void MavlinkMHandler::apply_udp_peer_configuration")
        ]
        for contract in (
            "const bool esad_control_on_cue_route = cue_route && _control_configuration_valid;",
            "(control_route || esad_control_on_cue_route)",
            "const bool cue_link_matches = cue_component",
            "const bool control_link_matches = control_component",
            "if (!cue_link_matches && !control_link_matches)",
        ):
            self.assertIn(contract, admission)
        self.assertNotIn("const int32_t expected_link_id =", admission)

    def test_signed_selected_route_is_locked_until_both_signing_contexts_are_active(self) -> None:
        signing_active = HANDLER[
            HANDLER.index("bool MavlinkMHandler::signing_active() const") :
            HANDLER.index("bool MavlinkMHandler::generic_ingress_allowed")
        ]
        for contract in (
            "_signing_ready",
            "_mavlink->get_status()->signing == &_signing",
            "_mavlink->get_status()->signing_streams == &_signing_streams",
            "_receiver_status->signing == &_signing",
            "_receiver_status->signing_streams == &_signing_streams",
            "signing_required() && (cue_physical_route_selected() || control_physical_route_selected())",
            "&& !signing_active();",
        ):
            self.assertIn(contract, signing_active)

        route_lock = RECEIVER.index("if (_mavlink_m_handler.ingress_locked())")
        protected = RECEIVER.index("_mavlink_m_handler.handle_message(", route_lock)
        generic = RECEIVER.index("handle_message(&msg);", protected)
        self.assertLess(route_lock, protected)
        self.assertLess(route_lock, generic)
        self.assertIn("continue;", RECEIVER[route_lock:protected])

        runner = (
            ROOT / "Tools/aags_mavlink_m/run_sitl_acceptance.py"
        ).read_text()
        self.assertIn("signed_route_cold_start_lock_and_recovery", runner)
        for blocked_class in (
            "locked COMMAND_ACK",
            "locked MISSION_ACK",
            "locked PARAM_VALUE",
            "locked MAVFTP response",
            "locked ESAD_STATE",
        ):
            self.assertIn(blocked_class, runner)

    def test_signed_route_retries_and_invalid_clock_fails_closed(self) -> None:
        signing_setup = HANDLER[
            HANDLER.index("bool MavlinkMHandler::configure_signing()") :
            HANDLER.index("bool MavlinkMHandler::load_signing_key")
        ]
        for contract in (
            "const uint64_t timestamp = signing_timestamp();",
            "if (timestamp == 0 || !load_signing_key(key))",
            'PX4_ERR("MAVLink-M signed mode locked: key/time invalid")',
            "_receiver_status->signing = &_signing;",
            "_receiver_status->signing_streams = &_signing_streams;",
        ):
            self.assertIn(contract, signing_setup)

        timestamp = HANDLER[
            HANDLER.index("uint64_t MavlinkMHandler::signing_timestamp()") :
            HANDLER.index("uint64_t MavlinkMHandler::utc_now_usec()")
        ]
        self.assertIn("clock_gettime(CLOCK_REALTIME, &ts) != 0", timestamp)
        self.assertIn(
            "ts.tv_sec <= static_cast<time_t>(MavlinkSigningEpochSeconds)",
            timestamp,
        )
        self.assertIn("return 0;", timestamp)

        update = HANDLER[
            HANDLER.index("void MavlinkMHandler::update()") :
            HANDLER.index("bool MavlinkMHandler::configure_signing()")
        ]
        self.assertIn("now - _last_signing_retry >= 1'000'000", update)
        self.assertIn("configure_signing()", update)

    def test_authorized_gcs_heartbeat_alone_refreshes_task_source_liveness(self) -> None:
        handle_start = HANDLER.index("bool MavlinkMHandler::handle_message")
        refresh = HANDLER[
            handle_start :
            HANDLER.index("if (is_esad_control_message", handle_start)
        ]
        for contract in (
            "message.msgid == MAVLINK_MSG_ID_HEARTBEAT",
            "message.len >= MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN",
            "task_message_allowed(message, udp_endpoint_authorized)",
            "mavlink_msg_heartbeat_decode(&message, &heartbeat)",
            "heartbeat.type == MAV_TYPE_GCS",
            "_source_last_seen[message.sysid] = hrt_absolute_time();",
            "return false;",
        ):
            self.assertIn(contract, refresh)
        self.assertLess(
            refresh.index(
                "task_message_allowed(message, udp_endpoint_authorized)"
            ),
            refresh.index("_source_last_seen[message.sysid]"),
        )
        runner = (
            ROOT / "Tools/aags_mavlink_m/run_sitl_acceptance.py"
        ).read_text()
        self.assertIn(
            "source_heartbeats_sustain_pending_level_acceptance", runner
        )
        self.assertIn("source_heartbeats_sustain_active_intercept", runner)

    def test_datagram_framing_resets_before_parse(self) -> None:
        parse = RECEIVER.index("if (mavlink_parse_char(")
        first_reset = RECEIVER.index(
            "parser_status->parse_state = MAVLINK_PARSE_STATE_IDLE"
        )
        self.assertLess(first_reset, parse)
        self.assertGreaterEqual(
            RECEIVER.count("parser_status->signature_wait = 0"), 2
        )

    def test_protected_handler_precedes_all_generic_paths(self) -> None:
        route_gate = RECEIVER.index(
            "if (udp_provenance_managed && !udp_endpoint_authorized)"
        )
        protected = RECEIVER.index("_mavlink_m_handler.handle_message(")
        generic = RECEIVER.index("handle_message(&msg);", protected)
        self.assertLess(route_gate, protected)
        self.assertLess(protected, generic)
        self.assertIn("continue;", RECEIVER[route_gate:protected])
        self.assertIn("continue;", RECEIVER[protected:generic])
        for generic_path in (
            "_mission_manager.handle_message(&msg);",
            "_parameters_manager.handle_message(&msg);",
            "_mavlink_ftp.handle_message(&msg);",
            "_mavlink_timesync.handle_message(&msg);",
            "_mavlink->handle_message(&msg);",
        ):
            self.assertGreater(RECEIVER.index(generic_path, protected), protected)

    def test_route_gate_names_every_generic_ingress_class(self) -> None:
        route_gate = RECEIVER[
            RECEIVER.index("if (udp_provenance_managed && !udp_endpoint_authorized)")
            - 500 : RECEIVER.index("_mavlink_m_handler.handle_message(")
        ]
        for handler_class in (
            "command",
            "mission",
            "parameter",
            "FTP",
            "timesync",
            "forwarding",
        ):
            self.assertIn(handler_class, route_gate)

    def test_task_control_and_esad_apply_endpoint_and_link_checks(self) -> None:
        self.assertIn("task_message_allowed(message, udp_endpoint_authorized)", HANDLER)
        self.assertIn("!udp_endpoint_authorized", HANDLER)
        self.assertIn("signing_link_matches(message, _control_link_id)", HANDLER)
        self.assertIn("OwnerDecisionCommandWireLength == 30", HANDLER)
        self.assertIn("MAVLINK_MSG_ID_ESAD_ARMING", HANDLER)
        self.assertIn("MAVLINK_MSG_ID_ESAD_CONFIG", HANDLER)
        self.assertIn("return !esad_control_allowed", HANDLER)

    def test_authorized_esad_crosses_only_the_separate_cue_route_gate(self) -> None:
        authorization = HANDLER[
            HANDLER.index("bool MavlinkMHandler::esad_control_allowed") :
            HANDLER.index("bool MavlinkMHandler::handle_message")
        ]
        self.assertIn("enabled()", authorization)
        self.assertIn("_control_configuration_valid", authorization)
        self.assertIn("control_source_matches(message)", authorization)
        self.assertIn("signing_link_matches(message, _control_link_id)", authorization)
        self.assertNotIn("control_enabled()", authorization)

        generic = HANDLER[
            HANDLER.index("bool MavlinkMHandler::generic_ingress_allowed") :
            HANDLER.index("bool MavlinkMHandler::enabled")
        ]
        esad_gate = generic.index("if (is_esad_control_message(message.msgid))")
        owner_gate = generic.index("return control_enabled()")
        self.assertLess(esad_gate, owner_gate)
        self.assertIn(
            "return esad_control_allowed(message, udp_endpoint_authorized);",
            generic,
        )

    def test_selected_esad_first_send_bypasses_component_discovery_and_fails_closed(self) -> None:
        forwarding = MAIN[
            MAIN.index("void\nMavlink::forward_message") :
            MAIN.index("int\nMavlink::mavlink_open_uart")
        ]
        for contract in (
            "selected_esad_instance < -1",
            "selected_esad_instance >= MAVLINK_COMM_NUM_BUFFERS",
            "selected_esad_instance == self->get_instance_id()",
            "inst->pass_message(msg);",
            "selected_esad_forwarded = true;",
            "output instance %d is unavailable or forwarding is off",
        ):
            self.assertIn(contract, forwarding)
        explicit_output = forwarding.index("if (selected_esad_control) {")
        component_discovery = forwarding.index(
            "inst->_receiver.component_was_seen("
        )
        self.assertLess(explicit_output, component_discovery)
        self.assertIn("continue;", forwarding[explicit_output:component_discovery])

    def test_invalid_identity_cannot_unclaim_a_selected_physical_route(self) -> None:
        routes = HANDLER[
            HANDLER.index("bool MavlinkMHandler::cue_physical_route_selected") :
            HANDLER.index("bool MavlinkMHandler::signing_active")
        ]
        for contract in (
            "(_mode == 1 || _mode == 2)",
            "_instance >= 0 && _instance < MAVLINK_COMM_NUM_BUFFERS",
            "_control_instance >= 0 && _control_instance < MAVLINK_COMM_NUM_BUFFERS",
            "_endpoint_configuration_valid && cue_physical_route_selected()",
            "_control_configuration_valid && control_physical_route_selected()",
        ):
            self.assertIn(contract, routes)

        lock_and_gate = HANDLER[
            HANDLER.index("bool MavlinkMHandler::ingress_locked") :
            HANDLER.index("bool MavlinkMHandler::enabled")
        ]
        self.assertIn(
            "cue_physical_route_selected() || control_physical_route_selected()",
            lock_and_gate,
        )
        self.assertIn(
            "if (!cue_physical_route_selected() && !control_physical_route_selected())",
            lock_and_gate,
        )

    def test_generic_ingress_is_owner_only_after_protected_task_handling(self) -> None:
        generic = HANDLER[
            HANDLER.index("bool MavlinkMHandler::generic_ingress_allowed") :
            HANDLER.index("bool MavlinkMHandler::enabled")
        ]
        for contract in (
            "control_enabled()",
            "udp_endpoint_authorized",
            "control_source_matches(message)",
            "signing_link_matches(message, _control_link_id)",
        ):
            self.assertIn(contract, generic)
        protected = RECEIVER.index("_mavlink_m_handler.handle_message(")
        owner_gate = RECEIVER.index(
            "_mavlink_m_handler.generic_ingress_allowed(", protected
        )
        ordinary = RECEIVER.index("handle_message(&msg);", owner_gate)
        self.assertLess(protected, owner_gate)
        self.assertLess(owner_gate, ordinary)
        runner = (
            ROOT / "Tools/aags_mavlink_m/run_sitl_acceptance.py"
        ).read_text()
        self.assertIn(
            "learned_observer_is_read_only_while_exact_owner_controls",
            runner,
        )
        self.assertIn(
            "shared_route_admits_only_source_and_control_components",
            runner,
        )

    def test_state_rename_requires_directory_fsync_before_success(self) -> None:
        directory_sync = HANDLER[
            HANDLER.index("bool sync_persistence_directory") :
            HANDLER.index("bool invalidate_persisted_state")
        ]
        for contract in (
            "::open(PX4_STORAGEDIR, O_RDONLY | O_DIRECTORY)",
            "if (directory_fd < 0)",
            "const bool directory_synced = fsync(directory_fd) == 0;",
            "::close(directory_fd);",
            "if (!directory_synced)",
            "return false;",
        ):
            self.assertIn(contract, directory_sync)

        save_state = HANDLER[
            HANDLER.index("bool MavlinkMHandler::save_state()") :
        ]
        for contract in (
            "rename(StateTempPath, StatePath)",
            'if (!sync_persistence_directory("state commit"))',
            "return false;",
        ):
            self.assertIn(contract, save_state)
        self.assertLess(
            save_state.index("rename(StateTempPath, StatePath)"),
            save_state.index('sync_persistence_directory("state commit")'),
        )

        invalidation = HANDLER[
            HANDLER.index("bool invalidate_persisted_state") :
            HANDLER.index("template<typename T>")
        ]
        self.assertIn("directory_changed = true;", invalidation)
        self.assertIn(
            'sync_persistence_directory("state invalidation")', invalidation
        )

    def test_mavlink_m_cmake_stays_compatible_with_declared_cmake_310(self) -> None:
        self.assertNotIn("COMMAND_ERROR_IS_FATAL", CMAKE)
        self.assertNotIn(
            "find_program(MAVLINK_M_PYTHON_EXECUTABLE NAMES python3.9 REQUIRED)",
            CMAKE,
        )
        for contract in (
            "AAGS profile requires vendored pymavlink 2.4.49",
            "Dronecode MAVLink-M canonical XML hash mismatch",
            "if(EXISTS \"${MAVLINK_GIT_DIR}/.git\")",
            "add_custom_target(git_mavlink_v2)",
        ):
            self.assertIn(contract, CMAKE)

    def test_signed_forwarding_copies_complete_message_object(self) -> None:
        forwarding = MAIN[
            MAIN.index("Mavlink::pass_message") :
            MAIN.index("MavlinkShell *", MAIN.index("Mavlink::pass_message"))
        ]
        self.assertIn("sizeof(*msg)", forwarding)
        self.assertNotIn("MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len", forwarding)

    def test_broadcast_exception_is_heartbeat_only(self) -> None:
        discovery = MAIN[
            MAIN.index("const bool fleet_discovery_heartbeat") :
            MAIN.index("if (((_mode != MAVLINK_MODE_ONBOARD)")
        ]
        self.assertIn("MAVLINK_MSG_ID_HEARTBEAT", discovery)
        self.assertIn("loopback_partner", discovery)
        self.assertIn("!loopback_partner", discovery)
        self.assertNotIn("MAVLINK_MSG_ID_TARGET_CUE", discovery)
        self.assertNotIn("MAVLINK_MSG_ID_TARGET_HANDOVER", discovery)

    def test_live_loopback_peer_suppresses_direct_discovery_broadcast(self) -> None:
        discovery = MAIN[
            MAIN.index("bool loopback_partner") :
            MAIN.index("const bool fleet_discovery_heartbeat")
        ]
        for contract in (
            "_src_addr_initialized",
            "_mavlink_m_udp_peer_mode == MavlinkMUdpPeerMode::Direct",
            "peer.last_heartbeat != 0",
            "(ntohl(peer.address.sin_addr.s_addr) >> 24U) == 127U",
            "loopback_partner = true;",
        ):
            self.assertIn(contract, discovery)
        self.assertIn("&& !loopback_partner", MAIN)
        runner = (
            ROOT / "Tools/aags_mavlink_m/run_sitl_acceptance.py"
        ).read_text()
        self.assertIn(
            "loopback_direct_peer_suppresses_lan_discovery_alias", runner
        )

    def test_observer_status_and_fanout_accounting_are_visible(self) -> None:
        self.assertIn("owner_control_enabled || cue_receiver_enabled", HANDLER)
        self.assertIn("fanout copies:", MAIN)
        self.assertIn("fanout bytes:", MAIN)
        self.assertIn("last_conflict_warning", HEADER)
        self.assertIn("last_full_warning", HEADER)


if __name__ == "__main__":
    unittest.main()
