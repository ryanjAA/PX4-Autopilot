#!/usr/bin/env python3
"""Run finalized MAVLink-M cue receipt/decision tests against PX4 SITL.

This starts the already-built PX4 binary in an isolated rootfs, configures the
route-selected endpoint, exchanges finalized MAVLink 2 messages, restarts PX4
against the same durable state, and reports machine-readable results.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import threading
import time
from collections.abc import Callable
import uuid

from endpoint_tool import Endpoint, UdpTransport, fixed_text, load_dialect, text_field


SOURCE_SYSTEM = 255
SOURCE_COMPONENT = 190
OWNER_SYSTEM = 254
OWNER_COMPONENT = 190
WILDCARD_SOURCE_SYSTEM = 253
WILDCARD_OWNER_SYSTEM = 252
WILDCARD_REJECT_SYSTEM = 251
SITL_INSTANCE = 1
VEHICLE_SYSTEM = SITL_INSTANCE + 1
VEHICLE_COMPONENT = 1
TEST_MAVLINK_INSTANCE = 5
CONTROL_MAVLINK_INSTANCE = 2
CONTROL_SIGNING_LINK_ID = 8
TEST_PX4_UDP_PORT = 18671
TEST_AAGS_UDP_PORT = 14551
CONTROL_PX4_UDP_PORT = 14281
CONTROL_AAGS_UDP_PORT = 14031
FORWARD_INGRESS_PX4_UDP_PORT = 18571
FORWARD_INGRESS_TEST_UDP_PORT = 14651
FORWARD_OBSERVER_PX4_UDP_PORT = 14581
FORWARD_OBSERVER_TEST_UDP_PORT = 14541
TRACK_SET = 45
TRACK_UID = uuid.UUID("00112233-4455-6677-8899-aabbccddeeff")
REPLACEMENT_TRACK_UID = uuid.UUID("fedcba98-7654-3210-fedc-ba9876543210")
PROFILE_ID = "dronecode-mavlink-military"
PROFILE_VERSION = "main@23cec367"
CORE_XML_SHA256 = "02a88f79b9b4e58b271d9b0012a06087ac4ec975c75a8b1d7ba704c43bb55a8f"
SIGNING_KEY = bytes(range(32))
ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


class AcceptanceFailure(RuntimeError):
    pass


class Px4Sitl:
    def __init__(self, binary: Path, etc: Path, rootfs: Path, instance: int):
        self.binary = binary
        self.etc = etc
        self.rootfs = rootfs
        self.instance = instance
        self.process: subprocess.Popen[str] | None = None
        self.lines: list[str] = []
        self.condition = threading.Condition()
        self.reader: threading.Thread | None = None

    def start(self) -> int:
        if self.process is not None:
            raise AcceptanceFailure("PX4 process is already running")
        start_line = self.mark()
        environment = os.environ.copy()
        environment["PX4_SIM_MODEL"] = "sihsim_quadx"
        environment["PX4_SIMULATOR"] = "sihsim"
        self.process = subprocess.Popen(
            [
                str(self.binary), "-i", str(self.instance), "-w", str(self.rootfs),
                "-s", "etc/init.d-posix/rcS", str(self.etc),
            ],
            cwd=self.rootfs,
            env=environment,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.reader = threading.Thread(target=self._read_output, daemon=True)
        self.reader.start()
        self.wait_for("Startup script returned successfully", start_line, 20.0)
        return start_line

    def _read_output(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        for line in self.process.stdout:
            with self.condition:
                self.lines.append(ANSI_ESCAPE.sub("", line).replace("\r", ""))
                self.condition.notify_all()

    def mark(self) -> int:
        with self.condition:
            return len(self.lines)

    def text_since(self, mark: int) -> str:
        with self.condition:
            return "".join(self.lines[mark:])

    def wait_for(self, needle: str, mark: int, timeout: float) -> str:
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                output = "".join(self.lines[mark:])
                if needle in output:
                    return output
                if self.process is not None and self.process.poll() is not None:
                    raise AcceptanceFailure(
                        f"PX4 exited with {self.process.returncode} while waiting for {needle!r}\n"
                        + output[-6000:]
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AcceptanceFailure(
                        f"timed out waiting for PX4 output {needle!r}\n" + output[-6000:]
                    )
                self.condition.wait(min(remaining, 0.25))

    def command(self, command: str) -> None:
        if self.process is None or self.process.stdin is None or self.process.poll() is not None:
            raise AcceptanceFailure(f"cannot send PX4 command: {command}")
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()

    def configure(self, max_age: int, signed: bool) -> None:
        commands = [
            f"param set MAV_M_MODE {2 if signed else 1}",
            f"param set MAV_M_INST {TEST_MAVLINK_INSTANCE}",
            f"param set MAV_M_SRC_SYS {SOURCE_SYSTEM}",
            f"param set MAV_M_SRC_CMP {SOURCE_COMPONENT}",
            "param set MAV_M_LNK_ID 7",
            f"param set MAV_M_CTL_INST {CONTROL_MAVLINK_INSTANCE}",
            f"param set MAV_M_CTL_SYS {OWNER_SYSTEM}",
            f"param set MAV_M_CTL_CMP {OWNER_COMPONENT}",
            f"param set MAV_M_CTL_LNK {CONTROL_SIGNING_LINK_ID}",
            "param set MAV_M_SAME_EP 0",
            "param set MAV_M_RC_CH 5",
            "param set MAV_M_RC_REJ 1300",
            "param set MAV_M_RC_ACC 1700",
            f"param set MAV_M_MAX_AGE {max_age}",
            "param set MAV_M_ACTION 0",
            "param set MAV_M_INT_RAD 25",
            "param set MAV_M_INT_DWL 3",
            "param set MAV_M_INT_DZ 100",
            "param save",
        ]
        for command in commands:
            self.command(command)
        time.sleep(0.75)
        mark = self.mark()
        self.command("param show MAV_M_*")
        output = self.wait_for("MAV_M_SRC_SYS", mark, 5.0)
        expected = {
            "MAV_M_MODE": 2 if signed else 1,
            "MAV_M_INST": TEST_MAVLINK_INSTANCE,
            "MAV_M_SRC_SYS": SOURCE_SYSTEM,
            "MAV_M_SRC_CMP": SOURCE_COMPONENT,
            "MAV_M_LNK_ID": 7,
            "MAV_M_CTL_INST": CONTROL_MAVLINK_INSTANCE,
            "MAV_M_CTL_SYS": OWNER_SYSTEM,
            "MAV_M_CTL_CMP": OWNER_COMPONENT,
            "MAV_M_SAME_EP": 0,
            "MAV_M_MAX_AGE": max_age,
            "MAV_M_ACTION": 0,
            "MAV_M_INT_RAD": 25,
            "MAV_M_INT_DWL": 3,
            "MAV_M_INT_DZ": 100,
        }
        missing = [
            f"{name}={value}"
            for name, value in expected.items()
            if re.search(
                rf"\b{re.escape(name)}\b[^\n]*:\s*{value}(?:\.0+)?\s*$",
                output,
                re.MULTILINE,
            ) is None
        ]
        if missing:
            raise AcceptanceFailure(f"PX4 endpoint parameter verification failed: {missing!r}\n{output}")
        time.sleep(0.25)

    def add_test_link(self) -> None:
        """Add a non-forwarding, test-owned link after the five standard links."""
        self.command(
            f"mavlink start -x -u {TEST_PX4_UDP_PORT} "
            f"-o {TEST_AAGS_UDP_PORT} -r 4000000"
        )
        time.sleep(0.75)
        self.command(f"mavlink stream -u {TEST_PX4_UDP_PORT} -s HEARTBEAT -r 2")
        time.sleep(0.25)

    def mavlink_status(self) -> str:
        mark = self.mark()
        self.command("mavlink status")
        self.wait_for(
            f"transport protocol: UDP ({TEST_PX4_UDP_PORT},", mark, 5.0
        )
        time.sleep(0.1)
        return self.text_since(mark)

    def status(self) -> str:
        mark = self.mark()
        self.command("listener mavlink_m_target_status 1")
        self.wait_for("track_identity_valid:", mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

    def wait_target_status(
        self,
        required: list[str],
        timeout: float,
        keepalive: Callable[[], None] | None = None,
        keepalive_interval: float = 5.0,
    ) -> str:
        deadline = time.monotonic() + timeout
        next_keepalive = 0.0
        last_status = ""
        while time.monotonic() < deadline:
            now = time.monotonic()
            if keepalive is not None and now >= next_keepalive:
                keepalive()
                next_keepalive = now + keepalive_interval
            last_status = self.status()
            if all(value in last_status for value in required):
                return last_status
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
        raise AcceptanceFailure(
            f"mavlink_m_target_status did not reach {required!r}\n{last_status}"
        )

    def input_rc_status(self) -> str:
        mark = self.mark()
        self.command("listener input_rc 1")
        self.wait_for("input_source:", mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

    def vehicle_status(self) -> str:
        mark = self.mark()
        self.command("listener vehicle_status 1")
        self.wait_for("arming_state:", mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

    def wait_vehicle_status(self, required: list[str], timeout: float) -> str:
        deadline = time.monotonic() + timeout
        last_status = ""
        while time.monotonic() < deadline:
            last_status = self.vehicle_status()
            if all(value in last_status for value in required):
                return last_status
            time.sleep(min(0.75, max(0.0, deadline - time.monotonic())))
        raise AcceptanceFailure(
            f"vehicle_status did not reach {required!r}\n{last_status}"
        )

    def land_detected_status(self) -> str:
        mark = self.mark()
        self.command("listener vehicle_land_detected 1")
        self.wait_for("landed:", mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

    def wait_land_detected(self, required: list[str], timeout: float) -> str:
        deadline = time.monotonic() + timeout
        last_status = ""
        while time.monotonic() < deadline:
            last_status = self.land_detected_status()
            if all(value in last_status for value in required):
                return last_status
            time.sleep(min(0.75, max(0.0, deadline - time.monotonic())))
        raise AcceptanceFailure(
            f"vehicle_land_detected did not reach {required!r}\n{last_status}"
        )

    def vehicle_command_status(self) -> str:
        mark = self.mark()
        self.command("listener vehicle_command 1")
        self.wait_for("command:", mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

    def global_position_status(self) -> str:
        mark = self.mark()
        self.command("listener vehicle_global_position 1")
        self.wait_for("alt:", mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

    def parameter_status(self, name: str) -> str:
        mark = self.mark()
        self.command(f"param show {name}")
        self.wait_for(name, mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

    def set_invalid_endpoint_parameter(self, name: str, value: int) -> None:
        mark = self.mark()
        self.command(f"param set {name} {value}")
        self.wait_for(
            "invalid MAVLink-M route/source/signing parameter; endpoint disabled",
            mark,
            5.0,
        )
        # Every MAVLink instance owns a handler and consumes parameter updates
        # on its own work cycle. Do not transmit the negative probe until the
        # selected cue instance has also observed the fail-closed value.
        time.sleep(0.5)

    def restore_endpoint_parameter(self, name: str, value: int) -> None:
        self.command(f"param set {name} {value}")
        time.sleep(0.75)
        status = self.parameter_status(name)
        if re.search(
            rf"\b{re.escape(name)}\b[^\n]*:\s*{value}\s*$",
            status,
            re.MULTILINE,
        ) is None:
            raise AcceptanceFailure(
                f"{name} did not restore to {value}\n{status}"
            )

    def stop(self) -> None:
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            try:
                self.command("shutdown")
                process.wait(timeout=8.0)
            except (AcceptanceFailure, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.0)
        self.process = None


def wait_for_heartbeat(endpoint: Endpoint, timeout: float = 12.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.5, deadline - time.monotonic())):
            if message.get_type() == "HEARTBEAT" and message.get_srcSystem() == VEHICLE_SYSTEM:
                return
    raise AcceptanceFailure("no PX4 heartbeat received on UDP 14550")


def establish_udp_peer(endpoint: Endpoint) -> None:
    endpoint.mav.heartbeat_send(
        endpoint.dialect.MAV_TYPE_GCS,
        endpoint.dialect.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )
def drain(endpoint: Endpoint, duration: float = 0.25) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        endpoint.receive(min(0.05, deadline - time.monotonic()))


def require_trimmed_payload(frame: bytes, required: int, full: int, label: str) -> None:
    if not frame or frame[0] != 0xFD:
        raise AcceptanceFailure(f"{label} was not encoded as MAVLink 2")
    payload_length = frame[1]
    if payload_length < required or payload_length >= full:
        raise AcceptanceFailure(
            f"{label} payload length {payload_length} did not exercise "
            f"semantic-prefix acceptance [{required}, {full})"
        )


def wait_for_ack(endpoint: Endpoint, message_id: int, instance: int, timeout: float = 4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.25, deadline - time.monotonic())):
            if (
                message.get_type() == "MAVLINK_M_ACK"
                and message.ack_msgid == message_id
                and message.ack_instance == instance
            ):
                return message
    raise AcceptanceFailure(f"no ACK for msgid={message_id} instance={instance}")


def wait_for_cue_ack_set(
    endpoint: Endpoint,
    expected: dict[int, tuple[int, str]],
    timeout: float = 4.0,
) -> None:
    """Collect multiple cue ACKs and require their specified wire order."""
    remaining = dict(expected)
    expected_order = list(expected)
    observed_order: list[int] = []
    deadline = time.monotonic() + timeout
    while remaining and time.monotonic() < deadline:
        for message in endpoint.receive(min(0.25, deadline - time.monotonic())):
            if (
                message.get_type() != "MAVLINK_M_ACK"
                or message.ack_msgid != endpoint.dialect.MAVLINK_MSG_ID_TARGET_CUE
                or message.ack_instance not in remaining
            ):
                continue
            expected_result, reason_fragment = remaining.pop(message.ack_instance)
            assert_ack(message, expected_result, reason_fragment)
            observed_order.append(message.ack_instance)
    if remaining:
        raise AcceptanceFailure(
            f"missing cue ACKs for instances {sorted(remaining)}"
        )
    if observed_order != expected_order:
        raise AcceptanceFailure(
            f"cue ACK order {observed_order} did not match {expected_order}"
        )


def expect_no_ack(endpoint: Endpoint, message_id: int, instance: int, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.2, deadline - time.monotonic())):
            if (
                message.get_type() == "MAVLINK_M_ACK"
                and message.ack_msgid == message_id
                and message.ack_instance == instance
            ):
                raise AcceptanceFailure(
                    f"unexpected ACK for wrong source msgid={message_id} instance={instance}"
                )

def send_owner_decision(
    endpoint: Endpoint,
    cue_id: int,
    accept: bool,
    requested_effect: int = 0,
) -> None:
    endpoint.mav.command_long_send(
        VEHICLE_SYSTEM,
        VEHICLE_COMPONENT,
        endpoint.dialect.MAV_CMD_USER_1,
        0,
        1 if accept else 2,
        cue_id & 0xFFFF,
        (cue_id >> 16) & 0xFFFF,
        endpoint.dialect.MAVLINK_MSG_ID_TARGET_CUE,
        requested_effect,
        0,
        0,
    )


def wait_for_command_ack(
    endpoint: Endpoint, expected_result: int, timeout: float = 3.0
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.25, deadline - time.monotonic())):
            if (
                message.get_type() == "COMMAND_ACK"
                and message.command == endpoint.dialect.MAV_CMD_USER_1
                and message.get_srcSystem() == VEHICLE_SYSTEM
                and message.get_srcComponent() == VEHICLE_COMPONENT
            ):
                if message.result != expected_result:
                    raise AcceptanceFailure(
                        f"owner decision COMMAND_ACK result={message.result}; "
                        f"wanted {expected_result}"
                    )
                return message
    raise AcceptanceFailure(
        f"no owner decision COMMAND_ACK result={expected_result}"
    )


def wait_for_colocated_accept_acks(
    endpoint: Endpoint, cue_id: int, timeout: float = 4.0
) -> None:
    """Collect both ACK types when cue and owner traffic share one socket."""
    deadline = time.monotonic() + timeout
    command_accepted = False
    cue_accepted = False
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.25, deadline - time.monotonic())):
            if (
                message.get_type() == "COMMAND_ACK"
                and message.command == endpoint.dialect.MAV_CMD_USER_1
                and message.get_srcSystem() == VEHICLE_SYSTEM
                and message.get_srcComponent() == VEHICLE_COMPONENT
            ):
                if message.result != endpoint.dialect.MAV_RESULT_ACCEPTED:
                    raise AcceptanceFailure(
                        "same-endpoint owner decision was not accepted: "
                        f"COMMAND_ACK result={message.result}"
                    )
                command_accepted = True
            elif (
                message.get_type() == "MAVLINK_M_ACK"
                and message.ack_msgid == endpoint.dialect.MAVLINK_MSG_ID_TARGET_CUE
                and message.ack_instance == cue_id
            ):
                assert_ack(
                    message,
                    endpoint.dialect.MAVLINK_M_ACK_ACCEPTED,
                    "local operator accepted",
                )
                cue_accepted = True
        if command_accepted and cue_accepted:
            return
    raise AcceptanceFailure(
        "same-endpoint acceptance did not return both COMMAND_ACK and "
        f"MAVLINK_M_ACK; command={command_accepted} cue={cue_accepted}"
    )


def wait_for_owner_snapshot(
    endpoint: Endpoint, cue_id: int, expected_state_name: str, timeout: float = 4.0
) -> dict:
    deadline = time.monotonic() + timeout
    snapshot: dict[str, int | float] = {}
    observed: list[str] = []
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.25, deadline - time.monotonic())):
            if len(observed) < 24:
                observed.append(
                    f"{message.get_type()}:{message.get_srcSystem()}/{message.get_srcComponent()}"
                )
            if (
                message.get_srcSystem() != VEHICLE_SYSTEM
                or message.get_srcComponent() != VEHICLE_COMPONENT
            ):
                continue
            if message.get_type() == "NAMED_VALUE_INT":
                name = text_field(message.name)
                value = int(message.value)
                if name == "AAGS_DID":
                    observed_cue = value & 0xFFFFFFFF
                    if snapshot.get("cue_id") != observed_cue:
                        snapshot = {"cue_id": observed_cue}
                elif name == "AAGS_LAT":
                    snapshot["lat"] = value
                elif name == "AAGS_LON":
                    snapshot["lon"] = value
                elif name == "AAGS_SRC":
                    snapshot["source"] = value
                elif name == "AAGS_CFG":
                    snapshot["configuration"] = value
                elif name == "AAGS_IPHS":
                    snapshot["intercept_phase"] = value
                elif name == expected_state_name:
                    state_cue = value & 0xFFFFFFFF
                    required = {
                        "cue_id",
                        "lat",
                        "lon",
                        "alt",
                        "source",
                        "configuration",
                        "intercept_phase",
                    }
                    if (
                        state_cue == cue_id
                        and snapshot.get("cue_id") == cue_id
                        and required.issubset(snapshot)
                    ):
                        return snapshot
            elif (
                message.get_type() == "NAMED_VALUE_FLOAT"
                and text_field(message.name) == "AAGS_ALT"
            ):
                snapshot["alt"] = float(message.value)
    raise AcceptanceFailure(
        f"owner link did not publish a correlated {expected_state_name} "
        f"snapshot for cue {cue_id}; partial={snapshot!r}; observed={observed!r}"
    )


def connect_owner_endpoint(dialect, signed: bool) -> tuple[Endpoint, UdpTransport]:
    transport = UdpTransport(
        ("127.0.0.1", CONTROL_AAGS_UDP_PORT),
        ("127.0.0.1", CONTROL_PX4_UDP_PORT),
    )
    endpoint = Endpoint(
        dialect,
        transport,
        OWNER_SYSTEM,
        OWNER_COMPONENT,
        SIGNING_KEY if signed else None,
        CONTROL_SIGNING_LINK_ID,
    )
    establish_udp_peer(endpoint)
    return endpoint, transport


def wait_for_source_message(
    endpoint: Endpoint,
    message_type: str,
    source_system: int,
    source_component: int,
    timeout: float,
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.2, deadline - time.monotonic())):
            if (
                message.get_type() == message_type
                and message.get_srcSystem() == source_system
                and message.get_srcComponent() == source_component
            ):
                return message
    raise AcceptanceFailure(
        f"forwarding control {message_type} from "
        f"{source_system}/{source_component} was not observed"
    )


def expect_no_local_only_forward(
    endpoint: Endpoint,
    source_system: int,
    source_component: int,
    timeout: float = 1.0,
) -> None:
    local_only_types = {
        "TRACK_IDENTITY",
        "TARGET_CUE",
        "TARGET_HANDOVER",
        "MAVLINK_M_ACK",
    }
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.2, deadline - time.monotonic())):
            if (
                message.get_type() in local_only_types
                and message.get_srcSystem() == source_system
                and message.get_srcComponent() == source_component
            ):
                raise AcceptanceFailure(
                    f"route-local {message.get_type()} leaked through generic forwarding"
                )


def assert_ack(
    message,
    result: int,
    reason_contains: str,
    source_system: int = SOURCE_SYSTEM,
) -> None:
    reason = text_field(message.reason)
    if message.result != result or reason_contains not in reason:
        raise AcceptanceFailure(
            f"unexpected ACK result={message.result} reason={reason!r}; "
            f"wanted result={result} containing {reason_contains!r}"
        )
    if message.origin_sysid != source_system or message.ack_sysid != VEHICLE_SYSTEM:
        raise AcceptanceFailure(
            "ACK did not preserve source/receiving-vehicle correlation"
        )


def make_track_identity(
    endpoint: Endpoint,
    now_usec: int,
    track_uid: uuid.UUID = TRACK_UID,
):
    return endpoint.mav.track_identity_encode(
        now_usec,
        track_uid.bytes,
        bytes(16),
        TRACK_SET,
        now_usec - 1_000_000,
        0.8,
        1,
        1,
        7,
        1,
        0,
        0,
        0,
        fixed_text("operator visual", 50),
        fixed_text("", 20),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def make_cue(
    endpoint: Endpoint,
    cue_id: int,
    now_usec: int,
    name: str,
    lat: int = 454671000,
    lon: int = -737578000,
    cue_type: int = 1,
    alt_msl: float = 50.0,
):
    return endpoint.mav.target_cue_encode(
        now_usec,
        cue_id,
        TRACK_SET,
        lat,
        lon,
        alt_msl,
        math.nan,
        math.nan,
        math.nan,
        0.8,
        1,
        cue_type,
        0,
        0,
        fixed_text(name, 20),
    )


def make_handover(endpoint: Endpoint, now_usec: int, target_set: int = 9002):
    return endpoint.mav.target_handover_encode(
        now_usec,
        now_usec - 1_000_000,
        now_usec + 30_000_000,
        454671000,
        -737578000,
        50.0,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        target_set,
        fixed_text("handover", 50),
        fixed_text("", 50),
        0.8,
        bytes(8),
        0,
        0,
        0,
        bytes(16),
    )


def record(results: list[dict], name: str, started: float, detail: str) -> None:
    results.append(
        {
            "name": name,
            "status": "PASS",
            "duration_seconds": round(time.monotonic() - started, 3),
            "detail": detail,
        }
    )
    print(f"{name}=PASS {detail}")


def output_float(output: str, field: str) -> float:
    match = re.search(
        rf"^\s*{re.escape(field)}:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$",
        output,
        re.MULTILINE,
    )
    if match is None:
        raise AcceptanceFailure(f"missing finite {field!r} field\n{output}")
    return float(match.group(1))


def run(binary: Path, etc: Path, rootfs: Path, max_age: int, signed: bool) -> dict:
    dialect = load_dialect()
    transport = UdpTransport(
        ("127.0.0.1", TEST_AAGS_UDP_PORT),
        ("127.0.0.1", TEST_PX4_UDP_PORT),
    )
    endpoint = Endpoint(
        dialect,
        transport,
        SOURCE_SYSTEM,
        SOURCE_COMPONENT,
        SIGNING_KEY if signed else None,
        7,
    )
    wrong_sender = dialect.MAVLink(
        transport, srcSystem=OWNER_SYSTEM, srcComponent=OWNER_COMPONENT
    )
    if signed:
        wrong_sender.signing.secret_key = SIGNING_KEY
        wrong_sender.signing.link_id = 7
        wrong_sender.signing.timestamp = max(
            1, int((time.time() - 1_420_070_400) * 100_000)
        )
        wrong_sender.signing.sign_outgoing = True
    px4 = Px4Sitl(binary, etc, rootfs, SITL_INSTANCE)
    results: list[dict] = []

    try:
        if signed:
            key_path = rootfs / "mavlink_m_signing.key"
            key_path.write_bytes(SIGNING_KEY)
            key_path.chmod(0o600)

        started = time.monotonic()
        px4.start()
        px4.add_test_link()
        px4.configure(max_age, signed)
        route_status = px4.mavlink_status()
        route_pattern = re.compile(
            rf"instance #(?P<instance>\d+):"
            rf"(?:(?!\ninstance #).)*?"
            rf"transport protocol:\s*UDP \({TEST_PX4_UDP_PORT},",
            re.DOTALL,
        )
        route_match = route_pattern.search(route_status)
        if route_match is None:
            raise AcceptanceFailure(
                f"test-owned MAVLink route {TEST_PX4_UDP_PORT} not found\n{route_status}"
            )
        actual_instance = int(route_match.group("instance"))
        if actual_instance != TEST_MAVLINK_INSTANCE:
            raise AcceptanceFailure(
                f"test route is MAVLink instance {actual_instance}, "
                f"but MAV_M_INST={TEST_MAVLINK_INSTANCE}\n{route_status}"
            )
        establish_udp_peer(endpoint)
        wait_for_heartbeat(endpoint)
        record(
            results,
            "sitl_start_and_endpoint_configuration",
            started,
            f"route instance {TEST_MAVLINK_INSTANCE}; "
            f"signing={'required' if signed else 'lab-unsigned'}",
        )

        started = time.monotonic()
        forward_ingress_transport = UdpTransport(
            ("127.0.0.1", FORWARD_INGRESS_TEST_UDP_PORT),
            ("127.0.0.1", FORWARD_INGRESS_PX4_UDP_PORT),
        )
        forward_observer_transport = UdpTransport(
            ("127.0.0.1", FORWARD_OBSERVER_TEST_UDP_PORT),
            ("127.0.0.1", FORWARD_OBSERVER_PX4_UDP_PORT),
        )
        forward_ingress = Endpoint(dialect, forward_ingress_transport, 253, 191)
        forward_observer = Endpoint(dialect, forward_observer_transport, 252, 192)
        try:
            establish_udp_peer(forward_observer)
            establish_udp_peer(forward_ingress)
            time.sleep(0.2)
            drain(forward_observer)
            forward_ingress.mav.heartbeat_send(
                dialect.MAV_TYPE_GCS,
                dialect.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            wait_for_source_message(
                forward_observer, "HEARTBEAT", 253, 191, 2.0
            )
            leak_time = int(time.time() * 1_000_000)
            forward_ingress.mav.send(
                make_track_identity(forward_ingress, leak_time)
            )
            forward_ingress.mav.send(
                make_cue(forward_ingress, 799, leak_time + 100_000, "no leak")
            )
            forward_ingress.mav.send(
                make_handover(forward_ingress, leak_time + 200_000, 9799)
            )
            forward_ingress.mav.mavlink_m_ack_send(
                leak_time + 300_000,
                dialect.MAVLINK_MSG_ID_TARGET_CUE,
                799,
                253,
                VEHICLE_SYSTEM,
                dialect.MAVLINK_M_ACK_RECEIVED,
                fixed_text("forwarding probe", 50),
            )
            expect_no_local_only_forward(
                forward_observer, 253, 191, timeout=1.0
            )
        finally:
            forward_ingress_transport.close()
            forward_observer_transport.close()
        record(
            results,
            "route_local_messages_not_forwarded",
            started,
            "forwarding path carried HEARTBEAT control but suppressed 53000/53001/53002/53004",
        )

        started = time.monotonic()
        now_usec = int(time.time() * 1_000_000)
        track_frame = endpoint.send_frozen(make_track_identity(endpoint, now_usec))
        require_trimmed_payload(track_frame, 58, 141, "TRACK_IDENTITY")
        time.sleep(0.1)
        cue_one = make_cue(endpoint, 731, now_usec + 100_000, "relay cue")
        cue_frame = endpoint.send_frozen(cue_one)
        require_trimmed_payload(cue_frame, 45, 68, "TARGET_CUE")
        ack = wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731)
        assert_ack(ack, dialect.MAVLINK_M_ACK_RECEIVED, "stored pending")
        record(
            results,
            "final_route_selected_cue_receipt",
            started,
            "trimmed TARGET_CUE 53001 stored and ACK RECEIVED correlated to cue 731",
        )

        started = time.monotonic()
        status = px4.status()
        required_status = [
            "instance_id: 731",
            f"target_set_id: {TRACK_SET}",
            "source_system: 255",
            "source_component: 190",
            "origin_system: 1",
            "track_identity_valid: True",
            "command_flags: 0",
        ]
        missing = [value for value in required_status if value not in status]
        if missing:
            raise AcceptanceFailure(f"normalized status missing {missing!r}\n{status}")
        record(
            results,
            "normalized_track_identity_projection",
            started,
            "cue, source endpoint, target-set, and track identity exposed on uORB",
        )

        owner_transport = UdpTransport(
            ("127.0.0.1", CONTROL_AAGS_UDP_PORT),
            ("127.0.0.1", CONTROL_PX4_UDP_PORT),
        )
        owner_endpoint = Endpoint(
            dialect,
            owner_transport,
            OWNER_SYSTEM,
            OWNER_COMPONENT,
            SIGNING_KEY if signed else None,
            CONTROL_SIGNING_LINK_ID,
        )
        try:
            started = time.monotonic()
            establish_udp_peer(owner_endpoint)
            owner_snapshot = wait_for_owner_snapshot(
                owner_endpoint, 731, "AAGS_PEND"
            )
            expected_source = (SOURCE_SYSTEM << 8) | SOURCE_COMPONENT
            if (
                owner_snapshot["lat"] != 454671000
                or owner_snapshot["lon"] != -737578000
                or not math.isclose(float(owner_snapshot["alt"]), 50.0)
                or owner_snapshot["source"] != expected_source
                or owner_snapshot["configuration"] != 1
            ):
                raise AcceptanceFailure(
                    f"owner snapshot did not preserve cue coordinates/alt/source: "
                    f"{owner_snapshot!r}"
                )
            record(
                results,
                "owner_control_prompt_snapshot",
                started,
                "control instance published correlated cue ID, source, exact lat/lon, MSL altitude, cue type, and motion policy before PENDING",
            )

            started = time.monotonic()
            wrong_sender.command_long_send(
                VEHICLE_SYSTEM,
                VEHICLE_COMPONENT,
                dialect.MAV_CMD_USER_1,
                0,
                1,
                731,
                0,
                dialect.MAVLINK_MSG_ID_TARGET_CUE,
                0,
                0,
                0,
            )
            wait_for_command_ack(endpoint, dialect.MAV_RESULT_DENIED)
            px4.wait_target_status(
                ["instance_id: 731", "state: 1", "prompt: True"], 3.0
            )
            unauthorized_sender = dialect.MAVLink(
                owner_transport,
                srcSystem=SOURCE_SYSTEM,
                srcComponent=SOURCE_COMPONENT,
            )
            if signed:
                unauthorized_sender.signing.secret_key = SIGNING_KEY
                unauthorized_sender.signing.link_id = CONTROL_SIGNING_LINK_ID
                unauthorized_sender.signing.timestamp = max(
                    1, int((time.time() - 1_420_070_400) * 100_000)
                )
                unauthorized_sender.signing.sign_outgoing = True
            unauthorized_sender.command_long_send(
                VEHICLE_SYSTEM,
                VEHICLE_COMPONENT,
                dialect.MAV_CMD_USER_1,
                0,
                1,
                731,
                0,
                dialect.MAVLINK_MSG_ID_TARGET_CUE,
                0,
                0,
                0,
            )
            wait_for_command_ack(owner_endpoint, dialect.MAV_RESULT_DENIED)
            px4.wait_target_status(
                ["instance_id: 731", "state: 1", "prompt: True"], 3.0
            )
            record(
                results,
                "owner_decision_authority_isolation",
                started,
                "owner identity on wrong instance and cue-sender identity on owner instance were denied; cue stayed Pending",
            )

            started = time.monotonic()
            send_owner_decision(owner_endpoint, 731, True, requested_effect=1)
            wait_for_command_ack(owner_endpoint, dialect.MAV_RESULT_DENIED)
            send_owner_decision(owner_endpoint, 731, True, requested_effect=3)
            wait_for_command_ack(owner_endpoint, dialect.MAV_RESULT_FAILED)
            px4.wait_target_status(
                ["instance_id: 731", "state: 1", "prompt: True"], 3.0
            )
            record(
                results,
                "owner_effect_permission_ceiling",
                started,
                "explicit level travel above MAV_M_ACTION=0 was denied, invalid effect 3 failed, and the exact cue stayed Pending",
            )

            started = time.monotonic()
            px4.command("param set MAV_M_ACTION 15")
            time.sleep(0.25)
            sanitized_action = px4.parameter_status("MAV_M_ACTION")
            if re.search(
                r"\bMAV_M_ACTION\b[^\n]*:\s*0\s*$",
                sanitized_action,
                re.MULTILINE,
            ) is None:
                raise AcceptanceFailure(
                    "obsolete MAV_M_ACTION was inert internally but not corrected in parameters\n"
                    + sanitized_action
                )
            send_owner_decision(owner_endpoint, 731, True)
            wait_for_command_ack(owner_endpoint, dialect.MAV_RESULT_ACCEPTED)
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731),
                dialect.MAVLINK_M_ACK_ACCEPTED,
                "local operator accepted",
            )
            wait_for_owner_snapshot(owner_endpoint, 731, "AAGS_ACTV")
            accepted_status = px4.status()
            if "state: 2" not in accepted_status or "command_flags: 0" not in accepted_status:
                raise AcceptanceFailure(
                    "owner acceptance or obsolete MAV_M_ACTION fail-closed state was wrong\n"
                    + accepted_status
                )
            record(
                results,
                "owner_control_accept_and_authoritative_ack",
                started,
                "owner COMMAND_ACK queued the decision; cue RX link returned MAVLINK_M_ACK ACCEPTED and owner status became ACTIVE",
            )
        finally:
            owner_transport.close()

        started = time.monotonic()
        wrong_cue = wrong_sender.target_cue_encode(
            int(time.time() * 1_000_000),
            734,
            TRACK_SET,
            454671000,
            -737578000,
            50.0,
            math.nan,
            math.nan,
            math.nan,
            0.8,
            1,
            1,
            0,
            0,
            fixed_text("wrong source", 20),
        )
        wrong_sender.send(wrong_cue)
        expect_no_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 734)
        record(results, "wrong_packet_source_isolation", started, "254/190 produced no ACK")

        started = time.monotonic()
        changed = make_cue(endpoint, 731, now_usec + 100_000, "changed cue", 454672000)
        endpoint.mav.send(changed)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731),
            dialect.MAVLINK_M_ACK_FAILED,
            "immutable instance collision",
        )
        record(results, "immutable_cue_collision", started, "changed cue 731 rejected")

        started = time.monotonic()
        px4.command("param set MAV_M_MAX_AGE 60")
        blocked_cue = make_cue(
            endpoint,
            732,
            int(time.time() * 1_000_000),
            "blocked active",
        )
        endpoint.send_frozen(blocked_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.command("mavlink task accept 732 53001")
        wait_for_cue_ack_set(
            endpoint,
            {
                731: (dialect.MAVLINK_M_ACK_REJECTED, "superseded"),
                732: (dialect.MAVLINK_M_ACK_ACCEPTED, "previous cue superseded"),
            },
        )
        px4.wait_target_status(
            [
                "instance_id: 732",
                "state: 2",
                "queue_depth: 0",
                "command_flags: 0",
                "prompt: False",
            ],
            3.0,
        )
        endpoint.mav.send(cue_one)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731),
            dialect.MAVLINK_M_ACK_REJECTED,
            "duplicate idempotent replay",
        )
        px4.command("mavlink task reject 732 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
        )
        record(
            results,
            "accepted_cue_explicitly_supersedes_active",
            started,
            "accepting cue B durably aborted cue A, ACKed A rejected before B accepted, and made B the sole Active cue",
        )

        px4.command("param set MAV_M_ACTION 1")
        time.sleep(1.0)
        px4.wait_vehicle_status(
            [
                "arming_state: 1",
                "failsafe: False",
                "failure_detector_status: 0",
                "pre_flight_checks_pass: True",
            ],
            20.0,
        )
        px4.command("commander mode auto:loiter")
        px4.wait_vehicle_status(
            ["arming_state: 1", "nav_state: 4", "failsafe: False"],
            5.0,
        )

        non_movement_cue = make_cue(
            endpoint,
            754,
            int(time.time() * 1_000_000),
            "observe while safe",
            cue_type=2,
        )
        endpoint.send_frozen(non_movement_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 754),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.command("mavlink task accept 754 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 754),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted",
        )
        px4.wait_target_status(
            ["instance_id: 754", "state: 2", "command_flags: 0"],
            3.0,
        )

        unsafe_cue = make_cue(
            endpoint,
            735,
            int(time.time() * 1_000_000),
            "state gated",
        )
        unsafe_frame = endpoint.send_frozen(unsafe_cue)
        require_trimmed_payload(unsafe_frame, 45, 68, "state-gated TARGET_CUE")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 735),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.command("mavlink task accept 735 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 735),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "movement blocked",
        )
        px4.wait_target_status(
            [
                "instance_id: 735",
                "state: 1",
                "queue_depth: 1",
                "command_flags: 0",
                "prompt: True",
            ],
            3.0,
        )
        px4.command("commander arm")
        px4.wait_vehicle_status(
            [
                "arming_state: 2",
                "nav_state: 4",
                "failsafe: False",
                "failure_detector_status: 0",
            ],
            8.0,
        )
        time.sleep(0.75)
        delayed_status = px4.status()
        if (
            "instance_id: 735" not in delayed_status
            or "state: 1" not in delayed_status
            or "command_flags: 0" not in delayed_status
        ):
            raise AcceptanceFailure(
                "blocked replacement cue did not remain Pending after a later armed-Hold transition\n"
                + delayed_status
            )
        endpoint.mav.send(non_movement_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 754),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "duplicate idempotent replay",
        )
        px4.command("mavlink task reject 735 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 735),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator rejected",
        )
        px4.wait_target_status(
            ["instance_id: 754", "state: 2", "command_flags: 0"],
            3.0,
        )
        px4.command("mavlink task reject 754 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 754),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
        )
        px4.wait_land_detected(["landed: True"], 5.0)
        ground_cue = make_cue(
            endpoint,
            745,
            int(time.time() * 1_000_000),
            "ground gated",
        )
        endpoint.mav.send(ground_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 745),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.command("mavlink task accept 745 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 745),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "movement blocked",
        )
        ground_status = px4.wait_target_status(
            ["instance_id: 745", "state: 1", "command_flags: 0", "prompt: True"],
            3.0,
        )
        px4.command("commander disarm")
        px4.wait_vehicle_status(["arming_state: 1"], 5.0)
        px4.command("commander takeoff")
        px4.wait_vehicle_status(
            ["arming_state: 2", "failsafe: False", "failure_detector_status: 0"],
            8.0,
        )
        airborne_fields = [
            "freefall: False",
            "ground_contact: False",
            "maybe_landed: False",
            "landed: False",
        ]
        px4.wait_land_detected(airborne_fields, 20.0)
        px4.wait_vehicle_status(
            [
                "arming_state: 2",
                "nav_state: 4",
                "failsafe: False",
                "failure_detector_status: 0",
            ],
            20.0,
        )
        px4.wait_land_detected(airborne_fields, 5.0)
        delayed_ground_status = px4.status()
        if (
            "instance_id: 745" not in delayed_ground_status
            or "state: 1" not in delayed_ground_status
            or "command_flags: 0" not in delayed_ground_status
        ):
            raise AcceptanceFailure(
                "ground-blocked cue did not remain Pending after a later takeoff\n"
                + delayed_ground_status
            )
        px4.command("mavlink task reject 745 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 745),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator rejected",
        )
        record(
            results,
            "reposition_requires_existing_safe_flight_state",
            started,
            "a blocked movement replacement stayed Pending while the existing Observe cue remained Active and never auto-promoted",
        )

        started = time.monotonic()
        stale_source_cue = make_cue(
            endpoint,
            758,
            int(time.time() * 1_000_000),
            "stale level travel",
        )
        endpoint.send_frozen(stale_source_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 758),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.wait_target_status(
            ["instance_id: 758", "state: 1", "source_fresh: False"],
            45.0,
        )
        px4.command("mavlink task accept 758 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 758),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "movement blocked: cue source is stale",
        )
        px4.wait_target_status(
            ["instance_id: 758", "state: 1", "command_flags: 0", "prompt: True"],
            3.0,
        )
        endpoint.mav.heartbeat_send(
            dialect.MAV_TYPE_GCS,
            dialect.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        time.sleep(0.25)
        px4.command("mavlink task reject 758 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 758),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator rejected",
        )
        record(
            results,
            "level_movement_requires_fresh_cue_source",
            started,
            "level travel from a source silent for 15 seconds stayed Pending and issued no navigation command",
        )

        started = time.monotonic()
        level_altitude_before = output_float(
            px4.global_position_status(), "alt"
        )
        wire_altitude_to_ignore = -321.25
        safe_cue = make_cue(
            endpoint,
            736,
            int(time.time() * 1_000_000),
            "safe one shot",
            alt_msl=wire_altitude_to_ignore,
        )
        safe_frame = endpoint.send_frozen(safe_cue)
        require_trimmed_payload(safe_frame, 45, 68, "safe-state TARGET_CUE")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 736),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.command("mavlink task accept 736 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 736),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted",
        )
        safe_status = px4.wait_target_status(
            ["instance_id: 736", "state: 2", "command_flags: 1"],
            3.0,
        )
        vehicle_command = px4.vehicle_command_status()
        required_command = [
            r"\bcommand:\s*192\b",
            r"\bparam2:\s*1(?:\.0+)?\b",
            r"\bsource_system:\s*255\b",
            r"\bsource_component:\s*190\b",
        ]
        missing_command = [
            pattern for pattern in required_command
            if re.search(pattern, vehicle_command) is None
        ]
        if missing_command:
            raise AcceptanceFailure(
                "one-shot reposition vehicle_command audit failed "
                f"{missing_command!r}\n{vehicle_command}"
            )
        commanded_level_altitude = output_float(vehicle_command, "param7")
        if (
            abs(commanded_level_altitude - level_altitude_before) > 5.0
            or abs(commanded_level_altitude - wire_altitude_to_ignore) < 1.0
        ):
            raise AcceptanceFailure(
                "MAV_M_ACTION=1 did not preserve acceptance-time aircraft AMSL "
                f"(before={level_altitude_before}, cue={wire_altitude_to_ignore}, "
                f"command={commanded_level_altitude})\n{vehicle_command}"
            )
        unchanged_mode = px4.vehicle_status()
        if "arming_state: 2" not in unchanged_mode or "nav_state: 4" not in unchanged_mode:
            raise AcceptanceFailure(
                "one-shot reposition changed armed Hold state\n" + unchanged_mode
            )
        record(
            results,
            "safe_state_one_shot_reposition",
            started,
            "fresh airborne acceptance emitted DO_REPOSITION at aircraft AMSL, ignored finite cue altitude, and stayed in Hold",
        )
        original_command_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", vehicle_command
        )
        if original_command_timestamp is None:
            raise AcceptanceFailure(
                "one-shot vehicle_command had no timestamp\n" + vehicle_command
            )

        supersede_started = time.monotonic()
        replacement_lat = 454691000
        replacement_lon = -737558000
        replacement_cue = make_cue(
            endpoint,
            759,
            int(time.time() * 1_000_000),
            "replacement travel",
            lat=replacement_lat,
            lon=replacement_lon,
            alt_msl=999.0,
        )
        endpoint.send_frozen(replacement_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 759),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.command("mavlink task accept 759 53001")
        wait_for_cue_ack_set(
            endpoint,
            {
                736: (dialect.MAVLINK_M_ACK_REJECTED, "superseded"),
                759: (dialect.MAVLINK_M_ACK_ACCEPTED, "previous cue superseded"),
            },
        )
        px4.wait_target_status(
            ["instance_id: 759", "state: 2", "command_flags: 1"],
            3.0,
        )
        replacement_command = px4.vehicle_command_status()
        replacement_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", replacement_command
        )
        if (
            replacement_timestamp is None
            or int(replacement_timestamp.group(1))
            <= int(original_command_timestamp.group(1))
            or re.search(r"\bcommand:\s*192\b", replacement_command) is None
            or abs(output_float(replacement_command, "param5") - replacement_lat * 1e-7) > 1.0e-5
            or abs(output_float(replacement_command, "param6") - replacement_lon * 1e-7) > 1.0e-5
        ):
            raise AcceptanceFailure(
                "moving cue supersession did not publish the replacement coordinate\n"
                + replacement_command
            )
        original_command_timestamp = replacement_timestamp
        record(
            results,
            "moving_cue_supersession_replaces_navigation",
            supersede_started,
            "fresh cue 759 durably aborted moving cue 736 and published one newer DO_REPOSITION to the replacement coordinate",
        )

        px4.command("param set MAV_M_ACTION 0")
        time.sleep(0.3)
        if re.search(
            r"\bMAV_M_ACTION\b[^\n]*:\s*0\s*$",
            px4.parameter_status("MAV_M_ACTION"),
            re.MULTILINE,
        ) is None:
            raise AcceptanceFailure("MAV_M_ACTION did not revoke future commands")
        stop_replacement_started = time.monotonic()
        stop_replacement = make_cue(
            endpoint,
            760,
            int(time.time() * 1_000_000),
            "observe replacement",
            cue_type=2,
        )
        endpoint.send_frozen(stop_replacement)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 760),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        px4.command("mavlink task accept 760 53001")
        wait_for_cue_ack_set(
            endpoint,
            {
                759: (dialect.MAVLINK_M_ACK_REJECTED, "superseded"),
                760: (dialect.MAVLINK_M_ACK_ACCEPTED, "previous cue superseded"),
            },
        )
        px4.wait_target_status(
            ["instance_id: 760", "state: 2", "command_flags: 0"],
            3.0,
        )
        cancellation_command = px4.vehicle_command_status()
        cancellation_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", cancellation_command
        )
        if (
            cancellation_timestamp is None
            or int(cancellation_timestamp.group(1))
            <= int(original_command_timestamp.group(1))
            or re.search(r"\bcommand:\s*192\b", cancellation_command) is None
            or re.search(r"\bparam2:\s*1(?:\.0+)?\b", cancellation_command) is None
        ):
            raise AcceptanceFailure(
                "moving-to-nonmoving supersession did not publish a hold\n"
                + cancellation_command
            )
        record(
            results,
            "moving_to_nonmoving_supersession_stops_navigation",
            stop_replacement_started,
            "Observe cue 760 rejected moving cue 759 before acceptance and published a newer hold even after MAV_M_ACTION was revoked",
        )
        px4.command("mavlink task reject 760 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 760),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
        )

        started = time.monotonic()
        px4.command("param set MAV_M_ACTION 2")
        px4.command("param set MAV_M_INT_DWL 60")
        px4.command("param set GF_ACTION 2")
        px4.command("param set GF_MAX_HOR_DIST 10000")
        time.sleep(1.0)
        restrictive_base_altitude = output_float(
            px4.global_position_status(), "alt"
        )
        restrictive_cue = make_cue(
            endpoint,
            749,
            int(time.time() * 1_000_000),
            "fence blocked",
            alt_msl=restrictive_base_altitude + 10.0,
        )
        endpoint.mav.send(restrictive_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 749),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending pilot decision",
        )
        px4.command("mavlink task accept 749 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 749),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted active target",
        )
        px4.wait_target_status(
            [
                "instance_id: 749",
                "state: 2",
                "command_flags: 3",
                "intercept_phase: 4",
            ],
            3.0,
        )
        px4.command("mavlink task reject 749 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 749),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted active target",
        )
        record(
            results,
            "configured_restrictive_geofence_rejects_exact_intercept",
            started,
            "GF_ACTION=2 with a configured horizontal fence limit rejected the private exact fly-through and permanently aborted that acceptance",
        )

        started = time.monotonic()
        px4.command("param set GF_MAX_HOR_DIST 0")
        time.sleep(1.0)
        no_fence_base_altitude = output_float(px4.global_position_status(), "alt")
        no_fence_altitude = no_fence_base_altitude + 10.0
        no_fence_cue = make_cue(
            endpoint,
            753,
            int(time.time() * 1_000_000),
            "no fence default",
            alt_msl=no_fence_altitude,
        )
        endpoint.mav.send(no_fence_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 753),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending pilot decision",
        )
        px4.command("mavlink task accept 753 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 753),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted active target",
        )
        no_fence_status = px4.wait_target_status(
            ["instance_id: 753", "state: 2", "command_flags: 3"],
            3.0,
        )
        if re.search(r"\bintercept_phase:\s*[12]\b", no_fence_status) is None:
            raise AcceptanceFailure(
                "default GF_ACTION=2 with no configured fence blocked intercept\n"
                + no_fence_status
            )
        no_fence_command = px4.vehicle_command_status()
        if (
            re.search(r"\bcommand:\s*100001\b", no_fence_command) is None
            or abs(output_float(no_fence_command, "param7") - no_fence_altitude) > 0.2
        ):
            raise AcceptanceFailure(
                "default GF_ACTION=2 with no configured fence did not publish the exact fly-through\n"
                + no_fence_command
            )
        px4.command("mavlink task reject 753 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 753),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted active target",
        )
        record(
            results,
            "default_geofence_action_without_fence_allows_exact_intercept",
            started,
            "GF_ACTION=2 with zero polygon, circle, horizontal, and vertical constraints began the exact fly-through",
        )

        started = time.monotonic()
        px4.command("param set MAV_M_ACTION 2")
        px4.command("param set MAV_M_INT_DWL 60")
        px4.command("param set GF_ACTION 1")
        time.sleep(1.0)
        intercept_base_altitude = output_float(
            px4.global_position_status(), "alt"
        )
        intercept_altitude = intercept_base_altitude + 12.5
        intercept_cue = make_cue(
            endpoint,
            746,
            int(time.time() * 1_000_000),
            "explicit intercept",
            alt_msl=intercept_altitude,
        )
        endpoint.mav.send(intercept_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 746),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending pilot decision",
        )
        px4.command("mavlink task accept 746 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 746),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted active target",
        )
        intercept_status = px4.wait_target_status(
            ["instance_id: 746", "state: 2", "command_flags: 3"],
            3.0,
        )
        if re.search(r"\bintercept_phase:\s*[12]\b", intercept_status) is None:
            raise AcceptanceFailure(
                "finite-altitude intercept did not enter approach transit/dwell\n"
                + intercept_status
            )
        intercept_command = px4.vehicle_command_status()
        commanded_intercept_altitude = output_float(intercept_command, "param7")
        intercept_token_low = output_float(intercept_command, "param1")
        intercept_token_high = output_float(intercept_command, "param2")
        intercept_token = int(intercept_token_low) | (int(intercept_token_high) << 16)
        first_intercept_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", intercept_command
        )
        if (
            re.search(r"\bcommand:\s*100001\b", intercept_command) is None
            or first_intercept_timestamp is None
            or intercept_token_low != int(intercept_token_low)
            or intercept_token_high != int(intercept_token_high)
            or intercept_token <= 0
            or intercept_token > 0x7FFFFFFF
            or abs(commanded_intercept_altitude - intercept_altitude) > 0.2
        ):
            raise AcceptanceFailure(
                "MAV_M_ACTION=2 finite-altitude intercept did not begin an "
                "internal cue-altitude fly-through "
                f"(acceptance={intercept_base_altitude}, cue={intercept_altitude}, "
                f"command={commanded_intercept_altitude})\n"
                + intercept_command
            )
        time.sleep(0.75)
        premature_command = px4.vehicle_command_status()
        premature_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", premature_command
        )
        if (
            premature_timestamp is None
            or int(premature_timestamp.group(1))
            != int(first_intercept_timestamp.group(1))
        ):
            raise AcceptanceFailure(
                "finite-altitude intercept emitted a duplicate command before crossing\n"
                + premature_command
            )
        px4.command("mavlink task reject 746 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 746),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted active target",
        )
        px4.command("param set MAV_M_INT_DWL 3")
        record(
            results,
            "trusted_source_intercept_starts_guarded_approach",
            started,
            "local mode 2 finite cue began one internal exact fly-through at cue altitude and emitted no duplicate command",
        )

        started = time.monotonic()
        px4.command("param set MAV_M_INT_RAD 500")
        px4.command("param set MAV_M_INT_DWL 0")
        time.sleep(1.0)
        arrival_position = px4.global_position_status()
        arrival_lat = output_float(arrival_position, "lat")
        arrival_lon = output_float(arrival_position, "lon")
        arrival_altitude = output_float(arrival_position, "alt")
        arrival_cue_altitude = arrival_altitude + 10.0
        arrival_cue = make_cue(
            endpoint,
            748,
            int(time.time() * 1_000_000),
            "arrival completion",
            lat=round(arrival_lat * 1e7),
            lon=round(arrival_lon * 1e7),
            alt_msl=arrival_cue_altitude,
        )
        endpoint.mav.send(arrival_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 748),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending pilot decision",
        )
        px4.command("mavlink task accept 748 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 748),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted active target",
        )
        radius_independent_command = px4.vehicle_command_status()
        radius_independent_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", radius_independent_command
        )
        if (
            re.search(r"\bcommand:\s*100001\b", radius_independent_command) is None
            or radius_independent_timestamp is None
            or (
                abs(output_float(radius_independent_command, "param1") - 500.0) < 0.001
                and abs(output_float(radius_independent_command, "param2")) < 0.001
            )
        ):
            raise AcceptanceFailure(
                "MAV_M_INT_RAD leaked into exact target-hit acceptance\n"
                + radius_independent_command
            )
        completion_status = px4.wait_target_status(
            [
                "instance_id: 748",
                "state: 2",
                "command_flags: 3",
                "intercept_phase: 3",
            ],
            90.0,
            keepalive=lambda: establish_udp_peer(endpoint),
        )
        completion_command = px4.vehicle_command_status()
        commanded_completion_altitude = output_float(completion_command, "param7")
        completion_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", completion_command
        )
        if (
            re.search(r"\bcommand:\s*100001\b", completion_command) is None
            or completion_timestamp is None
            or int(completion_timestamp.group(1))
            != int(radius_independent_timestamp.group(1))
            or abs(commanded_completion_altitude - arrival_cue_altitude) > 0.2
        ):
            raise AcceptanceFailure(
                "completion did not retain the one cue-altitude fly-through command "
                f"(cue={arrival_cue_altitude}, command={commanded_completion_altitude})\n"
                + completion_command
                + completion_status
            )
        px4.command("param set MAV_M_ACTION 0")
        aborted_completion_status = px4.wait_target_status(
            ["instance_id: 748", "intercept_phase: 4"],
            3.0,
        )
        stopped_completion_command = px4.vehicle_command_status()
        stopped_completion_timestamp = re.search(
            r"\btimestamp:\s*(\d+)\b", stopped_completion_command
        )
        if (
            stopped_completion_timestamp is None
            or int(stopped_completion_timestamp.group(1))
            <= int(completion_timestamp.group(1))
            or re.search(r"\bcommand:\s*192\b", stopped_completion_command) is None
        ):
            raise AcceptanceFailure(
                "changing MAV_M_ACTION did not permanently abort and stop intercept loiter\n"
                + stopped_completion_command
                + aborted_completion_status
            )
        px4.command("mavlink task reject 748 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 748),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted active target",
        )
        px4.command("param set MAV_M_ACTION 2")
        px4.command("param set MAV_M_INT_RAD 25")
        px4.command("param set MAV_M_INT_DWL 3")
        time.sleep(1.0)
        record(
            results,
            "trusted_source_intercept_completion",
            started,
            "guarded cue-altitude crossing and dwell latched COMPLETE without a second command; action change latched ABORTED and stopped it",
        )

        started = time.monotonic()
        nan_cue_level_before = output_float(
            px4.global_position_status(), "alt"
        )
        level_cue = make_cue(
            endpoint,
            747,
            int(time.time() * 1_000_000),
            "mode2 level",
            alt_msl=math.nan,
        )
        endpoint.mav.send(level_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 747),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending pilot decision",
        )
        px4.command("mavlink task accept 747 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 747),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted active target",
        )
        nan_status = px4.wait_target_status(
            ["instance_id: 747", "state: 2", "command_flags: 1"],
            3.0,
        )
        if "intercept_phase: 0" not in nan_status:
            raise AcceptanceFailure(
                "NaN-altitude action-2 cue unexpectedly entered two-phase intercept\n"
                + nan_status
            )
        level_command = px4.vehicle_command_status()
        commanded_nan_cue_altitude = output_float(level_command, "param7")
        if (
            re.search(r"\bcommand:\s*192\b", level_command) is None
            or abs(commanded_nan_cue_altitude - nan_cue_level_before) > 5.0
        ):
            raise AcceptanceFailure(
                "MAV_M_ACTION=2 NaN-altitude cue did not preserve aircraft AMSL "
                f"(before={nan_cue_level_before}, command={commanded_nan_cue_altitude})\n"
                + level_command
            )
        px4.command("mavlink task reject 747 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 747),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted active target",
        )
        record(
            results,
            "trusted_source_intercept_nan_altitude_stays_level",
            started,
            "local mode 2 treated NaN TARGET_CUE.alt as a level transit at acceptance-time aircraft AMSL",
        )

        owner_effect_endpoint, owner_effect_transport = connect_owner_endpoint(
            dialect, signed
        )
        try:
            started = time.monotonic()
            owner_nan_cue = make_cue(
                endpoint,
                755,
                int(time.time() * 1_000_000),
                "owner no altitude",
                alt_msl=math.nan,
            )
            endpoint.mav.send(owner_nan_cue)
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 755),
                dialect.MAVLINK_M_ACK_RECEIVED,
                "stored pending pilot decision",
            )
            owner_nan_snapshot = wait_for_owner_snapshot(
                owner_effect_endpoint, 755, "AAGS_PEND"
            )
            if (
                owner_nan_snapshot["configuration"] != 513
                or not math.isnan(float(owner_nan_snapshot["alt"]))
            ):
                raise AcceptanceFailure(
                    "owner NaN cue snapshot did not expose action ceiling 2 "
                    f"and unknown altitude: {owner_nan_snapshot!r}"
                )
            send_owner_decision(
                owner_effect_endpoint, 755, True, requested_effect=2
            )
            wait_for_command_ack(
                owner_effect_endpoint, dialect.MAV_RESULT_DENIED
            )
            px4.wait_target_status(
                [
                    "instance_id: 755",
                    "state: 1",
                    "action_mode: 2",
                    "command_flags: 0",
                    "prompt: True",
                ],
                3.0,
            )
            expect_no_ack(
                endpoint,
                dialect.MAVLINK_MSG_ID_TARGET_CUE,
                755,
                timeout=0.75,
            )
            send_owner_decision(owner_effect_endpoint, 755, False)
            wait_for_command_ack(
                owner_effect_endpoint, dialect.MAV_RESULT_ACCEPTED
            )
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 755),
                dialect.MAVLINK_M_ACK_REJECTED,
                "local operator rejected",
            )
            record(
                results,
                "owner_explicit_intercept_requires_altitude",
                started,
                "owner effect 2 was denied for the exact NaN-altitude cue; it stayed Pending until explicit owner rejection",
            )

            started = time.monotonic()
            owner_level_position = px4.global_position_status()
            owner_level_altitude = output_float(owner_level_position, "alt")
            owner_level_lat = (
                round(output_float(owner_level_position, "lat") * 1e7) + 5_000
            )
            owner_level_lon = round(
                output_float(owner_level_position, "lon") * 1e7
            )
            owner_level_cue_altitude = owner_level_altitude + 12.5
            owner_level_cue = make_cue(
                endpoint,
                756,
                int(time.time() * 1_000_000),
                "owner level travel",
                lat=owner_level_lat,
                lon=owner_level_lon,
                alt_msl=owner_level_cue_altitude,
            )
            endpoint.mav.send(owner_level_cue)
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 756),
                dialect.MAVLINK_M_ACK_RECEIVED,
                "stored pending pilot decision",
            )
            wait_for_owner_snapshot(owner_effect_endpoint, 756, "AAGS_PEND")
            send_owner_decision(
                owner_effect_endpoint, 756, True, requested_effect=1
            )
            wait_for_command_ack(
                owner_effect_endpoint, dialect.MAV_RESULT_ACCEPTED
            )
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 756),
                dialect.MAVLINK_M_ACK_ACCEPTED,
                "local operator accepted active target",
            )
            owner_level_status = px4.wait_target_status(
                [
                    "instance_id: 756",
                    "state: 2",
                    "action_mode: 1",
                    "command_flags: 1",
                    "intercept_phase: 0",
                ],
                3.0,
            )
            owner_level_command = px4.vehicle_command_status()
            commanded_owner_level_altitude = output_float(
                owner_level_command, "param7"
            )
            if (
                re.search(r"\bcommand:\s*192\b", owner_level_command) is None
                or abs(commanded_owner_level_altitude - owner_level_altitude)
                > 5.0
                or abs(
                    commanded_owner_level_altitude - owner_level_cue_altitude
                )
                < 1.0
            ):
                raise AcceptanceFailure(
                    "owner effect 1 under MAV_M_ACTION=2 did not remain level "
                    f"(before={owner_level_altitude}, cue={owner_level_cue_altitude}, "
                    f"command={commanded_owner_level_altitude})\n"
                    + owner_level_command
                    + owner_level_status
                )
            send_owner_decision(owner_effect_endpoint, 756, False)
            wait_for_command_ack(
                owner_effect_endpoint, dialect.MAV_RESULT_ACCEPTED
            )
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 756),
                dialect.MAVLINK_M_ACK_REJECTED,
                "local operator aborted",
            )
            record(
                results,
                "owner_explicit_level_under_intercept_ceiling",
                started,
                "owner effect 1 with MAV_M_ACTION=2 emitted a level DO_REPOSITION, ignored finite cue altitude, and reported action_mode 1",
            )

            started = time.monotonic()
            px4.command("param set MAV_M_INT_RAD 500")
            px4.command("param set MAV_M_INT_DWL 4")
            time.sleep(1.0)
            owner_intercept_position = px4.global_position_status()
            owner_intercept_lat = output_float(owner_intercept_position, "lat")
            owner_intercept_lon = output_float(owner_intercept_position, "lon")
            owner_intercept_altitude = output_float(
                owner_intercept_position, "alt"
            )
            owner_intercept_cue_altitude = owner_intercept_altitude + 10.0
            owner_intercept_cue = make_cue(
                endpoint,
                757,
                int(time.time() * 1_000_000),
                "owner intercept",
                lat=round(owner_intercept_lat * 1e7),
                lon=round(owner_intercept_lon * 1e7),
                alt_msl=owner_intercept_cue_altitude,
            )
            endpoint.mav.send(owner_intercept_cue)
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 757),
                dialect.MAVLINK_M_ACK_RECEIVED,
                "stored pending pilot decision",
            )
            wait_for_owner_snapshot(owner_effect_endpoint, 757, "AAGS_PEND")
            send_owner_decision(
                owner_effect_endpoint, 757, True, requested_effect=2
            )
            wait_for_command_ack(
                owner_effect_endpoint, dialect.MAV_RESULT_ACCEPTED
            )
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 757),
                dialect.MAVLINK_M_ACK_ACCEPTED,
                "local operator accepted active target",
            )
            owner_intercept_initial_status = px4.wait_target_status(
                [
                    "instance_id: 757",
                    "state: 2",
                    "action_mode: 2",
                    "command_flags: 3",
                ],
                3.0,
            )
            if re.search(
                r"\bintercept_phase:\s*[12]\b",
                owner_intercept_initial_status,
            ) is None:
                raise AcceptanceFailure(
                    "owner effect 2 did not enter intercept transit or dwell\n"
                    + owner_intercept_initial_status
                )
            owner_intercept_initial_command = px4.vehicle_command_status()
            commanded_owner_intercept_level = output_float(
                owner_intercept_initial_command, "param7"
            )
            owner_token_low = output_float(owner_intercept_initial_command, "param1")
            owner_token_high = output_float(owner_intercept_initial_command, "param2")
            owner_token = int(owner_token_low) | (int(owner_token_high) << 16)
            owner_intercept_initial_timestamp = re.search(
                r"\btimestamp:\s*(\d+)\b", owner_intercept_initial_command
            )
            if (
                re.search(
                    r"\bcommand:\s*100001\b", owner_intercept_initial_command
                )
                is None
                or owner_token_low != int(owner_token_low)
                or owner_token_high != int(owner_token_high)
                or owner_token <= 0
                or owner_token > 0x7FFFFFFF
                or owner_intercept_initial_timestamp is None
                or (abs(owner_token_low - 500.0) < 0.001 and owner_token_high == 0.0)
                or abs(
                    commanded_owner_intercept_level
                    - owner_intercept_cue_altitude
                )
                > 0.2
            ):
                raise AcceptanceFailure(
                    "owner effect 2 did not begin one cue-altitude approach\n"
                    + owner_intercept_initial_command
                )
            owner_intercept_completion_status = px4.wait_target_status(
                [
                    "instance_id: 757",
                    "action_mode: 2",
                    "command_flags: 3",
                    "intercept_phase: 3",
                ],
                90.0,
                keepalive=lambda: establish_udp_peer(endpoint),
            )
            owner_intercept_completion_command = px4.vehicle_command_status()
            commanded_owner_intercept_cue_altitude = output_float(
                owner_intercept_completion_command, "param7"
            )
            owner_intercept_completion_timestamp = re.search(
                r"\btimestamp:\s*(\d+)\b", owner_intercept_completion_command
            )
            if (
                re.search(
                    r"\bcommand:\s*100001\b", owner_intercept_completion_command
                )
                is None
                or owner_intercept_completion_timestamp is None
                or int(owner_intercept_completion_timestamp.group(1))
                != int(owner_intercept_initial_timestamp.group(1))
                or abs(
                    commanded_owner_intercept_cue_altitude
                    - owner_intercept_cue_altitude
                )
                > 0.2
            ):
                raise AcceptanceFailure(
                    "owner effect 2 did not complete without a second command\n"
                    + owner_intercept_completion_command
                    + owner_intercept_completion_status
                )
            send_owner_decision(owner_effect_endpoint, 757, False)
            wait_for_command_ack(
                owner_effect_endpoint, dialect.MAV_RESULT_ACCEPTED
            )
            try:
                owner_abort_ack = wait_for_ack(
                    endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 757
                )
                owner_abort_reason = "local operator aborted active target"

            except AcceptanceFailure:
                # MAVLink UDP is lossy. Replay the immutable cue after the
                # committed abort and require its terminal idempotent ACK.
                endpoint.mav.send(owner_intercept_cue)
                owner_abort_ack = wait_for_ack(
                    endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 757
                )
                owner_abort_reason = "duplicate idempotent replay"

            assert_ack(
                owner_abort_ack,
                dialect.MAVLINK_M_ACK_REJECTED,
                owner_abort_reason,
            )
            px4.command("param set MAV_M_INT_RAD 25")
            px4.command("param set MAV_M_INT_DWL 3")
            record(
                results,
                "owner_explicit_intercept_guarded_approach",
                started,
                "owner effect 2 entered one guarded cue-altitude approach, proved the exact target crossing, dwelled, and reported complete without a second command",
            )
        finally:
            owner_effect_transport.close()

        px4.command("param set MAV_M_ACTION 0")
        time.sleep(0.3)

        px4.command("commander land")
        px4.wait_land_detected(["landed: True"], 90.0)
        px4.command("commander disarm")
        px4.wait_vehicle_status(["arming_state: 1"], 5.0)
        px4.command(f"param set MAV_M_MAX_AGE {max_age}")
        px4.command("param save")
        time.sleep(0.25)

        started = time.monotonic()
        identity_expiry = now_usec / 1_000_000.0 + max_age + 0.5
        if time.time() < identity_expiry:
            time.sleep(identity_expiry - time.time())
        stale_identity_cue = make_cue(
            endpoint,
            743,
            int(time.time() * 1_000_000),
            "stale identity",
        )
        endpoint.mav.send(stale_identity_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 743),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        stale_identity_status = px4.status()
        if (
            "instance_id: 743" not in stale_identity_status
            or "track_identity_valid: False" not in stale_identity_status
        ):
            raise AcceptanceFailure(
                "expired cached TRACK_IDENTITY enriched a new cue\n"
                + stale_identity_status
            )
        px4.command("mavlink task reject 743 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 743),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator rejected",
        )
        replacement_now = int(time.time() * 1_000_000)
        endpoint.send_frozen(
            make_track_identity(endpoint, replacement_now, REPLACEMENT_TRACK_UID)
        )
        time.sleep(0.1)
        endpoint.mav.send(
            make_cue(endpoint, 744, replacement_now + 100_000, "new identity")
        )
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 744),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        replacement_status = px4.status()
        replacement_uid_text = (
            "track_uid: [" + ", ".join(str(value) for value in REPLACEMENT_TRACK_UID.bytes) + "]"
        )
        if (
            "instance_id: 744" not in replacement_status
            or "track_identity_valid: True" not in replacement_status
            or replacement_uid_text not in replacement_status
        ):
            raise AcceptanceFailure(
                "fresh identity did not replace an expired target-set cache entry\n"
                + replacement_status
            )
        px4.command("mavlink task reject 744 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 744),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator rejected",
        )
        record(
            results,
            "track_identity_cache_freshness",
            started,
            "expired identity did not enrich; fresh different UID replaced stale target-set entry",
        )

        started = time.monotonic()
        handover = make_handover(endpoint, int(time.time() * 1_000_000))
        handover_frame = endpoint.send_frozen(handover)
        require_trimmed_payload(handover_frame, 76, 207, "TARGET_HANDOVER")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        record(
            results,
            "final_handover_receipt",
            started,
            "trimmed TARGET_HANDOVER 53002 correlated by target_set_id 9002",
        )

        started = time.monotonic()
        ignored = 65535
        endpoint.mav.rc_channels_override_send(
            VEHICLE_SYSTEM,
            VEHICLE_COMPONENT,
            ignored,
            ignored,
            ignored,
            ignored,
            1500,
            ignored,
            ignored,
            ignored,
        )
        time.sleep(0.2)
        endpoint.mav.rc_channels_override_send(
            VEHICLE_SYSTEM,
            VEHICLE_COMPONENT,
            ignored,
            ignored,
            ignored,
            ignored,
            1800,
            ignored,
            ignored,
            ignored,
        )
        time.sleep(0.3)
        input_rc = px4.input_rc_status()
        values_match = re.search(r"values:\s*\[([^\]]+)\]", input_rc)
        values = (
            [int(value.strip()) for value in values_match.group(1).split(",")]
            if values_match is not None
            else []
        )
        if "input_source: 6" not in input_rc or len(values) < 5 or values[4] != 1800:
            raise AcceptanceFailure(
                "RC override did not reach input_rc as MAVLink channel 5=1800\n"
                + input_rc
            )
        endpoint.mav.send(handover)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "duplicate idempotent replay",
        )
        endpoint.mav.rc_channels_override_send(
            VEHICLE_SYSTEM,
            VEHICLE_COMPONENT,
            ignored,
            ignored,
            ignored,
            ignored,
            1500,
            ignored,
            ignored,
            ignored,
        )
        time.sleep(0.2)
        endpoint.mav.rc_channels_override_send(
            VEHICLE_SYSTEM,
            VEHICLE_COMPONENT,
            ignored,
            ignored,
            ignored,
            ignored,
            1200,
            ignored,
            ignored,
            ignored,
        )
        time.sleep(0.3)
        input_rc = px4.input_rc_status()
        values_match = re.search(r"values:\s*\[([^\]]+)\]", input_rc)
        values = (
            [int(value.strip()) for value in values_match.group(1).split(",")]
            if values_match is not None
            else []
        )
        if "input_source: 6" not in input_rc or len(values) < 5 or values[4] != 1200:
            raise AcceptanceFailure(
                "RC override did not reach input_rc as MAVLink channel 5=1200\n"
                + input_rc
            )
        endpoint.mav.send(handover)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "duplicate idempotent replay",
        )
        record(
            results,
            "remote_rc_override_cannot_accept",
            started,
            "MAVLink RC center-to-high and center-to-low overrides left handover pending",
        )

        state_file = rootfs / "mavlink_m_state.bin"
        if not state_file.is_file() or state_file.stat().st_size == 0:
            raise AcceptanceFailure("durable MAVLink-M state file was not created")

        started = time.monotonic()
        px4.stop()
        drain(endpoint)
        restart_mark = px4.start()
        px4.add_test_link()
        establish_udp_peer(endpoint)
        wait_for_heartbeat(endpoint)
        px4.wait_for("restored MAVLink-M assignment state", restart_mark, 5.0)
        endpoint.mav.send(handover)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "duplicate idempotent replay",
        )
        restored_status = px4.status()
        if "restored: True" not in restored_status:
            raise AcceptanceFailure("restored state was not exposed\n" + restored_status)
        record(
            results,
            "restart_restore_and_idempotent_replay",
            started,
            "pending finalized handover restored and replay ACKed",
        )

        started = time.monotonic()
        px4.command("mavlink task reject 9002 53002")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator rejected",
        )
        record(results, "local_reject", started, "ACK REJECTED returned for handover 9002")

        started = time.monotonic()
        zero_time = make_cue(endpoint, 734, 0, "zero timestamp")
        endpoint.mav.send(zero_time)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 734),
            dialect.MAVLINK_M_ACK_FAILED,
            "timestamp invalid",
        )
        record(
            results,
            "unset_timestamp_rejected",
            started,
            "time_usec=0 rejected independently of replay-window policy",
        )

        started = time.monotonic()
        expiring = make_cue(
            endpoint,
            733,
            int(time.time() * 1_000_000),
            "expiring cue",
            cue_type=2,
        )
        endpoint.mav.send(expiring)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 733),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        expired = wait_for_ack(
            endpoint,
            dialect.MAVLINK_MSG_ID_TARGET_CUE,
            733,
            max_age + 4.0,
        )
        assert_ack(expired, dialect.MAVLINK_M_ACK_EXPIRED, "assignment expired")
        px4.command("mavlink task accept 733 53001")
        expect_no_ack(
            endpoint,
            dialect.MAVLINK_MSG_ID_TARGET_CUE,
            733,
            timeout=1.0,
        )
        record(
            results,
            "derived_cue_expiry",
            started,
            f"finalized cue expired from MAV_M_MAX_AGE={max_age} and late accept could not revive it",
        )

        started = time.monotonic()
        for command in [
            f"param set MAV_M_CTL_INST {TEST_MAVLINK_INSTANCE}",
            f"param set MAV_M_CTL_SYS {SOURCE_SYSTEM}",
            f"param set MAV_M_CTL_CMP {SOURCE_COMPONENT}",
        ]:
            px4.command(command)
        px4.command("param set MAV_M_CTL_LNK 7")
        px4.command("param set MAV_M_SAME_EP 1")
        time.sleep(0.75)
        drain(endpoint)
        local_cue_id = 764
        endpoint.mav.send(
            make_cue(
                endpoint,
                local_cue_id,
                int(time.time() * 1_000_000),
                "local owner",
            )
        )
        assert_ack(
            wait_for_ack(
                endpoint,
                dialect.MAVLINK_MSG_ID_TARGET_CUE,
                local_cue_id,
            ),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        local_snapshot = wait_for_owner_snapshot(
            endpoint, local_cue_id, "AAGS_PEND"
        )
        expected_local_source = (SOURCE_SYSTEM << 8) | SOURCE_COMPONENT
        if local_snapshot.get("source") != expected_local_source:
            raise AcceptanceFailure(
                "same-endpoint owner snapshot did not preserve the exact "
                f"local source: {local_snapshot!r}"
            )
        send_owner_decision(endpoint, local_cue_id, True)
        wait_for_colocated_accept_acks(endpoint, local_cue_id)
        wait_for_owner_snapshot(endpoint, local_cue_id, "AAGS_ACTV")
        px4.command(f"mavlink task reject {local_cue_id} 53001")
        assert_ack(
            wait_for_ack(
                endpoint,
                dialect.MAVLINK_MSG_ID_TARGET_CUE,
                local_cue_id,
            ),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
        )
        record(
            results,
            "same_endpoint_local_owner_acceptance",
            started,
            "one exact source/link received the cue, exposed Pending, required explicit Accept, returned both ACKs, and became ACTIVE",
        )

        # Restore the default separated-authority layout before subsequent
        # corrupt-parameter checks.
        for command in [
            "param set MAV_M_SAME_EP 0",
            f"param set MAV_M_CTL_INST {CONTROL_MAVLINK_INSTANCE}",
            f"param set MAV_M_CTL_SYS {OWNER_SYSTEM}",
            f"param set MAV_M_CTL_CMP {OWNER_COMPONENT}",
            f"param set MAV_M_CTL_LNK {CONTROL_SIGNING_LINK_ID}",
        ]:
            px4.command(command)
        time.sleep(0.75)

        started = time.monotonic()
        px4.command("param set MAV_M_SRC_SYS -1")
        px4.command("param set MAV_M_CTL_SYS -1")
        time.sleep(0.75)
        wildcard_parameters = px4.parameter_status("MAV_M_SRC_SYS") + px4.parameter_status("MAV_M_CTL_SYS")
        for parameter in ("MAV_M_SRC_SYS", "MAV_M_CTL_SYS"):
            if re.search(
                rf"\b{parameter}\b[^\n]*:\s*-1\s*$",
                wildcard_parameters,
                re.MULTILINE,
            ) is None:
                raise AcceptanceFailure(
                    f"{parameter} wildcard was not retained\n{wildcard_parameters}"
                )

        px4.command("param set MAV_M_ACTION 1")
        px4.command("commander disarm")
        px4.wait_vehicle_status(["arming_state: 1"], 5.0)
        px4.command("commander takeoff")
        px4.wait_vehicle_status(
            ["arming_state: 2", "failsafe: False", "failure_detector_status: 0"],
            8.0,
        )
        wildcard_airborne_fields = [
            "freefall: False",
            "ground_contact: False",
            "maybe_landed: False",
            "landed: False",
        ]
        px4.wait_land_detected(wildcard_airborne_fields, 20.0)
        px4.wait_vehicle_status(
            ["arming_state: 2", "nav_state: 4", "failsafe: False"],
            20.0,
        )

        wildcard_source = Endpoint(
            dialect,
            transport,
            WILDCARD_SOURCE_SYSTEM,
            SOURCE_COMPONENT,
            SIGNING_KEY if signed else None,
            7,
        )
        wildcard_wrong_component = Endpoint(
            dialect,
            transport,
            WILDCARD_SOURCE_SYSTEM,
            SOURCE_COMPONENT + 1,
            SIGNING_KEY if signed else None,
            7,
        )
        wildcard_cue_id = 761
        wildcard_cue = make_cue(
            wildcard_source,
            wildcard_cue_id,
            int(time.time() * 1_000_000),
            "wildcard source",
        )
        wildcard_source.send_frozen(wildcard_cue)
        wildcard_received = wait_for_ack(
            endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, wildcard_cue_id
        )
        assert_ack(
            wildcard_received,
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
            WILDCARD_SOURCE_SYSTEM,
        )
        if wildcard_received.origin_sysid != WILDCARD_SOURCE_SYSTEM:
            raise AcceptanceFailure(
                "wildcard cue ACK did not preserve its actual source system"
            )

        # A second allowed system may not reuse an in-flight cue ID because
        # the owner-decision command has no source-system correlation field.
        endpoint.mav.send(
            make_cue(
                endpoint,
                wildcard_cue_id,
                int(time.time() * 1_000_000),
                "ambiguous source",
                cue_type=2,
            )
        )
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, wildcard_cue_id),
            dialect.MAVLINK_M_ACK_FAILED,
            "ambiguous across sources",
        )

        wildcard_wrong_component.mav.send(
            make_cue(
                wildcard_wrong_component,
                762,
                int(time.time() * 1_000_000),
                "wrong component",
                cue_type=2,
            )
        )
        expect_no_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 762)

        wrong_route_transport = UdpTransport(
            ("127.0.0.1", FORWARD_INGRESS_TEST_UDP_PORT),
            ("127.0.0.1", FORWARD_INGRESS_PX4_UDP_PORT),
        )
        wrong_route_source = Endpoint(
            dialect,
            wrong_route_transport,
            WILDCARD_SOURCE_SYSTEM,
            SOURCE_COMPONENT,
        )
        try:
            establish_udp_peer(wrong_route_source)
            wrong_route_source.mav.send(
                make_cue(
                    wrong_route_source,
                    763,
                    int(time.time() * 1_000_000),
                    "wrong route",
                    cue_type=2,
                )
            )
            expect_no_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 763)
        finally:
            wrong_route_transport.close()

        wildcard_owner_transport = UdpTransport(
            ("127.0.0.1", CONTROL_AAGS_UDP_PORT),
            ("127.0.0.1", CONTROL_PX4_UDP_PORT),
        )
        wildcard_owner = Endpoint(
            dialect,
            wildcard_owner_transport,
            WILDCARD_OWNER_SYSTEM,
            OWNER_COMPONENT,
            SIGNING_KEY if signed else None,
            CONTROL_SIGNING_LINK_ID,
        )
        wrong_owner_component = Endpoint(
            dialect,
            wildcard_owner_transport,
            WILDCARD_OWNER_SYSTEM,
            OWNER_COMPONENT + 1,
            SIGNING_KEY if signed else None,
            CONTROL_SIGNING_LINK_ID,
        )
        wildcard_reject_owner = Endpoint(
            dialect,
            wildcard_owner_transport,
            WILDCARD_REJECT_SYSTEM,
            OWNER_COMPONENT,
            SIGNING_KEY if signed else None,
            CONTROL_SIGNING_LINK_ID,
        )
        try:
            establish_udp_peer(wildcard_owner)
            wait_for_owner_snapshot(wildcard_owner, wildcard_cue_id, "AAGS_PEND")
            send_owner_decision(wrong_owner_component, wildcard_cue_id, True)
            wait_for_command_ack(wildcard_owner, dialect.MAV_RESULT_DENIED)
            send_owner_decision(wildcard_owner, wildcard_cue_id, True)
            wait_for_command_ack(wildcard_owner, dialect.MAV_RESULT_ACCEPTED)
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, wildcard_cue_id),
                dialect.MAVLINK_M_ACK_ACCEPTED,
                "local operator accepted",
                WILDCARD_SOURCE_SYSTEM,
            )
            wait_for_owner_snapshot(wildcard_owner, wildcard_cue_id, "AAGS_ACTV")
            px4.wait_target_status(
                [
                    f"instance_id: {wildcard_cue_id}",
                    "state: 2",
                    "command_flags: 1",
                    f"source_system: {WILDCARD_SOURCE_SYSTEM}",
                ],
                3.0,
            )
            wildcard_command = px4.vehicle_command_status()
            for pattern in (
                r"\bcommand:\s*192\b",
                rf"\bsource_system:\s*{WILDCARD_SOURCE_SYSTEM}\b",
                rf"\bsource_component:\s*{SOURCE_COMPONENT}\b",
            ):
                if re.search(pattern, wildcard_command) is None:
                    raise AcceptanceFailure(
                        "wildcard-authorized movement did not preserve the actual source\n"
                        + wildcard_command
                    )
            send_owner_decision(wildcard_reject_owner, wildcard_cue_id, False)
            wait_for_command_ack(wildcard_owner, dialect.MAV_RESULT_ACCEPTED)
            assert_ack(
                wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, wildcard_cue_id),
                dialect.MAVLINK_M_ACK_REJECTED,
                "local operator aborted",
                WILDCARD_SOURCE_SYSTEM,
            )
        finally:
            wildcard_owner_transport.close()

        px4.command(f"param set MAV_M_SRC_SYS {SOURCE_SYSTEM}")
        px4.command(f"param set MAV_M_CTL_SYS {OWNER_SYSTEM}")
        time.sleep(0.75)
        record(
            results,
            "system_wildcards_preserve_component_and_route",
            started,
            "system 253 published level movement on the cue route, systems 252 and 251 accepted and aborted it on the owner route, and wrong component, wrong route, and ambiguous cue ID failed closed",
        )

        started = time.monotonic()
        px4.command("param set MAV_M_MAX_AGE -1")
        time.sleep(0.3)
        corrected_age = px4.parameter_status("MAV_M_MAX_AGE")
        if re.search(r"\bMAV_M_MAX_AGE\b[^\n]*:\s*30\s*$", corrected_age, re.MULTILINE) is None:
            raise AcceptanceFailure(
                "negative MAV_M_MAX_AGE was not corrected to the safe default\n"
                + corrected_age
            )
        px4.set_invalid_endpoint_parameter("MAV_M_SRC_SYS", -2)
        endpoint.mav.send(
            make_cue(endpoint, 750, int(time.time() * 1_000_000), "bad sys")
        )
        expect_no_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 750)
        px4.restore_endpoint_parameter("MAV_M_SRC_SYS", SOURCE_SYSTEM)
        px4.set_invalid_endpoint_parameter("MAV_M_SRC_CMP", 446)
        endpoint.mav.send(
            make_cue(endpoint, 751, int(time.time() * 1_000_000), "bad comp")
        )
        expect_no_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 751)
        px4.restore_endpoint_parameter("MAV_M_SRC_CMP", SOURCE_COMPONENT)
        px4.set_invalid_endpoint_parameter("MAV_M_INST", 6)
        endpoint.mav.send(
            make_cue(endpoint, 752, int(time.time() * 1_000_000), "bad route")
        )
        expect_no_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 752)
        px4.restore_endpoint_parameter("MAV_M_INST", TEST_MAVLINK_INSTANCE)
        px4.command(f"param set MAV_M_MAX_AGE {max_age}")
        time.sleep(0.3)
        record(
            results,
            "corrupt_parameter_aliases_fail_closed",
            started,
            "negative age corrected; -2/446 source IDs and instance 6 could not modulo-alias a valid endpoint",
        )

        return {
            "schema": "px4.dronecode-mavlink-m.sitl-acceptance.v2",
            "status": "PASS",
            "profile_id": PROFILE_ID,
            "profile_version": PROFILE_VERSION,
            "core_xml_sha256": CORE_XML_SHA256,
            "field_release": True,
            "live_aags_transmit": True,
            "inert_only": True,
            "signing_required": signed,
            "tests": results,
        }
    except AcceptanceFailure as error:
        raise AcceptanceFailure(
            f"{error}\n--- PX4 console tail ---\n"
            + px4.text_since(max(0, px4.mark() - 120))[-12000:]
        ) from error
    finally:
        px4.stop()
        transport.close()


def parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[2]
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--px4-binary",
        type=Path,
        default=root / "build/px4_sitl_default/bin/px4",
    )
    result.add_argument(
        "--etc",
        type=Path,
        default=root / "build/px4_sitl_default/etc",
    )
    result.add_argument("--rootfs", type=Path, help="isolated run directory; default is temporary")
    result.add_argument("--keep-rootfs", action="store_true")
    result.add_argument("--json-output", type=Path)
    result.add_argument("--max-age", type=int, default=12)
    result.add_argument(
        "--test-aags-port",
        type=int,
        default=TEST_AAGS_UDP_PORT,
        help="UDP bind/output port for the isolated cue-source endpoint",
    )
    result.add_argument(
        "--signed",
        action="store_true",
        help="exercise physical mode with a temporary MAVLink 2 signing key",
    )
    return result


def main() -> int:
    global TEST_AAGS_UDP_PORT
    args = parser().parse_args()
    if not 1024 <= args.test_aags_port <= 65535:
        raise SystemExit("--test-aags-port must be between 1024 and 65535")
    TEST_AAGS_UDP_PORT = args.test_aags_port
    binary = args.px4_binary.resolve()
    etc = args.etc.resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise SystemExit(f"PX4 SITL binary is missing or not executable: {binary}")
    if not (etc / "init.d-posix/rcS").is_file():
        raise SystemExit(f"PX4 SITL etc tree is incomplete: {etc}")
    if args.max_age < 8 or args.max_age > 60:
        raise SystemExit("--max-age must be between 8 and 60 seconds")

    temporary: tempfile.TemporaryDirectory[str] | None = None
    if args.rootfs:
        rootfs = args.rootfs.resolve()
        rootfs.mkdir(parents=True, exist_ok=True)
    elif args.keep_rootfs:
        rootfs = Path(tempfile.mkdtemp(prefix="px4-mavlink-m-sitl-"))
        print(f"rootfs={rootfs}")
    else:
        temporary = tempfile.TemporaryDirectory(prefix="px4-mavlink-m-sitl-")
        rootfs = Path(temporary.name)

    try:
        report = run(binary, etc, rootfs, args.max_age, args.signed)
        encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
        if args.json_output:
            args.json_output.parent.mkdir(parents=True, exist_ok=True)
            args.json_output.write_text(encoded, encoding="utf-8")
        print(encoded, end="")
        return 0
    except (AcceptanceFailure, OSError, subprocess.SubprocessError) as error:
        print(f"acceptance=FAIL {error}")
        return 1
    finally:
        if temporary is not None:
            temporary.cleanup()

if __name__ == "__main__":
    raise SystemExit(main())
