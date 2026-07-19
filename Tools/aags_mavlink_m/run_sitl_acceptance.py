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
import uuid

from endpoint_tool import Endpoint, UdpTransport, fixed_text, load_dialect, text_field


SOURCE_SYSTEM = 255
SOURCE_COMPONENT = 190
OWNER_SYSTEM = 254
OWNER_COMPONENT = 190
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
        }
        missing = [
            f"{name}={value}"
            for name, value in expected.items()
            if re.search(rf"\b{re.escape(name)}\b[^\n]*:\s*{value}\s*$", output, re.MULTILINE) is None
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

    def wait_target_status(self, required: list[str], timeout: float) -> str:
        deadline = time.monotonic() + timeout
        last_status = ""
        while time.monotonic() < deadline:
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
        mark = self.mark()
        self.command(f"param set {name} {value}")
        self.wait_for("restored MAVLink-M assignment state", mark, 5.0)
        time.sleep(0.5)

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

def send_owner_decision(endpoint: Endpoint, cue_id: int, accept: bool) -> None:
    endpoint.mav.command_long_send(
        VEHICLE_SYSTEM,
        VEHICLE_COMPONENT,
        endpoint.dialect.MAV_CMD_USER_1,
        0,
        1 if accept else 2,
        cue_id & 0xFFFF,
        (cue_id >> 16) & 0xFFFF,
        endpoint.dialect.MAVLINK_MSG_ID_TARGET_CUE,
        0,
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
                elif name == expected_state_name:
                    state_cue = value & 0xFFFFFFFF
                    required = {
                        "cue_id",
                        "lat",
                        "lon",
                        "alt",
                        "source",
                        "configuration",
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


def assert_ack(message, result: int, reason_contains: str) -> None:
    reason = text_field(message.reason)
    if message.result != result or reason_contains not in reason:
        raise AcceptanceFailure(
            f"unexpected ACK result={message.result} reason={reason!r}; "
            f"wanted result={result} containing {reason_contains!r}"
        )
    if message.origin_sysid != SOURCE_SYSTEM or message.ack_sysid != VEHICLE_SYSTEM:
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
    cue_type: int = 1,
    alt_msl: float = 50.0,
):
    return endpoint.mav.target_cue_encode(
        now_usec,
        cue_id,
        TRACK_SET,
        lat,
        -737578000,
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
        expect_no_ack(
            endpoint,
            dialect.MAVLINK_MSG_ID_TARGET_CUE,
            732,
            timeout=1.0,
        )
        px4.wait_target_status(
            [
                "instance_id: 732",
                "state: 1",
                "queue_depth: 1",
                "command_flags: 0",
                "prompt: True",
            ],
            3.0,
        )
        endpoint.mav.send(cue_one)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "duplicate idempotent replay",
        )
        px4.command("mavlink task reject 731 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
        )
        px4.wait_target_status(
            [
                "instance_id: 732",
                "state: 1",
                "queue_depth: 1",
                "command_flags: 0",
                "prompt: True",
            ],
            3.0,
        )
        endpoint.mav.send(blocked_cue)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "duplicate idempotent replay",
        )
        px4.command("mavlink task accept 732 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted active target",
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
        px4.command("mavlink task reject 732 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
        )
        record(
            results,
            "active_assignment_requires_fresh_acceptance",
            started,
            "cue B stayed RECEIVED/Pending while A was active and after A aborted; a fresh B acceptance then succeeded",
        )

        px4.command("param set MAV_M_ACTION 1")
        time.sleep(0.3)
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
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted",
        )
        unsafe_status = px4.status()
        if "instance_id: 735" not in unsafe_status or "command_flags: 0" not in unsafe_status:
            raise AcceptanceFailure(
                "ACTION=1 moved or marked a vehicle that was not already armed in Hold\n"
                + unsafe_status
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
        if "instance_id: 735" not in delayed_status or "command_flags: 0" not in delayed_status:
            raise AcceptanceFailure(
                "previously accepted cue dispatched after a later armed-Hold transition\n"
                + delayed_status
            )
        px4.command("mavlink task reject 735 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 735),
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
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "local operator accepted",
        )
        ground_status = px4.wait_target_status(
            ["instance_id: 745", "state: 2", "command_flags: 0"],
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
            or "command_flags: 0" not in delayed_ground_status
        ):
            raise AcceptanceFailure(
                "ground-accepted cue dispatched after a later takeoff\n"
                + delayed_ground_status
            )
        px4.command("mavlink task reject 745 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 745),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
        )
        record(
            results,
            "reposition_requires_existing_safe_flight_state",
            started,
            "disarmed and armed-ground acceptances stayed inert through later arming and takeoff",
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
        px4.command("param set MAV_M_ACTION 0")
        time.sleep(0.3)
        if re.search(
            r"\bMAV_M_ACTION\b[^\n]*:\s*0\s*$",
            px4.parameter_status("MAV_M_ACTION"),
            re.MULTILINE,
        ) is None:
            raise AcceptanceFailure("MAV_M_ACTION did not revoke future commands")
        px4.command("mavlink task reject 736 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 736),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted",
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
                "revoking MAV_M_ACTION prevented safe reposition cancellation\n"
                + cancellation_command
            )

        started = time.monotonic()
        px4.command("param set MAV_M_ACTION 2")
        time.sleep(0.3)
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
        px4.wait_target_status(
            ["instance_id: 746", "state: 2", "command_flags: 1"],
            3.0,
        )
        intercept_command = px4.vehicle_command_status()
        commanded_intercept_altitude = output_float(intercept_command, "param7")
        if (
            re.search(r"\bcommand:\s*192\b", intercept_command) is None
            or abs(commanded_intercept_altitude - intercept_altitude) > 0.2
        ):
            raise AcceptanceFailure(
                "MAV_M_ACTION=2 finite-altitude intercept did not use cue AMSL "
                f"(cue={intercept_altitude}, command={commanded_intercept_altitude})\n"
                + intercept_command
            )
        px4.command("mavlink task reject 746 53001")
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 746),
            dialect.MAVLINK_M_ACK_REJECTED,
            "local operator aborted active target",
        )
        record(
            results,
            "trusted_source_intercept_uses_finite_cue_altitude",
            started,
            "local mode 2 used finite TARGET_CUE.alt only after explicit local acceptance",
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
        px4.wait_target_status(
            ["instance_id: 747", "state: 2", "command_flags: 1"],
            3.0,
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
        px4.command("param set MAV_M_ACTION 0")
        time.sleep(0.3)
        record(
            results,
            "trusted_source_intercept_nan_altitude_stays_level",
            started,
            "local mode 2 treated NaN TARGET_CUE.alt as a level transit at acceptance-time aircraft AMSL",
        )

        px4.command("commander land")
        px4.wait_land_detected(["landed: True"], 30.0)
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
        local_cue_id = 760
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
        px4.command("param set MAV_M_MAX_AGE -1")
        time.sleep(0.3)
        corrected_age = px4.parameter_status("MAV_M_MAX_AGE")
        if re.search(r"\bMAV_M_MAX_AGE\b[^\n]*:\s*30\s*$", corrected_age, re.MULTILINE) is None:
            raise AcceptanceFailure(
                "negative MAV_M_MAX_AGE was not corrected to the safe default\n"
                + corrected_age
            )
        px4.set_invalid_endpoint_parameter("MAV_M_SRC_SYS", -1)
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
            "negative age corrected; -1/446 source IDs and instance 6 could not modulo-alias a valid endpoint",
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
