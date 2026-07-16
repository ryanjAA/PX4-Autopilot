#!/usr/bin/env python3
"""Run the live UDP AAGS MAVLink-M acceptance suite against PX4 SITL.

This starts the already-built PX4 binary in an isolated rootfs, configures the
private inert endpoint, exchanges exactly addressed MAVLink 2 datagrams,
restarts PX4 against the same durable state, and reports machine-readable
results. It never commands navigation, flight mode, arming, or payloads.
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
WRONG_SOURCE_SYSTEM = 254
SITL_INSTANCE = 1
VEHICLE_SYSTEM = SITL_INSTANCE + 1
VEHICLE_COMPONENT = 1
TEST_MAVLINK_INSTANCE = 5
TEST_PX4_UDP_PORT = 18671
TEST_AAGS_UDP_PORT = 14551
TRACK_SET = 45
TRACK_UID = uuid.UUID("00112233-4455-6677-8899-aabbccddeeff")
PROFILE_ID = "aags-private-inert-54xxx"
PROFILE_VERSION = "private-inert-2026-07-16-v1"
CORE_XML_SHA256 = "699b9b9369180925b06b8b8c4efcb26f1f3323970d9e79ebfa2bef69692ff7a9"
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
            "param set MAV_M_RC_CH 5",
            "param set MAV_M_RC_REJ 1300",
            "param set MAV_M_RC_ACC 1700",
            f"param set MAV_M_MAX_AGE {max_age}",
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
            "MAV_M_MAX_AGE": max_age,
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
        """Add one test-owned GCS link after the five standard SITL links."""
        self.command(
            f"mavlink start -x -u {TEST_PX4_UDP_PORT} "
            f"-o {TEST_AAGS_UDP_PORT} -r 4000000 -f"
        )
        time.sleep(0.75)
        self.command(f"mavlink stream -u {TEST_PX4_UDP_PORT} -s HEARTBEAT -r 2")
        time.sleep(0.25)

    def status(self) -> str:
        mark = self.mark()
        self.command("listener mavlink_m_target_status 1")
        self.wait_for("track_identity_valid:", mark, 5.0)
        time.sleep(0.1)
        return self.text_since(mark)

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


def wait_for_px4_capability(endpoint: Endpoint, timeout: float = 7.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.5, deadline - time.monotonic())):
            if (
                message.get_type() == "MAVLINK_M_CAPABILITY"
                and message.endpoint_system == VEHICLE_SYSTEM
                and message.endpoint_component == VEHICLE_COMPONENT
                and message.endpoint_type == endpoint.dialect.MAVLINK_M_ENDPOINT_PX4_PILOT
            ):
                return message
    raise AcceptanceFailure("PX4 did not advertise the private inert capability")


def drain(endpoint: Endpoint, duration: float = 0.25) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        endpoint.receive(min(0.05, deadline - time.monotonic()))


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


def assert_ack(message, result: int, reason_contains: str) -> None:
    reason = text_field(message.reason)
    if message.result != result or reason_contains not in reason:
        raise AcceptanceFailure(
            f"unexpected ACK result={message.result} reason={reason!r}; "
            f"wanted result={result} containing {reason_contains!r}"
        )
    if message.target_system != SOURCE_SYSTEM or message.target_component != SOURCE_COMPONENT:
        raise AcceptanceFailure(
            f"ACK was not addressed back to {SOURCE_SYSTEM}/{SOURCE_COMPONENT}"
        )


def make_track_identity(endpoint: Endpoint, now_usec: int):
    return endpoint.mav.track_identity_encode(
        now_usec,
        TRACK_UID.bytes,
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
        1,
        5,
        255,
        0,
        0,
        2,
    )


def make_cue(endpoint: Endpoint, cue_id: int, now_usec: int, valid_until_usec: int,
             name: str, lat: int = 454671000):
    return endpoint.mav.target_cue_encode(
        now_usec,
        cue_id,
        TRACK_SET,
        lat,
        -737578000,
        50.0,
        math.nan,
        math.nan,
        math.nan,
        0.8,
        1,
        2,
        0,
        0,
        fixed_text(name, 20),
        valid_until_usec,
        VEHICLE_SYSTEM,
        VEHICLE_COMPONENT,
        TRACK_UID.bytes,
    )


def make_handover(endpoint: Endpoint, now_usec: int):
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
        TRACK_SET,
        fixed_text("handover", 50),
        fixed_text("", 50),
        0.8,
        bytes(8),
        0,
        0,
        0,
        9002,
        VEHICLE_SYSTEM,
        VEHICLE_COMPONENT,
        TRACK_UID.bytes,
    )


def make_control(endpoint: Endpoint, control_id: int, task_msgid: int,
                 task_instance: int, action: int, replacement_instance: int = 0):
    return endpoint.mav.mavlink_m_task_control_encode(
        int(time.time() * 1_000_000), task_msgid, task_instance, control_id,
        replacement_instance, 0, VEHICLE_SYSTEM, VEHICLE_COMPONENT, action,
        fixed_text("SITL inert control", 40),
    )


def wait_for_task_status(endpoint: Endpoint, task_msgid: int, task_instance: int,
                         state: int, timeout: float = 4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in endpoint.receive(min(0.25, deadline - time.monotonic())):
            if (
                message.get_type() == "MAVLINK_M_TASK_STATUS"
                and message.task_msgid == task_msgid
                and message.task_instance == task_instance
                and message.state == state
                and message.target_system == SOURCE_SYSTEM
                and message.target_component == SOURCE_COMPONENT
            ):
                return message
    raise AcceptanceFailure(
        f"no task status state={state} for msgid={task_msgid} instance={task_instance}"
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


def run(binary: Path, etc: Path, rootfs: Path, max_age: int, signed: bool) -> dict:
    dialect = load_dialect()
    transport = UdpTransport(
        ("127.0.0.1", TEST_AAGS_UDP_PORT),
        ("127.0.0.1", TEST_PX4_UDP_PORT),
    )
    endpoint = Endpoint(
        dialect, transport, SOURCE_SYSTEM, SOURCE_COMPONENT,
        SIGNING_KEY if signed else None, 7,
    )
    wrong_sender = dialect.MAVLink(
        transport, srcSystem=WRONG_SOURCE_SYSTEM, srcComponent=SOURCE_COMPONENT
    )
    px4 = Px4Sitl(binary, etc, rootfs, SITL_INSTANCE)
    results: list[dict] = []
    cue_time = 0
    cue_one = None
    cue_two = None
    cancel_frame = None
    cancel_message = None
    try:
        if signed:
            key_path = rootfs / "mavlink_m_signing.key"
            key_path.write_bytes(SIGNING_KEY)
            key_path.chmod(0o600)
        started = time.monotonic()
        px4.start()
        px4.add_test_link()
        establish_udp_peer(endpoint)
        if not signed:
            wait_for_heartbeat(endpoint)
        px4.configure(max_age, signed)
        wait_for_px4_capability(endpoint)
        record(results, "sitl_start_and_endpoint_configuration", started,
               f"isolated UDP 18671/14551; signing={'required' if signed else 'lab-unsigned'}")

        started = time.monotonic()
        cue_time = int(time.time() * 1_000_000)
        cue_expiry = cue_time + max_age * 1_000_000
        endpoint.send_capability()
        time.sleep(0.05)
        cue_one = make_cue(endpoint, 731, cue_time + 100_000, cue_expiry, "relay cue")
        endpoint.mav.send(cue_one)
        ack = wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731)
        assert_ack(ack, dialect.MAVLINK_M_ACK_RECEIVED, "stored pending")
        if ack.origin_sysid != SOURCE_SYSTEM or ack.ack_sysid != VEHICLE_SYSTEM:
            raise AcceptanceFailure("receipt ACK did not preserve sender/vehicle correlation")
        record(results, "capability_gated_addressed_cue_receipt", started,
               f"AAGS 255/190 to PX4 {VEHICLE_SYSTEM}/1; sensing origin 1")

        started = time.monotonic()
        status = px4.status()
        required_status = [
            "instance_id: 731",
            f"target_set_id: {TRACK_SET}",
            "source_system: 255",
            "source_component: 190",
            "origin_system: 1",
            "track_identity_valid: True",
            "track_uid: [0, 17, 34, 51, 68, 85, 102, 119, 136, 153, 170, 187, 204, 221, 238, 255]",
        ]
        missing = [value for value in required_status if value not in status]
        if missing:
            raise AcceptanceFailure(f"normalized status missing {missing!r}\n{status}")
        record(results, "normalized_track_identity_projection", started, "target-set and UUID retained")

        started = time.monotonic()
        drain(endpoint)
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
            2,
            0,
            0,
            fixed_text("wrong source", 20),
            int(time.time() * 1_000_000) + 30_000_000,
            VEHICLE_SYSTEM,
            VEHICLE_COMPONENT,
            TRACK_UID.bytes,
        )
        wrong_sender.send(wrong_cue)
        expect_no_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 734)
        record(results, "wrong_packet_source_isolation", started, "254/190 produced no ACK")

        started = time.monotonic()
        changed = make_cue(endpoint, 731, cue_time + 100_000, cue_expiry, "changed cue", 454672000)
        endpoint.mav.send(changed)
        ack = wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731)
        assert_ack(ack, dialect.MAVLINK_M_ACK_FAILED, "immutable instance collision")
        record(results, "immutable_cue_collision", started, "changed cue 731 rejected")

        started = time.monotonic()
        endpoint.mav.send(make_handover(endpoint, int(time.time() * 1_000_000)))
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        record(results, "addressed_unique_handover_receipt", started, "handover 9002 stored")

        started = time.monotonic()
        cancel_message = make_control(
            endpoint, 3001, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002,
            dialect.MAVLINK_M_TASK_CONTROL_CANCEL,
        )
        cancel_frame = endpoint.send_frozen(cancel_message)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL, 3001),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "cancelled",
        )
        wait_for_task_status(
            endpoint, dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, 9002,
            dialect.MAVLINK_M_TASK_STATE_ABORTED,
        )
        record(results, "durable_idempotent_cancel", started, "control 3001 and ABORTED status")

        started = time.monotonic()
        cue_two = make_cue(endpoint, 732, cue_time + 200_000, cue_expiry, "replacement cue")
        endpoint.mav.send(cue_two)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        endpoint.mav.send(make_control(
            endpoint, 3002, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731,
            dialect.MAVLINK_M_TASK_CONTROL_SUPERSEDE, 732,
        ))
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL, 3002),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "superseded",
        )
        wait_for_task_status(
            endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 731,
            dialect.MAVLINK_M_TASK_STATE_ABORTED,
        )
        endpoint.mav.send(make_cue(endpoint, 733, cue_time + 300_000, cue_expiry, "second pending"))
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 733),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "stored pending",
        )
        endpoint.mav.send(make_cue(endpoint, 734, cue_time + 400_000, cue_expiry, "queue overflow"))
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 734),
            dialect.MAVLINK_M_ACK_FAILED,
            "queue full",
        )
        record(results, "supersede_and_bounded_inbox", started,
               "control 3002; replacement 732 retained; third pending rejected")

        started = time.monotonic()
        ignored = 65535
        endpoint.mav.rc_channels_override_send(
            VEHICLE_SYSTEM, VEHICLE_COMPONENT,
            ignored, ignored, ignored, ignored, 1500, ignored, ignored, ignored,
        )
        time.sleep(0.25)
        endpoint.mav.rc_channels_override_send(
            VEHICLE_SYSTEM, VEHICLE_COMPONENT,
            ignored, ignored, ignored, ignored, 1800, ignored, ignored, ignored,
        )
        time.sleep(0.4)
        endpoint.mav.send(cue_two)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "duplicate idempotent replay",
        )
        record(results, "remote_rc_override_cannot_accept", started, "cue remained pending")

        state_file = rootfs / "mavlink_m_state.bin"
        if not state_file.is_file() or state_file.stat().st_size == 0:
            raise AcceptanceFailure("durable MAVLink-M state file was not created")

        started = time.monotonic()
        px4.stop()
        drain(endpoint)
        restart_mark = px4.start()
        px4.add_test_link()
        establish_udp_peer(endpoint)
        if not signed:
            wait_for_heartbeat(endpoint)
        px4.wait_for("restored MAVLink-M assignment state", restart_mark, 5.0)
        endpoint.send_capability()
        time.sleep(0.05)
        endpoint.mav.send(cue_two)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_TARGET_CUE, 732),
            dialect.MAVLINK_M_ACK_RECEIVED,
            "duplicate idempotent replay",
        )
        restored_status = px4.status()
        if "restored: True" not in restored_status or "track_identity_valid: True" not in restored_status:
            raise AcceptanceFailure("restored normalized state is incomplete\n" + restored_status)
        if signed:
            endpoint.mav.send(cancel_message)
        else:
            transport.write(cancel_frame)
        assert_ack(
            wait_for_ack(endpoint, dialect.MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL, 3001),
            dialect.MAVLINK_M_ACK_ACCEPTED,
            "duplicate idempotent control",
        )
        record(results, "restart_restore_and_idempotent_replay", started,
               "pending UUID and control history restored; "
               + ("fresh signed anti-replay envelope" if signed else "byte-identical frame replay"))

        started = time.monotonic()
        expected = {732, 733}
        expired: set[int] = set()
        deadline = time.monotonic() + max_age + 5.0
        while expected - expired and time.monotonic() < deadline:
            for message in endpoint.receive(min(0.25, deadline - time.monotonic())):
                if (
                    message.get_type() == "MAVLINK_M_ACK"
                    and message.ack_msgid == dialect.MAVLINK_MSG_ID_TARGET_CUE
                    and message.ack_instance in expected
                    and message.result == dialect.MAVLINK_M_ACK_EXPIRED
                ):
                    expired.add(message.ack_instance)
        if expired != expected:
            raise AcceptanceFailure(f"missing persisted expiry ACKs: {sorted(expected - expired)}")
        record(results, "durable_local_expiry", started, "cue 732 and 733 expired after restart")

        return {
            "schema": "px4.aags-mavlink-m.sitl-acceptance.v1",
            "status": "PASS",
            "profile_id": PROFILE_ID,
            "profile_version": PROFILE_VERSION,
            "core_xml_sha256": CORE_XML_SHA256,
            "field_release": False,
            "live_aags_transmit": True,
            "inert_only": True,
            "signing_required": signed,
            "tests": results,
        }
    except AcceptanceFailure as error:
        raise AcceptanceFailure(
            f"{error}\n--- PX4 console tail ---\n{px4.text_since(max(0, px4.mark() - 120))[-12000:]}"
        ) from error
    finally:
        px4.stop()
        transport.close()


def parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[2]
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--px4-binary", type=Path,
        default=root / "build/px4_sitl_default/bin/px4",
    )
    result.add_argument(
        "--etc", type=Path,
        default=root / "build/px4_sitl_default/etc",
    )
    result.add_argument("--rootfs", type=Path, help="isolated run directory; default is temporary")
    result.add_argument("--keep-rootfs", action="store_true")
    result.add_argument("--json-output", type=Path)
    result.add_argument("--max-age", type=int, default=12)
    result.add_argument("--signed", action="store_true",
                        help="exercise physical mode with a temporary MAVLink 2 signing key")
    return result


def main() -> int:
    args = parser().parse_args()
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
        rootfs = Path(tempfile.mkdtemp(prefix="px4-aags-sitl-"))
        print(f"rootfs={rootfs}")
    else:
        temporary = tempfile.TemporaryDirectory(prefix="px4-aags-sitl-")
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
