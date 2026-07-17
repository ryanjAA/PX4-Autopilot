#!/usr/bin/env python3
"""End-to-end private-inert MAVLink-M endpoint test for PX4 SITL."""

import argparse
import importlib.util
import math
import os
import pathlib
import pty
import select
import shutil
import socket
import subprocess
import sys
import tempfile
import time


PROFILE_ID = b"aags-private-inert-54xxx"
PROFILE_VERSION = b"private-inert-2026-07-17-v2"
PROFILE_SHA256 = bytes.fromhex(
    "f8089061a6a216ef5f1bd4ba5b3242f38f3167b81050c9667121f31e46c36d8e"
)
SOURCE_SYSTEM = 201
SOURCE_COMPONENT = 190
VEHICLE_SYSTEM = 1
VEHICLE_COMPONENT = 1
LOCAL_PORT = 18600
REMOTE_PORT = 18601


class UdpWriter:
    def __init__(self, udp_socket):
        self._socket = udp_socket

    def write(self, data):
        self._socket.sendto(data, ("127.0.0.1", LOCAL_PORT))


class Sitl:
    def __init__(self, repository, build_dir, working_dir):
        self.repository = repository
        self.build_dir = build_dir
        self.working_dir = working_dir
        self.process = None
        self.master_fd = None
        self.log = ""

    def start(self):
        binary = self.build_dir / "bin" / "px4"
        startup = self.build_dir / "etc" / "init.d-posix" / "rcS"
        if not binary.is_file() or not startup.is_file():
            raise RuntimeError(
                "SITL build is missing; run `make px4_sitl_mavlink_m` first"
            )

        master_fd, slave_fd = pty.openpty()
        environment = os.environ.copy()
        environment["PX4_SIM_MODEL"] = "shell"
        environment["R"] = f"{self.build_dir}/"
        environment["PATH"] = (
            f"{self.build_dir / 'bin'}:{environment.get('PATH', '')}"
        )
        self.process = subprocess.Popen(
            [
                str(binary),
                "-i",
                "0",
                "-s",
                str(startup),
                "-w",
                str(self.working_dir),
                str(self.build_dir),
            ],
            cwd=self.repository,
            env=environment,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            close_fds=True,
        )
        os.close(slave_fd)
        self.master_fd = master_fd
        os.set_blocking(self.master_fd, False)
        self.wait_for("Startup script returned successfully", 8)

    def pump(self):
        if self.master_fd is None:
            return

        while True:
            try:
                data = os.read(self.master_fd, 65536)
            except BlockingIOError:
                break
            except OSError:
                break

            if not data:
                break
            self.log += data.decode("utf-8", errors="replace")

    def command(self, command):
        if self.master_fd is None:
            raise RuntimeError("SITL is not running")
        os.write(self.master_fd, f"{command}\n".encode("utf-8"))

    def wait_for(self, text, timeout, start_at=0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if text in self.log[start_at:]:
                return
            if self.process is not None and self.process.poll() is not None:
                raise RuntimeError(
                    f"SITL exited while waiting for {text!r}:\n{self.log[-3000:]}"
                )
            time.sleep(0.02)
        raise RuntimeError(f"timed out waiting for {text!r}:\n{self.log[-3000:]}")

    def command_and_wait(self, command, text, timeout=5):
        start_at = len(self.log)
        self.command(command)
        self.wait_for(text, timeout, start_at)

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            try:
                self.command("shutdown")
                self.process.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                self.process.terminate()
                self.process.wait(timeout=5)
        self.pump()
        if self.master_fd is not None:
            os.close(self.master_fd)
        self.master_fd = None
        self.process = None


class WireHarness:
    def __init__(self, dialect, sitl):
        self.dialect = dialect
        self.sitl = sitl
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("127.0.0.1", REMOTE_PORT))
        self.socket.setblocking(False)
        self.transmitter = dialect.MAVLink(
            UdpWriter(self.socket),
            srcSystem=SOURCE_SYSTEM,
            srcComponent=SOURCE_COMPONENT,
        )
        self.receiver = dialect.MAVLink(None)
        self.receiver.robust_parsing = True
        self.next_heartbeat = 0.0

    def close(self):
        self.socket.close()

    def send_heartbeat(self):
        self.transmitter.heartbeat_send(
            self.dialect.MAV_TYPE_GCS,
            self.dialect.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
            3,
        )
        self.next_heartbeat = time.monotonic() + 0.4

    def send_capability(self):
        now = time.time_ns() // 1000
        flags = (
            self.dialect.MAVLINK_M_CAPABILITY_CUE_RECEIVE
            | self.dialect.MAVLINK_M_CAPABILITY_APPLICATION_RECEIPT
            | self.dialect.MAVLINK_M_CAPABILITY_LOCAL_DECISION
            | self.dialect.MAVLINK_M_CAPABILITY_TASK_STATUS
            | self.dialect.MAVLINK_M_CAPABILITY_TASK_CONTROL
            | self.dialect.MAVLINK_M_CAPABILITY_HANDOVER
            | self.dialect.MAVLINK_M_CAPABILITY_INERT_ONLY
        )
        self.transmitter.mavlink_m_capability_send(
            now,
            now + 15_000_000,
            flags,
            self.dialect.MAVLINK_M_ENDPOINT_AAGS,
            SOURCE_SYSTEM,
            SOURCE_COMPONENT,
            1,
            1,
            PROFILE_SHA256,
            PROFILE_ID,
            PROFILE_VERSION,
            VEHICLE_SYSTEM,
        )

    def make_cue_arguments(self, cue_id, target_system=VEHICLE_SYSTEM):
        now = time.time_ns() // 1000
        return (
            now,
            cue_id,
            17,
            436_532_000,
            -793_830_000,
            100.0,
            math.nan,
            math.nan,
            math.nan,
            0.9,
            SOURCE_SYSTEM,
            self.dialect.MAVLINK_M_CUE_TYPE_INVESTIGATE,
            self.dialect.MAVLINK_M_TARGET_CLASS_MBT_1,
            self.dialect.MAVLINK_M_TARGET_FORCE_FOE,
            b"SITL addressed cue",
            now + 300_000_000,
            target_system,
            VEHICLE_COMPONENT,
            bytes(range(16)),
            target_system,
        )

    def send_cue(self, arguments):
        self.transmitter.target_cue_send(*arguments)

    def pump(self, timeout=0.05):
        if time.monotonic() >= self.next_heartbeat:
            self.send_heartbeat()

        readable, _, _ = select.select(
            [self.socket, self.sitl.master_fd], [], [], timeout
        )
        messages = []
        if self.sitl.master_fd in readable:
            self.sitl.pump()
        if self.socket in readable:
            while True:
                try:
                    data, _ = self.socket.recvfrom(65536)
                except BlockingIOError:
                    break
                for byte in data:
                    message = self.receiver.parse_char(bytes([byte]))
                    if message is not None and message.get_type() != "BAD_DATA":
                        messages.append(message)
        return messages

    def collect_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        collected = []
        while time.monotonic() < deadline:
            for message in self.pump():
                collected.append(message)
                if predicate(message, collected):
                    return collected
        return collected

    def drain(self, duration=0.3):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.pump()


def load_dialect(repository, output_dir, mavgen):
    xml = (
        repository
        / "src"
        / "modules"
        / "mavlink"
        / "mavlink"
        / "message_definitions"
        / "v1.0"
        / "mavlink_m.xml"
    )
    output = output_dir / "mavlink_m.py"
    subprocess.run(
        [
            mavgen,
            "--lang=Python3",
            "--wire-protocol=2.0",
            f"--output={output}",
            str(xml),
        ],
        cwd=repository,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    spec = importlib.util.spec_from_file_location("mavlink_m_test_dialect", output)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def is_ack(dialect, message, cue_id, result):
    return (
        message.get_type() == "MAVLINK_M_ACK"
        and message.ack_msgid == dialect.MAVLINK_MSG_ID_TARGET_CUE
        and message.ack_instance == cue_id
        and message.result == result
        and message.target_system == SOURCE_SYSTEM
        and message.target_component == SOURCE_COMPONENT
    )


def is_status(dialect, message, cue_id, state, sequence):
    return (
        message.get_type() == "MAVLINK_M_TASK_STATUS"
        and message.task_msgid == dialect.MAVLINK_MSG_ID_TARGET_CUE
        and message.task_instance == cue_id
        and message.state == state
        and message.status_sequence == sequence
        and message.target_system == SOURCE_SYSTEM
        and message.target_component == SOURCE_COMPONENT
    )


def run_test(repository, build_dir, mavgen):
    with tempfile.TemporaryDirectory(prefix="px4-mavlink-m-") as temporary:
        temporary_path = pathlib.Path(temporary)
        dialect = load_dialect(repository, temporary_path, mavgen)
        sitl_work = temporary_path / "root"
        sitl_work.mkdir()
        sitl = Sitl(repository, build_dir, sitl_work)
        wire = None

        try:
            sitl.start()
            sitl.command_and_wait("param set MAV_SYS_ID 1", "pxh>")
            sitl.command_and_wait("dataman start", "data manager file")
            sitl.command_and_wait(
                f"mavlink start -x -u {LOCAL_PORT} -o {REMOTE_PORT} "
                "-t 127.0.0.1 -r 1000000",
                "mode: Normal",
            )
            sitl.command_and_wait("mavlink boot_complete", "MAVLink only on localhost")
            sitl.command_and_wait(
                "mavlink mavlink_m status", "signing_required=no"
            )

            wire = WireHarness(dialect, sitl)
            sitl.command_and_wait("param set MVM_SIGN_REQ 1", "pxh>")
            sitl.command_and_wait(
                "mavlink mavlink_m status",
                "signing_required=yes (endpoint disabled)",
            )
            wire.send_heartbeat()
            signing_blocked = wire.collect_until(
                lambda _message, _collected: False, 1.2
            )
            require(
                not any(
                    message.get_type() == "MAVLINK_M_CAPABILITY"
                    for message in signing_blocked
                ),
                "signing-required fail-closed mode advertised a capability",
            )
            sitl.command_and_wait("param set MVM_SIGN_REQ 0", "pxh>")
            sitl.command_and_wait(
                "mavlink mavlink_m status", "signing_required=no"
            )
            wire.send_capability()
            capabilities = wire.collect_until(
                lambda message, _: message.get_type()
                == "MAVLINK_M_CAPABILITY",
                7,
            )
            endpoint_capability = next(
                (
                    message
                    for message in capabilities
                    if message.get_type() == "MAVLINK_M_CAPABILITY"
                ),
                None,
            )
            require(endpoint_capability is not None, "PX4 capability was not received")
            require(
                bytes(endpoint_capability.profile_sha256) == PROFILE_SHA256,
                "PX4 advertised the wrong profile hash",
            )
            require(
                endpoint_capability.capability_flags
                == (
                    dialect.MAVLINK_M_CAPABILITY_CUE_RECEIVE
                    | dialect.MAVLINK_M_CAPABILITY_APPLICATION_RECEIPT
                    | dialect.MAVLINK_M_CAPABILITY_LOCAL_DECISION
                    | dialect.MAVLINK_M_CAPABILITY_TASK_STATUS
                    | dialect.MAVLINK_M_CAPABILITY_INERT_ONLY
                ),
                "PX4 advertised capabilities it does not implement",
            )

            cue_id = 0xA6171001
            cue_arguments = wire.make_cue_arguments(cue_id)
            wire.send_cue(cue_arguments)
            receipt_messages = wire.collect_until(
                lambda message, _: is_ack(
                    dialect, message, cue_id, dialect.MAVLINK_M_ACK_RECEIVED
                ),
                4,
            )
            receipt = next(
                (
                    message
                    for message in receipt_messages
                    if is_ack(
                        dialect,
                        message,
                        cue_id,
                        dialect.MAVLINK_M_ACK_RECEIVED,
                    )
                ),
                None,
            )
            require(receipt is not None, "durable RECEIVED ACK was not returned")
            require(receipt.reason == "durably stored", "receipt reason changed")

            sitl.command("mavlink mavlink_m accept")
            accepted_frames = []
            active_frames = []
            deadline = time.monotonic() + 6.5
            while time.monotonic() < deadline:
                for message in wire.pump():
                    if is_ack(
                        dialect,
                        message,
                        cue_id,
                        dialect.MAVLINK_M_ACK_ACCEPTED,
                    ):
                        accepted_frames.append(bytes(message.get_msgbuf()))
                    if is_status(
                        dialect,
                        message,
                        cue_id,
                        dialect.MAVLINK_M_TASK_STATE_ACTIVE,
                        1,
                    ):
                        active_frames.append(bytes(message.get_msgbuf()))
            require(accepted_frames, "local Accept ACK was not returned")
            require(active_frames, "wire-driven ACTIVE status was not returned")
            require(
                len(accepted_frames) >= 2
                and len(set(accepted_frames)) == 1
                and len(active_frames) >= 2
                and len(set(active_frames)) == 1,
                "exact-frame Accept/ACTIVE retry behavior was not observed",
            )

            lifecycle = (
                (
                    "enroute",
                    dialect.MAVLINK_M_TASK_STATE_EN_ROUTE,
                    2,
                ),
                ("ready", dialect.MAVLINK_M_TASK_STATE_READY, 3),
                ("complete", dialect.MAVLINK_M_TASK_STATE_COMPLETE, 4),
            )
            for command, state, sequence in lifecycle:
                sitl.command(f"mavlink mavlink_m {command}")
                messages = wire.collect_until(
                    lambda message, _collected, expected_state=state,
                    expected_sequence=sequence: is_status(
                        dialect,
                        message,
                        cue_id,
                        expected_state,
                        expected_sequence,
                    ),
                    4,
                )
                require(
                    any(
                        is_status(
                            dialect, message, cue_id, state, sequence
                        )
                        for message in messages
                    ),
                    f"{command} task status was not returned",
                )

            sitl.stop()
            sitl = Sitl(repository, build_dir, sitl_work)
            sitl.start()
            sitl.command_and_wait("param set MAV_SYS_ID 1", "pxh>")
            sitl.command_and_wait("dataman start", "data manager file")
            status_start = len(sitl.log)
            sitl.command("mavlink mavlink_m status")
            sitl.wait_for(
                f"cue={cue_id} from={SOURCE_SYSTEM}:{SOURCE_COMPONENT} state=complete",
                5,
                status_start,
            )

            sitl.command_and_wait(
                f"mavlink start -x -u {LOCAL_PORT} -o {REMOTE_PORT} "
                "-t 127.0.0.1 -r 1000000",
                "mode: Normal",
            )
            sitl.command_and_wait("mavlink boot_complete", "MAVLink only on localhost")
            wire.sitl = sitl
            wire.drain()
            wire.send_heartbeat()
            wire.send_capability()
            reject_id = 0xA6171002
            wire.send_cue(wire.make_cue_arguments(reject_id))
            reject_receipt = wire.collect_until(
                lambda message, _: is_ack(
                    dialect,
                    message,
                    reject_id,
                    dialect.MAVLINK_M_ACK_RECEIVED,
                ),
                4,
            )
            require(
                any(
                    is_ack(
                        dialect,
                        message,
                        reject_id,
                        dialect.MAVLINK_M_ACK_RECEIVED,
                    )
                    for message in reject_receipt
                ),
                "post-restart cue receipt failed",
            )
            sitl.command("mavlink mavlink_m reject")
            rejected = wire.collect_until(
                lambda message, _: is_ack(
                    dialect,
                    message,
                    reject_id,
                    dialect.MAVLINK_M_ACK_REJECTED,
                ),
                4,
            )
            require(
                any(
                    is_ack(
                        dialect,
                        message,
                        reject_id,
                        dialect.MAVLINK_M_ACK_REJECTED,
                    )
                    for message in rejected
                ),
                "local Reject ACK was not returned",
            )

            wrong_id = 0xA6171003
            wire.drain()
            wire.send_cue(wire.make_cue_arguments(wrong_id, target_system=2))
            wrong_address = wire.collect_until(
                lambda _message, _collected: False, 1.5
            )
            require(
                not any(
                    message.get_type() == "MAVLINK_M_ACK"
                    and message.ack_instance == wrong_id
                    for message in wrong_address
                ),
                "wrongly addressed cue received an application ACK",
            )

            print(
                "PASS: capability, addressed durable receipt, Accept/Reject, "
                "exact retries, lifecycle, persistence, signing override, "
                "and wrong-address filtering"
            )
        finally:
            if wire is not None:
                wire.close()
            sitl.stop()


def main():
    repository = pathlib.Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--build-dir",
        type=pathlib.Path,
        default=repository / "build" / "px4_sitl_mavlink_m",
    )
    parser.add_argument(
        "--mavgen",
        default=shutil.which("mavgen.py"),
        help="pymavlink mavgen.py executable",
    )
    arguments = parser.parse_args()
    if not arguments.mavgen:
        parser.error("mavgen.py was not found on PATH; activate the PX4 Python venv")

    try:
        run_test(
            repository,
            arguments.build_dir.resolve(),
            arguments.mavgen,
        )
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
