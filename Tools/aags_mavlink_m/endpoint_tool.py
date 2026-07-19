#!/usr/bin/env python3
"""Send and inspect finalized Dronecode MAVLink-M cue/handover traffic.

The utility generates a Python binding from the dialect checked into this PX4
tree, so its wire layout cannot silently drift to a system pymavlink dialect.
The chosen UDP/serial route selects the receiving vehicle; finalized cue and
handover payloads intentionally contain no destination fields.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import math
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import tempfile
import time
import uuid


PROFILE_ID = "dronecode-mavlink-military"
PROFILE_VERSION = "main@23cec367"
CORE_XML_SHA256 = "02a88f79b9b4e58b271d9b0012a06087ac4ec975c75a8b1d7ba704c43bb55a8f"
ACK_NAMES = {
    0: "RECEIVED",
    1: "ACCEPTED",
    2: "REJECTED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "EXPIRED",
}


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def fixed_text(value: str, length: int) -> bytes:
    encoded = value.encode("utf-8")
    if len(encoded) >= length:
        raise ValueError(f"text must encode to fewer than {length} bytes")
    return encoded + bytes(length - len(encoded))


def text_field(value) -> str:
    if isinstance(value, bytes):
        return value.decode(errors="replace").rstrip("\0")
    return str(value).rstrip("\0")


def parse_host_port(value: str) -> tuple[str, int]:
    try:
        host, port = value.rsplit(":", 1)
        return host, int(port)
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected HOST:PORT") from error


def load_dialect():
    root = repository_root()
    xml = root / "src/modules/mavlink/mavlink/message_definitions/v1.0/mavlink_m.xml"
    generator = root / "src/modules/mavlink/mavlink/pymavlink/tools/mavgen.py"

    actual_hash = hashlib.sha256(xml.read_bytes()).hexdigest()
    if actual_hash != CORE_XML_SHA256:
        raise RuntimeError(
            f"dialect hash mismatch: expected {CORE_XML_SHA256}, got {actual_hash}"
        )

    temporary_directory = tempfile.TemporaryDirectory(prefix="aags-mavlink-m-")
    module_path = Path(temporary_directory.name) / "mavlink_m.py"
    command = [
        sys.executable,
        str(generator),
        "--lang",
        "Python3",
        "--wire-protocol",
        "2.0",
        "--output",
        str(module_path),
        str(xml),
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"mavgen failed: {detail}")

    spec = importlib.util.spec_from_file_location("aags_mavlink_m", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load generated dialect")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # The loaded module remains valid after generation. Retaining this object
    # prevents TemporaryDirectory cleanup until process exit.
    module._aags_temporary_directory = temporary_directory
    return module


class UdpTransport:
    def __init__(self, bind: tuple[str, int], destination: tuple[str, int]):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(bind)
        self.socket.setblocking(False)
        self.destination = destination

    def write(self, data: bytes) -> None:
        self.socket.sendto(data, self.destination)

    def read(self, timeout: float) -> bytes:
        readable, _, _ = select.select([self.socket], [], [], timeout)
        return self.socket.recvfrom(65535)[0] if readable else b""

    def close(self) -> None:
        self.socket.close()


class SerialTransport:
    def __init__(self, device: str, baud: int):
        try:
            import serial
        except ImportError as error:
            raise RuntimeError("serial transport requires pyserial") from error
        self.serial = serial.Serial(device, baudrate=baud, timeout=0)

    def write(self, data: bytes) -> None:
        self.serial.write(data)

    def read(self, timeout: float) -> bytes:
        readable, _, _ = select.select([self.serial.fileno()], [], [], timeout)
        if not readable:
            return b""
        return self.serial.read(max(self.serial.in_waiting, 1))

    def close(self) -> None:
        self.serial.close()


class Endpoint:
    def __init__(self, dialect, transport, source_system: int, source_component: int,
                 signing_key: bytes | None = None, signing_link_id: int = 0):
        self.dialect = dialect
        self.transport = transport
        self.mav = dialect.MAVLink(
            transport, srcSystem=source_system, srcComponent=source_component
        )
        self.mav.robust_parsing = True
        if signing_key is not None:
            self.mav.signing.secret_key = signing_key
            self.mav.signing.link_id = signing_link_id
            self.mav.signing.timestamp = max(
                1, int((time.time() - 1_420_070_400) * 100_000)
            )
            self.mav.signing.sign_outgoing = True

    def send_frozen(self, message) -> bytes:
        """Pack once so any caller retry is byte-identical, including signature."""
        frame = bytes(message.pack(self.mav))
        self.transport.write(frame)
        return frame

    def receive(self, timeout: float):
        deadline = time.monotonic() + timeout
        messages = []
        while time.monotonic() < deadline:
            data = self.transport.read(max(0.0, deadline - time.monotonic()))
            if not data:
                break
            message = self.mav.parse_char(data)
            while message is not None:
                if message.get_type() != "BAD_DATA":
                    messages.append(message)
                message = self.mav.parse_char(b"")
            if messages:
                break
        return messages

    def wait_for_ack(self, message_id: int, instance: int, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for message in self.receive(deadline - time.monotonic()):
                if (
                    message.get_type() == "MAVLINK_M_ACK"
                    and message.ack_msgid == message_id
                    and message.ack_instance == instance
                ):
                    return message
                print_message(message)
        raise TimeoutError(
            f"no MAVLINK_M_ACK for msgid={message_id} instance={instance}"
        )


def print_message(message) -> None:
    if message.get_type() == "MAVLINK_M_ACK":
        reason = text_field(message.reason)
        result = ACK_NAMES.get(message.result, str(message.result))
        print(
            f"ACK msgid={message.ack_msgid} instance={message.ack_instance} "
            f"origin={message.origin_sysid} vehicle={message.ack_sysid} "
            f"result={result} reason={reason!r}"
        )
    elif message.get_type() == "PARTICIPANT_POSITION":
        callsign = text_field(message.callsign)
        print(
            f"PPLI vehicle={message.origin_sysid} callsign={callsign!r} "
            f"lat={message.lat / 1e7:.7f} lon={message.lon / 1e7:.7f} "
            f"alt_msl={message.alt:.1f} type={message.ppli_type}"
        )
    elif message.get_type() == "DEBUG_VECT" and text_field(message.name) == "AAGS_TGT":
        print(
            f"OSD bearing={message.x:.1f} relative={message.y:.1f} "
            f"range_m={message.z:.1f}"
        )


def default_instance() -> int:
    value = int(time.time_ns() & 0xFFFFFFFF)
    return value or 1


def send_cue(endpoint: Endpoint, args) -> tuple[int, int]:
    instance = args.instance or default_instance()
    origin_system = args.origin_system or args.source_system
    now_usec = int(time.time() * 1_000_000)
    message = endpoint.mav.target_cue_encode(
        now_usec,
        instance,
        args.target_set,
        round(args.lat * 1e7),
        round(args.lon * 1e7),
        args.alt,
        math.nan,
        math.nan,
        math.nan,
        args.confidence,
        origin_system,
        args.cue_type,
        args.target_class,
        args.target_force,
        fixed_text(args.name, 20),
    )
    frame = endpoint.send_frozen(message)
    args.frozen_frame = frame
    print(f"sent route-selected TARGET_CUE cue_id={instance}")
    return endpoint.dialect.MAVLINK_MSG_ID_TARGET_CUE, instance


def send_track_identity(endpoint: Endpoint, args) -> None:
    now_usec = int(time.time() * 1_000_000)
    track_uid = uuid.UUID(args.track_uid).bytes if args.track_uid else uuid.uuid4().bytes
    origin_system = args.origin_system or args.source_system
    message = endpoint.mav.track_identity_encode(
        now_usec,
        track_uid,
        bytes(16),
        args.target_set,
        now_usec - round(args.track_age * 1_000_000),
        args.confidence,
        origin_system,
        args.id_method,
        args.id_method,
        args.pid_status,
        0,
        args.target_class,
        args.target_force,
        fixed_text(args.id_basis, 50),
        fixed_text("", 20),
        0,
        args.stanag_identity,
        args.environment,
        255,
        0,
        0,
        args.sidc_context,
    )
    endpoint.mav.send(message)
    print(
        f"sent TRACK_IDENTITY target_set_id={args.target_set} "
        f"track_uid={uuid.UUID(bytes=track_uid)} origin_system={origin_system}; no ACK is defined"
    )


def send_handover(endpoint: Endpoint, args) -> tuple[int, int]:
    target_set = args.target_set or args.instance or default_instance()
    now_usec = int(time.time() * 1_000_000)
    track_uid = uuid.UUID(args.track_uid).bytes if args.track_uid else uuid.uuid4().bytes
    message = endpoint.mav.target_handover_encode(
        now_usec,
        now_usec,
        now_usec + round(args.valid_for * 1_000_000),
        round(args.lat * 1e7),
        round(args.lon * 1e7),
        args.alt,
        args.vx,
        args.vy,
        args.vz,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        math.nan,
        target_set,
        fixed_text(args.name, 50),
        fixed_text("", 50),
        args.confidence,
        bytes(8),
        args.target_class,
        args.target_force,
        0,
        track_uid,
    )
    frame = endpoint.send_frozen(message)
    args.frozen_frame = frame
    print(
        f"sent route-selected TARGET_HANDOVER target_set_id={target_set} "
        f"track_uid={uuid.UUID(bytes=track_uid)}"
    )
    return endpoint.dialect.MAVLINK_MSG_ID_TARGET_HANDOVER, target_set


def add_target_arguments(parser, require_alt: bool) -> None:
    parser.add_argument("--lat", type=float, required=True, help="WGS84 latitude, degrees")
    parser.add_argument("--lon", type=float, required=True, help="WGS84 longitude, degrees")
    parser.add_argument("--alt", type=float, required=require_alt, default=math.nan, help="MSL altitude, metres")
    parser.add_argument("--instance", type=int, default=0, help="nonzero cue/target-set ID; default is generated")
    parser.add_argument("--name", default="AAGS target")
    parser.add_argument("--confidence", type=float, default=0.8)
    parser.add_argument("--target-class", type=int, default=0)
    parser.add_argument("--target-force", type=int, default=0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument("--serial", help="serial device for point-to-point radio")
    transport.add_argument("--udp-destination", type=parse_host_port, default=("127.0.0.1", 18570))
    parser.add_argument("--udp-bind", type=parse_host_port, default=("127.0.0.1", 14550))
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--source-component", type=int, default=190)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--signing-key", type=Path, help="32 raw bytes or 64 hexadecimal bytes")
    parser.add_argument("--signing-link-id", type=int, default=0)
    parser.add_argument("--retries", type=int, choices=range(0, 4), default=2,
                        help="additional byte-identical retries after ACK timeout")

    commands = parser.add_subparsers(dest="command", required=True)
    cue = commands.add_parser("cue", help="send a route-selected TARGET_CUE")
    add_target_arguments(cue, require_alt=False)
    cue.add_argument("--target-set", type=int, default=0)
    cue.add_argument(
        "--origin-system", type=int, default=0,
        help="sensing-platform sysid; default is the packet source sysid",
    )
    cue.add_argument("--cue-type", type=int, choices=(1, 2, 3), default=2)
    cue.add_argument("--no-wait", action="store_true")

    track = commands.add_parser(
        "track", help="send approved TRACK_IDENTITY context for a target set"
    )
    track.add_argument("--target-set", type=int, required=True)
    track.add_argument("--track-uid", help="UUID; default is generated")
    track.add_argument("--origin-system", type=int, default=0)
    track.add_argument("--track-age", type=float, default=1.0, help="seconds since first detection")
    track.add_argument("--confidence", type=float, default=0.8)
    track.add_argument("--id-method", type=int, choices=range(0, 10), default=0)
    track.add_argument("--pid-status", type=int, choices=range(0, 4), default=1)
    track.add_argument("--target-class", type=int, default=0)
    track.add_argument("--target-force", type=int, default=0)
    track.add_argument("--id-basis", default="operator observation")
    track.add_argument("--stanag-identity", type=int, choices=range(0, 7), default=1)
    track.add_argument("--environment", type=int, choices=range(0, 6), default=0)
    track.add_argument("--sidc-context", type=int, choices=range(0, 3), default=0)

    handover = commands.add_parser("handover", help="send a route-selected TARGET_HANDOVER")
    add_target_arguments(handover, require_alt=True)
    handover.add_argument("--valid-for", type=float, default=30.0, help="validity duration, seconds")
    handover.add_argument("--target-set", type=int, default=0)
    handover.add_argument("--track-uid", help="UUID; default is generated")
    handover.add_argument("--vx", type=float, default=math.nan)
    handover.add_argument("--vy", type=float, default=math.nan)
    handover.add_argument("--vz", type=float, default=math.nan)
    handover.add_argument("--no-wait", action="store_true")

    listen = commands.add_parser("listen", help="print ACK, PPLI, and OSD telemetry")
    listen.add_argument("--duration", type=float, default=30.0)

    commands.add_parser("profile", help="verify and print the local profile identity")
    return parser


def validate_args(args) -> None:
    for name in ("source_system", "source_component"):
        value = getattr(args, name)
        if not 1 <= value <= 255:
            raise ValueError(f"--{name.replace('_', '-')} must be 1..255")
    if not 0 <= args.signing_link_id <= 255:
        raise ValueError("--signing-link-id must be 0..255")
    if args.command in ("cue", "handover"):
        if not -90.0 <= args.lat <= 90.0 or not -180.0 <= args.lon <= 180.0:
            raise ValueError("latitude/longitude out of WGS84 bounds")
        if args.instance < 0 or args.instance > 0xFFFFFFFF:
            raise ValueError("--instance must be 0..4294967295")
        if not math.isnan(args.confidence) and not 0.0 <= args.confidence <= 1.0:
            raise ValueError("--confidence must be 0..1 or NaN")
    if args.command == "handover":
        if args.valid_for <= 0 or args.valid_for > 300:
            raise ValueError("--valid-for must be greater than zero and at most 300 seconds")
    if args.command == "cue" and args.origin_system and not 1 <= args.origin_system <= 255:
        raise ValueError("--origin-system must be 1..255")
    if args.command == "track":
        if not 1 <= args.target_set <= 0xFFFFFFFF:
            raise ValueError("--target-set must be 1..4294967295")
        if args.origin_system and not 1 <= args.origin_system <= 255:
            raise ValueError("--origin-system must be 1..255")
        if args.track_age < 0:
            raise ValueError("--track-age must be non-negative")
        if not math.isnan(args.confidence) and not 0.0 <= args.confidence <= 1.0:
            raise ValueError("--confidence must be 0..1 or NaN")
def read_signing_key(path: Path | None) -> bytes | None:
    if path is None:
        return None
    if os.name == "posix" and path.stat().st_mode & 0o077:
        raise ValueError("signing key file must not be accessible by group or other")
    data = path.read_bytes()
    if len(data) == 64:
        try:
            data = bytes.fromhex(data.decode("ascii"))
        except (UnicodeDecodeError, ValueError) as error:
            raise ValueError("signing key must be 32 raw bytes or 64 hexadecimal bytes") from error
    if len(data) != 32 or not any(data):
        raise ValueError("signing key must be 32 nonzero raw bytes or 64 hexadecimal bytes")
    return data


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        validate_args(args)
        dialect = load_dialect()
        print(f"profile_id={PROFILE_ID}")
        print(f"profile_version={PROFILE_VERSION}")
        print(f"core_xml_sha256={CORE_XML_SHA256}")
        if args.command == "profile":
            return 0

        if args.serial:
            transport = SerialTransport(args.serial, args.baud)
        else:
            transport = UdpTransport(args.udp_bind, args.udp_destination)

        endpoint = Endpoint(
            dialect, transport, args.source_system, args.source_component,
            read_signing_key(args.signing_key), args.signing_link_id,
        )
        try:
            if args.command == "cue":
                message_id, instance = send_cue(endpoint, args)
            elif args.command == "track":
                send_track_identity(endpoint, args)
                return 0
            elif args.command == "handover":
                message_id, instance = send_handover(endpoint, args)
            else:
                deadline = time.monotonic() + args.duration
                while time.monotonic() < deadline:
                    for message in endpoint.receive(deadline - time.monotonic()):
                        print_message(message)
                return 0

            if not args.no_wait:
                for attempt in range(args.retries + 1):
                    try:
                        print_message(endpoint.wait_for_ack(message_id, instance, args.timeout))
                        break
                    except TimeoutError:
                        if attempt >= args.retries:
                            raise
                        transport.write(args.frozen_frame)
                        print(f"byte-identical retry {attempt + 1}/{args.retries}")
            return 0
        finally:
            transport.close()
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        parser.exit(1, f"error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
