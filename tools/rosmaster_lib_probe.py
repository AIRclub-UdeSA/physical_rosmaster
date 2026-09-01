#!/usr/bin/env python3
"""Inspect Rosmaster_Lib and passively sample controller telemetry."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import struct
import sys
import time
from pathlib import Path


PUBLIC_V3_3_9_SHA256 = (
    "e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c"
)
DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def load_rosmaster_class():
    """Import and return the installed Rosmaster class, or report failure."""
    try:
        from Rosmaster_Lib import Rosmaster  # type: ignore
    except Exception as exc:  # pragma: no cover - hardware dependency
        print(f"ERROR: failed to import Rosmaster_Lib: {exc}", file=sys.stderr)
        print(
            "Check that /root/Rosmaster_Lib is present and exposed "
            "through a .pth file.",
            file=sys.stderr,
        )
        return None
    return Rosmaster


def hash_source(rosmaster_class) -> tuple[Path, str, int, str]:
    """Return source path, SHA-256, byte size, and library version line."""
    source_file = inspect.getsourcefile(rosmaster_class)
    if source_file is None:
        raise RuntimeError("Could not locate Rosmaster source file")

    path = Path(source_file)
    try:
        data = path.read_bytes()
    except OSError:
        module = sys.modules.get(rosmaster_class.__module__)
        loader = getattr(module, "__loader__", None)
        get_data = getattr(loader, "get_data", None)
        if get_data is None:
            raise
        data = get_data(source_file)
    digest = hashlib.sha256(data).hexdigest()
    version_line = ""
    for line in data.decode("utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if stripped.startswith("# V"):
            version_line = stripped
            break
    return path, digest, len(data), version_line


def print_source_report(rosmaster_class) -> bool:
    """Print provenance and return whether the installed hash matches."""
    path, digest, size, version_line = hash_source(rosmaster_class)
    matches_public = digest == PUBLIC_V3_3_9_SHA256

    print(f"source_file: {path}")
    print(f"sha256: {digest}")
    print(f"size_bytes: {size}")
    print(f"version_line: {version_line or '<not found>'}")
    print(f"matches_public_v3_3_9: {str(matches_public).lower()}")
    return matches_public


def extract_frames(buffer: bytearray):
    """Remove and return checksum-valid controller frames from ``buffer``."""
    frames = []
    while len(buffer) >= 4:
        frame_start = buffer.find(b"\xff\xfb")
        if frame_start < 0:
            if buffer[-1:] == b"\xff":
                del buffer[:-1]
            else:
                buffer.clear()
            break
        if frame_start:
            del buffer[:frame_start]
        if len(buffer) < 4:
            break

        ext_len = buffer[2]
        frame_len = ext_len + 2
        if ext_len < 3:
            del buffer[0]
            continue
        if len(buffer) < frame_len:
            break

        ext_type = buffer[3]
        ext_data = bytes(buffer[4:frame_len])
        if (
            ext_data
            and (ext_len + ext_type + sum(ext_data[:-1])) & 0xFF
            == ext_data[-1]
        ):
            frames.append((ext_type, ext_data[:-1]))
            del buffer[:frame_len]
        else:
            del buffer[0]
    return frames


def sample_hardware(port: str, samples: int, period: float) -> int:
    """Passively print motion, encoder, and battery auto-report samples."""
    try:
        import serial  # type: ignore

        serial_port = serial.Serial(
            port=None,
            baudrate=115200,
            timeout=min(period, 0.1),
            exclusive=True,
        )
        # Avoid control-line transitions when the port is opened.  This probe
        # never calls Serial.write() and sends no controller request frames.
        serial_port.dtr = False
        serial_port.rts = False
        serial_port.port = port
        serial_port.open()
    except Exception as exc:  # pragma: no cover - hardware dependency
        print(
            f"ERROR: failed to open Rosmaster on {port}: {exc}",
            file=sys.stderr,
        )
        return 3

    print(
        "sample,motion_vx,motion_vy,motion_vz,encoder_m1,encoder_m2,"
        "encoder_m3,encoder_m4,battery"
    )
    buffer = bytearray()
    motion = None
    encoders = None
    battery = None
    emitted = 0
    next_sample = time.monotonic()
    deadline = next_sample + max(2.0, samples * period + 2.0)
    try:
        while emitted < samples and time.monotonic() < deadline:
            buffer.extend(serial_port.read(serial_port.in_waiting or 1))
            for ext_type, payload in extract_frames(buffer):
                if ext_type == 0x0A and len(payload) >= 7:
                    vx, vy, vz, battery_raw = struct.unpack(
                        "<hhhB", payload[:7]
                    )
                    motion = (vx / 1000.0, vy / 1000.0, vz / 1000.0)
                    battery = battery_raw / 10.0
                elif ext_type == 0x0D and len(payload) >= 16:
                    encoders = struct.unpack("<iiii", payload[:16])

            now = time.monotonic()
            if (
                motion is not None
                and encoders is not None
                and battery is not None
                and now >= next_sample
            ):
                vx, vy, vz = motion
                m1, m2, m3, m4 = encoders
                print(
                    f"{emitted},{vx:.6f},{vy:.6f},{vz:.6f},"
                    f"{m1},{m2},{m3},{m4},{battery:.3f}"
                )
                emitted += 1
                next_sample = now + period
    except Exception as exc:  # pragma: no cover - hardware dependency
        print(f"ERROR: passive sample failed: {exc}", file=sys.stderr)
        return 4
    finally:
        serial_port.close()

    if emitted != samples:
        print(
            "ERROR: timed out waiting for passive speed and encoder "
            "auto-report frames",
            file=sys.stderr,
        )
        return 4
    return 0


def parse_args() -> argparse.Namespace:
    """Parse probe and optional sampling arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Inspect Rosmaster_Lib source and optionally sample "
            "motion/encoder feedback."
        )
    )
    parser.add_argument(
        "--hash-only",
        action="store_true",
        help=(
            "Only print source path/hash/version; do not open the serial "
            "port."
        ),
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=(
            "Serial port passed to Rosmaster when sampling. "
            f"Default: {DEFAULT_PORT}"
        ),
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=0,
        help=(
            "Number of passive motion/encoder/battery auto-report samples "
            "to print. Opens the serial port but transmits no bytes."
        ),
    )
    parser.add_argument(
        "--period",
        type=float,
        default=0.1,
        help="Seconds between samples. Default: 0.1",
    )
    return parser.parse_args()


def main() -> int:
    """Report library provenance and optionally sample controller telemetry."""
    args = parse_args()
    rosmaster_class = load_rosmaster_class()
    if rosmaster_class is None:
        return 2

    try:
        supported_source = print_source_report(rosmaster_class)
    except Exception as exc:
        print(
            f"ERROR: failed to inspect Rosmaster source: {exc}",
            file=sys.stderr,
        )
        return 2

    if not supported_source:
        print(
            "ERROR: unsupported Rosmaster_Lib source; expected SHA256 "
            f"{PUBLIC_V3_3_9_SHA256}",
            file=sys.stderr,
        )
        return 2

    if args.hash_only or args.samples <= 0:
        return 0

    return sample_hardware(args.port, args.samples, args.period)


if __name__ == "__main__":
    raise SystemExit(main())
