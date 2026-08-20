#!/usr/bin/env python3
"""Inspect Rosmaster_Lib without commanding robot motion."""

from __future__ import annotations

import argparse
import hashlib
import inspect
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
    data = path.read_bytes()
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


def sample_hardware(
    rosmaster_class, port: str, samples: int, period: float
) -> int:
    """Print read-only motion, encoder, and battery samples from hardware."""
    try:
        car = rosmaster_class(com=port)
        car.create_receive_threading()
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
    for index in range(samples):
        try:
            vx, vy, vz = car.get_motion_data()
            m1, m2, m3, m4 = car.get_motor_encoder()
            battery = car.get_battery_voltage()
        except Exception as exc:  # pragma: no cover - hardware dependency
            print(f"ERROR: sample failed: {exc}", file=sys.stderr)
            return 4

        print(
            f"{index},{vx:.6f},{vy:.6f},{vz:.6f},"
            f"{m1},{m2},{m3},{m4},{battery:.3f}"
        )
        time.sleep(period)

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
            "Number of motion/encoder samples to print. Opens the serial "
            "port."
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
        print_source_report(rosmaster_class)
    except Exception as exc:
        print(
            f"ERROR: failed to inspect Rosmaster source: {exc}",
            file=sys.stderr,
        )
        return 2

    if args.hash_only or args.samples <= 0:
        return 0

    return sample_hardware(
        rosmaster_class, args.port, args.samples, args.period
    )


if __name__ == "__main__":
    raise SystemExit(main())
