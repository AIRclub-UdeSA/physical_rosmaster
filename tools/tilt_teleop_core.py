"""Packet decoding, tilt mapping and safety gates for the tilt controller.

Pure Python, no ROS: ``tilt_teleop.py`` wraps this in a node.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

MAGIC = 0x5443  # "TC"
VERSION = 1
PACKET_FORMAT = "<HBBIhhBBH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # 16

FLAG_HOLD = 0x01
FLAG_TURN_MODE = 0x02  # clear: roll strafes, set: roll turns
FLAG_CALIBRATED = 0x04

# A gap longer than this means the controller rebooted or reconnected.
RESYNC_GAP_S = 1.0


@dataclass(frozen=True)
class Packet:
    """One decoded controller packet; tilt angles are in degrees."""

    seq: int
    forward_deg: float
    right_deg: float
    hold: bool
    turn_mode: bool
    calibrated: bool


def checksum(data: bytes) -> int:
    """Sum of the first 14 bytes, modulo 65536."""
    return sum(data[:14]) & 0xFFFF


def encode_packet(
    robot_id: int,
    seq: int,
    forward_cdeg: int,
    right_cdeg: int,
    flags: int,
) -> bytes:
    """Build a packet; used by tests and by the fake sender."""
    head = struct.pack(
        "<HBBIhhBB",
        MAGIC,
        VERSION,
        robot_id,
        seq,
        forward_cdeg,
        right_cdeg,
        flags,
        0,
    )
    return head + struct.pack("<H", checksum(head))


def decode_packet(data: bytes, robot_id: int) -> Optional[Packet]:
    """Return the packet, or None if it is malformed or for another robot."""
    if len(data) != PACKET_SIZE:
        return None
    magic, version, rid, seq, fwd, right, flags, _res, csum = struct.unpack(
        PACKET_FORMAT, data
    )
    if magic != MAGIC or version != VERSION or rid != robot_id:
        return None
    if csum != checksum(data):
        return None
    return Packet(
        seq=seq,
        forward_deg=fwd / 100.0,
        right_deg=right / 100.0,
        hold=bool(flags & FLAG_HOLD),
        turn_mode=bool(flags & FLAG_TURN_MODE),
        calibrated=bool(flags & FLAG_CALIBRATED),
    )


def axis_speed(
    angle_deg: float,
    deadzone_deg: float,
    full_scale_deg: float,
    min_speed: float,
    max_speed: float,
) -> float:
    """Deadzone, then a linear ramp from min_speed to max_speed."""
    magnitude = abs(angle_deg)
    if magnitude < deadzone_deg:
        return 0.0
    if magnitude >= full_scale_deg:
        speed = max_speed
    else:
        span = (magnitude - deadzone_deg) / (full_scale_deg - deadzone_deg)
        speed = min_speed + (max_speed - min_speed) * span
    return math.copysign(speed, angle_deg)


@dataclass(frozen=True)
class Limits:
    """Mapping parameters (the node exposes them as ROS parameters)."""

    deadzone_deg: float = 6.0
    full_scale_deg: float = 30.0
    min_linear: float = 0.10
    max_linear: float = 0.35
    min_angular: float = 0.6
    max_angular: float = 1.0


def map_packet(packet: Packet, limits: Limits) -> Tuple[float, float, float]:
    """Return (linear.x, linear.y, angular.z) for a driving packet.

    Forward tilt drives +x. A right tilt is negative y (strafe) or negative
    angular.z (turn), as in ROS conventions.
    """
    x = axis_speed(
        packet.forward_deg,
        limits.deadzone_deg,
        limits.full_scale_deg,
        limits.min_linear,
        limits.max_linear,
    )
    if packet.turn_mode:
        z = axis_speed(
            -packet.right_deg,
            limits.deadzone_deg,
            limits.full_scale_deg,
            limits.min_angular,
            limits.max_angular,
        )
        return x, 0.0, z
    y = axis_speed(
        -packet.right_deg,
        limits.deadzone_deg,
        limits.full_scale_deg,
        limits.min_linear,
        limits.max_linear,
    )
    return x, y, 0.0


class TiltGate:
    """Accepts packets, rejects stale ones, and decides when to drive."""

    def __init__(self, robot_id: int, timeout_s: float, limits: Limits):
        """Start disarmed: no motion until a hold-released packet is seen."""
        self.robot_id = robot_id
        self.timeout_s = timeout_s
        self.limits = limits
        self._last: Optional[Packet] = None
        self._last_rx = 0.0
        self._armed = False

    def feed(self, data: bytes, now: float) -> bool:
        """Process one datagram; return True if it was accepted."""
        packet = decode_packet(data, self.robot_id)
        if packet is None:
            return False
        gap = self._last is None or now - self._last_rx > RESYNC_GAP_S
        if gap:
            # Controller (re)started: its sequence may have reset, and a
            # held button must not start motion, so wait for a release.
            self._armed = False
        else:
            delta = (packet.seq - self._last.seq) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:  # duplicate or older
                return False
        if not packet.hold:
            self._armed = True
        self._last = packet
        self._last_rx = now
        return True

    def command(self, now: float) -> Tuple[float, float, float]:
        """Return the velocity to publish now; zeros whenever unsafe."""
        packet = self._last
        if packet is None or now - self._last_rx > self.timeout_s:
            return 0.0, 0.0, 0.0
        if not (self._armed and packet.hold and packet.calibrated):
            return 0.0, 0.0, 0.0
        return map_packet(packet, self.limits)
