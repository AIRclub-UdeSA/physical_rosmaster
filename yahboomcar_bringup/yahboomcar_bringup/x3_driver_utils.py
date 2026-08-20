"""Pure helpers for safe X3 motion commands and encoder processing."""

from __future__ import annotations

import math
from typing import Callable, Optional, Sequence, Tuple


EncoderCounts = Tuple[int, int, int, int]
MotionCommand = Tuple[float, float, float]


class MotionSafetyController:
    """Clamp motion commands and stop them when updates become stale."""

    def __init__(
        self,
        send_motion: Callable[[float, float, float], None],
        x_limit: float,
        y_limit: float,
        angular_limit: float,
        timeout_seconds: float,
    ) -> None:
        """Configure the command sink, limits, and watchdog timeout."""
        self._send_motion = send_motion
        self._limits = (x_limit, y_limit, angular_limit)
        self.timeout_seconds = timeout_seconds
        self.last_command_time = 0.0
        self.motion_stopped = True

    def command(
        self, vx: float, vy: float, wz: float, now_seconds: float
    ) -> MotionCommand:
        """Clamp, send, and record a motion command."""
        command = clamp_motion_command(vx, vy, wz, *self._limits)
        self._send_motion(*command)
        self.last_command_time = now_seconds
        self.motion_stopped = command == (0.0, 0.0, 0.0)
        return command

    def stop(self, repeat: int = 1) -> None:
        """Send redundant zero commands and mark motion stopped."""
        for _ in range(max(1, repeat)):
            self._send_motion(0.0, 0.0, 0.0)
        self.motion_stopped = True

    def enforce_timeout(self, now_seconds: float) -> bool:
        """Stop a stale nonzero command and report whether it expired."""
        if self.motion_stopped:
            return False
        elapsed = now_seconds - self.last_command_time
        if watchdog_expired(elapsed, self.timeout_seconds):
            self.stop(repeat=3)
            return True
        return False


def clamp_motion_command(
    vx: float,
    vy: float,
    wz: float,
    x_limit: float,
    y_limit: float,
    angular_limit: float,
) -> MotionCommand:
    """Return a finite motion command clamped to configured absolute limits."""
    values = (vx, vy, wz, x_limit, y_limit, angular_limit)
    if not all(math.isfinite(value) for value in values):
        return 0.0, 0.0, 0.0

    limits = (
        max(0.0, x_limit),
        max(0.0, y_limit),
        max(0.0, angular_limit),
    )
    commands = (vx, vy, wz)
    return tuple(
        max(-limit, min(command, limit))
        for command, limit in zip(commands, limits)
    )


def validate_encoder_config(
    order: Sequence[int], signs: Sequence[float]
) -> None:
    """Validate a raw-channel order and sign configuration for four wheels."""
    if len(order) != 4 or sorted(order) != [0, 1, 2, 3]:
        raise ValueError("encoder_order must be a permutation of [0, 1, 2, 3]")
    if len(signs) != 4 or any(
        sign not in (-1, -1.0, 1, 1.0) for sign in signs
    ):
        raise ValueError(
            "encoder_signs must contain exactly four values of -1 or 1"
        )


def map_encoder_counts(
    raw_counts: Sequence[int],
    order: Sequence[int],
    signs: Sequence[float],
) -> EncoderCounts:
    """Map raw m1..m4 counters into signed FL, FR, BL, BR counters."""
    if len(raw_counts) != 4:
        raise ValueError("raw encoder data must contain exactly four counters")
    validate_encoder_config(order, signs)
    return tuple(
        int(signs[index]) * int(raw_counts[channel])
        for index, channel in enumerate(order)
    )


def signed_int32_delta(current: int, previous: int) -> int:
    """Return a wrap-safe delta between signed 32-bit encoder counters."""
    return ((int(current) - int(previous) + 2**31) % 2**32) - 2**31


def compute_encoder_deltas(
    current: Sequence[int],
    previous: Sequence[int],
    max_abs_delta: int,
) -> Optional[EncoderCounts]:
    """Return wrap-safe deltas, or None when a sample is implausible."""
    if len(current) != 4 or len(previous) != 4:
        raise ValueError("encoder samples must contain exactly four counters")
    if max_abs_delta <= 0:
        raise ValueError("max_abs_delta must be positive")

    deltas = tuple(
        signed_int32_delta(current_value, previous_value)
        for current_value, previous_value in zip(current, previous)
    )
    if any(abs(delta) > max_abs_delta for delta in deltas):
        return None
    return deltas


def watchdog_expired(elapsed_seconds: float, timeout_seconds: float) -> bool:
    """Return whether a nonzero command has exceeded its watchdog timeout."""
    return (
        math.isfinite(elapsed_seconds)
        and math.isfinite(timeout_seconds)
        and timeout_seconds > 0.0
        and elapsed_seconds >= timeout_seconds
    )
