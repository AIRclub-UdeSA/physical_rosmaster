# Copyright 2026 AIRclub UdeSA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Hardware-free regression tests for X3 driver safety helpers."""

import math

import pytest

from yahboomcar_bringup.x3_driver_utils import clamp_motion_command
from yahboomcar_bringup.x3_driver_utils import compute_encoder_deltas
from yahboomcar_bringup.x3_driver_utils import map_encoder_counts
from yahboomcar_bringup.x3_driver_utils import MotionSafetyController
from yahboomcar_bringup.x3_driver_utils import signed_int32_delta
from yahboomcar_bringup.x3_driver_utils import validate_encoder_config
from yahboomcar_bringup.x3_driver_utils import watchdog_expired


def test_encoder_mapping_and_signs() -> None:
    """Raw channels are reordered and signed into FL, FR, BL, BR."""
    result = map_encoder_counts(
        raw_counts=(10, 20, 30, 40),
        order=(0, 2, 1, 3),
        signs=(1, 1, 1, 1),
    )

    assert result == (10, 30, 20, 40)


def test_encoder_configuration_rejects_duplicate_channels() -> None:
    """Each raw encoder channel must map to exactly one physical wheel."""
    with pytest.raises(ValueError):
        validate_encoder_config((0, 0, 2, 3), (1, 1, 1, 1))


def test_signed_int32_delta_handles_wraparound() -> None:
    """Counter wrap produces a small physical delta, not a pose jump."""
    assert signed_int32_delta(-2147483648, 2147483647) == 1
    assert signed_int32_delta(2147483647, -2147483648) == -1


def test_encoder_discontinuity_is_rejected() -> None:
    """Implausible reset-like jumps force a rebase sample."""
    deltas = compute_encoder_deltas(
        (100, 200, 300, 400), (90, 190, 290, 390), 50
    )
    assert deltas == (
        10,
        10,
        10,
        10,
    )
    reset = compute_encoder_deltas(
        (0, 0, 0, 0), (1000, 1000, 1000, 1000), 50
    )
    assert reset is None


def test_motion_command_is_clamped_and_nonfinite_input_stops() -> None:
    """Commands cannot exceed configured limits and NaN fails closed."""
    assert clamp_motion_command(2.0, -3.0, 8.0, 1.0, 1.5, 5.0) == (
        1.0,
        -1.5,
        5.0,
    )
    assert clamp_motion_command(math.nan, 0.2, 0.1, 1.0, 1.0, 5.0) == (
        0.0,
        0.0,
        0.0,
    )


def test_watchdog_timeout_boundary() -> None:
    """A persistent command expires at the configured timeout."""
    assert not watchdog_expired(0.499, 0.5)
    assert watchdog_expired(0.5, 0.5)
    assert watchdog_expired(0.75, 0.5)
    assert not watchdog_expired(10.0, 0.0)


def test_motion_safety_controller_clamps_and_repeatedly_stops() -> None:
    """The wired safety state clamps a command and expires it to zero."""
    commands = []
    safety = MotionSafetyController(
        lambda vx, vy, wz: commands.append((vx, vy, wz)),
        x_limit=1.0,
        y_limit=1.0,
        angular_limit=5.0,
        timeout_seconds=0.5,
    )

    assert safety.command(2.0, -2.0, 8.0, now_seconds=10.0) == (
        1.0,
        -1.0,
        5.0,
    )
    assert not safety.enforce_timeout(now_seconds=10.49)
    assert safety.enforce_timeout(now_seconds=10.5)
    assert safety.motion_stopped
    assert commands[-3:] == [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    ]
