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

"""Safety regressions for the configurable joystick mapping."""

import pytest
import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

from yahboomcar_ctrl.yahboom_joy_X3 import _axis, _button
from yahboomcar_ctrl.yahboom_joy_X3 import JoyTeleop


class _CapturePublisher:
    def __init__(self):
        self.commands = []

    def publish(self, message):
        self.commands.append(
            (message.linear.x, message.linear.y, message.angular.z)
        )


def test_axis_applies_deadzone_sign_and_bounds():
    message = Joy(axes=[0.05, 2.0])

    assert _axis(message, 0, 1.0, 0.15) == 0.0
    assert _axis(message, 1, -1.0, 0.15) == -1.0


def test_axis_rejects_missing_and_nonfinite_inputs():
    with pytest.raises(IndexError):
        _axis(Joy(axes=[]), 0, 1.0, 0.15)
    with pytest.raises(ValueError):
        _axis(Joy(axes=[float('nan')]), 0, 1.0, 0.15)


def test_button_requires_a_valid_configured_index():
    message = Joy(buttons=[0, 1])

    assert _button(message, 1)
    assert not _button(message, -1)
    with pytest.raises(IndexError):
        _button(message, 2)


def test_deadman_release_malformed_timeout_and_shutdown_all_stop(
    monkeypatch, tmp_path
):
    """Every safety boundary emits zero after a previously active command."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    node = JoyTeleop()
    capture = _CapturePublisher()
    node.cmd_publisher = capture
    active_buttons = [0, 0, 0, 0, 1, 0, 0]
    released_buttons = [0] * len(active_buttons)
    try:
        node.joy_callback(
            Joy(axes=[0.2, 0.8, -0.4], buttons=active_buttons)
        )
        assert capture.commands[-1] == pytest.approx((0.08, 0.02, -0.2))

        node.joy_callback(
            Joy(axes=[0.2, 0.8, -0.4], buttons=released_buttons)
        )
        assert capture.commands[-1] == (0.0, 0.0, 0.0)

        node.joy_callback(
            Joy(axes=[0.2, 0.8, -0.4], buttons=active_buttons)
        )
        node.joy_callback(Joy(axes=[], buttons=active_buttons))
        assert capture.commands[-1] == (0.0, 0.0, 0.0)

        node.joy_callback(
            Joy(axes=[0.2, 0.8, -0.4], buttons=active_buttons)
        )
        node.last_input_time = node.get_clock().now() - Duration(seconds=1.0)
        node._input_watchdog()
        assert capture.commands[-1] == (0.0, 0.0, 0.0)

        node.joy_callback(
            Joy(axes=[0.2, 0.8, -0.4], buttons=active_buttons)
        )
        node.stop()
        assert capture.commands[-1] == (0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
