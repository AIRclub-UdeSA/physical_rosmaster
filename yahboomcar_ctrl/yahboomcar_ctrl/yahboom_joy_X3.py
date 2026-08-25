#!/usr/bin/env python3
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

"""Safe, explicit-deadman joystick teleoperation for the X3."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32


def _in_bounds(values, index: int) -> bool:
    return 0 <= index < len(values)


def _axis(message: Joy, index: int, sign: float, deadzone: float) -> float:
    if not _in_bounds(message.axes, index):
        raise IndexError("axis %d is not present" % index)
    value = float(message.axes[index]) * sign
    if not math.isfinite(value):
        raise ValueError("axis %d is non-finite" % index)
    return 0.0 if abs(value) < deadzone else max(-1.0, min(1.0, value))


def _button(message: Joy, index: int) -> bool:
    if index < 0:
        return False
    if not _in_bounds(message.buttons, index):
        raise IndexError("button %d is not present" % index)
    return bool(message.buttons[index])


class JoyTeleop(Node):
    """Publish bounded commands only while a configured deadman is held."""

    def __init__(self, name: str = "x3_joy_teleop") -> None:
        super().__init__(name)
        defaults = {
            "joy_topic": "/joy",
            "cmd_vel_topic": "/cmd_vel",
            "x_axis": 1,
            "y_axis": 0,
            "yaw_axis": 2,
            "x_sign": 1.0,
            "y_sign": 1.0,
            "yaw_sign": 1.0,
            "deadman_button": 4,
            "gear_up_button": 5,
            "gear_down_button": 6,
            "buzzer_button": 1,
            "rgb_button": 3,
            "deadzone": 0.15,
            "max_linear_speed": 0.20,
            "max_angular_speed": 1.0,
            "input_timeout": 0.30,
            "gear_scales": [0.25, 0.5, 1.0],
        }
        for parameter, default in defaults.items():
            self.declare_parameter(parameter, default)

        self.x_axis = int(self.get_parameter("x_axis").value)
        self.y_axis = int(self.get_parameter("y_axis").value)
        self.yaw_axis = int(self.get_parameter("yaw_axis").value)
        self.x_sign = float(self.get_parameter("x_sign").value)
        self.y_sign = float(self.get_parameter("y_sign").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.gear_up_button = int(self.get_parameter("gear_up_button").value)
        self.gear_down_button = int(self.get_parameter("gear_down_button").value)
        self.buzzer_button = int(self.get_parameter("buzzer_button").value)
        self.rgb_button = int(self.get_parameter("rgb_button").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.gear_scales = sorted(
            float(value) for value in self.get_parameter("gear_scales").value
        )

        if (
            not 0.0 <= self.deadzone < 1.0
            or self.max_linear <= 0.0
            or self.max_linear > 0.20
            or self.max_angular <= 0.0
            or self.max_angular > 1.0
            or self.input_timeout <= 0.0
            or not self.gear_scales
            or any(not 0.0 < scale <= 1.0 for scale in self.gear_scales)
        ):
            raise ValueError("unsafe joystick limits, timeout, deadzone, or gears")

        self.gear_index = min(1, len(self.gear_scales) - 1)
        self.command_active = False
        self.buzzer_active = False
        self.rgb_index = 0
        self.last_input_time = self.get_clock().now()
        self.previous_buttons = []

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        joy_topic = str(self.get_parameter("joy_topic").value)
        self.cmd_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.buzzer_publisher = self.create_publisher(Bool, "/Buzzer", 1)
        self.rgb_publisher = self.create_publisher(Int32, "/RGBLight", 1)
        self.subscription = self.create_subscription(
            Joy, joy_topic, self.joy_callback, 10
        )
        self.watchdog = self.create_timer(0.05, self._input_watchdog)

    def _rising_edge(self, message: Joy, index: int) -> bool:
        current = _button(message, index)
        previous = (
            bool(self.previous_buttons[index])
            if _in_bounds(self.previous_buttons, index)
            else False
        )
        return current and not previous

    def _publish_zero(self) -> None:
        self.cmd_publisher.publish(Twist())
        self.command_active = False

    def joy_callback(self, message: Joy) -> None:
        """Validate the whole mapping before publishing one bounded command."""
        self.last_input_time = self.get_clock().now()
        try:
            deadman = _button(message, self.deadman_button)
            gear_up = self._rising_edge(message, self.gear_up_button)
            gear_down = self._rising_edge(message, self.gear_down_button)
            buzzer_edge = self._rising_edge(message, self.buzzer_button)
            rgb_edge = self._rising_edge(message, self.rgb_button)
            x = _axis(message, self.x_axis, self.x_sign, self.deadzone)
            y = _axis(message, self.y_axis, self.y_sign, self.deadzone)
            yaw = _axis(message, self.yaw_axis, self.yaw_sign, self.deadzone)
        except (IndexError, ValueError) as error:
            if self.command_active:
                self._publish_zero()
            self.get_logger().error(
                "Invalid joystick mapping/data: %s" % error,
                throttle_duration_sec=5.0,
            )
            self.previous_buttons = list(message.buttons)
            return

        if gear_up:
            self.gear_index = min(
                self.gear_index + 1, len(self.gear_scales) - 1
            )
        if gear_down:
            self.gear_index = max(self.gear_index - 1, 0)
        if buzzer_edge:
            self.buzzer_active = not self.buzzer_active
            self.buzzer_publisher.publish(Bool(data=self.buzzer_active))
        if rgb_edge:
            self.rgb_index = (self.rgb_index + 1) % 6
            self.rgb_publisher.publish(Int32(data=self.rgb_index))

        if deadman:
            scale = self.gear_scales[self.gear_index]
            command = Twist()
            command.linear.x = x * self.max_linear * scale
            command.linear.y = y * self.max_linear * scale
            command.angular.z = yaw * self.max_angular * scale
            self.cmd_publisher.publish(command)
            self.command_active = True
        elif self.command_active:
            self._publish_zero()

        self.previous_buttons = list(message.buttons)

    def _input_watchdog(self) -> None:
        if not self.command_active:
            return
        age = (self.get_clock().now() - self.last_input_time).nanoseconds / 1e9
        if age > self.input_timeout:
            self.get_logger().warning("Joystick input timeout; commanding stop")
            self._publish_zero()

    def stop(self) -> None:
        """Guarantee a final stop while the ROS context is still valid."""
        self._publish_zero()


def main(args=None) -> None:
    """Run joystick teleoperation independently of platform bringup."""
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
