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

"""Bounded keyboard teleoperation with release timeout and shutdown stop."""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
ROSMASTER X3 keyboard control (run explicitly)

   u    i    o        i/, forward/back
   j    k    l        j/l strafe left/right
   m    ,    .        u/o/m/. combine translation and rotation

q/z all speeds +/-10%; w/x linear; e/c angular
space or k stops; Ctrl-C exits. Commands stop automatically on key timeout.
"""

MOVEMENT = {
    "i": (1.0, 0.0, 0.0),
    ",": (-1.0, 0.0, 0.0),
    "j": (0.0, 1.0, 0.0),
    "l": (0.0, -1.0, 0.0),
    "u": (1.0, 0.0, 1.0),
    "o": (1.0, 0.0, -1.0),
    "m": (-1.0, 0.0, -1.0),
    ".": (-1.0, 0.0, 1.0),
}
SPEED = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}


class KeyboardTeleop(Node):
    """Own the terminal and publish commands only after explicit key input."""

    def __init__(self) -> None:
        super().__init__("x3_keyboard_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("initial_linear_speed", 0.10)
        self.declare_parameter("initial_angular_speed", 0.50)
        self.declare_parameter("key_timeout", 0.30)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.linear_speed = float(
            self.get_parameter("initial_linear_speed").value
        )
        self.angular_speed = float(
            self.get_parameter("initial_angular_speed").value
        )
        self.key_timeout = float(self.get_parameter("key_timeout").value)
        if (
            not 0.0 < self.linear_speed <= self.max_linear <= 0.20
            or not 0.0 < self.angular_speed <= self.max_angular <= 1.0
            or self.key_timeout <= 0.0
        ):
            raise ValueError("unsafe keyboard speed or timeout configuration")
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.active = False
        self.last_motion_key = time.monotonic()

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())
        self.active = False

    def publish_motion(self, x: float, y: float, yaw: float) -> None:
        command = Twist()
        command.linear.x = x * self.linear_speed
        command.linear.y = y * self.linear_speed
        command.angular.z = yaw * self.angular_speed
        self.publisher.publish(command)
        self.last_motion_key = time.monotonic()
        self.active = True

    def adjust_speed(self, linear_scale: float, angular_scale: float) -> None:
        self.linear_speed = min(
            self.max_linear, max(0.02, self.linear_speed * linear_scale)
        )
        self.angular_speed = min(
            self.max_angular, max(0.10, self.angular_speed * angular_scale)
        )
        print(
            "linear %.2f m/s; angular %.2f rad/s"
            % (self.linear_speed, self.angular_speed)
        )


def _read_key(settings, timeout: float) -> str:
    tty.setraw(sys.stdin.fileno())
    try:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else ""
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None) -> None:
    """Run interactive keyboard control with a fail-safe release timeout."""
    if not sys.stdin.isatty():
        raise RuntimeError("keyboard teleop requires an interactive terminal")
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()
    print(HELP)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key(settings, min(0.05, node.key_timeout / 2.0))
            normalized = key.lower()
            if key == "\x03":
                break
            if normalized in MOVEMENT:
                node.publish_motion(*MOVEMENT[normalized])
            elif normalized in SPEED:
                node.adjust_speed(*SPEED[normalized])
            elif normalized in (" ", "k"):
                node.publish_stop()
            elif node.active and (
                time.monotonic() - node.last_motion_key > node.key_timeout
            ):
                node.publish_stop()
    finally:
        node.publish_stop()
        rclpy.spin_once(node, timeout_sec=0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
