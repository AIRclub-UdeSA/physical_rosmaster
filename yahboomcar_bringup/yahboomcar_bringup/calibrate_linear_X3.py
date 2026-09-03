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

"""Explicitly activated, bounded X3 linear odometry calibration."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class CalibrateLinear(Node):
    """Move one linear axis while comparing distance against odometry TF."""

    def __init__(self, name: str = "calibrate_linear") -> None:
        super().__init__(name)
        defaults = {
            "start_test": False,
            "axis": "x",
            "direction": 1.0,
            "test_distance": 0.50,
            "speed": 0.10,
            "tolerance": 0.02,
            "odom_linear_scale_correction": 1.0,
            "max_duration": 15.0,
            "rate": 20.0,
            "base_frame": "base_footprint",
            "odom_frame": "odom",
        }
        for parameter, default in defaults.items():
            self.declare_parameter(parameter, default)

        self.axis = str(self.get_parameter("axis").value)
        self.direction = float(self.get_parameter("direction").value)
        self.test_distance = float(self.get_parameter("test_distance").value)
        self.speed = float(self.get_parameter("speed").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.scale = float(
            self.get_parameter("odom_linear_scale_correction").value
        )
        self.max_duration = float(self.get_parameter("max_duration").value)
        rate = float(self.get_parameter("rate").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        if (
            self.axis not in ("x", "y")
            or self.direction not in (-1.0, 1.0)
            or not 0.0 < self.test_distance <= 2.0
            or not 0.0 < self.speed <= 0.20
            or not 0.0 < self.tolerance < self.test_distance
            or not 0.1 <= self.scale <= 10.0
            or not 0.0 < self.max_duration <= 60.0
            or not 1.0 <= rate <= 50.0
        ):
            raise ValueError("unsafe linear calibration parameters")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 5)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.active = False
        self.origin = (0.0, 0.0)
        self.started_at = 0.0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def _position(self):
        transform = self.tf_buffer.lookup_transform(
            self.odom_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _set_inactive(self) -> None:
        self.set_parameters([Parameter("start_test", value=False)])
        self.active = False

    def stop(self, reason: str | None = None) -> None:
        """Stop motion first, then make the calibration inert."""
        self.publisher.publish(Twist())
        if reason:
            self.get_logger().warning(reason)
        self._set_inactive()

    def _start(self) -> None:
        try:
            self.origin = self._position()
        except TransformException as error:
            self.stop("Cannot start calibration without odometry TF: %s" % error)
            return
        self.started_at = time.monotonic()
        self.active = True
        self.get_logger().warning(
            "Starting bounded %s-axis calibration: %.2fm at %.2fm/s"
            % (self.axis, self.test_distance, self.speed)
        )

    def on_timer(self) -> None:
        """Advance only while start_test remains explicitly true."""
        requested = bool(self.get_parameter("start_test").value)
        if not requested:
            if self.active:
                self.stop("Linear calibration cancelled")
            return
        if not self.active:
            self._start()
            if not self.active:
                return
        if time.monotonic() - self.started_at > self.max_duration:
            self.stop("Linear calibration timed out")
            return

        try:
            current = self._position()
        except TransformException as error:
            self.stop("Odometry TF lost during calibration: %s" % error)
            return
        distance = math.hypot(
            current[0] - self.origin[0], current[1] - self.origin[1]
        ) * self.scale
        if not math.isfinite(distance):
            self.stop("Non-finite odometry distance")
            return
        if distance >= self.test_distance - self.tolerance:
            self.stop("Linear calibration target reached")
            return

        command = Twist()
        if self.axis == "x":
            command.linear.x = self.direction * self.speed
        else:
            command.linear.y = self.direction * self.speed
        self.publisher.publish(command)


def main(args=None) -> None:
    """Run the inert-until-enabled calibration node."""
    rclpy.init(args=args)
    node = CalibrateLinear()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
