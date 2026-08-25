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

"""Explicitly activated, bounded X3 angular odometry calibration."""

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


def _yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CalibrateAngular(Node):
    """Rotate through a bounded target while accumulating odometry yaw."""

    def __init__(self, name: str = "calibrate_angular") -> None:
        super().__init__(name)
        defaults = {
            "start_test": False,
            "direction": 1.0,
            "test_angle_degrees": 90.0,
            "speed": 0.30,
            "tolerance_degrees": 2.0,
            "odom_angular_scale_correction": 1.0,
            "max_duration": 20.0,
            "rate": 20.0,
            "base_frame": "base_footprint",
            "odom_frame": "odom",
        }
        for parameter, default in defaults.items():
            self.declare_parameter(parameter, default)

        self.direction = float(self.get_parameter("direction").value)
        angle_degrees = float(self.get_parameter("test_angle_degrees").value)
        self.target = math.radians(angle_degrees) * self.direction
        self.speed = float(self.get_parameter("speed").value)
        tolerance_degrees = float(
            self.get_parameter("tolerance_degrees").value
        )
        self.tolerance = math.radians(tolerance_degrees)
        self.scale = float(
            self.get_parameter("odom_angular_scale_correction").value
        )
        self.max_duration = float(self.get_parameter("max_duration").value)
        rate = float(self.get_parameter("rate").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        if (
            self.direction not in (-1.0, 1.0)
            or not 0.0 < angle_degrees <= 360.0
            or not 0.0 < self.speed <= 1.0
            or not 0.0 < tolerance_degrees < angle_degrees
            or not 0.1 <= self.scale <= 10.0
            or not 0.0 < self.max_duration <= 60.0
            or not 1.0 <= rate <= 50.0
        ):
            raise ValueError("unsafe angular calibration parameters")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 5)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.active = False
        self.last_yaw = 0.0
        self.accumulated_yaw = 0.0
        self.started_at = 0.0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def _odom_yaw(self) -> float:
        transform = self.tf_buffer.lookup_transform(
            self.odom_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        rotation = transform.transform.rotation
        return _yaw(rotation.x, rotation.y, rotation.z, rotation.w)

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
            self.last_yaw = self._odom_yaw()
        except TransformException as error:
            self.stop("Cannot start calibration without odometry TF: %s" % error)
            return
        self.accumulated_yaw = 0.0
        self.started_at = time.monotonic()
        self.active = True
        self.get_logger().warning(
            "Starting bounded angular calibration: %.1fdeg at %.2frad/s"
            % (math.degrees(self.target), self.speed)
        )

    def on_timer(self) -> None:
        """Advance only while start_test remains explicitly true."""
        requested = bool(self.get_parameter("start_test").value)
        if not requested:
            if self.active:
                self.stop("Angular calibration cancelled")
            return
        if not self.active:
            self._start()
            if not self.active:
                return
        if time.monotonic() - self.started_at > self.max_duration:
            self.stop("Angular calibration timed out")
            return

        try:
            current_yaw = self._odom_yaw()
        except TransformException as error:
            self.stop("Odometry TF lost during calibration: %s" % error)
            return
        delta = _normalize(current_yaw - self.last_yaw) * self.scale
        self.last_yaw = current_yaw
        if not math.isfinite(delta):
            self.stop("Non-finite odometry angle")
            return
        self.accumulated_yaw += delta
        error = self.target - self.accumulated_yaw
        if abs(error) <= self.tolerance:
            self.stop("Angular calibration target reached")
            return

        command = Twist()
        command.angular.z = math.copysign(self.speed, error)
        self.publisher.publish(command)


def main(args=None) -> None:
    """Run the inert-until-enabled calibration node."""
    rclpy.init(args=args)
    node = CalibrateAngular()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
