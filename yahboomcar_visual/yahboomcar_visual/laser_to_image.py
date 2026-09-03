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

"""Render a LaserScan as a parameterized top-down inspection image."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


class LaserToImage(Node):
    """Convert valid scan returns into a ROS mono8 image without opening a GUI."""

    def __init__(self) -> None:
        super().__init__("laser_to_image")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan/image")
        self.declare_parameter("image_size", 800)
        self.declare_parameter("metres_per_pixel", 0.01)
        self.image_size = int(self.get_parameter("image_size").value)
        self.metres_per_pixel = float(
            self.get_parameter("metres_per_pixel").value
        )
        if self.image_size < 100 or self.metres_per_pixel <= 0.0:
            raise ValueError("invalid scan image dimensions or scale")
        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )

    def scan_callback(self, message: LaserScan) -> None:
        """Preserve the scan header and plot finite returns in the scan frame."""
        canvas = np.zeros((self.image_size, self.image_size), dtype=np.uint8)
        center = self.image_size // 2
        canvas[max(0, center - 2) : center + 3, max(0, center - 2) : center + 3] = 128
        for index, distance in enumerate(message.ranges):
            if (
                not math.isfinite(distance)
                or distance < message.range_min
                or distance > message.range_max
            ):
                continue
            angle = message.angle_min + index * message.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            column = center - int(round(y / self.metres_per_pixel))
            row = center - int(round(x / self.metres_per_pixel))
            if 0 <= row < self.image_size and 0 <= column < self.image_size:
                canvas[row, column] = 255

        output = Image()
        output.header = message.header
        output.height = self.image_size
        output.width = self.image_size
        output.encoding = "mono8"
        output.is_bigendian = False
        output.step = self.image_size
        output.data = canvas.tobytes()
        self.publisher.publish(output)


def main(args=None) -> None:
    """Run the opt-in scan inspection conversion."""
    rclpy.init(args=args)
    node = LaserToImage()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
