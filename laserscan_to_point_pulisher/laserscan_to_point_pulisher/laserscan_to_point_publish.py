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

"""Convert a LaserScan into a standard XYZ PointCloud2."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def scan_points(message: LaserScan):
    """Yield finite, in-range Cartesian points in the scan frame."""
    for index, distance in enumerate(message.ranges):
        if (
            not math.isfinite(distance)
            or distance < message.range_min
            or distance > message.range_max
        ):
            continue
        angle = message.angle_min + index * message.angle_increment
        yield (
            distance * math.cos(angle),
            distance * math.sin(angle),
            0.0,
        )


class LaserScanToPointPublisher(Node):
    """Publish a configurable PointCloud2 view of a LaserScan."""

    def __init__(self):
        super().__init__("laserscan_to_point_pulisher")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_points")
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.laserscan_callback,
            qos_profile_sensor_data,
        )

    def laserscan_callback(self, message: LaserScan) -> None:
        """Preserve the scan header and publish only valid returns."""
        cloud = point_cloud2.create_cloud_xyz32(
            message.header, list(scan_points(message))
        )
        self.publisher.publish(cloud)


def main(args=None):
    """Run the scan conversion utility."""
    rclpy.init(args=args)
    node = LaserScanToPointPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
