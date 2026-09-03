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

"""Generic parameterized ROS image resize/conversion utility."""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class PublishImage(Node):
    """Resize a subscribed image while preserving timestamp and frame."""

    def __init__(self) -> None:
        super().__init__("image_resize")
        self.declare_parameter("input_topic", "/cam_1/color/image_raw")
        self.declare_parameter("output_topic", "/inspection/image")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("encoding", "rgb8")
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.encoding = str(self.get_parameter("encoding").value)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image dimensions must be positive")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter("input_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )

    def image_callback(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, self.encoding)
        resized = cv2.resize(frame, (self.width, self.height))
        output = self.bridge.cv2_to_imgmsg(resized, self.encoding)
        output.header = message.header
        self.publisher.publish(output)


def main(args=None) -> None:
    """Run the opt-in image inspection conversion."""
    rclpy.init(args=args)
    node = PublishImage()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
