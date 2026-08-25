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

"""Validate the physical X3 against the simulator-facing robot contract.

Derived from the simulator probe at commit 772ba250bafeb0e93e651b7d8d78a4598feba118.
Simulation clock, ground truth, simulator-specific rates, and ideal camera FOV
checks are intentionally replaced with physical-hardware checks.
"""

from __future__ import annotations

import math
import struct
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


EXPECTED_WHEEL_JOINTS = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
}
REQUIRED_DYNAMIC_TF_EDGE = ("odom", "base_footprint")
REQUIRED_STATIC_FRAMES = {
    "base_link",
    "laser_link",
    "imu_link",
    "cam_1_link",
    "cam_1_color_frame",
    "cam_1_color_optical_frame",
    "cam_1_depth_frame",
    "cam_1_depth_optical_frame",
}
TOPIC_TYPES = {
    "/scan": (LaserScan, "sensor_msgs/msg/LaserScan"),
    "/imu/data": (Imu, "sensor_msgs/msg/Imu"),
    "/cam_1/color/image_raw": (Image, "sensor_msgs/msg/Image"),
    "/cam_1/depth/image_raw": (Image, "sensor_msgs/msg/Image"),
    "/cam_1/color/camera_info": (CameraInfo, "sensor_msgs/msg/CameraInfo"),
    "/cam_1/depth/camera_info": (CameraInfo, "sensor_msgs/msg/CameraInfo"),
    "/cam_1/depth/color/points": (PointCloud2, "sensor_msgs/msg/PointCloud2"),
    "/joint_states": (JointState, "sensor_msgs/msg/JointState"),
    "/odom": (Odometry, "nav_msgs/msg/Odometry"),
    "/tf": (TFMessage, "tf2_msgs/msg/TFMessage"),
    "/tf_static": (TFMessage, "tf2_msgs/msg/TFMessage"),
    "/diagnostics": (DiagnosticArray, "diagnostic_msgs/msg/DiagnosticArray"),
}
SENSOR_TOPICS = {
    "/scan",
    "/imu/data",
    "/cam_1/color/image_raw",
    "/cam_1/depth/image_raw",
    "/cam_1/color/camera_info",
    "/cam_1/depth/camera_info",
    "/cam_1/depth/color/points",
}
UNIQUE_PUBLISHER_TOPICS = set(TOPIC_TYPES) - {
    "/diagnostics",
    "/tf",
    "/tf_static",
}
REQUIRED_DIAGNOSTIC_SOURCES = {
    "yahboomcar_base_node: wheel encoder odometry",
    "yahboomcar_bringup: motor controller and onboard sensors",
}


class PhysicalContractProbe(Node):
    """Collect consecutive messages and validate the hardware contract."""

    def __init__(self):
        super().__init__("physical_contract_probe")
        self.declare_parameter("timeout", 35.0)
        self.declare_parameter("samples", 5)
        self.timeout = float(self.get_parameter("timeout").value)
        self.samples = max(3, int(self.get_parameter("samples").value))
        self.required_counts = {
            topic: 2 if topic == "/tf_static" else self.samples
            for topic in TOPIC_TYPES
        }
        self.messages = {topic: [] for topic in self.required_counts}
        self.observed_dynamic_tf_edges = set()
        self.started_at = time.monotonic()
        self.first_arrivals = {}
        self.subscription_handles = []

        default_qos = QoSProfile(depth=20)
        sensor_qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        static_tf_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        for topic, (message_type, _) in TOPIC_TYPES.items():
            qos = (
                static_tf_qos
                if topic == "/tf_static"
                else sensor_qos if topic in SENSOR_TOPICS else default_qos
            )
            self.subscription_handles.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, topic_name=topic: self.capture(
                        topic_name, message
                    ),
                    qos,
                )
            )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0), node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self.get_logger().info(
            "Waiting up to %.1fs for the physical platform contract" % self.timeout
        )

    def capture(self, topic, message):
        """Keep bounded samples and track the canonical dynamic TF edge."""
        self.first_arrivals.setdefault(topic, time.monotonic())
        if topic == "/tf":
            self.observed_dynamic_tf_edges.update(
                (
                    transform.header.frame_id.lstrip("/"),
                    transform.child_frame_id.lstrip("/"),
                )
                for transform in message.transforms
            )
        if len(self.messages[topic]) < self.required_counts[topic]:
            self.messages[topic].append(message)

    def complete(self):
        """Return whether all messages and odometry TF were observed."""
        return (
            all(
                len(self.messages[topic]) >= count
                for topic, count in self.required_counts.items()
            )
            and REQUIRED_DYNAMIC_TF_EDGE in self.observed_dynamic_tf_edges
        )

    @staticmethod
    def stamp_seconds(message):
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def finite(values):
        return all(math.isfinite(float(value)) for value in values)

    def validate_header(self, topic, errors):
        messages = self.messages[topic]
        stamps = [self.stamp_seconds(message) for message in messages]
        if any(stamp <= 0.0 for stamp in stamps):
            errors.append("%s: header timestamp is zero or negative" % topic)
        if any(
            current <= previous
            for previous, current in zip(stamps, stamps[1:])
        ):
            errors.append("%s: header timestamps are not increasing" % topic)
        if any(not message.header.frame_id for message in messages):
            errors.append("%s: frame_id is empty" % topic)

    def validate_rate(self, topic, minimum, maximum, errors):
        stamps = [self.stamp_seconds(message) for message in self.messages[topic]]
        periods = [
            current - previous
            for previous, current in zip(stamps, stamps[1:])
            if current > previous
        ]
        if not periods:
            return
        period = sorted(periods)[len(periods) // 2]
        rate = 1.0 / period
        if not minimum <= rate <= maximum:
            errors.append(
                "%s: measured %.2f Hz outside %.1f..%.1f Hz"
                % (topic, rate, minimum, maximum)
            )

    def validate_graph(self, errors):
        graph_types = dict(self.get_topic_names_and_types())
        for topic, (_, expected_type) in TOPIC_TYPES.items():
            actual_types = graph_types.get(topic, [])
            if expected_type not in actual_types:
                errors.append(
                    "%s: expected type %s, graph reports %s"
                    % (topic, expected_type, actual_types)
                )
            if topic in UNIQUE_PUBLISHER_TOPICS:
                publishers = self.get_publishers_info_by_topic(topic)
                if len(publishers) != 1:
                    errors.append(
                        "%s: expected exactly one publisher, found %d"
                        % (topic, len(publishers))
                    )
        cmd_publishers = self.get_publishers_info_by_topic("/cmd_vel")
        if cmd_publishers:
            errors.append(
                "/cmd_vel: default bringup must have no publisher; found %s"
                % sorted(endpoint.node_name for endpoint in cmd_publishers)
            )

    def validate_camera_info(self, label, info, image, errors):
        if (info.width, info.height) != (image.width, image.height):
            errors.append("%s camera_info dimensions do not match image" % label)
        if len(info.k) != 9 or not self.finite(info.k):
            errors.append("%s camera_info has invalid intrinsics" % label)
        elif (
            info.k[0] <= 0.0
            or info.k[4] <= 0.0
            or not 0.0 <= info.k[2] <= image.width
            or not 0.0 <= info.k[5] <= image.height
        ):
            errors.append("%s camera_info is not a valid calibration" % label)
        if len(info.p) != 12 or not self.finite(info.p):
            errors.append("%s camera_info has invalid projection" % label)
        if info.header.frame_id != image.header.frame_id:
            errors.append("%s camera_info frame does not match image" % label)

    def validate_depth_units(self, image, errors):
        if image.encoding != "32FC1":
            errors.append("depth image: expected 32FC1, got %s" % image.encoding)
            return
        endian = ">" if image.is_bigendian else "<"
        count = min(image.width * image.height, 4096)
        if count == 0 or len(image.data) < count * 4:
            errors.append("depth image: no sampleable pixels")
            return
        step = max(1, (image.width * image.height) // count)
        values = []
        for index in range(0, image.width * image.height, step):
            byte_index = (index // image.width) * image.step + (index % image.width) * 4
            if byte_index + 4 <= len(image.data):
                values.append(struct.unpack_from(endian + "f", image.data, byte_index)[0])
            if len(values) >= count:
                break
        plausible = [value for value in values if math.isfinite(value) and 0.05 < value < 20.0]
        if not plausible:
            errors.append("depth image: no plausible metric depth samples")

    def validate_timestamped_tf(self, errors):
        for topic in (
            "/scan",
            "/imu/data",
            "/cam_1/color/image_raw",
            "/cam_1/depth/image_raw",
            "/cam_1/color/camera_info",
            "/cam_1/depth/camera_info",
            "/cam_1/depth/color/points",
            "/odom",
        ):
            message = self.messages[topic][-1]
            frame = message.child_frame_id if topic == "/odom" else message.header.frame_id
            if not self.tf_buffer.can_transform(
                "odom",
                frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.2),
            ):
                errors.append("%s: cannot resolve odom -> %s at message time" % (topic, frame))

    def validate_diagnostics(self, errors):
        """Require current healthy status from both hardware-facing owners."""
        latest_status = {}
        for message in self.messages["/diagnostics"]:
            for status in message.status:
                latest_status[status.name] = status
        missing = REQUIRED_DIAGNOSTIC_SOURCES - set(latest_status)
        if missing:
            errors.append("/diagnostics missing %s" % sorted(missing))
        for name in REQUIRED_DIAGNOSTIC_SOURCES & set(latest_status):
            status = latest_status[name]
            if status.level != DiagnosticStatus.OK:
                errors.append(
                    "/diagnostics %s is level %d: %s"
                    % (name, status.level, status.message)
                )

    def validate(self):
        errors = []
        for topic, required in self.required_counts.items():
            received = len(self.messages[topic])
            if received < required:
                errors.append("%s: received %d/%d messages" % (topic, received, required))
        if errors:
            return errors

        self.validate_graph(errors)
        for topic in TOPIC_TYPES:
            if topic not in ("/diagnostics", "/tf", "/tf_static"):
                self.validate_header(topic, errors)
        for topic, limits in {
            "/scan": (3.0, 20.0),
            "/imu/data": (5.0, 30.0),
            "/cam_1/color/image_raw": (3.0, 40.0),
            "/cam_1/depth/image_raw": (3.0, 40.0),
            "/cam_1/color/camera_info": (3.0, 40.0),
            "/cam_1/depth/camera_info": (3.0, 40.0),
            "/cam_1/depth/color/points": (3.0, 40.0),
            "/joint_states": (5.0, 20.0),
            "/odom": (5.0, 20.0),
        }.items():
            self.validate_rate(topic, *limits, errors)
            if self.first_arrivals[topic] - self.started_at > 25.0:
                errors.append("%s: first message took more than 25s" % topic)

        color = self.messages["/cam_1/color/image_raw"][-1]
        depth = self.messages["/cam_1/depth/image_raw"][-1]
        if color.encoding != "rgb8":
            errors.append("color image: expected rgb8, got %s" % color.encoding)
        for label, image in (("color", color), ("depth", depth)):
            if image.width == 0 or image.height == 0:
                errors.append("%s image dimensions are zero" % label)
            if len(image.data) != image.step * image.height:
                errors.append("%s image data length does not match layout" % label)
        self.validate_depth_units(depth, errors)
        self.validate_camera_info(
            "color", self.messages["/cam_1/color/camera_info"][-1], color, errors
        )
        self.validate_camera_info(
            "depth", self.messages["/cam_1/depth/camera_info"][-1], depth, errors
        )

        points = self.messages["/cam_1/depth/color/points"][-1]
        if points.header.frame_id != "cam_1_depth_frame":
            errors.append("point cloud frame is %s" % points.header.frame_id)
        if not {"x", "y", "z", "rgb"}.issubset(
            {field.name for field in points.fields}
        ):
            errors.append("point cloud does not contain XYZRGB")
        if not points.data or len(points.data) != points.row_step * points.height:
            errors.append("point cloud data/layout is invalid")

        scan = self.messages["/scan"][-1]
        if scan.header.frame_id != "laser_link":
            errors.append("scan frame is %s" % scan.header.frame_id)
        if (
            not scan.ranges
            or scan.angle_increment <= 0.0
            or not 0.0 < scan.range_min < scan.range_max
            or not self.finite(
                (
                    scan.angle_min,
                    scan.angle_max,
                    scan.angle_increment,
                    scan.range_min,
                    scan.range_max,
                )
            )
        ):
            errors.append("scan metadata or ranges are invalid")

        imu = self.messages["/imu/data"][-1]
        if imu.header.frame_id != "imu_link":
            errors.append("IMU frame is %s" % imu.header.frame_id)
        imu_values = (
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
        )
        if not self.finite(imu_values):
            errors.append("IMU contains non-finite values")
        gravity = math.sqrt(
            imu.linear_acceleration.x**2
            + imu.linear_acceleration.y**2
            + imu.linear_acceleration.z**2
        )
        if not 5.0 < gravity < 15.0:
            errors.append("IMU acceleration magnitude is %.3f" % gravity)

        joints = self.messages["/joint_states"][-1]
        if not EXPECTED_WHEEL_JOINTS.issubset(set(joints.name)):
            errors.append("joint_states is missing canonical wheel joints")
        if len(joints.position) < len(joints.name) or len(joints.velocity) < len(joints.name):
            errors.append("joint_states position/velocity arrays are incomplete")
        elif not self.finite([*joints.position, *joints.velocity]):
            errors.append("joint_states contains non-finite values")

        odometry = self.messages["/odom"][-1]
        if odometry.header.frame_id != "odom" or odometry.child_frame_id != "base_footprint":
            errors.append("odometry frames do not match odom -> base_footprint")
        odom_values = (
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w,
            odometry.twist.twist.linear.x,
            odometry.twist.twist.linear.y,
            odometry.twist.twist.angular.z,
        )
        if not self.finite(odom_values):
            errors.append("odometry contains non-finite values")
        if REQUIRED_DYNAMIC_TF_EDGE not in self.observed_dynamic_tf_edges:
            errors.append("/tf did not contain odom -> base_footprint")

        static_children = {
            transform.child_frame_id
            for message in self.messages["/tf_static"]
            for transform in message.transforms
        }
        missing_frames = REQUIRED_STATIC_FRAMES - static_children
        if missing_frames:
            errors.append("/tf_static missing %s" % sorted(missing_frames))
        self.validate_diagnostics(errors)
        self.validate_timestamped_tf(errors)
        return errors

    def summary(self):
        return ", ".join(
            "%s=%d" % (topic, len(messages))
            for topic, messages in self.messages.items()
        )


def main():
    """Collect, validate, and return a shell-friendly status code."""
    rclpy.init()
    node = PhysicalContractProbe()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.2)
        tf_deadline = min(deadline, time.monotonic() + 0.5)
        while rclpy.ok() and time.monotonic() < tf_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        errors = node.validate()
        if errors:
            node.get_logger().error("Physical contract FAILED: " + "; ".join(errors))
            node.get_logger().error("Received: " + node.summary())
            return 1
        node.get_logger().info("Physical contract PASSED: " + node.summary())
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
