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

"""
Measure physical X3 sensor capabilities for transfer to the simulator.

tools/physical_contract_probe.py is a pass/fail gate against
config/robot_contract.yaml: it answers "is this interface present and legal".
This tool answers the different question the simulator needs -- *how well*
does each interface actually behave: arrival rate and jitter, publish-to-
receive latency, dropouts, intrinsics and derived field of view, depth
validity, and stationary noise floors.

It changes nothing and commands nothing. It has no /cmd_vel publisher.

Run it ON THE ROBOT. Latency is measured as (receive clock - header.stamp),
so a workstation whose clock is not synchronized to the robot reports a
constant clock offset rather than true pipeline latency.

Measurement load is itself a result. On a Raspberry Pi, subscribing to the
XYZRGB cloud is not free, so --sequential measures one topic at a time and
--content-samples bounds heavy per-message parsing. Record which mode
produced a number before comparing two numbers.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import (
    CameraInfo,
    Image,
    Imu,
    JointState,
    LaserScan,
    MagneticField,
    PointCloud2,
)
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy ships with the camera adapter
    np = None


TOPIC_TYPES = {
    "/cam_1/color/image_raw": Image,
    "/cam_1/color/camera_info": CameraInfo,
    "/cam_1/depth/image_raw": Image,
    "/cam_1/depth/camera_info": CameraInfo,
    "/cam_1/depth/color/points": PointCloud2,
    "/_hardware/astra/color/image_raw": Image,
    "/_hardware/astra/color/camera_info": CameraInfo,
    "/_hardware/astra/depth/image_raw": Image,
    "/_hardware/astra/depth/camera_info": CameraInfo,
    "/_hardware/astra/depth/color/points": PointCloud2,
    "/scan": LaserScan,
    "/scan_filtered": LaserScan,
    "/imu/data": Imu,
    "/imu/data_raw": Imu,
    "/imu/mag": MagneticField,
    "/odom": Odometry,
    "/joint_states": JointState,
    "/vel_raw": Twist,
    "/voltage": Float32,
    "/diagnostics": DiagnosticArray,
}

GROUPS = {
    "camera": [
        "/cam_1/color/image_raw",
        "/cam_1/color/camera_info",
        "/cam_1/depth/image_raw",
        "/cam_1/depth/camera_info",
        "/cam_1/depth/color/points",
    ],
    "camera_hardware": [
        "/_hardware/astra/color/image_raw",
        "/_hardware/astra/color/camera_info",
        "/_hardware/astra/depth/image_raw",
        "/_hardware/astra/depth/camera_info",
        "/_hardware/astra/depth/color/points",
    ],
    "lidar": ["/scan", "/scan_filtered"],
    "imu": ["/imu/data", "/imu/data_raw", "/imu/mag"],
    "odom": ["/odom", "/joint_states", "/vel_raw"],
    "health": ["/voltage", "/diagnostics"],
}

# base_link is the platform-owned root for every sensor mount. The cam_1_*
# frames below cam_1_link are owned by the Orbbec driver's calibration.
TF_TARGETS = [
    "base_footprint",
    "laser_link",
    "imu_link",
    "cam_1_link",
    "cam_1_color_frame",
    "cam_1_color_optical_frame",
    "cam_1_depth_frame",
    "cam_1_depth_optical_frame",
]

# Messages with no header carry no stamp, so latency is undefined for them.
UNSTAMPED_TYPES = (Twist, Float32)


def percentile(values, fraction):
    """Return a linearly interpolated percentile of an unsorted sequence."""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def describe(values, scale=1.0):
    """Summarize a numeric series, scaling for readable units."""
    if not values:
        return None
    scaled = [value * scale for value in values]
    mean = sum(scaled) / len(scaled)
    variance = sum((value - mean) ** 2 for value in scaled) / len(scaled)
    return {
        "count": len(scaled),
        "min": min(scaled),
        "max": max(scaled),
        "mean": mean,
        "median": percentile(scaled, 0.5),
        "p95": percentile(scaled, 0.95),
        "p99": percentile(scaled, 0.99),
        "stddev": math.sqrt(variance),
    }


def field_of_view(intrinsic, size):
    """Derive a field of view in degrees from a focal length and image size."""
    if not intrinsic or intrinsic <= 0.0 or not size:
        return None
    return math.degrees(2.0 * math.atan(size / (2.0 * intrinsic)))


@dataclass
class TopicRecord:
    """Accumulated arrival, latency, and content evidence for one topic."""

    topic: str
    message_type: str
    arrivals: list = field(default_factory=list)
    latencies: list = field(default_factory=list)
    content: list = field(default_factory=list)
    series: dict = field(default_factory=dict)
    publisher_qos: list = field(default_factory=list)
    errors: list = field(default_factory=list)

    def add_series(self, name, value):
        """Record one cheap scalar sample kept for every message."""
        if value is None or not math.isfinite(value):
            return
        self.series.setdefault(name, []).append(value)


def image_content(message):
    """Describe image geometry, encoding, and depth validity."""
    content = {
        "frame_id": message.header.frame_id,
        "width": message.width,
        "height": message.height,
        "encoding": message.encoding,
        "step": message.step,
        "is_bigendian": bool(message.is_bigendian),
        "payload_bytes": len(message.data),
    }
    if message.encoding == "32FC1" and np is not None:
        pixels = np.frombuffer(bytes(message.data), dtype=np.float32)
        finite = pixels[np.isfinite(pixels)]
        valid = finite[finite > 0.0]
        content["depth"] = {
            "unit": "metre",
            "pixel_count": int(pixels.size),
            "nan_fraction": float(np.count_nonzero(np.isnan(pixels)) / pixels.size),
            "zero_fraction": float(np.count_nonzero(pixels == 0.0) / pixels.size),
            "valid_fraction": float(valid.size / pixels.size),
        }
        if valid.size:
            content["depth"].update(
                {
                    "min_m": float(valid.min()),
                    "max_m": float(valid.max()),
                    "median_m": float(np.median(valid)),
                    "p05_m": float(np.percentile(valid, 5)),
                    "p95_m": float(np.percentile(valid, 95)),
                }
            )
    return content


def camera_info_content(message):
    """Describe calibration, distortion, and the field of view it implies."""
    intrinsics = list(message.k)
    content = {
        "frame_id": message.header.frame_id,
        "width": message.width,
        "height": message.height,
        "distortion_model": message.distortion_model,
        "d": list(message.d),
        "k": intrinsics,
        "p": list(message.p),
        "binning": [message.binning_x, message.binning_y],
        "roi": {
            "x_offset": message.roi.x_offset,
            "y_offset": message.roi.y_offset,
            "width": message.roi.width,
            "height": message.roi.height,
            "do_rectify": bool(message.roi.do_rectify),
        },
    }
    if len(intrinsics) == 9:
        fx, fy = intrinsics[0], intrinsics[4]
        content["intrinsics"] = {
            "fx": fx,
            "fy": fy,
            "cx": intrinsics[2],
            "cy": intrinsics[5],
            "horizontal_fov_deg": field_of_view(fx, message.width),
            "vertical_fov_deg": field_of_view(fy, message.height),
        }
        content["calibrated"] = bool(fx > 0.0 and fy > 0.0)
    return content


def cloud_content(message):
    """Describe cloud layout and how much of it carries usable geometry."""
    fields = [
        {
            "name": entry.name,
            "offset": entry.offset,
            "datatype": entry.datatype,
            "count": entry.count,
        }
        for entry in message.fields
    ]
    content = {
        "frame_id": message.header.frame_id,
        "width": message.width,
        "height": message.height,
        "point_step": message.point_step,
        "row_step": message.row_step,
        "is_dense": bool(message.is_dense),
        "is_bigendian": bool(message.is_bigendian),
        "fields": fields,
        "field_names": [entry["name"] for entry in fields],
        "declared_points": message.width * message.height,
        "payload_bytes": len(message.data),
    }
    offsets = {entry["name"]: entry["offset"] for entry in fields}
    if np is None or not {"x", "y", "z"} <= set(offsets):
        return content
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    usable = (raw.size // message.point_step) * message.point_step
    if usable == 0:
        return content
    matrix = raw[:usable].reshape(-1, message.point_step)

    def column(name):
        start = offsets[name]
        raw_column = matrix[:, start:start + 4]
        return raw_column.copy().view(np.float32).ravel()

    x, y, z = column("x"), column("y"), column("z")
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    content["valid_points"] = int(np.count_nonzero(finite))
    content["valid_fraction"] = float(np.count_nonzero(finite) / x.size)
    if np.any(finite):
        content["extent_m"] = {
            "x": [float(x[finite].min()), float(x[finite].max())],
            "y": [float(y[finite].min()), float(y[finite].max())],
            "z": [float(z[finite].min()), float(z[finite].max())],
        }
    return content


def scan_content(message):
    """Describe scan geometry and how many returns survive as finite."""
    ranges = list(message.ranges)
    finite = [value for value in ranges if math.isfinite(value)]
    content = {
        "frame_id": message.header.frame_id,
        "ray_count": len(ranges),
        "angle_min": message.angle_min,
        "angle_max": message.angle_max,
        "angle_increment": message.angle_increment,
        "range_min": message.range_min,
        "range_max": message.range_max,
        "scan_time": message.scan_time,
        "time_increment": message.time_increment,
        "finite_returns": len(finite),
        "non_finite_returns": len(ranges) - len(finite),
        "intensities_present": bool(len(message.intensities)),
    }
    if finite:
        content["finite_range_m"] = [min(finite), max(finite)]
    return content


def record_series(record, message):
    """Keep cheap per-message scalars that support noise and drift numbers."""
    if isinstance(message, Imu):
        record.add_series("gyro_x", message.angular_velocity.x)
        record.add_series("gyro_y", message.angular_velocity.y)
        record.add_series("gyro_z", message.angular_velocity.z)
        record.add_series("accel_x", message.linear_acceleration.x)
        record.add_series("accel_y", message.linear_acceleration.y)
        record.add_series("accel_z", message.linear_acceleration.z)
        record.add_series(
            "accel_magnitude",
            math.sqrt(
                message.linear_acceleration.x**2
                + message.linear_acceleration.y**2
                + message.linear_acceleration.z**2
            ),
        )
    elif isinstance(message, Odometry):
        record.add_series("pose_x", message.pose.pose.position.x)
        record.add_series("pose_y", message.pose.pose.position.y)
        record.add_series("twist_x", message.twist.twist.linear.x)
        record.add_series("twist_y", message.twist.twist.linear.y)
        record.add_series("twist_yaw", message.twist.twist.angular.z)
    elif isinstance(message, JointState):
        for index, name in enumerate(message.name):
            if index < len(message.position):
                record.add_series("position:%s" % name, message.position[index])
            if index < len(message.velocity):
                record.add_series("velocity:%s" % name, message.velocity[index])
    elif isinstance(message, Float32):
        record.add_series("value", message.data)


def content_for(message):
    """Dispatch heavy per-message description by message type."""
    if isinstance(message, Image):
        return image_content(message)
    if isinstance(message, CameraInfo):
        return camera_info_content(message)
    if isinstance(message, PointCloud2):
        return cloud_content(message)
    if isinstance(message, LaserScan):
        return scan_content(message)
    return None


class SensorCapabilityProbe(Node):
    """Subscribe to the requested interfaces and accumulate their evidence."""

    def __init__(self, topics, content_samples):
        """Subscribe to every requested topic with a matched QoS profile."""
        super().__init__("sensor_capability_probe")
        self.content_samples = content_samples
        self.records = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for topic in topics:
            message_type = TOPIC_TYPES[topic]
            record = TopicRecord(topic=topic, message_type=message_type.__name__)
            self.records[topic] = record
            self.create_subscription(
                message_type,
                topic,
                self._make_callback(record, message_type),
                self._match_publisher_qos(topic),
            )

    def _match_publisher_qos(self, topic):
        """Adopt the publisher's reliability so no stream is silently dropped."""
        profile = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for endpoint in self.get_publishers_info_by_topic(topic):
            self.records[topic].publisher_qos.append(
                {
                    "node": endpoint.node_name,
                    "namespace": endpoint.node_namespace,
                    "reliability": endpoint.qos_profile.reliability.name,
                    "durability": endpoint.qos_profile.durability.name,
                    "history": endpoint.qos_profile.history.name,
                    "depth": endpoint.qos_profile.depth,
                }
            )
        # BEST_EFFORT subscribers cannot match a RELIABLE-only publisher, so
        # only relax reliability when nothing on the topic requires it.
        reliabilities = {
            entry["reliability"] for entry in self.records[topic].publisher_qos
        }
        if reliabilities and reliabilities != {"BEST_EFFORT"}:
            profile.reliability = ReliabilityPolicy.RELIABLE
        return profile

    def _make_callback(self, record, message_type):
        stamped = not issubclass(message_type, UNSTAMPED_TYPES)

        def callback(message):
            arrival = time.monotonic()
            record.arrivals.append(arrival)
            if stamped:
                stamp = message.header.stamp
                stamp_seconds = stamp.sec + stamp.nanosec * 1e-9
                if stamp_seconds > 0.0:
                    record.latencies.append(time.time() - stamp_seconds)
            try:
                record_series(record, message)
                if len(record.content) < self.content_samples:
                    described = content_for(message)
                    if described is not None:
                        record.content.append(described)
            except Exception as error:  # keep measuring the other topics
                if len(record.errors) < 5:
                    record.errors.append("%s: %s" % (type(error).__name__, error))

        return callback

    def transforms(self, source="base_link"):
        """Report each sensor mount as translation plus roll, pitch, and yaw."""
        results = {}
        for target in TF_TARGETS:
            entry = {"parent": source, "child": target}
            try:
                transform = self.tf_buffer.lookup_transform(
                    source, target, rclpy.time.Time()
                )
            except Exception as error:
                entry["error"] = "%s: %s" % (type(error).__name__, error)
                results[target] = entry
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            entry["xyz_m"] = [translation.x, translation.y, translation.z]
            entry["quaternion_xyzw"] = [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ]
            entry["rpy_deg"] = [
                math.degrees(angle)
                for angle in quaternion_to_rpy(
                    rotation.x, rotation.y, rotation.z, rotation.w
                )
            ]
            results[target] = entry
        return results


def quaternion_to_rpy(x, y, z, w):
    """Convert a quaternion to intrinsic roll, pitch, and yaw in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def summarize(record, duration, dropout_factor):
    """Turn raw arrivals and samples into the numbers the simulator needs."""
    arrivals = record.arrivals
    summary = {
        "topic": record.topic,
        "type": record.message_type,
        "publisher_count": len(record.publisher_qos),
        "publisher_qos": record.publisher_qos,
        "message_count": len(arrivals),
        "observed_duration_s": duration,
    }
    if record.errors:
        summary["errors"] = record.errors
    if len(arrivals) < 2:
        summary["rate_hz"] = None
        summary["note"] = "fewer than two messages observed"
        return summary
    gaps = [later - earlier for earlier, later in zip(arrivals, arrivals[1:])]
    span = arrivals[-1] - arrivals[0]
    # Headline rate is measured between the first and last message, not across
    # the whole window, so a slow subscription match does not understate a
    # healthy sensor. That alone would hide a topic that burst briefly and then
    # went silent, so coverage is checked separately and warned about loudly.
    summary["rate_hz"] = (len(arrivals) - 1) / span if span > 0 else None
    summary["rate_hz_over_window"] = len(arrivals) / duration if duration > 0 else None
    summary["active_span_s"] = span
    if duration > 0 and span < 0.8 * duration:
        summary["coverage_warning"] = (
            "messages covered only %.1f s of the %.1f s window; the publisher "
            "was late, went silent, or stopped -- the rate above describes "
            "only the active span" % (span, duration)
        )
    summary["period_ms"] = describe(gaps, scale=1000.0)
    median_gap = percentile(gaps, 0.5)
    if median_gap and median_gap > 0:
        threshold = median_gap * dropout_factor
        dropouts = [gap for gap in gaps if gap > threshold]
        summary["dropouts"] = {
            "threshold_ms": threshold * 1000.0,
            "count": len(dropouts),
            "worst_gap_ms": max(dropouts) * 1000.0 if dropouts else None,
            "estimated_missed_messages": sum(
                int(round(gap / median_gap)) - 1 for gap in dropouts
            ),
        }
    if record.latencies:
        summary["latency_ms"] = describe(record.latencies, scale=1000.0)
        summary["latency_note"] = (
            "receive clock minus header.stamp; valid only when measured on the "
            "publishing host or against a synchronized clock"
        )
    if record.series:
        summary["series"] = {
            name: describe(values) for name, values in sorted(record.series.items())
        }
    if record.content:
        summary["content_samples"] = len(record.content)
        summary["content"] = record.content[0]
        if len(record.content) > 1:
            summary["content_last"] = record.content[-1]
    return summary


def format_report(result):
    """Render a terminal report that is readable without the JSON file."""
    lines = []
    lines.append("=" * 78)
    lines.append("X3 sensor capability measurement")
    lines.append("=" * 78)
    meta = result["measurement"]
    lines.append("host            : %s" % meta["host"])
    lines.append("started (UTC)   : %s" % meta["started_utc"])
    lines.append("duration        : %.1f s per topic" % meta["duration_s"])
    lines.append("mode            : %s" % meta["mode"])
    lines.append("content samples : %d per topic" % meta["content_samples"])
    lines.append("")
    for summary in result["topics"]:
        lines.append("-" * 78)
        lines.append("%s  [%s]" % (summary["topic"], summary["type"]))
        lines.append("-" * 78)
        if not summary["message_count"]:
            lines.append("  NO MESSAGES RECEIVED "
                         "(publishers seen: %d)" % summary["publisher_count"])
            lines.append("")
            continue
        rate = summary.get("rate_hz")
        lines.append(
            "  messages %d over %.1f s%s"
            % (
                summary["message_count"],
                summary["observed_duration_s"],
                "  ->  %.2f Hz" % rate if rate else "",
            )
        )
        warning = summary.get("coverage_warning")
        if warning:
            over_window = summary.get("rate_hz_over_window")
            lines.append("  !! %s" % warning)
            if over_window:
                lines.append("     (%.2f Hz averaged over the full window)" % over_window)
        period = summary.get("period_ms")
        if period:
            lines.append(
                "  period   median %.1f ms  p95 %.1f ms  max %.1f ms  "
                "(jitter sd %.1f ms)"
                % (
                    period["median"],
                    period["p95"],
                    period["max"],
                    period["stddev"],
                )
            )
        dropouts = summary.get("dropouts")
        if dropouts and dropouts["count"]:
            lines.append(
                "  dropouts %d gaps over %.0f ms, worst %.0f ms, ~%d messages missed"
                % (
                    dropouts["count"],
                    dropouts["threshold_ms"],
                    dropouts["worst_gap_ms"],
                    dropouts["estimated_missed_messages"],
                )
            )
        latency = summary.get("latency_ms")
        if latency:
            lines.append(
                "  latency  median %.1f ms  p95 %.1f ms  max %.1f ms"
                % (latency["median"], latency["p95"], latency["max"])
            )
        content = summary.get("content")
        if content:
            for line in format_content(content):
                lines.append("  %s" % line)
        lines.append("")
    lines.append("-" * 78)
    lines.append("Sensor mounts relative to base_link")
    lines.append("-" * 78)
    for name, entry in result["transforms"].items():
        if "error" in entry:
            lines.append("  %-30s unavailable (%s)" % (name, entry["error"]))
            continue
        x, y, z = entry["xyz_m"]
        roll, pitch, yaw = entry["rpy_deg"]
        lines.append(
            "  %-30s xyz [%+.5f %+.5f %+.5f] m   rpy [%+.2f %+.2f %+.2f] deg"
            % (name, x, y, z, roll, pitch, yaw)
        )
    lines.append("")
    return "\n".join(lines)


def format_content(content):
    """Render the type-specific part of one topic's report."""
    lines = []
    if "encoding" in content:
        lines.append(
            "content  %dx%d %s, step %d, %d bytes, frame %s"
            % (
                content["width"],
                content["height"],
                content["encoding"],
                content["step"],
                content["payload_bytes"],
                content["frame_id"],
            )
        )
        depth = content.get("depth")
        if depth:
            lines.append(
                "depth    valid %.1f%%  zero %.1f%%  nan %.1f%%"
                % (
                    depth["valid_fraction"] * 100.0,
                    depth["zero_fraction"] * 100.0,
                    depth["nan_fraction"] * 100.0,
                )
            )
            if "median_m" in depth:
                lines.append(
                    "range    %.3f - %.3f m (median %.3f, p05 %.3f, p95 %.3f)"
                    % (
                        depth["min_m"],
                        depth["max_m"],
                        depth["median_m"],
                        depth["p05_m"],
                        depth["p95_m"],
                    )
                )
    elif "distortion_model" in content:
        lines.append(
            "content  %dx%d, model '%s', frame %s"
            % (
                content["width"],
                content["height"],
                content["distortion_model"],
                content["frame_id"],
            )
        )
        intrinsics = content.get("intrinsics")
        if intrinsics:
            lines.append(
                "intrinsics fx %.3f fy %.3f cx %.3f cy %.3f  (calibrated: %s)"
                % (
                    intrinsics["fx"],
                    intrinsics["fy"],
                    intrinsics["cx"],
                    intrinsics["cy"],
                    content.get("calibrated"),
                )
            )
            if intrinsics["horizontal_fov_deg"]:
                lines.append(
                    "fov      horizontal %.2f deg  vertical %.2f deg"
                    % (
                        intrinsics["horizontal_fov_deg"],
                        intrinsics["vertical_fov_deg"],
                    )
                )
        lines.append("distortion d = %s" % content["d"])
    elif "point_step" in content:
        lines.append(
            "content  %d x %d points, step %d, %d bytes, frame %s"
            % (
                content["width"],
                content["height"],
                content["point_step"],
                content["payload_bytes"],
                content["frame_id"],
            )
        )
        lines.append("fields   %s" % ", ".join(content["field_names"]))
        if "valid_fraction" in content:
            lines.append(
                "geometry valid %d/%d points (%.1f%%)"
                % (
                    content["valid_points"],
                    content["declared_points"],
                    content["valid_fraction"] * 100.0,
                )
            )
        extent = content.get("extent_m")
        if extent:
            lines.append(
                "extent   x [%.3f %.3f]  y [%.3f %.3f]  z [%.3f %.3f] m"
                % (
                    extent["x"][0],
                    extent["x"][1],
                    extent["y"][0],
                    extent["y"][1],
                    extent["z"][0],
                    extent["z"][1],
                )
            )
    elif "ray_count" in content:
        lines.append(
            "content  %d rays, frame %s, %.4f rad increment"
            % (content["ray_count"], content["frame_id"], content["angle_increment"])
        )
        lines.append(
            "geometry angle [%.5f %.5f] rad, range [%.3f %.3f] m"
            % (
                content["angle_min"],
                content["angle_max"],
                content["range_min"],
                content["range_max"],
            )
        )
        lines.append(
            "timing   scan_time %.4f s, time_increment %.7f s"
            % (content["scan_time"], content["time_increment"])
        )
        lines.append(
            "returns  finite %d, non-finite %d"
            % (content["finite_returns"], content["non_finite_returns"])
        )
    return lines


def resolve_topics(args):
    """Expand group names and explicit topics into one ordered topic list."""
    selected = []
    for group in args.group:
        if group == "all":
            for name in GROUPS:
                selected.extend(GROUPS[name])
        elif group in GROUPS:
            selected.extend(GROUPS[group])
        else:
            raise SystemExit(
                "unknown group '%s'; choose from: all, %s"
                % (group, ", ".join(sorted(GROUPS)))
            )
    selected.extend(args.topic)
    unknown = [topic for topic in selected if topic not in TOPIC_TYPES]
    if unknown:
        raise SystemExit("unknown topic(s): %s" % ", ".join(unknown))
    ordered = []
    for topic in selected:
        if topic not in ordered:
            ordered.append(topic)
    return ordered


def measure(topics, duration, content_samples, dropout_factor, sequential):
    """Run the probe and return one structured measurement result."""
    import socket

    started = time.gmtime()
    summaries = []
    transforms = {}
    batches = [[topic] for topic in topics] if sequential else [topics]
    for batch in batches:
        probe = SensorCapabilityProbe(batch, content_samples)
        # The TF listener needs the tree before the first lookup, and a fresh
        # subscription needs a moment to match its publisher.
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        observed = duration
        for topic in batch:
            summaries.append(
                summarize(probe.records[topic], observed, dropout_factor)
            )
        if not transforms:
            transforms = probe.transforms()
        probe.destroy_node()
    return {
        "measurement": {
            "host": socket.gethostname(),
            "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", started),
            "duration_s": duration,
            "mode": "sequential" if sequential else "concurrent",
            "content_samples": content_samples,
            "dropout_factor": dropout_factor,
            "topics": topics,
        },
        "topics": summaries,
        "transforms": transforms,
    }


def main():
    """Parse arguments, measure the selected interfaces, and report."""
    parser = argparse.ArgumentParser(
        description="Measure physical X3 sensor capabilities for the simulator."
    )
    parser.add_argument(
        "--group",
        action="append",
        default=[],
        help="topic group: all, %s (repeatable)" % ", ".join(sorted(GROUPS)),
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="explicit topic to measure (repeatable)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="seconds to observe, per batch (default: 15)",
    )
    parser.add_argument(
        "--content-samples",
        type=int,
        default=3,
        help="heavy per-message descriptions to keep per topic (default: 3)",
    )
    parser.add_argument(
        "--dropout-factor",
        type=float,
        default=2.5,
        help="gap over this multiple of the median period counts as a dropout",
    )
    parser.add_argument(
        "--sequential",
        action="store_true",
        help="measure one topic at a time so the probe does not load the others",
    )
    parser.add_argument("--output", help="write the full JSON result to this path")
    args = parser.parse_args()

    if not args.group and not args.topic:
        args.group = ["camera"]
    topics = resolve_topics(args)
    if args.duration <= 0.0 or args.content_samples < 0:
        raise SystemExit("duration must be positive and content samples non-negative")

    rclpy.init()
    try:
        result = measure(
            topics,
            args.duration,
            args.content_samples,
            args.dropout_factor,
            args.sequential,
        )
    finally:
        rclpy.shutdown()

    print(format_report(result))
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2, sort_keys=True)
        print("JSON written to %s" % args.output)

    silent = [entry["topic"] for entry in result["topics"] if not entry["message_count"]]
    if silent:
        print("WARNING: no messages on: %s" % ", ".join(silent), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
