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
produced a number before comparing two numbers. For the same reason every
topic's other subscribers are recorded at the end of its window.

Timing is reported two ways. Arrival gaps (receive clock) describe what a
subscriber experiences; header-stamp gaps describe what the publisher
produced. The camera captures on a fixed frame clock, so camera stamp gaps
are also counted in whole frames, which is how the point cloud loses data.
For topics the boot gate checks, the gate's own rate statistic from
physical_contract_probe.py is applied to every window of the capture, so the
margin against its floor is measured in the gate's terms. --per-message keeps
the raw per-message columns so a gap model can be fitted afterwards.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import hashlib
import json
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
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

from physical_contract_probe import DEFAULT_SAMPLES, median_stamp_rate, RATE_LIMITS_HZ

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

# Types whose stamps come from the camera's frame clock, so their gaps are
# whole multiples of one frame period.
FRAME_QUANTIZED_TYPES = {"Image", "CameraInfo", "PointCloud2"}

# Adapter settings that change the cloud's shape and cost. They are recorded
# with every measurement so two numbers are never compared across settings.
ADAPTER_NODE = "/astra_sensor_adapter"
ADAPTER_PARAMETERS = ["cloud_strip_nan", "cloud_decimation", "target_cloud_frame"]


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
    """
    Accumulated arrival, stamp, and content evidence for one topic.

    ``arrivals``, ``receipts``, ``stamps`` and ``payload_bytes`` hold one entry
    per received message, in arrival order: the monotonic receive time, the
    wall-clock receive time, the header stamp (None when there is none), and
    the payload size (None for types without a bulk payload).
    """

    topic: str
    message_type: str
    arrivals: list = field(default_factory=list)
    receipts: list = field(default_factory=list)
    stamps: list = field(default_factory=list)
    payload_bytes: list = field(default_factory=list)
    content: list = field(default_factory=list)
    series: dict = field(default_factory=dict)
    publisher_qos: list = field(default_factory=list)
    other_subscribers: list = None
    errors: list = field(default_factory=list)

    def add_series(self, name, value):
        """Record one cheap scalar sample kept for every message."""
        if value is None or not math.isfinite(value):
            return
        self.series.setdefault(name, []).append(value)


def payload_size(message):
    """Return the bulk payload size in bytes, or None for small messages."""
    if isinstance(message, (Image, PointCloud2)):
        return len(message.data)
    return None


def frame_cadence(stamp_gaps, frame_rate):
    """
    Count header-stamp gaps in whole camera frames.

    A consumer that cannot keep up with the camera loses whole frames, so a
    gap of three frame periods means two frames were skipped. Gaps that do
    not land near a frame boundary are counted separately, since they mean
    the stamps do not follow the frame clock the way this assumes.
    """
    if not frame_rate or frame_rate <= 0.0:
        return None
    frame_period = 1.0 / frame_rate
    frames = [gap / frame_period for gap in stamp_gaps if gap > 0.0]
    if not frames:
        return None
    whole = [max(1, int(round(value))) for value in frames]
    histogram = {}
    for count in whole:
        histogram[count] = histogram.get(count, 0) + 1
    return {
        "frame_rate_hz": frame_rate,
        "frames_per_gap": {str(count): histogram[count] for count in sorted(histogram)},
        "skipped_frames": sum(count - 1 for count in whole),
        "delivered_fraction": len(whole) / sum(whole),
        "off_grid_gaps": sum(1 for value in frames if abs(value - round(value)) > 0.25),
    }


def contract_windows(stamps, limits, window=DEFAULT_SAMPLES):
    """
    Apply the boot gate's rate statistic to every window of consecutive stamps.

    The gate judges a topic on the first few messages after it subscribes, so
    its verdict depends on which messages it happens to catch. Sliding its
    exact statistic across the capture shows how much margin it has.
    """
    rates = []
    for start in range(len(stamps) - window + 1):
        rate = median_stamp_rate(stamps[start:start + window])
        if rate is not None:
            rates.append(rate)
    if not rates:
        return None
    minimum, maximum = limits
    return {
        "window_messages": window,
        "limits_hz": [minimum, maximum],
        "windows": len(rates),
        "first_window_hz": rates[0],
        "min_hz": min(rates),
        "p05_hz": percentile(rates, 0.05),
        "median_hz": percentile(rates, 0.5),
        "windows_outside_limits": sum(
            1 for rate in rates if not minimum <= rate <= maximum
        ),
    }


def other_endpoints(endpoints, own_name, own_namespace):
    """Describe graph endpoints that do not belong to this probe."""
    return [
        {"node": endpoint.node_name, "namespace": endpoint.node_namespace}
        for endpoint in endpoints
        if (endpoint.node_name, endpoint.node_namespace) != (own_name, own_namespace)
    ]


def git_blob_id(content):
    """Return the id ``git hash-object`` gives this content."""
    header = b"blob %d\0" % len(content)
    return hashlib.sha1(header + content).hexdigest()


def host_uptime():
    """Return seconds since the host booted, or None off Linux."""
    try:
        with open("/proc/uptime", encoding="utf-8") as handle:
            return float(handle.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def parse_notes(entries):
    """Turn repeated KEY=VALUE arguments into a dictionary."""
    notes = {}
    for entry in entries:
        key, separator, value = entry.partition("=")
        if not separator or not key.strip():
            raise SystemExit("--note expects KEY=VALUE, got '%s'" % entry)
        notes[key.strip()] = value.strip()
    return notes


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
            receipt = time.time()
            stamp_seconds = None
            if stamped:
                stamp = message.header.stamp
                value = stamp.sec + stamp.nanosec * 1e-9
                if value > 0.0:
                    stamp_seconds = value
            record.arrivals.append(arrival)
            record.receipts.append(receipt)
            record.stamps.append(stamp_seconds)
            record.payload_bytes.append(payload_size(message))
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

    def record_other_subscribers(self):
        """Note who else was subscribed to each topic by the end of the window."""
        for topic, record in self.records.items():
            record.other_subscribers = other_endpoints(
                self.get_subscriptions_info_by_topic(topic),
                self.get_name(),
                self.get_namespace(),
            )

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


def read_parameters(node_name, names, timeout=2.0):
    """
    Read parameters from another node, reporting why when it cannot.

    Uses its own short-lived node: spinning the probe itself here would run
    its subscription callbacks and record messages outside the window.
    """
    node = rclpy.create_node("sensor_capability_probe_parameters")
    client = node.create_client(GetParameters, node_name + "/get_parameters")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return {"error": "%s is not running" % node_name}
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            return {"error": "no parameter reply from %s" % node_name}
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, future.result().values)
        }
    finally:
        node.destroy_node()


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


def summarize(
    record,
    duration,
    dropout_factor,
    window_start=None,
    frame_rate=0.0,
    per_message=False,
):
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
    if record.other_subscribers is not None:
        summary["other_subscriber_count"] = len(record.other_subscribers)
        summary["other_subscribers"] = record.other_subscribers
    if record.errors:
        summary["errors"] = record.errors
    if per_message and arrivals:
        origin = arrivals[0] if window_start is None else window_start
        summary["per_message"] = {
            "arrival_s": [round(arrival - origin, 6) for arrival in arrivals],
            "stamp_s": [
                None if stamp is None else round(stamp, 6) for stamp in record.stamps
            ],
            "latency_ms": [
                None if stamp is None else round((receipt - stamp) * 1000.0, 3)
                for receipt, stamp in zip(record.receipts, record.stamps)
            ],
            "payload_bytes": record.payload_bytes,
        }
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
    latencies = [
        receipt - stamp
        for receipt, stamp in zip(record.receipts, record.stamps)
        if stamp is not None
    ]
    if latencies:
        summary["latency_ms"] = describe(latencies, scale=1000.0)
        summary["latency_note"] = (
            "receive clock minus header.stamp; valid only when measured on the "
            "publishing host or against a synchronized clock"
        )
    stamps = [stamp for stamp in record.stamps if stamp is not None]
    stamp_gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
    if stamp_gaps:
        summary["stamp_period_ms"] = describe(stamp_gaps, scale=1000.0)
        backwards = sum(1 for gap in stamp_gaps if gap <= 0.0)
        if backwards:
            summary["non_increasing_stamps"] = backwards
        if record.message_type in FRAME_QUANTIZED_TYPES:
            cadence = frame_cadence(stamp_gaps, frame_rate)
            if cadence:
                summary["frame_cadence"] = cadence
    if record.topic in RATE_LIMITS_HZ:
        gate = contract_windows(stamps, RATE_LIMITS_HZ[record.topic])
        if gate:
            summary["contract_rate_hz"] = gate
    payloads = [size for size in record.payload_bytes if size is not None]
    if payloads:
        summary["payload_bytes"] = describe(payloads)
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
    if meta.get("host_uptime_s") is not None:
        lines.append("host uptime     : %.0f s at start" % meta["host_uptime_s"])
    if meta.get("adapter_parameters"):
        lines.append(
            "adapter         : %s"
            % ", ".join(
                "%s=%s" % item for item in sorted(meta["adapter_parameters"].items())
            )
        )
    for key, value in sorted(meta.get("notes", {}).items()):
        lines.append("note            : %s=%s" % (key, value))
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
        stamp_period = summary.get("stamp_period_ms")
        if stamp_period:
            lines.append(
                "  stamps   period median %.1f ms  p95 %.1f ms  max %.1f ms%s"
                % (
                    stamp_period["median"],
                    stamp_period["p95"],
                    stamp_period["max"],
                    "  (%d non-increasing)" % summary["non_increasing_stamps"]
                    if summary.get("non_increasing_stamps")
                    else "",
                )
            )
        cadence = summary.get("frame_cadence")
        if cadence:
            lines.append(
                "  frames   %.1f%% of %.0f Hz frames delivered, %d skipped, "
                "%d off-grid gaps"
                % (
                    cadence["delivered_fraction"] * 100.0,
                    cadence["frame_rate_hz"],
                    cadence["skipped_frames"],
                    cadence["off_grid_gaps"],
                )
            )
            lines.append(
                "           frames per gap: %s"
                % "  ".join(
                    "%sx%d" % item for item in cadence["frames_per_gap"].items()
                )
            )
        gate = summary.get("contract_rate_hz")
        if gate:
            lines.append(
                "  gate     %d-message rate: first %.2f  min %.2f  p05 %.2f  "
                "median %.2f Hz; %d/%d windows outside %.1f..%.1f Hz"
                % (
                    gate["window_messages"],
                    gate["first_window_hz"],
                    gate["min_hz"],
                    gate["p05_hz"],
                    gate["median_hz"],
                    gate["windows_outside_limits"],
                    gate["windows"],
                    gate["limits_hz"][0],
                    gate["limits_hz"][1],
                )
            )
        payload = summary.get("payload_bytes")
        if payload:
            lines.append(
                "  payload  median %.0f bytes  min %.0f  max %.0f"
                % (payload["median"], payload["min"], payload["max"])
            )
        if "other_subscribers" in summary:
            lines.append(
                "  others   %s"
                % (
                    ", ".join(
                        entry["namespace"].rstrip("/") + "/" + entry["node"]
                        for entry in summary["other_subscribers"]
                    )
                    or "none subscribed"
                )
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


def measure(
    topics,
    duration,
    content_samples,
    dropout_factor,
    sequential,
    frame_rate=0.0,
    per_message=False,
    notes=None,
):
    """Run the probe and return one structured measurement result."""
    import socket

    started = time.gmtime()
    uptime = host_uptime()
    adapter_parameters = read_parameters(ADAPTER_NODE, ADAPTER_PARAMETERS)
    summaries = []
    transforms = {}
    batches = [[topic] for topic in topics] if sequential else [topics]
    for batch in batches:
        probe = SensorCapabilityProbe(batch, content_samples)
        # The TF listener needs the tree before the first lookup, and a fresh
        # subscription needs a moment to match its publisher.
        window_start = time.monotonic()
        deadline = window_start + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        observed = duration
        probe.record_other_subscribers()
        for topic in batch:
            summaries.append(
                summarize(
                    probe.records[topic],
                    observed,
                    dropout_factor,
                    window_start=window_start,
                    frame_rate=frame_rate,
                    per_message=per_message,
                )
            )
        if not transforms:
            transforms = probe.transforms()
        probe.destroy_node()
    try:
        with open(__file__, "rb") as handle:
            probe_blob = git_blob_id(handle.read())
    except OSError:
        probe_blob = None
    return {
        "measurement": {
            "host": socket.gethostname(),
            "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", started),
            "host_uptime_s": uptime,
            "duration_s": duration,
            "mode": "sequential" if sequential else "concurrent",
            "content_samples": content_samples,
            "dropout_factor": dropout_factor,
            "camera_frame_rate_hz": frame_rate,
            "per_message": per_message,
            "topics": topics,
            "adapter_parameters": adapter_parameters,
            "notes": notes or {},
            "probe_git_blob": probe_blob,
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
    parser.add_argument(
        "--camera-frame-rate",
        type=float,
        default=30.0,
        help="camera frame clock used to count stamp gaps in whole frames; the "
        "Astra is configured for 30 Hz in astra_platform.launch.py (0 disables)",
    )
    parser.add_argument(
        "--per-message",
        action="store_true",
        help="keep per-message arrival, stamp, latency and size columns in the JSON",
    )
    parser.add_argument(
        "--note",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="condition recorded with the result, e.g. commit=b144d31 (repeatable)",
    )
    parser.add_argument("--output", help="write the full JSON result to this path")
    args = parser.parse_args()

    if not args.group and not args.topic:
        args.group = ["camera"]
    topics = resolve_topics(args)
    if args.duration <= 0.0 or args.content_samples < 0:
        raise SystemExit("duration must be positive and content samples non-negative")
    if args.camera_frame_rate < 0.0:
        raise SystemExit("camera frame rate must not be negative")
    notes = parse_notes(args.note)

    rclpy.init()
    try:
        result = measure(
            topics,
            args.duration,
            args.content_samples,
            args.dropout_factor,
            args.sequential,
            frame_rate=args.camera_frame_rate,
            per_message=args.per_message,
            notes=notes,
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
