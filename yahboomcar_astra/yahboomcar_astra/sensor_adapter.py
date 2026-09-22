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

"""Normalize the Orbbec Astra driver to the shared simulator contract."""

from __future__ import annotations

import math
import sys
from typing import Iterable

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


_FIELD_BYTES = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}


def metric_depth(message: Image, scale: float) -> Image:
    """Return a tightly packed 32FC1 depth image measured in metres."""
    if message.width == 0 or message.height == 0:
        raise ValueError("depth image dimensions must be non-zero")

    if message.encoding == "16UC1":
        byte_order = ">" if message.is_bigendian else "<"
        source = np.frombuffer(message.data, dtype=byte_order + "u2")
        row_width = message.step // 2
        if row_width < message.width or source.size < row_width * message.height:
            raise ValueError("malformed 16UC1 depth image")
        source = source.reshape(message.height, row_width)[:, : message.width]
        converted = source.astype(np.float32) * np.float32(scale)
        converted[source == 0] = np.nan
    elif message.encoding == "32FC1":
        byte_order = ">" if message.is_bigendian else "<"
        source = np.frombuffer(message.data, dtype=byte_order + "f4")
        row_width = message.step // 4
        if row_width < message.width or source.size < row_width * message.height:
            raise ValueError("malformed 32FC1 depth image")
        converted = np.asarray(
            source.reshape(message.height, row_width)[:, : message.width],
            dtype=np.float32,
        )
    else:
        raise ValueError("unsupported depth encoding: %s" % message.encoding)

    output = Image()
    output.header = message.header
    output.height = message.height
    output.width = message.width
    output.encoding = "32FC1"
    output.is_bigendian = False
    output.step = message.width * 4
    # Fill the generated array directly.  Assigning through the ROS property
    # makes its Python setter validate every byte before doing the same copy.
    output.data.frombytes(
        np.ascontiguousarray(converted, dtype="<f4").tobytes()
    )
    return output


def rgb_image(message: Image) -> Image:
    """Return a tightly packed RGB8 image without requiring cv_bridge."""
    channels_by_encoding = {"rgb8": (0, 1, 2), "bgr8": (2, 1, 0)}
    if message.encoding not in channels_by_encoding:
        raise ValueError("unsupported color encoding: %s" % message.encoding)
    if message.step < message.width * 3:
        raise ValueError("malformed color image")

    source = np.frombuffer(message.data, dtype=np.uint8)
    if source.size < message.step * message.height:
        raise ValueError("truncated color image")
    rows = source.reshape(message.height, message.step)
    pixels = rows[:, : message.width * 3].reshape(
        message.height, message.width, 3
    )
    converted = pixels[:, :, channels_by_encoding[message.encoding]]

    output = Image()
    output.header = message.header
    output.height = message.height
    output.width = message.width
    output.encoding = "rgb8"
    output.is_bigendian = False
    output.step = message.width * 3
    output.data.frombytes(np.ascontiguousarray(converted).tobytes())
    return output


def _rotation_matrix(quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_cloud(
    message: PointCloud2,
    translation: Iterable[float],
    quaternion: Iterable[float],
    target_frame: str,
    decimation: int = 1,
    strip_nan: bool = False,
) -> PointCloud2:
    """
    Return the cloud rotated into ``target_frame``, tightly packed.

    The Orbbec driver publishes 32-byte points that carry only 16 bytes of
    x, y, z and rgb, so half of every cloud is padding -- 1.2 MB of the
    2.4 MB measured on ``x3-c``.  This topic is the platform's measured
    bottleneck (see ``docs/sensor_capabilities.md``), and it is slow enough
    at boot to fail the contract probe's lower bound and suppress the
    boot-ready signal, so the padding is worth removing rather than
    forwarding.

    ``decimation`` optionally subsamples the point grid (e.g. keeping every
    2nd row and column) before coordinate transformation.

    ``strip_nan`` drops non-finite (NaN / Inf) depth returns, producing an
    unorganized cloud (height=1) of valid 3D points. Because ~61.5% of the
    Astra depth returns on hardware are NaN, this drops message size and DDS
    overhead significantly with no information loss.

    Unlike the other conversions here this one cannot edit the callback's
    buffer in place, because changing the point stride is the whole point of
    the pass.  The cost of the extra copy is bounded and local; the bytes it
    saves are serialized and delivered to every subscriber.  As in
    ``metric_depth``, the output buffer is filled through the generated array
    directly, since assigning through the ROS property makes its Python
    setter validate every byte before doing the same copy.
    """
    if not isinstance(decimation, int) or decimation < 1:
        raise ValueError("point cloud decimation must be a positive integer")

    fields_by_name = {field.name: field for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields_by_name):
        raise ValueError("point cloud does not contain x, y, and z fields")
    if message.point_step <= 0 or message.row_step != message.point_step * message.width:
        raise ValueError("point cloud contains unsupported row padding")

    expected_size = message.row_step * message.height
    if len(message.data) < expected_size:
        raise ValueError("truncated point cloud")

    source = message.data
    byte_order = ">" if message.is_bigendian else "<"

    if message.height > 1:
        row_indices = slice(0, message.height, decimation)
        col_indices = slice(0, message.width, decimation)
        kept_height = (message.height + decimation - 1) // decimation
        kept_width = (message.width + decimation - 1) // decimation
        coordinates = [
            np.ndarray(
                shape=(message.height, message.width),
                dtype=byte_order + "f4",
                buffer=source,
                offset=fields_by_name[name].offset,
                strides=(message.row_step, message.point_step),
            )[row_indices, col_indices].ravel()
            for name in ("x", "y", "z")
        ]
    else:
        col_indices = slice(0, message.width, decimation)
        kept_height = 1
        kept_width = (message.width + decimation - 1) // decimation
        coordinates = [
            np.ndarray(
                shape=(message.width,),
                dtype=byte_order + "f4",
                buffer=source,
                offset=fields_by_name[name].offset,
                strides=(message.point_step,),
            )[col_indices]
            for name in ("x", "y", "z")
        ]

    # Colour is copied as raw bytes rather than reinterpreted, so whatever
    # packing the driver chose survives untouched.  That is only safe while
    # the field really is the four bytes the packed layout reserves for it.
    colour = fields_by_name.get("rgb")
    if colour is not None and (
        _FIELD_BYTES.get(colour.datatype) != 4 or colour.count != 1
    ):
        raise ValueError("point cloud rgb field is not a single four-byte value")

    decimated_colour = None
    if colour is not None:
        if message.height > 1:
            colour_grid = np.ndarray(
                shape=(message.height, message.width, 4),
                dtype=np.uint8,
                buffer=source,
                offset=colour.offset,
                strides=(message.row_step, message.point_step, 1),
            )
            decimated_colour = colour_grid[row_indices, col_indices, :].reshape(-1, 4)
        else:
            colour_1d = np.ndarray(
                shape=(message.width, 4),
                dtype=np.uint8,
                buffer=source,
                offset=colour.offset,
                strides=(message.point_step, 1),
            )
            decimated_colour = colour_1d[col_indices, :]

    xyz = np.column_stack(coordinates).astype(np.float64, copy=False)
    finite = np.all(np.isfinite(xyz), axis=1)
    if np.any(finite):
        rotation = _rotation_matrix(quaternion)
        offset = np.asarray(tuple(translation), dtype=np.float64)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("invalid transform translation")
        xyz[finite] = xyz[finite] @ rotation.T + offset

    if strip_nan:
        out_xyz = xyz[finite]
        out_colour = decimated_colour[finite] if colour is not None else None
        output_height = 1
        output_width = len(out_xyz)
        is_dense = True
    else:
        out_xyz = xyz
        out_colour = decimated_colour
        output_height = kept_height
        output_width = kept_width
        is_dense = bool(message.is_dense and not np.any(~finite))

    packed_step = 12 if colour is None else 16
    num_output_points = output_height * output_width
    packed = np.empty((num_output_points, packed_step), dtype=np.uint8)
    if num_output_points > 0:
        packed[:, 0:12] = np.ascontiguousarray(out_xyz, dtype="<f4").view(np.uint8)
        if colour is not None:
            packed[:, 12:16] = np.ascontiguousarray(out_colour, dtype=np.uint8)

    output = PointCloud2()
    output.header.stamp = message.header.stamp
    output.header.frame_id = target_frame
    output.height = output_height
    output.width = output_width
    output.is_dense = is_dense
    output.is_bigendian = False
    output.point_step = packed_step
    output.row_step = packed_step * output_width
    output.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if colour is not None:
        output.fields.append(
            PointField(name="rgb", offset=12, datatype=colour.datatype, count=1)
        )
    output.data.frombytes(packed.tobytes())
    return output


class AstraSensorAdapter(Node):
    """Expose one strict, normalized RGB-D interface for physical hardware."""

    def __init__(self) -> None:
        super().__init__("astra_sensor_adapter")
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("startup_timeout", 20.0)
        self.declare_parameter("target_cloud_frame", "cam_1_depth_frame")
        self.declare_parameter("cloud_decimation", 1)
        self.declare_parameter("cloud_strip_nan", True)
        self.depth_unit_scale = float(self.get_parameter("depth_unit_scale").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.target_cloud_frame = str(
            self.get_parameter("target_cloud_frame").value
        )
        self.cloud_decimation = int(self.get_parameter("cloud_decimation").value)
        self.cloud_strip_nan = bool(self.get_parameter("cloud_strip_nan").value)
        if self.depth_unit_scale <= 0.0 or self.startup_timeout <= 0.0:
            raise ValueError("camera adapter scale and timeout must be positive")
        if self.cloud_decimation < 1:
            raise ValueError("cloud_decimation must be at least 1")

        self.color_publisher = self.create_publisher(
            Image, "/cam_1/color/image_raw", qos_profile_sensor_data
        )
        self.color_info_publisher = self.create_publisher(
            CameraInfo, "/cam_1/color/camera_info", qos_profile_sensor_data
        )
        self.depth_publisher = self.create_publisher(
            Image, "/cam_1/depth/image_raw", qos_profile_sensor_data
        )
        self.depth_info_publisher = self.create_publisher(
            CameraInfo, "/cam_1/depth/camera_info", qos_profile_sensor_data
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2, "/cam_1/depth/color/points", qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            "/_hardware/astra/color/image_raw",
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/_hardware/astra/color/camera_info",
            self._color_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/_hardware/astra/depth/image_raw",
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/_hardware/astra/depth/camera_info",
            self._depth_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/_hardware/astra/depth/color/points",
            self._cloud_callback,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.required_streams = {
            "color": False,
            "color_info": False,
            "depth": False,
            "depth_info": False,
            "cloud": False,
        }
        self.start_time = self.get_clock().now()
        self.create_timer(1.0, self._startup_watchdog)

    def _conversion_error(self, stream: str, error: Exception) -> None:
        self.get_logger().error(
            "Dropping invalid %s message: %s" % (stream, error),
            throttle_duration_sec=5.0,
        )

    def _color_callback(self, message: Image) -> None:
        try:
            self.color_publisher.publish(rgb_image(message))
            self.required_streams["color"] = True
        except ValueError as error:
            self._conversion_error("color", error)

    @staticmethod
    def _calibration_valid(message: CameraInfo) -> bool:
        return (
            message.width > 0
            and message.height > 0
            and len(message.k) == 9
            and math.isfinite(message.k[0])
            and math.isfinite(message.k[4])
            and message.k[0] > 0.0
            and message.k[4] > 0.0
        )

    def _color_info_callback(self, message: CameraInfo) -> None:
        if not self._calibration_valid(message):
            self._conversion_error("color camera_info", ValueError("invalid intrinsics"))
            return
        self.color_info_publisher.publish(message)
        self.required_streams["color_info"] = True

    def _depth_callback(self, message: Image) -> None:
        try:
            self.depth_publisher.publish(metric_depth(message, self.depth_unit_scale))
            self.required_streams["depth"] = True
        except ValueError as error:
            self._conversion_error("depth", error)

    def _depth_info_callback(self, message: CameraInfo) -> None:
        if not self._calibration_valid(message):
            self._conversion_error("depth camera_info", ValueError("invalid intrinsics"))
            return
        self.depth_info_publisher.publish(message)
        self.required_streams["depth_info"] = True

    def _cloud_callback(self, message: PointCloud2) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_cloud_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.05),
            ).transform
            output = transform_cloud(
                message,
                (transform.translation.x, transform.translation.y, transform.translation.z),
                (
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ),
                self.target_cloud_frame,
                decimation=self.cloud_decimation,
                strip_nan=self.cloud_strip_nan,
            )
            self.cloud_publisher.publish(output)
            self.required_streams["cloud"] = True
        except (TransformException, ValueError) as error:
            self._conversion_error("point cloud", error)

    def _startup_watchdog(self) -> None:
        if all(self.required_streams.values()):
            return
        age = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if age > self.startup_timeout:
            missing = sorted(
                name for name, seen in self.required_streams.items() if not seen
            )
            raise RuntimeError(
                "Astra strict startup failed; missing valid streams: %s"
                % ", ".join(missing)
            )


def main(args=None) -> None:
    """Run the adapter and return non-zero on strict startup failure."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = AstraSensorAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - process must fail closed
        exit_code = 1
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print("Astra adapter failed: %s" % error, file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
