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

"""Unit tests for camera encoding and coordinate normalization."""

import math
import struct

import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField

from yahboomcar_astra.sensor_adapter import metric_depth, rgb_image, transform_cloud


def test_metric_depth_converts_millimetres_and_marks_zero_invalid():
    message = Image(height=1, width=2, encoding='16UC1', step=4)
    message.data = np.asarray([1000, 0], dtype='<u2').tobytes()

    output = metric_depth(message, 0.001)
    values = np.frombuffer(output.data, dtype='<f4')

    assert output.encoding == '32FC1'
    assert output.step == 8
    assert math.isclose(values[0], 1.0)
    assert math.isnan(values[1])


def test_rgb_image_swaps_bgr_channels():
    message = Image(height=1, width=1, encoding='bgr8', step=3)
    message.data = bytes((10, 20, 30))

    output = rgb_image(message)

    assert output.encoding == 'rgb8'
    assert bytes(output.data) == bytes((30, 20, 10))


def test_transform_cloud_changes_xyz_and_preserves_rgb_bytes():
    message = PointCloud2(height=1, width=1, point_step=16, row_step=16)
    message.header.frame_id = 'source_frame'
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.data = struct.pack('<fffBBBB', 1.0, 2.0, 3.0, 4, 5, 6, 7)

    output = transform_cloud(
        message, (10.0, 20.0, 30.0), (0.0, 0.0, 0.0, 1.0), 'target_frame'
    )
    x, y, z = struct.unpack_from('<fff', output.data)

    assert output.header.frame_id == 'target_frame'
    assert (x, y, z) == (11.0, 22.0, 33.0)
    assert bytes(output.data[12:16]) == bytes((4, 5, 6, 7))


def _orbbec_shaped_cloud(points):
    """Build a cloud with the driver's 32-byte stride and rgb at offset 16."""
    message = PointCloud2(
        height=1, width=len(points), point_step=32, row_step=32 * len(points)
    )
    message.header.frame_id = 'cam_1_color_optical_frame'
    message.is_dense = False
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    buffer = bytearray(32 * len(points))
    for index, (xyz, colour) in enumerate(points):
        struct.pack_into('<fff', buffer, index * 32, *xyz)
        buffer[index * 32 + 16:index * 32 + 20] = colour
    message.data = bytes(buffer)
    return message


def test_transform_cloud_packs_out_the_drivers_point_padding():
    message = _orbbec_shaped_cloud([((1.0, 2.0, 3.0), bytes((4, 5, 6, 7)))])

    output = transform_cloud(
        message, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame'
    )

    assert output.point_step == 16
    assert output.row_step == 16
    assert len(output.data) == len(message.data) // 2
    assert [(field.name, field.offset) for field in output.fields] == [
        ('x', 0), ('y', 4), ('z', 8), ('rgb', 12)
    ]
    assert bytes(output.data[12:16]) == bytes((4, 5, 6, 7))


def test_transform_cloud_repack_keeps_values_and_invalid_points():
    message = _orbbec_shaped_cloud(
        [
            ((1.0, 2.0, 3.0), bytes((1, 2, 3, 4))),
            ((math.nan, math.nan, math.nan), bytes((9, 8, 7, 6))),
        ]
    )

    output = transform_cloud(
        message, (10.0, 20.0, 30.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame'
    )

    assert struct.unpack_from('<fff', output.data, 0) == (11.0, 22.0, 33.0)
    # A dropped depth pixel must stay dropped rather than land at the origin.
    assert all(math.isnan(value) for value in struct.unpack_from('<fff', output.data, 16))
    assert bytes(output.data[28:32]) == bytes((9, 8, 7, 6))


def test_transform_cloud_does_not_mutate_the_incoming_message():
    message = _orbbec_shaped_cloud([((1.0, 2.0, 3.0), bytes((4, 5, 6, 7)))])
    original = bytes(message.data)

    transform_cloud(message, (5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame')

    assert message.header.frame_id == 'cam_1_color_optical_frame'
    assert bytes(message.data) == original


def test_transform_cloud_rejects_an_rgb_field_it_cannot_pack():
    message = _orbbec_shaped_cloud([((1.0, 2.0, 3.0), bytes((4, 5, 6, 7)))])
    message.fields[3].datatype = PointField.UINT8

    try:
        transform_cloud(
            message, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame'
        )
    except ValueError:
        return
    raise AssertionError('an unpackable rgb field must be reported, not dropped')
