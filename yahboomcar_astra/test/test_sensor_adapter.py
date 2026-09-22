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


def test_transform_cloud_strips_nan_points():
    message = _orbbec_shaped_cloud(
        [
            ((1.0, 2.0, 3.0), bytes((1, 2, 3, 4))),
            ((math.nan, math.nan, math.nan), bytes((9, 8, 7, 6))),
            ((4.0, 5.0, 6.0), bytes((5, 6, 7, 8))),
        ]
    )

    output = transform_cloud(
        message, (10.0, 20.0, 30.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame',
        strip_nan=True,
    )

    assert output.height == 1
    assert output.width == 2
    assert output.is_dense is True
    assert output.point_step == 16
    assert output.row_step == 32
    assert len(output.data) == 32

    first_xyz = struct.unpack_from('<fff', output.data, 0)
    second_xyz = struct.unpack_from('<fff', output.data, 16)
    assert first_xyz == (11.0, 22.0, 33.0)
    assert second_xyz == (14.0, 25.0, 36.0)
    assert bytes(output.data[12:16]) == bytes((1, 2, 3, 4))
    assert bytes(output.data[28:32]) == bytes((5, 6, 7, 8))


def test_transform_cloud_all_nan_strip_nan():
    message = _orbbec_shaped_cloud(
        [
            ((math.nan, math.nan, math.nan), bytes((1, 2, 3, 4))),
            ((math.nan, math.nan, math.nan), bytes((5, 6, 7, 8))),
        ]
    )

    output = transform_cloud(
        message, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame',
        strip_nan=True,
    )

    assert output.height == 1
    assert output.width == 0
    assert output.is_dense is True
    assert len(output.data) == 0


def _orbbec_shaped_grid(rows, cols, values_fn):
    """Build an organized 2D cloud with height=rows, width=cols."""
    message = PointCloud2(
        height=rows, width=cols, point_step=32, row_step=32 * cols
    )
    message.header.frame_id = 'cam_1_color_optical_frame'
    message.is_dense = False
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    buffer = bytearray(32 * rows * cols)
    for r in range(rows):
        for c in range(cols):
            xyz, colour = values_fn(r, c)
            idx = (r * cols + c) * 32
            struct.pack_into('<fff', buffer, idx, *xyz)
            buffer[idx + 16:idx + 20] = colour
    message.data = bytes(buffer)
    return message


def test_transform_cloud_decimates_grid():
    # 4x4 grid -> decimation 2 -> 2x2 grid
    def values_fn(r, c):
        return (float(r), float(c), 1.0), bytes((r, c, 10, 255))

    message = _orbbec_shaped_grid(4, 4, values_fn)

    output = transform_cloud(
        message, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame',
        decimation=2, strip_nan=False,
    )

    assert output.height == 2
    assert output.width == 2
    assert output.point_step == 16
    assert output.row_step == 32
    assert len(output.data) == 64

    # Subsampled points must be (0,0), (0,2), (2,0), (2,2)
    expected_samples = [(0, 0), (0, 2), (2, 0), (2, 2)]
    for i, (r, c) in enumerate(expected_samples):
        x, y, z = struct.unpack_from('<fff', output.data, i * 16)
        rgb = bytes(output.data[i * 16 + 12:i * 16 + 16])
        assert (x, y, z) == (float(r), float(c), 1.0)
        assert rgb == bytes((r, c, 10, 255))


def test_transform_cloud_decimates_and_strips_nan():
    # 4x4 grid with (0,0) as NaN
    def values_fn(r, c):
        if r == 0 and c == 0:
            return (math.nan, math.nan, math.nan), bytes((0, 0, 0, 0))
        return (float(r), float(c), 1.0), bytes((r, c, 10, 255))

    message = _orbbec_shaped_grid(4, 4, values_fn)

    output = transform_cloud(
        message, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame',
        decimation=2, strip_nan=True,
    )

    # 4 points decimated, 1 was NaN -> 3 points remaining
    assert output.height == 1
    assert output.width == 3
    assert output.is_dense is True
    assert len(output.data) == 48

    expected_samples = [(0, 2), (2, 0), (2, 2)]
    for i, (r, c) in enumerate(expected_samples):
        x, y, z = struct.unpack_from('<fff', output.data, i * 16)
        rgb = bytes(output.data[i * 16 + 12:i * 16 + 16])
        assert (x, y, z) == (float(r), float(c), 1.0)
        assert rgb == bytes((r, c, 10, 255))


def test_transform_cloud_rejects_invalid_decimation():
    message = _orbbec_shaped_cloud([((1.0, 2.0, 3.0), bytes((4, 5, 6, 7)))])
    for invalid in (0, -1, '2'):
        try:
            transform_cloud(
                message, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'cam_1_depth_frame',
                decimation=invalid,
            )
        except ValueError:
            continue
        raise AssertionError(f'decimation={invalid} must be rejected')
