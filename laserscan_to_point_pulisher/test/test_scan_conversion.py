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

"""Regression tests for generic LaserScan conversion."""

import math

from sensor_msgs.msg import LaserScan

from laserscan_to_point_pulisher.laserscan_to_point_publish import scan_points


def test_scan_points_uses_angle_min_and_filters_invalid_ranges():
    message = LaserScan(
        angle_min=math.pi / 2.0,
        angle_increment=math.pi / 2.0,
        range_min=0.1,
        range_max=5.0,
        ranges=[1.0, float('inf'), 0.01, 2.0],
    )

    points = list(scan_points(message))

    assert len(points) == 2
    assert math.isclose(points[0][0], 0.0, abs_tol=1e-12)
    assert math.isclose(points[0][1], 1.0, abs_tol=1e-12)
    assert math.isclose(points[1][0], 2.0, abs_tol=1e-12)
    assert math.isclose(points[1][1], 0.0, abs_tol=1e-12)
