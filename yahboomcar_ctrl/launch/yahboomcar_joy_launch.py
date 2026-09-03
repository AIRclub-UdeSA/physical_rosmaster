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

"""Start opt-in joystick acquisition and safe X3 teleoperation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the generic joy driver plus deadman-gated X3 mapping."""
    config = os.path.join(
        get_package_share_directory("yahboomcar_ctrl"),
        "config",
        "joystick.yaml",
    )
    device_id = LaunchConfiguration("device_id")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_id", default_value="0", description="Linux joystick index"
            ),
            Node(
                package="joy",
                executable="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_id": ParameterValue(device_id, value_type=int),
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
            Node(
                package="yahboomcar_ctrl",
                executable="yahboom_joy_X3",
                output="screen",
                parameters=[config],
            ),
        ]
    )
