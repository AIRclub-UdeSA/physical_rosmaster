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

"""Launch the pinned Orbbec driver behind the public camera contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the Astra driver and fail the enclosing launch if either node exits."""
    serial_number = LaunchConfiguration("serial_number")
    startup_timeout = LaunchConfiguration("startup_timeout")

    serial_argument = DeclareLaunchArgument(
        "serial_number",
        default_value="",
        description="Astra serial number; configure this per robot after discovery",
    )
    timeout_argument = DeclareLaunchArgument(
        "startup_timeout",
        default_value="20.0",
        description="Seconds allowed for all normalized RGB-D streams to become valid",
    )

    driver = Node(
        package="astra_camera",
        executable="astra_camera_node",
        namespace="/_hardware/astra",
        name="camera",
        output="screen",
        parameters=[
            {
                "camera_name": "cam_1",
                "camera_link_frame_id": "cam_1_link",
                "serial_number": ParameterValue(serial_number, value_type=str),
                "depth_registration": True,
                "enable_color": True,
                "enable_depth": True,
                "enable_ir": False,
                "enable_point_cloud": False,
                "enable_colored_point_cloud": True,
                "color_depth_synchronization": True,
                "publish_tf": True,
                "tf_publish_rate": 0.0,
                "color_qos": "sensor_data",
                "depth_qos": "sensor_data",
                "point_cloud_qos": "sensor_data",
            }
        ],
    )
    adapter = Node(
        package="yahboomcar_astra",
        executable="astra_sensor_adapter",
        output="screen",
        parameters=[
            {
                "startup_timeout": ParameterValue(
                    startup_timeout, value_type=float
                )
            }
        ],
    )

    required_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="required camera process exited: %s" % name
                        )
                    )
                ],
            )
        )
        for node, name in ((driver, "driver"), (adapter, "adapter"))
    ]

    return LaunchDescription(
        [serial_argument, timeout_argument, driver, adapter, *required_handlers]
    )
