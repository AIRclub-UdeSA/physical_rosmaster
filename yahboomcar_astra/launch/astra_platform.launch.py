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
                # Keep the full 30 Hz cadence while bounding XYZRGB conversion
                # work on the Raspberry Pi.  Both Astra functions provide this
                # native mode, so registration and camera_info stay aligned.
                "color_width": 320,
                "color_height": 240,
                "depth_width": 320,
                "depth_height": 240,
                # The X3 Astra exposes OpenNI depth as 2bc5:060f and RGB as a
                # separate UVC function at 2bc5:050f. The pinned driver's
                # Astra Pro Plus launch uses this same split-device path.
                "use_uvc_camera": True,
                "uvc_vendor_id": 0x2BC5,
                "uvc_product_id": 0x050F,
                "uvc_camera_format": "mjpeg",
                "publish_tf": True,
                "tf_publish_rate": 0.0,
                "color_qos": "sensor_data",
                "depth_qos": "sensor_data",
                # Upstream's XYZRGB synchronizer uses this setting for its
                # color camera-info subscription, despite the parameter name.
                "depth_camera_info_qos": "sensor_data",
                "point_cloud_qos": "sensor_data",
                "rgb_qos_profile": "sensor_data",
                "rgb_info_qos_profile": "sensor_data",
            }
        ],
    )
    adapter = Node(
        package="yahboomcar_astra",
        executable="astra_sensor_adapter",
        output="screen",
        # The 320x240 XYZ transform is a tall-by-3 matrix operation. Letting
        # OpenBLAS create one worker per core makes this small operation slower
        # and can starve the RGB-D graph on the Raspberry Pi. Keep the limit
        # local to the adapter instead of depending on an operator shell or
        # constraining unrelated platform processes.
        additional_env={"OPENBLAS_NUM_THREADS": "1"},
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
