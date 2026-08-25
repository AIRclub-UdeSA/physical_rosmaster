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

"""Launch the complete, non-autonomous ROSMASTER X3 hardware platform."""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _required_process(node, label):
    """Shut the platform down instead of leaving a partial sensor graph."""
    return RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="required platform process exited: %s" % label
                    )
                )
            ],
        )
    )


def generate_launch_description():
    """Create the strict X3 driver, sensor, preprocessing, odometry, and TF graph."""
    description_path = get_package_share_path("yahboomcar_description")
    bringup_share = get_package_share_directory("yahboomcar_bringup")
    astra_share = get_package_share_directory("yahboomcar_astra")

    model = LaunchConfiguration("model")
    motor_port = LaunchConfiguration("motor_serial_port")
    lidar_port = LaunchConfiguration("lidar_serial_port")
    camera_serial = LaunchConfiguration("camera_serial_number")

    launch_arguments = [
        DeclareLaunchArgument(
            "model",
            default_value=str(
                description_path / "urdf/yahboomcar_X3.urdf.xacro"
            ),
            description="Absolute path to the canonical X3 Xacro",
        ),
        DeclareLaunchArgument(
            "motor_serial_port",
            default_value=EnvironmentVariable(
                "ROSMASTER_MOTOR_PORT",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            description="Stable motor-controller serial path",
        ),
        DeclareLaunchArgument(
            "lidar_serial_port",
            default_value=EnvironmentVariable(
                "ROSMASTER_LIDAR_PORT", default_value="/dev/robot/lidar"
            ),
            description="Stable A1 LiDAR serial path or udev alias",
        ),
        DeclareLaunchArgument(
            "camera_serial_number",
            default_value=EnvironmentVariable(
                "ROSMASTER_ASTRA_SERIAL", default_value=""
            ),
            description="Stable Astra serial number selected per robot",
        ),
    ]

    robot_description = ParameterValue(
        Command(["xacro ", model]), value_type=str
    )
    driver_config = os.path.join(bringup_share, "param", "x3_driver.yaml")
    odometry_config = os.path.join(bringup_share, "param", "x3_odometry.yaml")
    imu_filter_config = os.path.join(
        bringup_share, "param", "imu_filter_param.yaml"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    driver = Node(
        package="yahboomcar_bringup",
        executable="Mcnamu_driver_X3",
        name="driver_node",
        output="screen",
        parameters=[
            driver_config,
            {"serial_port": ParameterValue(motor_port, value_type=str)},
        ],
    )
    wheel_odometry = Node(
        package="yahboomcar_base_node",
        executable="base_node_X3",
        output="screen",
        parameters=[odometry_config, {"pub_odom_tf": True}],
    )
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[imu_filter_config],
    )
    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": ParameterValue(lidar_port, value_type=str),
                "serial_baudrate": 115200,
                "frame_id": "laser_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
                # Physical cable/self-return preprocessing. /scan is canonical;
                # rejected returns remain observable on /scan_filtered.
                "cable_angle_min": 152.0,
                "cable_angle_max": 170.0,
                "cable_max_distance": 0.14,
            }
        ],
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(astra_share, "launch", "astra_platform.launch.py")
        ),
        launch_arguments={"serial_number": camera_serial}.items(),
    )

    required_nodes = (
        (robot_state_publisher, "robot_state_publisher"),
        (driver, "motor driver"),
        (wheel_odometry, "wheel odometry"),
        (imu_filter, "IMU filter"),
        (lidar, "LiDAR driver"),
    )

    return LaunchDescription(
        [
            *launch_arguments,
            robot_state_publisher,
            driver,
            wheel_odometry,
            imu_filter,
            lidar,
            camera,
            *(_required_process(node, label) for node, label in required_nodes),
        ]
    )
