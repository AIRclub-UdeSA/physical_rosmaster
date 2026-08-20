"""Launch the ROSMASTER X3 driver, odometry, IMU filter, and EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the deterministic, joystick-optional X3 core graph."""
    description_path = get_package_share_path("yahboomcar_description")
    default_model_path = description_path / "urdf/yahboomcar_X3.urdf"
    bringup_share = get_package_share_directory("yahboomcar_bringup")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to the robot URDF or Xacro file",
    )
    pub_odom_tf_arg = DeclareLaunchArgument(
        "pub_odom_tf",
        default_value="false",
        choices=["true", "false"],
        description="Publish raw odom TF instead of leaving TF to the EKF",
    )
    use_joy_arg = DeclareLaunchArgument(
        "use_joy",
        default_value="false",
        choices=["true", "false"],
        description="Launch the joystick controller node",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )
    imu_filter_config = os.path.join(
        bringup_share, "param", "imu_filter_param.yaml"
    )
    driver_config = os.path.join(
        bringup_share, "param", "x3_driver.yaml"
    )
    odometry_config = os.path.join(
        bringup_share, "param", "x3_odometry.yaml"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    driver_node = Node(
        package="yahboomcar_bringup",
        executable="Mcnamu_driver_X3",
        parameters=[driver_config],
    )
    base_node = Node(
        package="yahboomcar_base_node",
        executable="base_node_X3",
        parameters=[
            odometry_config,
            {
                "pub_odom_tf": ParameterValue(
                    LaunchConfiguration("pub_odom_tf"), value_type=bool
                )
            },
        ],
    )
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[imu_filter_config],
    )
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "ekf_x1_x3_launch.py")
        )
    )
    joystick_node = Node(
        package="yahboomcar_ctrl",
        executable="yahboom_joy_X3",
        condition=IfCondition(LaunchConfiguration("use_joy")),
    )

    return LaunchDescription(
        [
            model_arg,
            pub_odom_tf_arg,
            use_joy_arg,
            robot_state_publisher_node,
            driver_node,
            base_node,
            imu_filter_node,
            ekf_node,
            joystick_node,
        ]
    )
