from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

print("---------------------robot_type = x3---------------------")
def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_X3.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/yahboomcar.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    imu_filter_config = os.path.join(              
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    ) 

    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
    )

    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        # 当使用ekf融合时，该tf有ekf发布
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/ekf_x1_x3_launch.py'])
    )

    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        robot_state_publisher_node,
        driver_node,
        base_node,
        imu_filter_node,
        ekf_node,
        yahboom_joy_node
    ])
