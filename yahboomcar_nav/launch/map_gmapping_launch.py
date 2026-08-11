from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE')
    rplidar_type_arg = DeclareLaunchArgument(name='rplidar_type', default_value=RPLIDAR_TYPE, 
                                              choices=['a1','s2','4ROS'],
                                              description='The type of robot')

    gmapping_4ros_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_4ros_s2_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )
    gmapping_s2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_4ros_s2_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 's2')
    )
    gmapping_a1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_a1_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 'a1')
    )

    return LaunchDescription([
        rplidar_type_arg,
        gmapping_4ros_launch, 
        gmapping_s2_launch, 
        gmapping_a1_launch
    ])
