from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    linefollow_node = Node(
        package='yahboomcar_linefollow',
        executable='follow_line_a1_X3',
    )
    lidar_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
'/sllidar_launch.py'])
)
    bringup_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
'/yahboomcar_bringup_X3_launch.py'])
)
    Joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    launch_description = LaunchDescription([linefollow_node,lidar_node,bringup_node,Joy_node]) 
    return launch_description
