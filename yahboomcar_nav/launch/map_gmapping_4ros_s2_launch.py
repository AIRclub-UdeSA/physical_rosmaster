from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/laser_bringup_launch.py'])
    )
    # 4ROS和s2的雷达单帧激光点数大于1440，gmapping只适用于单帧二维激光点数小于1440的点，所以做过滤
    scan_filter_node = Node(
        package='yahboomcar_nav',
        executable='scan_filter',
    )

    slam_gmapping_node = Node(
            package='slam_gmapping', 
            executable='slam_gmapping', 
            output='screen', 
            parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")],
            remappings = [("/scan","/downsampled_scan")],
    )

    return LaunchDescription([
        laser_bringup_launch,
        scan_filter_node, 
        slam_gmapping_node
    ])
