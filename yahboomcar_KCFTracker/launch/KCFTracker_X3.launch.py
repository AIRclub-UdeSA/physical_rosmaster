from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    KCFTracker_node = Node(
        package='yahboomcar_KCFTracker',
        executable='KCF_Tracker_Node',
    )

    uabcam_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('usb_cam'), 'launch'),
'/demo_launch.py'])
)
    
    launch_description = LaunchDescription([uabcam_node,KCFTracker_node]) 
    return launch_description
