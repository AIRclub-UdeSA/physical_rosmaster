from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_slam')
    default_orbvoc_path = os.path.join(package_path, 'params', 'ORBvoc.txt')
    default_config_path = os.path.join(package_path, 'params', 'rgbd.yaml')
    
    orbslam_rgbd_pose = Node(
        package='ros2_orbslam',
        executable='rgbd_pose',
        arguments=[default_orbvoc_path, default_config_path],
    )

    return LaunchDescription([
        orbslam_rgbd_pose
    ])