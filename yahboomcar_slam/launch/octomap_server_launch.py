from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    octomap_mapping_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name="octomap_server",
        remappings=[('/cloud_in','/Local/PointCloudOutput')],
        parameters=[
            {"resolution": 0.05},
            {"frame_id": "odom"},
            {"colored_map": False},
            {"sensor_model.max_range": 5.0}
        ]
    )

    return LaunchDescription([
        octomap_mapping_node
    ])