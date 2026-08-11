from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
        '/yahboomcar_bringup_x1_launch.py'])
    )

    octomap_mapping_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name="octomap_server",
        remappings=[('/cloud_in','/camera/depth/points')],
        parameters=[
            {"resolution": 0.05},
            {"frame_id": "odom"},
            {"colored_map": False},
            {"sensor_model.max_range": 5.0}
        ]
    )


    return LaunchDescription([
        bringup_launch,
        octomap_mapping_node
    ])