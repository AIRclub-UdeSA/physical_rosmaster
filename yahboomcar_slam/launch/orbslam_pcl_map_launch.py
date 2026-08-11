from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_slam')

    tf_odom_to_world = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '1.57', '3.14', '1.57', 'odom', 'world']
    )

    pointcloud_mapping_config = os.path.join(              
        package_path,
        'params',
        'pointcloud_map.yaml'
    )

    pointcloud_mapping_node = Node(
        package='yahboomcar_slam',
        executable='pointcloud_mapping',
        name='pointcloud_mapping_node',
        output="screen",
        parameters=[pointcloud_mapping_config]
    )
    
    return LaunchDescription([
        tf_odom_to_world,
        pointcloud_mapping_node
    ])