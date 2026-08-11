from launch import LaunchDescription
from launch_ros.actions import Node 
# 参数声明与获取
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_slam')
    
    # orbslam_pcl_launch中全局坐标系是world, 所以需要提供world的tf
    tf_odom_to_world = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '1.57', '3.14', '1.57', 'odom', 'world']
    )

    pointcloud_mapping_config = os.path.join(              
        package_path,
        'params',
        'pointcloud_octomap.yaml'
    )

    pointcloud_mapping_node = Node(
        package='yahboomcar_slam',
        executable='pointcloud_mapping',
        name='pointcloud_mapping_node',
        output="screen",
        parameters=[pointcloud_mapping_config]
    )

    octomap_server_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(package_path, 'launch'),
         '/octomap_server_launch.py'])
    )
    
    return LaunchDescription([
        tf_odom_to_world,
        pointcloud_mapping_node,
        octomap_server_launch
    ])