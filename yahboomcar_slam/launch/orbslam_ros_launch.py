from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_slam')
    orbvoc_path = os.path.join(package_path, 'params', 'ORBvoc.txt')
    mono_yaml_path = os.path.join(package_path, 'params', 'mono.yaml')
    rgbd_yaml_path = os.path.join(package_path, 'params', 'rgbd.yaml')

    orb_slam_type_arg = DeclareLaunchArgument(name='orb_slam_type', default_value='mono', 
                                              choices=['mono','stereo','rgbd'],
                                              description='The type of orbslam2')

    # 单目相机
    orbslam_mono = Node(
        package='ros2_orbslam',
        executable='mono',
        arguments=[orbvoc_path, mono_yaml_path],
        condition=LaunchConfigurationEquals('orb_slam_type', 'mono')
    )

    # 双目相机
    orbslam_stereo = Node(
        package='ros2_orbslam',
        executable='stereo',
        arguments=[orbvoc_path, mono_yaml_path],
        condition=LaunchConfigurationEquals('orb_slam_type', 'stereo')
    )
    
    # RGBD相机
    orbslam_rgbd = Node(
        package='ros2_orbslam',
        executable='rgbd',
        arguments=[orbvoc_path, rgbd_yaml_path],
        condition=LaunchConfigurationEquals('orb_slam_type', 'rgbd')
    )

    return LaunchDescription([
        orb_slam_type_arg,
        orbslam_mono,
        orbslam_stereo,
        orbslam_rgbd
    ])