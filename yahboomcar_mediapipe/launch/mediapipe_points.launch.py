from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    #package_path = get_package_share_path('yahboomcar_mediapipe')
    #default_rviz_config_path = package_path / 'rviz/mediapipe_points.rviz'

    pub_point_node = Node(
        package='yahboomcar_point',
        executable='pub_point',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/nx-ros2/yahboomcar_lucio_ws/src/yahboomcar_mediapipe/rviz/mediapipe_points.rviz'],
    )

    return LaunchDescription([
        pub_point_node,
        rviz_node
    ])
