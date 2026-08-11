import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    '''astra_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('astra_camera'),'launch'),'/astra.launch.xml'])
    )'''
    driver_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_bringup'),'launch'),'/yahboomcar_bringup_X3_launch.py']                            
    ))
    colorIdentify_node =  Node(
                                package='yahboomcar_astra',
                                node_executable='colorHSV',
                                node_name='coloridentify'
                                )
    '''colorTracker_node = Node(
                                package='yahboomcar_astra',
                                node_executable='colorTraker',
                                node_name='colortracker'
                                )'''
    return LaunchDescription([#astra_node,
                              driver_node,
                              colorIdentify_node,
                              #colorTracker_node
                            ])
