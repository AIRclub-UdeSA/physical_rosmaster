import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_launch_path =os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch')

    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path,'/laser_bringup_launch.py'])
    )

    rtabmap_localization_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path, '/rtabmap_localization_launch.py'])
    )

    rtabmap_nav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [package_launch_path, '/rtabmap_nav_launch.py'])
    )

    return LaunchDescription([
        laser_bringup_launch, 
        rtabmap_localization_launch,
        rtabmap_nav_launch
    ])