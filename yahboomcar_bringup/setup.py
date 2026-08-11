from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        #(os.path.join('share','yahboomcar_description','urdf'),glob(os.path.join('urdf','*.*'))),
		#(os.path.join('share','yahboomcar_description','meshes'),glob(os.path.join('meshes','*.*'))),
        (os.path.join('share','yahboomcar_description','rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'Mcnamu_driver_X3	= yahboomcar_bringup.Mcnamu_driver_X3:main',
        'Mcnamu_driver_x1 = yahboomcar_bringup.Mcnamu_driver_x1:main',
        'calibrate_linear_X3 = yahboomcar_bringup.calibrate_linear_X3:main', 
        'calibrate_angular_X3 = yahboomcar_bringup.calibrate_angular_X3:main',
        'patrol_4ROS = yahboomcar_bringup.patrol_4ROS:main',
        'patrol_a1_X3 = yahboomcar_bringup.patrol_a1_X3:main',
        'Ackman_driver_R2	= yahboomcar_bringup.Ackman_driver_R2:main',
        'calibrate_linear_R2 = yahboomcar_bringup.calibrate_linear_R2:main',
        'calibrate_angular_R2 = yahboomcar_bringup.calibrate_angular_R2:main',
        'patrol_4ROS_R2 = yahboomcar_bringup.patrol_4ROS_R2:main',
        'patrol_a1_R2 = yahboomcar_bringup.patrol_a1_R2:main',   
        ],
    },
)
