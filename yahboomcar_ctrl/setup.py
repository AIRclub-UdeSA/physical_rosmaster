from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
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
        'yahboom_joy_X3	= yahboomcar_ctrl.yahboom_joy_X3:main',
        'yahboom_keyboard	= yahboomcar_ctrl.yahboom_keyboard:main',
        'yahboom_joy_R2	= yahboomcar_ctrl.yahboom_joy_R2:main',
        ],
    },
)
