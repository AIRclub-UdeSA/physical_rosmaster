from setuptools import setup

package_name = 'yahboomcar_voice_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'Voice_Ctrl_Mcnamu_driver_X3 = yahboomcar_voice_ctrl.Voice_Ctrl_Mcnamu_driver_X3:main',
        'Voice_Ctrl_follow_line_a1_X3 = yahboomcar_voice_ctrl.Voice_Ctrl_follow_line_a1_X3:main',
        'Voice_Ctrl_colorTracker = yahboomcar_voice_ctrl.Voice_Ctrl_colorTracker:main',
        'Voice_Ctrl_colorHSV = yahboomcar_voice_ctrl.Voice_Ctrl_colorHSV:main',
        'voice_Ctrl_send_mark = yahboomcar_voice_ctrl.voice_Ctrl_send_mark:main',
        'Voice_Ctrl_Ackman_driver_R2 = yahboomcar_voice_ctrl.Voice_Ctrl_Ackman_driver:main',  
        'Voice_Ctrl_follow_line_a1_R2 = yahboomcar_voice_ctrl.Voice_Ctrl_follow_line_a1_R2:main',
        'Voice_Ctrl_follow_line_4ROS_R2 = yahboomcar_voice_ctrl.Voice_Ctrl_follow_line_4ROS_R2:main',
        ],
    },
)
