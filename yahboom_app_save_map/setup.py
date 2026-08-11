from setuptools import setup

package_name = 'yahboom_app_save_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/yahboom_app_save_map.launch.py',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "server=yahboom_app_save_map.yahboom_app_save_map:main",
        "client=yahboom_app_save_map.yahboom_app_save_map_client:main"
        ],
    },
)
