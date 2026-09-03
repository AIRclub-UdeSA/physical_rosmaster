"""Packaging for generic ROS sensor inspection utilities."""

from setuptools import setup


package_name = "yahboomcar_visual"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIRclub UdeSA",
    maintainer_email="airclub@udesa.edu.ar",
    description="Generic ROS sensor conversion and inspection utilities",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "laser_to_image = yahboomcar_visual.laser_to_image:main",
            "pub_image = yahboomcar_visual.pub_image:main",
        ],
    },
)
