"""Packaging for the ROSMASTER X3 bringup nodes and launch files."""

import os
from glob import glob

from setuptools import setup


package_name = "yahboomcar_bringup"

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
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
        (
            os.path.join("share", package_name, "param"),
            glob(os.path.join("param", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIRclub UdeSA",
    maintainer_email="airclub@udesa.edu.ar",
    description="Strict non-autonomous ROSMASTER X3 hardware bringup",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "Mcnamu_driver_X3 = yahboomcar_bringup.Mcnamu_driver_X3:main",
            "calibrate_linear_X3 = yahboomcar_bringup.calibrate_linear_X3:main",
            "calibrate_angular_X3 = yahboomcar_bringup.calibrate_angular_X3:main",
        ],
    },
)
