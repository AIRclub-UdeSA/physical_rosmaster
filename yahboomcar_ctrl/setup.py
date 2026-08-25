"""Packaging for opt-in ROSMASTER X3 operator controls."""

import os
from glob import glob

from setuptools import setup


package_name = "yahboomcar_ctrl"

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
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIRclub UdeSA",
    maintainer_email="airclub@udesa.edu.ar",
    description="Opt-in safe operator controls for the ROSMASTER X3",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yahboom_joy_X3 = yahboomcar_ctrl.yahboom_joy_X3:main",
            "yahboom_keyboard = yahboomcar_ctrl.yahboom_keyboard:main",
        ],
    },
)
