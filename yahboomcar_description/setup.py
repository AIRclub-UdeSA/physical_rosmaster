"""Package the canonical ROSMASTER X3 robot description."""

import os
from glob import glob

from setuptools import setup


package_name = "yahboomcar_description"

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
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.*")),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob(os.path.join("meshes", "*.*")),
        ),
        (
            os.path.join("share", package_name, "meshes", "cad_visual"),
            glob(os.path.join("meshes", "cad_visual", "*.*")),
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
    description="Canonical ROSMASTER X3 hardware description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
