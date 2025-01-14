# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from setuptools import setup
import os
from glob import glob

package_name = "ada_imu"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ethan K. Gordon",
    maintainer_email="ekgordon@cs.washington.edu",
    description="IMU jointstate publisher",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_jointstate_publisher = ada_imu.imu_jointstate_publisher:main"
        ],
    },
)
