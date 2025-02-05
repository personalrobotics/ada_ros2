# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from glob import glob
import os
from setuptools import find_packages, setup

package_name = "ada_calibrate_camera"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    # Include all launch files.
    (
        os.path.join("share", package_name, "launch"),
        glob(os.path.join("launch", "*launch.[pxy][yma]*")),
    ),
    # Include all config files.
    (
        os.path.join("share", package_name, "config"),
        glob(os.path.join("config", "*.yaml")),
    ),
    (
        os.path.join("share", package_name, "config/calibs"),
        glob(os.path.join("config/calibs", "*.yaml")),
    ),
]
for path in os.listdir("data"):
    if os.path.isdir(os.path.join("data", path)):
        data_files.append(
            (
                os.path.join("share", package_name, "data", path),
                glob(os.path.join("data", path, "*.[pn][np][gz]")),
            )
        )

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Amal Nanavati",
    maintainer_email="amaln@cs.washington.edu",
    description=("This package contains a script to calibrate ADA's camera."),
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibrate_camera = ada_calibrate_camera.calibrate_camera_node:main",
            "publish_camera_extrinsics = ada_calibrate_camera.publish_camera_extrinsics_node:main",
        ],
    },
)
