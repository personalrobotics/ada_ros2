# Copyright (c) 2024, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from setuptools import setup

package_name = "ada_watchdog_listener"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Amal Nanavati",
    maintainer_email="amaln@cs.washington.edu",
    description="This package contains libraries and nodes related to listening to the output of a watchdog.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ada_watchdog_listener_node = ada_watchdog_listener.ada_watchdog_listener_node:main",
        ],
    },
)
