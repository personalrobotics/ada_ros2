from setuptools import setup
import os
from glob import glob

package_name = "ada_fork_joint"

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
    maintainer="Amal Nanavati",
    maintainer_email="amaln@cs.washington.edu",
    description=(
        "This package has a node that localizes the fork using Apriltags in known "
        "positions and publishes the joint states of the fork handle."
    ),
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fork_jointstate_publisher = ada_fork_joint.fork_jointstate_publisher:main"
        ],
    },
)
