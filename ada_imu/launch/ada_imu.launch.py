#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file for imu_jointstate_publisher with params from config/imu_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """ Creates launch description to run imu_jointstate_publisher """
    launch_description = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("ada_imu"), "config", "imu_params.yaml"
    )

    imu_jointstate_publisher = Node(
        package="ada_imu",
        name="imu_jointstate_publisher",
        executable="imu_jointstate_publisher",
        parameters=[config],
    )

    launch_description.add_action(imu_jointstate_publisher)
    return launch_description
