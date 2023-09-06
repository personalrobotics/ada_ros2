#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file for imu_jointstate_publisher with params from config/imu_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Creates launch description to run imu_jointstate_publisher"""
    launch_description = LaunchDescription()

    # Declare launch arguments
    sim_da = DeclareLaunchArgument(
        "sim",
        default_value="real",
        description="Which sim to use: 'mock' or 'real'",
    )
    sim = LaunchConfiguration("sim")
    launch_description.add_action(sim_da)

    # Load the parameters
    config = os.path.join(
        get_package_share_directory("ada_imu"), "config", "imu_params.yaml"
    )
    sim_param = {"sim": ParameterValue(sim, value_type=str)}

    imu_jointstate_publisher = Node(
        package="ada_imu",
        name="imu_jointstate_publisher",
        executable="imu_jointstate_publisher",
        parameters=[config, sim_param],
    )

    launch_description.add_action(imu_jointstate_publisher)
    return launch_description
