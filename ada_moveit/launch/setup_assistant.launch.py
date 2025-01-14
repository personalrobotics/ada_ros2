# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    ld = LaunchDescription()
    # Log Level
    log_level_da = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")
    log_level_cmd_line_args = ["--ros-args", "--log-level", log_level]
    ld.add_action(log_level_da)

    moveit_config = MoveItConfigsBuilder(
        "ada", package_name="ada_moveit"
    ).to_moveit_configs()
    entities = generate_setup_assistant_launch(moveit_config).entities
    for entity in entities:
        if isinstance(entity, Node):
            entity.cmd.extend(
                [
                    normalize_to_list_of_substitutions(arg)
                    for arg in log_level_cmd_line_args
                ]
            )
        ld.add_action(entity)

    return ld
