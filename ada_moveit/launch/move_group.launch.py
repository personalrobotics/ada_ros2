# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def get_move_group_launch(context):
    """
    Gets the launch description for MoveGroup, after removing sensors_3d
    if sim is mock.

    Adapted from https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file
    """
    sim = LaunchConfiguration("sim").perform(context)
    use_octomap = LaunchConfiguration("use_octomap").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    end_effector_tool = LaunchConfiguration("end_effector_tool").perform(context)

    # Get MoveIt Configs
    builder = MoveItConfigsBuilder("ada", package_name="ada_moveit")
    builder = builder.robot_description(
        mappings={"sim": sim, "end_effector_tool": end_effector_tool}
    )
    builder = builder.robot_description_semantic(
        mappings={"end_effector_tool": end_effector_tool}
    )
    moveit_config = builder.to_moveit_configs()

    # If sim is mock, set moveit_config.sensors_3d to an empty dictionary
    if sim == "mock" or use_octomap == "false":
        moveit_config.sensors_3d = {}

    entities = generate_move_group_launch(moveit_config).entities
    log_level_cmd_line_args = ["--ros-args", "--log-level", log_level]
    for entity in entities:
        if isinstance(entity, Node):
            entity.cmd.extend(
                [
                    normalize_to_list_of_substitutions(arg)
                    for arg in log_level_cmd_line_args
                ]
            )
    return entities


def generate_launch_description():
    # Sim Launch Argument
    sim_da = DeclareLaunchArgument(
        "sim",
        default_value="real",
        description="Which sim to use: 'mock', 'isaac', or 'real'",
    )
    # Use Octomap
    octomap_da = DeclareLaunchArgument(
        "use_octomap",
        default_value="true",
        description="Whether to use octomap for collision checking",
    )
    # Log Level
    log_level_da = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    eet_da = DeclareLaunchArgument(
        "end_effector_tool",
        default_value="fork",
        description="The end-effector tool being used: 'none', 'fork', 'articulable_fork'",
        choices=["none", "fork", "articulable_fork"],
    )

    ld = LaunchDescription()
    ld.add_action(sim_da)
    ld.add_action(octomap_da)
    ld.add_action(log_level_da)
    ld.add_action(eet_da)
    ld.add_action(OpaqueFunction(function=get_move_group_launch))
    return ld
