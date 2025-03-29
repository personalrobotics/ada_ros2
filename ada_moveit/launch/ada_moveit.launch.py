# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node, PushRosNamespace

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from moveit_configs_utils.launches import (
    generate_rsp_launch,
    generate_move_group_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
    generate_moveit_rviz_launch,
)


def generate_launch_description():
    # Sim Launch Argument
    sim_da = DeclareLaunchArgument(
        "sim",
        default_value="mock",
        description="Which sim to use:",
        choices=["mock", "real"],
    )
    sim = LaunchConfiguration("sim")

    # Use Forque Launch Argument
    use_forque_da = DeclareLaunchArgument(
        "use_forque",
        default_value="false",
        description="Whether to use the standard forque tool",
        choices=["true", "false"],
    )
    use_forque = LaunchConfiguration("use_forque")

    # Controllers File
    ctrl_da = DeclareLaunchArgument(
        "controllers_file",
        default_value=[sim, "_controllers.yaml"],
        description="ROS2 Controller YAML configuration in config folder",
    )
    controllers_file = LaunchConfiguration("controllers_file")

    # Log Level
    log_level_da = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch MoveIt launch file.",
    )
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Copy from generate_demo_launch
    ld = LaunchDescription()
    ld.add_action(sim_da)
    ld.add_action(use_forque_da)
    ld.add_action(ctrl_da)
    ld.add_action(log_level_da)
    ld.add_action(launch_rviz_arg)

    # Get MoveIt Configs
    builder = MoveItConfigsBuilder("ada", package_name="ada_moveit")
    builder = builder.robot_description(
        mappings={
            "sim": sim,
            "use_forque": use_forque,
        }
    )
    moveit_config = builder.to_moveit_configs()

    # If sim is mock, set moveit_config.sensors_3d to an empty dictionary
    if sim == "mock":
        moveit_config.sensors_3d = {}

    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    # Launch the IMU joint state publisher
    ada_imu_package_path = get_package_share_directory("ada_imu")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ada_imu_package_path, "launch/ada_imu.launch.py")
            ),
            launch_arguments={
                "sim": sim,
                "log_level": log_level,
            }.items(),
        ),
    )


    actions = [
        PushRosNamespace("ada"),
        # Combined Joint States Node
        Node(
            package='ada_moveit', #replace with your package name.
            executable='combined_joint_states.py',
            name='combined_joint_states',
        ),
        # Robot State Publisher
        # *generate_rsp_launch(moveit_config).entities,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[moveit_config.robot_description],
            remappings=[('/joint_states', '/ada/combined_joint_states')],
        ),
        # Move Group
        *generate_move_group_launch(moveit_config).entities,
        # RViz
        GroupAction(
            actions=generate_moveit_rviz_launch(moveit_config).entities,
            condition=IfCondition(launch_rviz)
        ),
        # Spawn Controllers
        *generate_spawn_controllers_launch(moveit_config).entities,
        # Static Virtual Joints
        *generate_static_virtual_joint_tfs_launch(moveit_config).entities,
        # Joint Controllers
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                PathJoinSubstitution(
                    [str(moveit_config.package_path), "config", controllers_file]
                ),
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ]

    ada_group = GroupAction(actions=actions)
    ld.add_action(ada_group)

    return ld
