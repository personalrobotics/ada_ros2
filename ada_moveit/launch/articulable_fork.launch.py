# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
    TextSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF


def generate_launch_description():

    # MoveIt Config
    moveit_config = MoveItConfigsBuilder(
        "ada", package_name="ada_moveit"
    ).to_moveit_configs()

    # Sim Launch Argument
    sim_da = DeclareLaunchArgument(
        "sim",
        default_value="real",
        description="Which sim to use: 'mock', 'isaac', or 'real'",
    )
    sim = LaunchConfiguration("sim")

    ctrl_da = DeclareLaunchArgument(
        "controllers_file",
        default_value=[sim, "_controllers.yaml"],
        # default_value=["hybrid_controllers.yaml"],
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

    # Copy from generate_demo_launch
    ld = LaunchDescription()
    ld.add_action(sim_da)
    ld.add_action(ctrl_da)
    ld.add_action(log_level_da)

    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
                launch_arguments={
                    "log_level": log_level,
                }.items(),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
            launch_arguments={
                "log_level": log_level,
            }.items(),
        )
    )

    # Launch the Move Group
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
            launch_arguments={
                "sim": sim,
                "log_level": log_level,
            }.items(),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            launch_arguments={
                "log_level": log_level,
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                str(moveit_config.package_path / "config/ada.urdf.xacro")
            ),
            " ",
            "sim:=",
            sim,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [str(moveit_config.package_path), "config", controllers_file]
    )

    # Joint Controllers
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            arguments=["--ros-args", "--log-level", log_level],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
            launch_arguments={
                "log_level": log_level,
            }.items(),
        )
    )

    return ld

    # OLD CODE STARTS HERE

    # robot_name = "ada"
    # package_name = robot_name + "_moveit"
    # robot_description = os.path.join(get_package_share_directory(
    #     package_name), "config", robot_name + ".urdf.xacro")
    # robot_description_config = xacro.process_file(robot_description)
    #
    # controller_config = os.path.join(
    #     get_package_share_directory(
    #         package_name), "config", "real_controllers.yaml"
    # )
    #
    # return LaunchDescription([
    #     Node(
    #         package="controller_manager",
    #         executable="ros2_control_node",
    #         parameters=[
    #             {"robot_description": robot_description_config.toxml()}, controller_config],
    #         output="screen",
    #     ),
    #
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #         output="screen",
    #     ),
    #
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["af_velocity_controller", "-c", "/controller_manager"],
    #         output="screen",
    #     ),
    #
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["af_joint_trajectory_controller", "-c", "/controller_manager"],
    #         output="screen",
    #     ),
    #
    #     Node(
    #         package="robot_state_publisher",
    #         executable="robot_state_publisher",
    #         name="robot_state_publisher",
    #         parameters=[
    #             {"robot_description": robot_description_config.toxml()}],
    #         output="screen",
    #     ),
    #
    # ])

