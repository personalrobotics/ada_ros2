# Copyright (c) 2024, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
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

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    # MoveIt Config
    moveit_config = MoveItConfigsBuilder(
        "ada", package_name="ada_moveit"
    ).to_moveit_configs()

    # Calibration Launch Argument
    calib_da = DeclareLaunchArgument(
        "calib",
        default_value="auto",
        description="Which calibration folder to use. Files are located in the `ada_calibrate_camera` package.",
    )
    calib = LaunchConfiguration("calib")

    # Sim Launch Argument
    sim_da = DeclareLaunchArgument(
        "sim",
        default_value="real",
        description="Which sim to use: 'mock', 'isaac', or 'real'",
    )
    sim = LaunchConfiguration("sim")

    # Use Octomap Launch Argument
    use_octomap_da = DeclareLaunchArgument(
        "use_octomap",
        default_value="true",
        description="Whether to use octomap for collision checking",
    )
    use_octomap = LaunchConfiguration("use_octomap")

    # Controllers File
    ctrl_da = DeclareLaunchArgument(
        "controllers_file",
        default_value=[sim, "_controllers.yaml"],
        description="ROS2 Controller YAML configuration in config folder",
    )
    controllers_file = LaunchConfiguration("controllers_file")

    # Servo File
    servo_da = DeclareLaunchArgument(
        "servo_file",
        default_value=[sim, "_servo.yaml"],
        description="MoveIt Servo YAML configuration in config folder",
    )
    servo_file = LaunchConfiguration("servo_file")

    # Log Level
    log_level_da = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    # Copy from generate_demo_launch
    ld = LaunchDescription()
    ld.add_action(calib_da)
    ld.add_action(sim_da)
    ld.add_action(use_octomap_da)
    ld.add_action(ctrl_da)
    ld.add_action(servo_da)
    ld.add_action(log_level_da)

    # Launch argument for whether to use moveit servo or not
    ld.add_action(DeclareBooleanLaunchArg("use_servo", default_value=False))

    # Camera Extrinsics Calibration
    ada_calibrate_camera_package_path = get_package_share_directory(
        "ada_calibrate_camera"
    )
    ld.add_action(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    ada_calibrate_camera_package_path,
                    "launch/publish_camera_extrinsics_launch.xml",
                )
            ),
            launch_arguments={
                "log_level": log_level,
                "calibration_file_name": calib,
            }.items(),
        ),
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

    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
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
                "use_octomap": use_octomap,
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

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            launch_arguments={
                "log_level": log_level,
            }.items(),
            condition=IfCondition(LaunchConfiguration("db")),
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

    # Launch MoveIt Servo
    servo_config = PathJoinSubstitution(
        [str(moveit_config.package_path), "config", servo_file]
    )
    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            parameters=[
                servo_config,
                robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,  # If set, use IK instead of the inverse jacobian
            ],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            condition=IfCondition(LaunchConfiguration("use_servo")),
        )
    )

    robot_controllers = PathJoinSubstitution(
        [str(moveit_config.package_path), "config", controllers_file]
    )

    # Joint Controllers
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            # Commented out the log-level since the joint state publisher logs every joint read
            # when on debug level
            arguments=["--ros-args"],  # , "--log-level", log_level],
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
