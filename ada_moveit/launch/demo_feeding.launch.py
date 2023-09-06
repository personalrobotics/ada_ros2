from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF


def generate_launch_description():
    # MoveIt Config
    moveit_config = MoveItConfigsBuilder(
        "ada", package_name="ada_moveit"
    ).to_moveit_configs()

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the default demo.launch.py
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/demo.launch.py")
            ),
        )
    )

    # Add the watchdog listener node
    watchdog_timeout = {"watchdog_timeout": ParameterValue(0.1, value_type=float)}
    watchdog_check_hz = {"watchdog_check_hz": ParameterValue(60.0, value_type=float)}
    initial_wait_time_sec = {
        "initial_wait_time_sec": ParameterValue(2.0, value_type=float)
    }
    ld.add_action(
        Node(
            package="ada_watchdog_listener",
            executable="ada_watchdog_listener_node",
            parameters=[
                watchdog_timeout,
                watchdog_check_hz,
                initial_wait_time_sec,
            ],
            remappings=[("~/watchdog", "/ada_watchdog/watchdog")],
            on_exit=Shutdown(),
        )
    )

    return ld
