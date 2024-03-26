from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def get_move_group_launch(context):
    """
    Gets the launch description for MoveGroup, after removing sensors_3d
    if sim is mock.

    Adapted from https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file
    """
    sim = LaunchConfiguration("sim").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    # Get MoveIt Configs
    moveit_config = MoveItConfigsBuilder(
        "ada", package_name="ada_moveit"
    ).to_moveit_configs()

    # If sim is mock, set moveit_config.sensors_3d to an empty dictionary
    if sim == "mock":
        moveit_config.sensors_3d = {}

    entities = generate_move_group_launch(moveit_config).entities
    log_level_cmd_line_args = ["--ros-args", "--log-level", log_level]
    for entity in entities:
        if isinstance(entity, Node):
            entity.cmd.extend([normalize_to_list_of_substitutions(arg) for arg in log_level_cmd_line_args])
    return entities

def generate_launch_description():
    # Sim Launch Argument
    sim_da = DeclareLaunchArgument(
        "sim",
        default_value="real",
        description="Which sim to use: 'mock', 'isaac', or 'real'",
    )
    # Log Level
    log_level_da = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )

    ld = LaunchDescription()
    ld.add_action(sim_da)
    ld.add_action(log_level_da)
    ld.add_action(
        OpaqueFunction(function=get_move_group_launch)
    )
    return ld
