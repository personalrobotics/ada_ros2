from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    # Get MoveIt Configs
    moveit_config = MoveItConfigsBuilder(
        "ada", package_name="ada_moveit"
    ).to_moveit_configs()

    # If sim is mock, set moveit_config.sensors_3d to an empty dictionary
    if sim == "mock":
        moveit_config.sensors_3d = {}

    return generate_move_group_launch(moveit_config).entities

def generate_launch_description():
    # Sim Launch Argument
    sim_da = DeclareLaunchArgument(
        "sim",
        default_value="real",
        description="Which sim to use: 'mock', 'isaac', or 'real'",
    )

    ld = LaunchDescription()
    ld.add_action(sim_da)
    ld.add_action(
        OpaqueFunction(function=get_move_group_launch)
    )
    return ld
