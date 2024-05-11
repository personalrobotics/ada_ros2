""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: j2n6s200_end_effector -> camera_color_optical_frame """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# NOTE: This was hand-tweaked, assimng 0 pitch and -3.14 yaw
# (i.e., assuming the only unknown orientation was roll)
# by having the camera observe the robot itself (point at
# joint 1) and comparing the depthcloud to the robot's mesh
# in RVIZ. The transform was then adjusted until the depthcloud
# visually matched the mesh.
def generate_launch_description() -> LaunchDescription:
    # Log Level
    log_level_da = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error, fatal)",
    )
    log_level = LaunchConfiguration("log_level")

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "j2n6s200_end_effector",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "0.038",
                "--y",
                "0.129",
                "--z",
                "-0.155",
                "--roll",
                "-0.2395",
                "--pitch",
                "0.0",
                "--yaw",
                "-3.14",
                "--ros-args",
                "--log-level",
                log_level,
            ],
        ),
    ]
    return LaunchDescription([log_level_da] + nodes)
