""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: j2n6s200_end_effector -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
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
                "0.032072",
                "--y",
                "0.13256",
                "--z",
                "-0.165098",
                "--roll",
                "-0.2707118",
                "--pitch",
                "-0.024086",
                "--yaw",
                "-3.1331081",
            ],
        ),
    ]
    return LaunchDescription(nodes)
