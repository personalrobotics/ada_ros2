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
                "0.0300509",
                "--y",
                "0.134319",
                "--z",
                "-0.138905",
                "--qx",
                "0.0017638",
                "--qy",
                "-0.134985",
                "--qz",
                "0.990838",
                "--qw",
                "0.00400733",
                # "--roll",
                # "0.27081",
                # "--pitch",
                # "0.00241343",
                # "--yaw",
                # "3.13318",
            ],
        ),
    ]
    return LaunchDescription(nodes)
