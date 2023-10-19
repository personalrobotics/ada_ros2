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
                "0.029564",
                "--y",
                "0.131007",
                "--z",
                "-0.170077",
                "--qx",
                "-0.0128317",
                "--qy",
                "0.1297282",
                "--qz",
                "-0.9914663",
                "--qw",
                "0.0007435",
            ],
        ),
    ]
    return LaunchDescription(nodes)
