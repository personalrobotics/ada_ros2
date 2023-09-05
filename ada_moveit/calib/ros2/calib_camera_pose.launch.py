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
                "0.0580205",
                "--y",
                "0.142803",
                "--z",
                "-0.144791",
                "--qx",
                "-0.00638382",
                "--qy",
                "-0.154877",
                "--qz",
                "0.984639",
                "--qw",
                "0.0803667",
                # "--roll",
                # "0.30908",
                # "--pitch",
                # "-0.0374741",
                # "--yaw",
                # "2.98455",
            ],
        ),
    ]
    return LaunchDescription(nodes)
