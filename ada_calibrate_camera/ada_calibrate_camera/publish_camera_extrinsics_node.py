#!/usr/bin/env python3
# Copyright (c) 2024, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

"""
This module contains a rclpy ROS2 node that takes in parameters for the translation
and rotation (quaternion) as well as the frame_id's and publishes the requested transform.
"""

# Standard imports

# Third-party imports
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# Local imports


class PublishCameraExtrinsics(Node):
    """
    This class defines a ROS2 node that takes in parameters for the translation
    and rotation (quaternion) as well as the frame_id's and publishes the requested transform.
    """

    def __init__(self):
        """
        Initializes the node and sets up the timer to publish the transform
        """
        super().__init__("publish_camera_extrinsics")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("frame_id", "j2n6s200_end_effector"),
                ("child_frame_id", "camera_color_optical_frame"),
                ("x", 0.0),
                ("y", 0.0),
                ("z", 0.0),
                ("roll", 0.0),
                ("pitch", 0.0),
                ("yaw", 0.0),
                ("rate_hz", 10.0),
            ],
        )

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        timer_interval = (
            1.0 / self.get_parameter("rate_hz").get_parameter_value().double_value
        )
        self.timer = self.create_timer(timer_interval, self.publish_transform)

    def publish_transform(self):
        """
        Publishes the transform from the frame_id to the child_frame_id
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        transform.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )
        transform.transform.translation.x = (
            self.get_parameter("x").get_parameter_value().double_value
        )
        transform.transform.translation.y = (
            self.get_parameter("y").get_parameter_value().double_value
        )
        transform.transform.translation.z = (
            self.get_parameter("z").get_parameter_value().double_value
        )
        quat = R.from_euler(
            "ZYX",
            [
                self.get_parameter("yaw").get_parameter_value().double_value,
                self.get_parameter("pitch").get_parameter_value().double_value,
                self.get_parameter("roll").get_parameter_value().double_value,
            ],
        ).as_quat()
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(transform)


def main():
    """
    The main function to create and spin the node
    """
    rclpy.init()
    node = PublishCameraExtrinsics()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
