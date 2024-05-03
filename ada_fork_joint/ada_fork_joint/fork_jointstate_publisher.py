#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module defines a node that perceives the pose of known Apriltags, and publishes
the joint states of the fork handle accordingly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import numpy as np
import serial


class ForkJointstatePublisher(Node):
    """
    """

    # pylint: disable=too-many-instance-attributes
    # Smoothing and calibration vectors require more variables than pylint allows
    def __init__(self):
        super().__init__("fork_jointstate_publisher")
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)

        # Load parameters
        self.yaw_joint_name = "fork_yaw_joint"
        self.pitch_joint_name = "fork_pitch_joint"

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish)

    def publish(self):
        """
        Reads in IMU data, applies smoothing, thresholds, and rounding, and then publishes
        the position and velocity in a JointState message.
        """
        msg = JointState()
        msg.name = [self.yaw_joint_name, self.pitch_joint_name]
        msg.position = [0.0, 0.0]
        msg.velocity = [0.0, 0.0]
        msg.effort = []

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""

        self.publisher_.publish(msg)
        self.get_logger().debug(f"Publishing: {msg}")


def main(args=None):
    """Spins fork jointstate publisher node"""
    rclpy.init(args=args)

    minimal_publisher = ForkJointstatePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
