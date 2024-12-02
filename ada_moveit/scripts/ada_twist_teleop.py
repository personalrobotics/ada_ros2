#!/usr/bin/env python3

import time
import yaml
import numpy as np
import argparse

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_geometry_msgs import Vector3Stamped  # pylint: disable=unused-import
import tf2_py as tf2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState

BASE_FRAME = "j2n6s200_link_base"
EE_FRAME = "forkTip"
JOINT_NAMES = [
    'j2n6s200_joint_1',
    'j2n6s200_joint_2',
    'j2n6s200_joint_3',
    'j2n6s200_joint_4',
    'j2n6s200_joint_5',
    'j2n6s200_joint_6',
    'af_joint_1',
    'af_joint_2',
]
DEFAULT_ANGULAR = [0.0, 0.0, 0.0]
DEFAULT_LINEAR = [0.0, 0.0, 0.0]
DEFAULT_DURATION_S = 1.0
WAIT_FOR_TF_S = 1.0

def parse_args():
    parser = argparse.ArgumentParser(description='Control robot arm with specified angular and linear velocities.')
    parser.add_argument('--angular', nargs=3, type=float, default=DEFAULT_ANGULAR,
                        help='Angular velocity in x, y, z directions (rad/s)')
    parser.add_argument('--linear', nargs=3, type=float, default=DEFAULT_LINEAR,
                        help='Linear velocity in x, y, z directions (m/s)')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION_S,
                        help='Duration of the motion (seconds)')
    parser.add_argument('--frame_id', type=str, default=BASE_FRAME, choices=[BASE_FRAME, EE_FRAME], help='Frame ID for the twist message')
    return parser.parse_args()

def publish_zero_twist(node, twist_pub):
    twist_msg = TwistStamped()
    twist_msg.header.stamp = node.get_clock().now().to_msg()
    twist_msg.header.frame_id = BASE_FRAME
    twist_msg.twist.linear.x = 0.0
    twist_msg.twist.linear.y = 0.0
    twist_msg.twist.linear.z = 0.0
    twist_msg.twist.angular.x = 0.0
    twist_msg.twist.angular.y = 0.0
    twist_msg.twist.angular.z = 0.0

    twist_pub.publish(twist_msg)

def main(args=None):
    # Initialize ROS context
    rclpy.init(args=args)
    node = rclpy.create_node("ada_twist_teleop")
    twist_pub = node.create_publisher(
        TwistStamped, "/jaco_af_cartesian_controller/twist_cmd", 1
    )

    # Initialize the tf2 buffer and listener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # pylint: disable=unused-variable

    # Parse command-line arguments
    args = parse_args()
    frame_id = args.frame_id
    angular = args.angular
    linear = args.linear
    duration_s = args.duration
    print(f"angular (rad/s) = {angular}, linear (m/s) = {linear}, duration (s) = {duration_s}")

    # Create the cartesian control messages
    # The linear velocity is always in the base frame
    linear_msg = Vector3Stamped()
    linear_msg.header.stamp = Time().to_msg()  # use latest time
    linear_msg.header.frame_id = frame_id
    # The angular velocity is always in the end effector frame
    angular_msg = Vector3Stamped()
    angular_msg.header.stamp = Time().to_msg()  # use latest time
    angular_msg.header.frame_id = EE_FRAME
    # The final message should be either in the base or end effector frame.
    # It should match the `robot_link_command_frame`` servo param.
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = BASE_FRAME

    # Delay for transforms to populate
    start_time_s = time.time()
    while time.time() - start_time_s < WAIT_FOR_TF_S:
        rclpy.spin_once(node, timeout_sec=0)

    start_time_s = time.time()
    while time.time() - start_time_s < duration_s:
        rclpy.spin_once(node, timeout_sec=0)

        linear_msg.vector.x = linear[0]
        linear_msg.vector.y = linear[1]
        linear_msg.vector.z = linear[2]
        twist_msg.twist.linear = linear_msg.vector
        if linear_msg.header.frame_id != twist_msg.header.frame_id:
            try:
                linear_transformed = tf_buffer.transform(
                    linear_msg, twist_msg.header.frame_id
                )
                twist_msg.twist.linear = linear_transformed.vector
            except tf2.ExtrapolationException as exc:
                node.get_logger().warning(
                    f"Transform from {linear_msg.header.frame_id} to "
                    f"{twist_msg.header.frame_id} failed: {type(exc)}: {exc}\n"
                    f"Interpreting the linear velocity in {twist_msg.header.frame_id} "
                    "without transforming."
                )

        angular_msg.vector.x = angular[0]
        angular_msg.vector.y = angular[1]
        angular_msg.vector.z = angular[2]
        twist_msg.twist.angular = angular_msg.vector
        if angular_msg.header.frame_id != twist_msg.header.frame_id:
            try:
                angular_transformed = tf_buffer.transform(
                    angular_msg, twist_msg.header.frame_id
                )
                twist_msg.twist.angular = angular_transformed.vector
            except tf2.ExtrapolationException as exc:
                node.get_logger().warning(
                    f"Transform from {angular_msg.header.frame_id} to "
                    f"{twist_msg.header.frame_id} failed: {type(exc)}: {exc}\n"
                    f"Interpreting the angular velocity in {twist_msg.header.frame_id}"
                    " without transforming."
                )

        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_pub.publish(twist_msg)
        publish_zero_twist(node, twist_pub)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
