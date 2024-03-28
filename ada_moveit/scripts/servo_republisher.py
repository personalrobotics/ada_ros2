#!/usr/bin/env python3
"""
This module defines the ServoRepublisher node, which listens for servo commands
(either TwistStamped or JointJog) on input topics, and republishes them on output
topics with the following modifications:
  1. The header timestamp is replaced with the current time. This is useful if
     the device sending the servo commands (e.g., a personal smartphone) does not
     have a clock that is synchronized to the robot's clock.
  2. For cartesian TwistStamped commands, this script interprets the linear velocity
     in the **base frame** of the robot and angular velocity in the **end effector
     frame** of the robot. It transforms both to the **base frame** before republishing.
"""

# Standard imports
from typing import List

# Third-party imports
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_geometry_msgs import Vector3Stamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Local imports


class ServoRepublisher(Node):
    """
    This class defines the ServoRepublisher node, which listens for servo commands
    (either TwistStamped or JointJog) on input topics, and republishes them on output
    topics with the following modifications:
    1. The header timestamp is replaced with the current time. This is useful if
        the device sending the servo commands (e.g., a personal smartphone) does not
        have a clock that is synchronized to the robot's clock.
    2. For cartesian TwistStamped commands, this script interprets the linear velocity
        in the **base frame** of the robot and angular velocity in the **end effector
        frame** of the robot. It transforms both to the **base frame** before republishing.
    """
    def __init__(self):
        """
        Initialize the node.
        """
        # pylint: disable=too-many-instance-attributes
        # This number is fine because we need to maintain references to all pubs/subs
        super().__init__("servo_republisher")

        # Read the parameters
        self.read_params()

        # Create the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create the publishers
        self.cartesian_output_pub = self.create_publisher(
            TwistStamped, "~/cartesian_commands_output", 1
        )
        self.joint_output_pub = self.create_publisher(
            JointJog, "~/joint_commands_output", 1
        )

        # Create the subscribers
        self.cartesian_input_sub = self.create_subscription(
            TwistStamped,
            "~/cartesian_commands_input",
            self.cartesian_input_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.joint_input_sub = self.create_subscription(
            JointJog,
            "~/joint_commands_input",
            self.joint_input_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def read_params(self):
        """
        Read parameters from the ROS 2 parameter server.
        """
        # pylint: disable=attribute-defined-outside-init
        input_linear_velocity_frame = self.declare_parameter(
            "input_linear_velocity_frame",
            "j2n6s200_link_base",
            descriptor=ParameterDescriptor(
                name="input_linear_velocity_frame",
                type=ParameterType.PARAMETER_STRING,
                description=(
                    "The frame in which the input linear velocity is interpreted."
                ),
                read_only=True,
            ),
        ).value
        self.linear_velocity = Vector3Stamped()
        self.linear_velocity.header.frame_id = input_linear_velocity_frame
        input_angular_velocity_frame = self.declare_parameter(
            "input_angular_velocity_frame",
            "forkTip",
            descriptor=ParameterDescriptor(
                name="input_angular_velocity_frame",
                type=ParameterType.PARAMETER_STRING,
                description=(
                    "The frame in which the input angular velocity is interpreted."
                ),
                read_only=True,
            ),
        ).value
        self.angular_velocity = Vector3Stamped()
        self.angular_velocity.header.frame_id = input_angular_velocity_frame
        output_twist_frame = self.declare_parameter(
            "output_twist_frame",
            "j2n6s200_link_base",
            descriptor=ParameterDescriptor(
                name="output_twist_frame",
                type=ParameterType.PARAMETER_STRING,
                description=("The frame in which the output twist is published."),
                read_only=True,
            ),
        ).value
        self.output_twist = TwistStamped()
        self.output_twist.header.frame_id = output_twist_frame

    def cartesian_input_callback(self, msg: TwistStamped):
        """
        Callback for the cartesian input subscriber. Transform the frames,
        update the timestamp, and republish.
        """
        # Transform the linear velocity to the base frame
        self.linear_velocity.vector = msg.twist.linear
        if (
            self.linear_velocity.header.frame_id
            == self.output_twist.header.frame_id
        ):
            final_linear_velocity = self.linear_velocity
        else:
            final_linear_velocity = self.tf_buffer.transform(
                self.linear_velocity, self.output_twist.header.frame_id
            )
        # Transform the angular velocity to the base frame
        self.angular_velocity.vector = msg.twist.angular
        if (
            self.angular_velocity.header.frame_id
            == self.output_twist.header.frame_id
        ):
            final_angular_velocity = self.angular_velocity
        else:
            final_angular_velocity = self.tf_buffer.transform(
                self.angular_velocity, self.output_twist.header.frame_id
            )

        # Create the output message
        self.output_twist.header.stamp = self.get_clock().now().to_msg()
        self.output_twist.twist.linear = final_linear_velocity.vector
        self.output_twist.twist.angular = final_angular_velocity.vector

        # Publish the output message
        self.cartesian_output_pub.publish(self.output_twist)

    def joint_input_callback(self, msg: JointJog):
        """
        Callback for the joint input subscriber. Update the timestamp and republish.
        """
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_output_pub.publish(msg)


def main(args: List = None) -> None:
    """
    Create the ROS2 node and run the action servers.
    """
    rclpy.init(args=args)

    servo_republisher = ServoRepublisher()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(servo_republisher, executor=executor)

    # Destroy the node explicitly
    servo_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
