#!/usr/bin/env python3

# -*- coding: utf-8 -*-
# License: BSD 3-Clause. See LICENSE.md file in root directory.

"""
ROS 2 Node to publish the orientation of a target TF frame relative
to a reference TF frame, intended for simulating orientation sensors
based on kinematic information published to TF2.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from geometry_msgs.msg import QuaternionStamped, Quaternion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SimulatedOrientationPublisherNode(Node):
    """
    Listens to TF2 transforms and publishes the orientation of a specified
    target link relative to a specified reference frame as a QuaternionStamped message.
    Useful for providing kinematic orientation feedback in simulation environments
    where TF is populated by a robot_state_publisher.
    """

    def __init__(self):
        super().__init__("simulated_orientation_publisher_node")

        # --- Declare Parameters with descriptions ---
        target_link_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='The name of the TF frame whose orientation should be published (e.g., "atool_imu_frame").',
        )
        reference_frame_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='The name of the TF frame relative to which the orientation should be expressed (e.g., "base_link" or "world").',
        )
        publish_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="The topic name on which to publish the QuaternionStamped orientation data.",
        )
        publish_rate_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="The rate (in Hz) at which to look up the transform and publish the orientation.",
        )

        self.declare_parameter("target_link", "atool_imu_frame", target_link_descriptor)
        self.declare_parameter(
            "reference_frame", "base_link", reference_frame_descriptor
        )
        self.declare_parameter(
            "publish_topic",
            "/articutool/estimated_orientation",
            publish_topic_descriptor,
        )
        self.declare_parameter("publish_rate", 50.0, publish_rate_descriptor)

        # --- Get Parameters ---
        self.target_link_ = (
            self.get_parameter("target_link").get_parameter_value().string_value
        )
        self.reference_frame_ = (
            self.get_parameter("reference_frame").get_parameter_value().string_value
        )
        self.publish_topic_ = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )
        self.publish_rate_ = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        # --- Validate Parameters ---
        if not self.target_link_ or not self.reference_frame_:
            self.get_logger().fatal(
                "Parameters 'target_link' and 'reference_frame' must be non-empty strings. Shutting down."
            )
            # Proper way to exit from init is tricky, rely on context shutdown
            raise ValueError("Missing required parameters")

        if self.publish_rate_ <= 0:
            self.get_logger().fatal(
                "Parameter 'publish_rate' must be positive. Shutting down."
            )
            raise ValueError("Invalid publish rate")

        self.timer_period_ = 1.0 / self.publish_rate_

        self.get_logger().info(
            f"Initializing SimulatedOrientationPublisherNode:\n"
            f"\tTarget Link:      '{self.target_link_}'\n"
            f"\tReference Frame:  '{self.reference_frame_}'\n"
            f"\tPublish Topic:    '{self.publish_topic_}'\n"
            f"\tPublish Rate:     {self.publish_rate_} Hz ({self.timer_period_:.4f} s period)"
        )

        # --- TF2 Setup ---
        # Buffer stores received transforms, cache_time determines how long they are stored
        self.tf_buffer_ = Buffer(cache_time=Duration(seconds=5.0))
        # Listener subscribes to /tf and /tf_static and fills the buffer
        self.tf_listener_ = TransformListener(
            self.tf_buffer_, self, spin_thread=True
        )  # Use spin_thread=True for automatic background processing

        # --- Publisher Setup ---
        self.publisher_ = self.create_publisher(
            QuaternionStamped,
            self.publish_topic_,
            10,  # QoS history depth
        )

        # --- Timer Setup ---
        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

        self.get_logger().info(
            "Simulated Orientation Publisher node started and listening to TF."
        )

    def timer_callback(self):
        """
        Periodically looks up the TF transform and publishes the orientation quaternion.
        """
        try:
            # Lookup the transform at the latest available time using rclpy.time.Time()
            # This asks TF2 for the most recent data relating the two frames.
            lookup_time = (
                Time()
            )  # Equivalent to Time(seconds=0, nanoseconds=0) for latest
            transform_stamped = self.tf_buffer_.lookup_transform(
                target_frame=self.reference_frame_,
                source_frame=self.target_link_,
                time=lookup_time,
                timeout=Duration(seconds=0.1),
            )

            # Create and populate the QuaternionStamped message
            quat_stamped_msg = QuaternionStamped()

            # Header: Timestamp of the message itself, and the frame the data is relative to
            quat_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            quat_stamped_msg.header.frame_id = self.reference_frame_

            # Quaternion: Copy the orientation directly from the lookup result
            quat_stamped_msg.quaternion = transform_stamped.transform.rotation

            # Publish the message
            self.publisher_.publish(quat_stamped_msg)

        except Exception as e:
            # Catch any other unexpected errors
            self.get_logger().error(f"Unexpected error in timer callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimulatedOrientationPublisherNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as e:
        if node:
            node.get_logger().fatal(f"Node initialization failed: {e}")
        else:
            print(f"Node initialization failed: {e}")
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {e}")
        else:
            print(f"Unhandled exception in main: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
