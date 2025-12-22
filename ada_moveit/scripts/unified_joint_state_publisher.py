# Copyright (c) 2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class UnifiedJointStatePublisher(Node):
    def __init__(self):
        super().__init__("unified_joint_state_publisher")
        # Declare and get the lock_joints parameter
        self.declare_parameter("lock_joints", False)
        self.lock_joints = (
            self.get_parameter("lock_joints").get_parameter_value().bool_value
        )
        if self.lock_joints:
            self.get_logger().info(
                "Articutool joints are LOCKED. Will not publish their state."
            )
        self.ada_joint_state = JointState()
        self.articutool_joint_state = JointState()

        # Store the previous state
        self.previous_ada_joint_state = JointState()
        self.previous_articutool_joint_state = JointState()

        self.ada_sub = self.create_subscription(
            JointState, "/ada/joint_states", self.ada_joint_state_callback, 10
        )
        self.articutool_sub = self.create_subscription(
            JointState,
            "/articutool/joint_states",
            self.articutool_joint_state_callback,
            10,
        )
        self.unified_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish_unified_joint_state)

    def ada_joint_state_callback(self, msg: JointState):
        self.previous_ada_joint_state = self.ada_joint_state  # Update previous
        self.ada_joint_state = msg

    def articutool_joint_state_callback(self, msg: JointState):
        self.previous_articutool_joint_state = (
            self.articutool_joint_state
        )  # Update previous
        self.articutool_joint_state = msg

    def publish_unified_joint_state(self):
        unified_msg = JointState()
        unified_msg.header.stamp = self.get_clock().now().to_msg()

        unified_msg.name = []
        unified_msg.position = []
        unified_msg.velocity = []
        unified_msg.effort = []

        ada_names_set = set(self.ada_joint_state.name)
        # Conditionally include Articutool joint states
        if self.lock_joints:
            articutool_names_set = set()
        else:
            articutool_names_set = set(self.articutool_joint_state.name)
        unified_names = list(ada_names_set.union(articutool_names_set))
        unified_msg.name = unified_names

        for name in unified_names:
            # Use current data if available, otherwise use previous
            ada_pos = None
            ada_vel = None
            ada_eff = None
            articutool_pos = None
            articutool_vel = None
            articutool_eff = None

            if name in self.ada_joint_state.name:
                idx = self.ada_joint_state.name.index(name)
                ada_pos = self.ada_joint_state.position[idx]
                if self.ada_joint_state.velocity:
                    ada_vel = self.ada_joint_state.velocity[idx]
                if self.ada_joint_state.effort:
                    ada_eff = self.ada_joint_state.effort[idx]
            elif name in self.previous_ada_joint_state.name:
                idx = self.previous_ada_joint_state.name.index(name)
                ada_pos = self.previous_ada_joint_state.position[idx]
                if self.previous_ada_joint_state.velocity:
                    ada_vel = self.previous_ada_joint_state.velocity[idx]
                if self.previous_ada_joint_state.effort:
                    ada_eff = self.previous_ada_joint_state.effort[idx]

            if name in self.articutool_joint_state.name:
                idx = self.articutool_joint_state.name.index(name)
                articutool_pos = self.articutool_joint_state.position[idx]
                if self.articutool_joint_state.velocity:
                    articutool_vel = self.articutool_joint_state.velocity[idx]
                if self.articutool_joint_state.effort:
                    articutool_eff = self.articutool_joint_state.effort[idx]
            elif name in self.previous_articutool_joint_state.name:
                idx = self.previous_articutool_joint_state.name.index(name)
                articutool_pos = self.previous_articutool_joint_state.position[idx]
                if self.previous_articutool_joint_state.velocity:
                    articutool_vel = self.previous_articutool_joint_state.velocity[idx]
                if self.previous_articutool_joint_state.effort:
                    articutool_eff = self.previous_articutool_joint_state.effort[idx]

            # Append the appropriate values to the unified message
            unified_msg.position.append(
                ada_pos
                if ada_pos is not None
                else articutool_pos
                if articutool_pos is not None
                else 0.0
            )
            if ada_vel is not None or articutool_vel is not None:
                unified_msg.velocity.append(
                    ada_vel
                    if ada_vel is not None
                    else articutool_vel
                    if articutool_vel is not None
                    else 0.0
                )
            if ada_eff is not None or articutool_eff is not None:
                unified_msg.effort.append(
                    ada_eff
                    if ada_eff is not None
                    else articutool_eff
                    if articutool_eff is not None
                    else 0.0
                )

        self.unified_pub.publish(unified_msg)


def main(args=None):
    rclpy.init(args=args)
    unified_publisher = UnifiedJointStatePublisher()
    rclpy.spin(unified_publisher)
    unified_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
