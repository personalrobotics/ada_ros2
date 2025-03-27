#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CombinedJointStates(Node):
    def __init__(self):
        super().__init__('combined_joint_states')
        self.articutool_sub = self.create_subscription(
            JointState,
            '/articutool/joint_states',
            self.articutool_callback,
            10
        )
        self.ada_sub = self.create_subscription(
            JointState,
            '/ada/joint_states',
            self.ada_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/ada/combined_joint_states', 10)
        self.articutool_joint_state = JointState()
        self.ada_joint_state = JointState()

    def articutool_callback(self, msg):
        self.articutool_joint_state = msg
        self.publish_combined()

    def ada_callback(self, msg):
        self.ada_joint_state = msg
        self.publish_combined()

    def publish_combined(self):
        combined_msg = JointState()
        combined_msg.header = self.articutool_joint_state.header if self.articutool_joint_state.header.stamp.sec != 0 else self.ada_joint_state.header #use the header from the most recent message.
        combined_msg.name = self.articutool_joint_state.name + self.ada_joint_state.name
        combined_msg.position = self.articutool_joint_state.position + self.ada_joint_state.position
        combined_msg.velocity = self.articutool_joint_state.velocity + self.ada_joint_state.velocity
        combined_msg.effort = self.articutool_joint_state.effort + self.ada_joint_state.effort

        self.publisher.publish(combined_msg)

def main(args=None):
    rclpy.init(args=args)
    combined_node = CombinedJointStates()
    rclpy.spin(combined_node)
    combined_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
