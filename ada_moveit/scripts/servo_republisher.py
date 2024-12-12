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
from threading import Lock
from typing import List

# Third-party imports
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped, Vector3
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_geometry_msgs import Vector3Stamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Local imports


JOINT_NAMES = [
    "j2n6s200_joint_1",
    "j2n6s200_joint_2",
    "j2n6s200_joint_3",
    "j2n6s200_joint_4",
    "j2n6s200_joint_5",
    "j2n6s200_joint_6",
]
POSITION_INTERFACE_NAME = "position"
VELOCITY_INTERFACE_NAME = "velocity"


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

    # pylint: disable=too-many-instance-attributes
    # This number is fine because we need to maintain references to all pubs/subs

    def __init__(self):
        """
        Initialize the node.
        """
        super().__init__("servo_republisher")

        # Read the parameters
        self.read_params()

        # Determine if the joint controller is position or velocity.
        self.joint_controller_interface_name = None
        self.joint_controller_get_parameter = self.create_client(
            GetParameters,
            "~/joint_controller_get_parameters",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.joint_controller_get_parameter_invoke_time = None
        self.joint_controller_get_parameter_future = None
        self.last_published_joint_positions = None
        self.last_published_joint_positions_time = None
        self.joint_states_sub = None
        self.latest_joint_states_lock = Lock()
        self.latest_joint_states = {}
        self.get_joint_controller_interface_name()

        # Create the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create the timer to check for stale messages
        self.latest_msg_lock = Lock()
        self.latest_msg = None
        # In the worst case, the timer can take up to self.servo_timeout_sec +
        # timer_duration to find a stale message. Thus, we run the timer at a
        # shorter interval than self.servo_timeout_sec
        self.stale_msg_timer = self.create_timer(
            self.servo_timeout_sec / 2.0,
            self.stale_msg_check,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.servo_timeout_sec = Duration(seconds=self.servo_timeout_sec)

        # Store the previous joint velocities for the exponential moving average
        self.prev_joint_velocities = [0.0 for _ in JOINT_NAMES]

        # Create the publishers
        self.cartesian_output_pub = self.create_publisher(
            TwistStamped, "~/cartesian_commands_output", 1
        )
        self.joint_output_pub = self.create_publisher(
            Float64MultiArray, "~/joint_commands_output", 1
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
        # Parameters for cartesian control
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

        # Parameters for joint control
        self.exponential_moving_average_alpha = self.declare_parameter(
            "exponential_moving_average_alpha",
            0.5,
            descriptor=ParameterDescriptor(
                name="exponential_moving_average_alpha",
                type=ParameterType.PARAMETER_DOUBLE,
                description=(
                    "To smoothen velocities, this node will take the exponential "
                    "weighted average of the non-zero joint velocities it gets. "
                    "Zero velocities are sent to the controller without modification. "
                    "To disable the exponential moving average, set this to 1.0."
                ),
                read_only=True,
            ),
        ).value
        if self.exponential_moving_average_alpha < 0.0:
            self.get_logger().warn("exponential_moving_average_alpha must be >= 0")
            self.exponential_moving_average_alpha = 0.0
        if self.exponential_moving_average_alpha > 1.0:
            self.get_logger().warn("exponential_moving_average_alpha must be <= 1")
            self.exponential_moving_average_alpha = 1.0

        # Parameters for both cartesian and joint control
        self.servo_timeout_sec = self.declare_parameter(
            "servo_timeout_sec",
            0.33,
            descriptor=ParameterDescriptor(
                name="servo_timeout_sec",
                type=ParameterType.PARAMETER_DOUBLE,
                description=(
                    "Stop moving if you haven't received a command in these many "
                    "seconds."
                ),
                read_only=True,
            ),
        ).value

    def get_joint_controller_interface_name(
        self, parameter_name: str = "interface_name", timeout: float = 1.0
    ) -> None:
        """
        Gets the interface name of the joint controller, either POSITION_INTERFACE_NAME or
        VELOCITY_INTERFACE_NAME, by getting that controller's parameter value.
        """
        # If we already got the joint controller interface name, ignore this call
        if self.joint_controller_interface_name is not None:
            return

        # Check if we are already waiting for a response
        if (self.joint_controller_get_parameter_future is not None):
            # If the future is done, wait for the callback to be called
            if self.joint_controller_get_parameter_future.done():
                return
            # If the future is not done, then wait for it to finish
            if (self.get_clock().now() - self.joint_controller_get_parameter_invoke_time) < Duration(seconds=timeout):
                return
            # If the future is not done and it has been too long, then cancel it
            self.joint_controller_get_parameter.remove_pending_request(self.joint_controller_get_parameter_future)
        
        # Send the request
        request = GetParameters.Request(
            names=[parameter_name],
        )
        self.joint_controller_get_parameter_future = self.joint_controller_get_parameter.call_async(request)
        self.joint_controller_get_parameter_invoke_time = self.get_clock().now()
        self.joint_controller_get_parameter_future.add_done_callback(self.get_joint_controller_interface_name_callback)
        

    def get_joint_controller_interface_name_callback(self, future: rclpy.task.Future) -> None:
        """
        Callback for the get_joint_controller_interface_name service call.
        """
        # If we already got the joint controller interface name, ignore this call
        if self.joint_controller_interface_name is not None:
            return

        # If the future succeeded, get the response
        try:
            response = future.result()
        except Exception as e: # pylint: disable=broad-except
            self.get_logger().error(
                f"Failed to get the joint controller interface name: {e}"
            )
            return

        # Check if the response is valid
        if len(response.values) > 0 and response.values[0].type == 4:
            if response.values[0].string_value in [
                POSITION_INTERFACE_NAME,
                VELOCITY_INTERFACE_NAME,
            ]:
                self.joint_controller_interface_name = response.values[0].string_value
                # If it is a position controller, then subscribe to the joint states
                if response.values[0].string_value == POSITION_INTERFACE_NAME:
                    self.joint_states_sub = self.create_subscription(
                        JointState,
                        "~/joint_states",
                        self.joint_state_callback,
                        1,
                        callback_group=MutuallyExclusiveCallbackGroup(),
                    )
            else:
                self.get_logger().error(
                    "This node only supports position or velocity joint controllers. "
                    "JointJog messages will be ignored."
                )
        
        # Reset the future
        self.joint_controller_get_parameter_future = None

    def joint_state_callback(self, msg: JointState) -> None:
        """
        Stores the latest joint states.
        """
        joint_states = {}
        for i, name in enumerate(msg.name):
            position = msg.position[i]
            joint_states[name] = (position, Time.from_msg(msg.header.stamp))

        with self.latest_joint_states_lock:
            self.latest_joint_states.update(joint_states)

    def cartesian_input_callback(self, msg: TwistStamped) -> None:
        """
        Callback for the cartesian input subscriber. Transform the frames,
        update the timestamp, and republish.
        """
        # Store the latest message
        with self.latest_msg_lock:
            self.latest_msg = msg

        # Transform the linear velocity to the base frame
        self.linear_velocity.vector = msg.twist.linear
        if self.linear_velocity.header.frame_id == self.output_twist.header.frame_id:
            final_linear_velocity = self.linear_velocity
        else:
            final_linear_velocity = self.tf_buffer.transform(
                self.linear_velocity, self.output_twist.header.frame_id
            )
        # Transform the angular velocity to the base frame
        self.angular_velocity.vector = msg.twist.angular
        if self.angular_velocity.header.frame_id == self.output_twist.header.frame_id:
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

    def joint_input_callback(self, msg: JointJog) -> None:
        """
        Callback for the joint input subscriber. Update the timestamp and republish.
        """
        # First, determine the interface type of the joint controller
        if self.joint_controller_interface_name is None:
            self.get_logger().warn(
                "Cannot process JointJog messages until the joint controller interface name is known.",
                throttle_duration_sec=1.0,
            )
            self.get_joint_controller_interface_name()
            return

        # Check the message
        if len(msg.joint_names) == 0:
            self.get_logger().warn(
                "Must have a non-empty list of joints in JointJog messages"
            )
            return
        if len(msg.displacements) > 0:
            self.get_logger().warn(
                "The node does not support joint displacements. Ignoring them."
            )
        if len(msg.velocities) != len(msg.joint_names):
            self.get_logger().warn(
                "Velocities and joint_names must have the same length. Will truncate or zero-pad."
            )

        # Store the latest message
        with self.latest_msg_lock:
            self.latest_msg = msg

        # Process the JointJog message
        curr_joint_velocities = []
        for i in range(len(JOINT_NAMES)):  # pylint: disable=consider-using-enumerate
            try:
                j = msg.joint_names.index(JOINT_NAMES[i])
                if j < len(msg.velocities):
                    curr_joint_velocities.append(msg.velocities[j])
                else:
                    curr_joint_velocities.append(0.0)
            except ValueError:  # The joint's velocity was unspecified
                curr_joint_velocities.append(0.0)

        # Apply the exponential moving average
        joint_velocities = []
        if np.all(np.isclose(curr_joint_velocities, 0.0)):
            joint_velocities = curr_joint_velocities
        else:
            joint_velocities = [
                curr_joint_velocities[i] * self.exponential_moving_average_alpha
                + self.prev_joint_velocities[i]
                * (1.0 - self.exponential_moving_average_alpha)
                for i in range(
                    len(JOINT_NAMES)
                )  # pylint: disable=consider-using-enumerate
            ]

        # Publish the joint_velocities
        self.publish_joint_velocity(joint_velocities, msg.duration)
        self.prev_joint_velocities = joint_velocities

    def publish_joint_velocity(
        self, joint_velocities: List[float], duration_secs: float = 0.0
    ) -> None:
        """
        Publish the specified joint velocities to the controller.
        Converts them to positions if that is the interface name of the controller.
        """
        # At this point, self.joint_controller_interface_name is either POSITION_INTERFACE_NAME
        # or VELOCITY_INTERFACE_NAME.
        # We are also guaranteed that we are given velocities equal to the number of joints.
        if self.joint_controller_interface_name == VELOCITY_INTERFACE_NAME:
            self.joint_output_pub.publish(Float64MultiArray(data=joint_velocities))
        else:
            missing_joint_names = []
            curr_joint_positions = []
            with self.latest_joint_states_lock:
                for i, joint_name in enumerate(JOINT_NAMES):
                    if joint_name in self.latest_joint_states:
                        # Get the joint position
                        joint_position, joint_state_time = self.latest_joint_states[joint_name]
                        if (
                            self.last_published_joint_positions is not None
                            and self.last_published_joint_positions_time is not None
                            and joint_state_time < self.last_published_joint_positions_time
                        ):
                            # If the last published joint positions are newer than the joint state, use them
                            joint_position = self.last_published_joint_positions[i]
                        # Add the joint position to the list
                        curr_joint_positions.append(
                            joint_position
                        )
                    else:
                        missing_joint_names.append(joint_name)
            if len(missing_joint_names) > 0:
                self.get_logger().warn(
                    f"Haven't received joint states for {missing_joint_names}. Ignoring "
                    "JointJog message."
                )
            else:
                next_joint_positions = [
                    curr_joint_positions[i] + joint_velocities[i] * duration_secs
                    for i in range(len(JOINT_NAMES))
                ]
                self.joint_output_pub.publish(
                    Float64MultiArray(data=next_joint_positions)
                )
                self.last_published_joint_positions = next_joint_positions
                self.last_published_joint_positions_time = self.get_clock().now()

    def stale_msg_check(self) -> None:
        """
        If it has been over `self.servo_timeout_sec` since the latest message
        and that message was non-zero, this node publishes a zero message to
        that topic.
        """
        # Get the latest message
        with self.latest_msg_lock:
            latest_msg = self.latest_msg

        # If we haven't received a message, continue
        if latest_msg is None:
            return

        # If the message is not stale, continue
        if (
            self.get_clock().now() - Time.from_msg(latest_msg.header.stamp)
            < self.servo_timeout_sec
        ):
            return

        # If the message is non-zero, publish a zero-message on the corresponding topic.
        if isinstance(latest_msg, TwistStamped):
            # pylint: disable=too-many-boolean-expressions
            if (
                latest_msg.twist.linear.x != 0.0
                or latest_msg.twist.linear.y != 0.0
                or latest_msg.twist.linear.z != 0.0
                or latest_msg.twist.angular.x != 0.0
                or latest_msg.twist.angular.y != 0.0
                or latest_msg.twist.angular.z != 0.0
            ):
                # Publish a zero-velocity message
                self.output_twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
                self.output_twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
                self.cartesian_output_pub.publish(self.output_twist)
                self.get_logger().warn(
                    f"Stopping the robot's caresian control because of a stale message"
                )
        elif isinstance(latest_msg, JointJog):
            if len(latest_msg.velocities) != len(JOINT_NAMES) or not np.all(
                np.isclose(latest_msg.velocities, 0.0)
            ):
                # Publish a zero-velocity message
                self.publish_joint_velocity([0.0 for _ in JOINT_NAMES])
                self.prev_joint_velocities = [0.0 for _ in JOINT_NAMES]
                self.get_logger().warn(
                    f"Stopping the robot's joint control because of a stale message"
                )

        # Once the zero-message is send, reset the latest_msg
        with self.latest_msg_lock:
            if self.latest_msg == latest_msg:
                self.latest_msg = None


def main(args: List = None) -> None:
    """
    Create the ROS2 node and run the action servers.
    """
    rclpy.init(args=args)

    servo_republisher = ServoRepublisher()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    rclpy.spin(servo_republisher, executor=executor)

    # Destroy the node explicitly
    servo_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
