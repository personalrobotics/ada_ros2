#!/usr/bin/env python3

# Standard imports
import readline
import sys
import threading

# Third-party imports
from controller_manager_msgs.srv import SwitchController
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from moveit_msgs.msg import MoveItErrorCodes
import numpy as np
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import kinova
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.timer import Rate
import ros2_numpy
from sensor_msgs.msg import CompressedImage, JointState
import tf2_py as tf2
import tf2_ros

# Local imports

def print_and_flush(message: str):
    """
    Print a message and flush the output.

    Parameters
    ----------
    message : str
        The message to print.
    """
    print(message)
    sys.stdout.flush()

class CalibrateCameraNode(Node):
    """
    This node uses a command-line interface to enable users to calibrate ADA's
    eye-on-hand camera using a charucoboard of known size. The robot base should
    be mounted on the tripod, for ease of changing height, and in a location with
    at least 1m empty space on all sides of the robot's base. After detecting the
    initial charucoboard, this node automatically moves the robot around it, and
    saves the images and joint states. It then computes the camera calibration, 
    given those images. Users are then encouraged to change the height of the robot
    relative to the charucoboard and repeat the process to improve the calibration.
    """
    
    def __init__(self):
        """
        Do the initialization steps that don't require rclpy to be spinning.
        """
        super().__init__("calibrate_camera")

        # Read the parameters
        self.active_controller = None
        self.read_params()

        # Create the TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create the twist publisher
        self.twist_pub = self.node.create_publisher(
            TwistStamped,
            "~/servo_twist_cmds",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # Create the MoveIt2 object
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=node,
            joint_names=kinova.joint_names(),
            base_link_name=kinova.base_link_name(),
            end_effector_name="forkTip",
            group_name="jaco_arm",
            callback_group=callback_group,
        )

        # Subscribe to the RGB camera feed
        self.bridge = CvBridge()
        self.latest_rgb_img_lock = threading.Lock()
        self.latest_rgb_img = None
        self.rgb_img_sub = self.create_subscription(
            CompressedImage,
            "~/compressed_image",
            self.rgb_img_callback,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Create the service to (de)activate the controller
        self.switch_controller_client = self.create_client(
            SwitchController, 
            "~/switch_controller",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

    def read_params(self):
        """
        Read the parameters from the parameter server.
        """
        self.all_controllers = self.declare_parameter(
            "all_controllers",
            [
                "jaco_arm_cartesian_controller",
                "jaco_arm_controller",
            ],
            ParameterDescriptor(
                name="all_controllers",
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="The names of all the controllers.",
                read_only=True,
            ),
        ).value
        self.default_moveit2_controller = self.declare_parameter(
            "default_moveit2_controller",
            "jaco_arm_controller",
            ParameterDescriptor(
                name="default_moveit2_controller",
                type=ParameterType.PARAMETER_STRING,
                description="The default controller for MoveIt2.",
                read_only=True,
            ),
        ).value
        if self.default_moveit2_controller not in self.all_controllers:
            self.get_logger().error(
                "The default controller for MoveIt2 is not in the list of all controllers."
            )
            sys.exit(1)

        self.starting_arm_configuration = self.declare_parameter(
            "starting_arm_configuration",
            [
                -2.3149168248766614 # j2n6s200_joint_1
                3.1444595465032634 # j2n6s200_joint_2
                1.7332586075115999 # j2n6s200_joint_3
                -2.3609596843308234 # j2n6s200_joint_4
                4.43936623280362 # j2n6s200_joint_5
                3.06866544924739 # j2n6s200_joint_6
            ],
            ParameterDescriptor(
                name="starting_arm_configuration",
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="The starting configuration of the arm.",
                read_only=True,
            ),
        ).value

        self.wait_before_capture_secs = self.declare_parameter(
            "wait_before_capture_secs",
            2.0,
            ParameterDescriptor(
                name="wait_before_capture_secs",
                type=ParameterType.PARAMETER_DOUBLE,
                description="The time to wait between the end of motion and capturing the image.",
                read_only=True,
            ),
        ).value

    def initialize(
        self,
        timeout_secs: float = 10.0,
        rate: Union[Rate, float] = 10.0,
    ) -> bool:
        """
        Do the initialization steps that require rclpy to be spinning.
        """
        # Configuration for the timeout
        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_secs)
        created_rate = False
        if isinstance(rate, float):
            rate = self.create_rate(rate)
            created_rate = True
        def cleanup(retval: bool) -> bool:
            if created_rate: self.destroy_rate(rate)
            return retval

        # Wait for the joint states
        while self.moveit2.joint_state is None:
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the joint states.")
                return cleanup(False)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error("Timed out while gettine the joint states.")
                return cleanup(False)
            rate.sleep()

        # Wait for the RGB image
        latest_rgb_img = None
        while latest_rgb_img is None:
            with self.latest_rgb_img_lock:
                latest_rgb_img = self.latest_rgb_img
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the RGB image.")
                return cleanup(False)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error("Timed out while getting the RGB image.")
                return cleanup(False)
            rate.sleep()

        return cleanup(True)

    def get_remaining_time(self, start_time: Time, timeout_secs: float) -> float:
        """
        Get the remaining time before the timeout.

        Parameters
        ----------
        start_time : Time
            The start time.
        timeout_secs : float
            The timeout in seconds.

        Returns
        -------
        float
            The remaining time in seconds.
        """
        return timeout_secs - (self.get_clock().now() - start_time).nanoseconds / 1e9

    def activate_controller(
        self, 
        controller_name: str = "jaco_arm_cartesian_controller",
        timeout_secs: float = 10.0,
        rate: Union[Rate, float] = 10.0,
    ) -> bool:
        """
        Activate the specified controller and deactivate all others.
        """
        if self.active_controller == controller_name:
            return True

        # Configuration for the timeout
        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_secs)
        created_rate = False
        if isinstance(rate, float):
            rate = self.create_rate(rate)
            created_rate = True
        def cleanup(retval: bool) -> bool:
            if created_rate: self.destroy_rate(rate)
            return retval

        # Activate the controller
        request = SwitchController.Request(
            activate_controllers=[controller_name],
            deactivate_controllers=[
                controller
                for controller in self.all_controllers
                if controller != controller_name
            ],
            activate_asap=True,
            strictness=SwitchController.Request.BEST_EFFORT,
        )
        if not self.switch_controller_client.wait_for_service(
            timeout_sec=get_remaining_time(start_time, timeout_secs),
        ):
            self.get_logger().error("Failed to connect to the switch_controller service.")
            return cleanup(False)
        future = self.switch_controller_client.call_async(request)
        while not future.done():
            if not rclpy.ok():
                self.get_logger().error("Interrupted while activating the controller.")
                return cleanup(False)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error("Timeout while activating the controller.")
                return cleanup(False)
            rate.sleep()
        response = future.result()
        if not response.ok:
            self.get_logger().error("Failed to activate the controller.")
            return cleanup(False)
        
        self.active_controller = controller_name
        return cleanup(True)

    def move_to_configuration(
        self, 
        configuration: List[float],
        timeout_secs: float = 10.0,
        rate: Union[Rate, float] = 10.0,
    ) -> bool:
        """
        Move the robot to the specified configuration.
        """
        # Configuration for the timeout
        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_secs)
        created_rate = False
        if isinstance(rate, float):
            rate = self.create_rate(rate)
            created_rate = True
        def cleanup(retval: bool) -> bool:
            if created_rate: self.destroy_rate(rate)
            return retval

        # Plan the motion to the configuration
        future = self.moveit2.move_to_configuration(
            joint_positions=configuration,
        )
        while not future.done():
            if not rclpy.ok():
                self.get_logger().error("Interrupted while moving to the configuration.")
                return cleanup(False)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error("Timeout while moving to the configuration.")
                return cleanup(False)
            rate.sleep()
        traj = self.moveit2.get_trajectory(future)
        if traj is None:
            self.get_logger().error("Failed to plan to the configuration.")
            return cleanup(False)

        # Execute the motion
        self.moveit2.execute(traj)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            if not rclpy.ok():
                self.get_logger().error("Interrupted while executing the trajectory.")
                return cleanup(False)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error("Timeout while executing the trajectory.")
                return cleanup(False)
            rate.sleep()
        self.active_controller = self.default_moveit2_controller  # MoveIt2 automatically activates this controller
        future = self.moveit2.get_execution_future()
        while not future.done():
            if not rclpy.ok():
                self.get_logger().error("Interrupted while executing the trajectory.")
                return cleanup(False)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error("Timeout while executing the trajectory.")
                return cleanup(False)
            rate.sleep()
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Failed to execute the trajectory.")
            return cleanup(False)
        return cleanup(True)

    def pose_to_twist(
        self,
        pose_stamped: PoseStamped,
        max_linear_speed: float = 0.1, # m/s
        max_angular_speed: float = 0.3, # rad/s
        round_decimals: Optional[int] = 3,
        rate_hz: float = 10.0,
    ) -> TwistStamped:
        """
        Convert a PoseStamped message to a TwistStamped message. Essentially, it
        returns the lienar and angular velocities to execute for 1/rate_hz sec to
        move the pose's frame_id to the pose. Based on
        ada_feeding/ada_feeding/behaviors/ros/msgs.py's PoseStampedToTwistStamped behavior.
        """
        # For the linear velocity, normalize the pose's position and multiply
        # it by the linear_speed
        linear_displacement = ros2_numpy.numpify(pose_stamped.pose.position)
        linear_distance = np.linalg.norm(linear_displacement)
        linear_speed = min(linear_distance * rate_hz, max_linear_speed)
        linear_velocity = linear_displacement / linear_distance * linear_speed

        # Round it
        if round_decimals is not None:
            linear_velocity = np.round(linear_velocity, round_decimals)

        # Convert to a msg
        linear_msg = ros2_numpy.msgify(Vector3, linear_velocity)

        # For the angular velocity, convert the pose's orientation to a
        # rotation vector, normalize it, and multiply it by the angular_speed
        angular_displacement = R.from_quat(
            ros2_numpy.numpify(pose_stamped.pose.orientation)
        ).as_rotvec()
        angular_distance = np.linalg.norm(angular_displacement)
        angular_speed = min(angular_distance * rate_hz, max_angular_speed)
        angular_velocity = angular_displacement / angular_distance * angular_speed

        # Round it
        if round_decimals is not None:
            angular_velocity = np.round(angular_velocity, round_decimals)

        # Convert to a msg
        angular_msg = ros2_numpy.msgify(Vector3, angular_velocity)

        # Create the twist stamped message
        twist_stamped = TwistStamped()
        twist_stamped.header = pose_stamped.header
        twist_stamped.twist.linear = linear_msg
        twist_stamped.twist.angular = angular_msg

        return twist_stamped

    def transform_pose_stamped(
        self,
        pose_stamped: PoseStamped,
        target_frame: str,
        timeout_secs: float = 10.0,
    ) -> Optional[PoseStamped]:
        """
        Transform the pose stamped to the target frame.
        """
        try:
            return self.tf_buffer.transform(
                pose_stamped,
                target_frame,
                timeout=Duration(seconds=timeout_secs),
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
            tf2_ros.TypeException,
        ) as e:
            self.get_logger().error(f"Failed to transform the pose: {e}")
            return None

    def move_end_effector_to_pose_cartesian(
        self,
        pose_stamped: PoseStamped,
        timeout_secs: float = 10.0,
        rate: Union[Rate, float] = 10.0,
        linear_threshold: float = 0.005, # meters
        angular_threshold: float = 0.01, # radians
    ):
        """
        Move the end-effector to the specified pose via Cartesian motion.
        """
        # Configuration for the timeout
        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_secs)
        created_rate = False
        if isinstance(rate, float):
            rate = self.create_rate(rate)
            created_rate = True
        def cleanup(retval: bool) -> bool:
            self.twist_pub.publish(TwistStamped())  # Stop the motion
            if created_rate: self.destroy_rate(rate)
            return retval

        # Activate the controller
        if not self.activate_controller("jaco_arm_cartesian_controller", timeout_secs, rate):
            return cleanup(False)

        # Convert pose to base link frame if it isn't already
        if pose_stamped.header.frame_id != self.moveit2.base_link_name:
            pose_stamped_base = self.transform_pose_stamped(
                pose_stamped,
                self.moveit2.base_link_name,
                get_remaining_time(start_time, timeout_secs),
            )
            if pose_stamped_base is None:
                return cleanup(False)
        else:
            pose_stamped_base = pose_stamped

        # Move towards the pose until it is reached or timeout
        while self.get_clock().now() - start_time < timeout:
            if not rclpy.ok():
                return cleanup(False)

            # Convert the target pose to the end-effector frame
            pose_stamped_ee = self.transform_pose_stamped(
                pose_stamped_base,
                self.moveit2.end_effector_frame,
                get_remaining_time(start_time, timeout_secs),
            )
            if pose_stamped_ee is None:
                return cleanup(False)

            # Check if the pose is reached
            position_diff = np.linalg.norm(
                ros2_numpy.numpify(pose_stamped_ee.pose.position)
            )
            angular_diff = np.linalg.norm(
                R.from_quat(ros2_numpy.numpify(pose_stamped_ee.pose.orientation)).as_rotvec()
            )
            if position_diff < linear_threshold and angular_diff < angular_threshold:
                return cleanup(True)
            
            # Compute the twist required to move the end effector to the target pose
            twist_stamped = self.pose_to_twist(pose_stamped_ee, rate_hz=1.0e9 / rate._timer.timer_period_ns)

            # Publish the twist
            self.twist_pub.publish(twist_stamped)

            rate.sleep()

        self.get_logger().error("Timeout while moving the end-effector to the pose.")
        return cleanup(False)


    def rgb_img_callback(self, msg: CompressedImage):
        """
        Callback for the RGB image.

        Parameters
        ----------
        msg : CompressedImage
            The compressed image message.
        """
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        with self.latest_rgb_img_lock:
            self.latest_rgb_img = img

    def run(
        self,
        rate: Union[Rate, float] = 10.0,
    ):
        """
        Run the node.
        """
        try:
            created_rate = False
            if isinstance(rate, float):
                rate = self.create_rate(rate)
                created_rate = True
            # Wait for the user to place the robot on its tripod mount
            _ = input(
                "Place the robot on its tripod mount. Ensure there is at least 1m of empty space "
                "on all sides of the robot's base. Press Enter when done."
            )

            # Move the robot to the starting configuration
            self.move_to_configuration(
                configuration=self.starting_arm_configuration,
                rate = rate,
            )

            # Wait for the user to place the charucoboard in front of the robot
            _ = input(
                "Place the charucoboard on the hospital table. Roll the hospital table "
                "under the robot's end-effector. Adjust the height of the tripod and hospital "
                "table so the end-effector is as close as possible to the charuboard while "
                "the charucoboard is still fully visible in the camera. Press Enter when done."
            )

            # Capture the poses and images. TODO: Consider adding EE rotation here
            lateral_radius = 0.2  # meters
            lateral_intervals = 5
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.moveit2.end_effector_frame
            target_pose.pose.orientation.w = 1.0
            wait_before_capture = Duration(seconds=self.wait_before_capture_secs)
            for d_z in [0.0, -0.1, -0.2, -0.3, -0.4]:
                for lateral_i in range(lateral_intervals):
                    # Get the target pose
                    theta = 2 * np.pi * lateral_i / lateral_intervals
                    d_x = lateral_radius * np.cos(theta)
                    d_y = lateral_radius * np.sin(theta)
                    target_pose.pose.position.x = d_x
                    target_pose.pose.position.y = d_y
                    target_pose.pose.position.z = d_z
                    target_pose.header.stamp = self.get_clock().now().to_msg()

                    # Move to the target pose
                    self.move_end_effector_to_pose_cartesian(target_pose, rate)

                    # Wait for the joint states to update
                    wait_start_time = self.get_clock().now()
                    while self.get_clock().now() - wait_start_time < wait_before_capture:
                        if not rclpy.ok():
                            return
                        rate.sleep()

                    # Capture the sample
                    joint_state = self.moveit2.joint_state
                    # TODO

            if created_rate:
                self.destroy_rate(rate)
        except KeyboardInterrupt:
            return

    def show_img(self, rate_hz: float = 10.0):
        """
        Show the latest RGB image.
        """
        # Configuration for the timeout
        rate = self.create_rate(rate_hz)
        try:
            while rclpy.ok():
                with self.latest_rgb_img_lock:
                    img = self.latest_rgb_img
                if img is not None:
                    cv2.imshow("RGB Image", img)
                    cv2.waitKey(1000//rate_hz)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        self.destroy_rate(rate)

def spin(node: Node, executor: rclpy.executors.Executor):
    """
    Spin the node in the background.

    Parameters
    ----------
    node : Node
        The node to spin.
    executor : rclpy.executors.Executor
        The executor to spin.
    """
    try:
        rclpy.spin(node, executor)
    except rclpy.executors.ExternalShutdownException:
        pass

def main():
    # Initialize the node
    rclpy.init()
    node = CalibrateCameraNode()

    # Spin in the background, as the node initializes
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    spin_thread = threading.Thread(
        target=spin,
        args=(node,),
        kwargs={"executor": executor},
        daemon=True,
    )
    spin_thread.start()

    # Run the node
    initialized = node.initialize()
    if initialized:
        # Run the node in the background
        run_thread = threading.Thread(
            target=node.run,
            daemon=True,
        )
        run_thread.start()

        # Show the image stream in the main thread
        try:
            node.show_img()
        except KeyboardInterrupt:
            pass
    
    # Cleanly terminate the node
    node.destroy_node()
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass
    print_and_flush("")
    run_thread.join()
    spin_thread.join()
    print_and_flush("Cleanly terminated.")

if __name__ == "__main__":
    main()
