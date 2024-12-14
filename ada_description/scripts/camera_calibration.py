#!/usr/bin/env python3
# Copyright (c) 2024, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

# Adapted from:
# https://github.com/crigroup/handeye
"""
TODO: Module
"""

# Standard imports
import select
import sys
import termios
import threading
import tty

# Third-party imports
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Transform,
    TransformStamped,
)
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import ros2_numpy
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf2_py as tf2


class CameraCalibrator(Node):
    """
    ROS Node for camera calibration
    TODO: Document
    """

    def __init__(self):
        """
        Define node params
        """

        super().__init__("camera_calib")
        self._declare_parameters()

        # Add subscribers to RealSense's compressed color image and aligned depth
        # image topics
        self.latest_image = None
        self.latest_image_lock = threading.Lock()
        self.latest_image_sub = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.image_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.camera_info = None
        self.camera_info_lock = threading.Lock()
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.info_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        cv2.namedWindow("camera_monitor")
        cv2.namedWindow("3d_pose")

        # Create a CvBridge to convert ROS messages to OpenCV images
        self.bridge = CvBridge()

        # Add TF Subscriber
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.get_logger().info("Initialized CameraCalibrator")
        self.static_transform_broadcaster = StaticTransformBroadcaster(self)

        # Transformation Matrix Cache
        self.chess_to_cam_rot = []
        self.chess_to_cam_tran = []
        self.ee_to_base_rot = []
        self.ee_to_base_tran = []

    def _declare_parameters(self):
        """
        Declare ROS2 params
        """
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "chessboard_square_size_m",
                    0.022,
                    ParameterDescriptor(
                        name="chessboard_square_size_m",
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Size of chessboard square in m",
                        read_only=True,
                    ),
                ),
                (
                    "chessboard_dim",
                    [9, 4],
                    ParameterDescriptor(
                        name="chessboard_dim",
                        type=ParameterType.PARAMETER_INTEGER_ARRAY,
                        description="Number of *corners* in the chessboard",
                        read_only=True,
                    ),
                ),
                (
                    "chess_frame",
                    "chessboard",
                    ParameterDescriptor(
                        name="chess_frame",
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of chessboard reference frame",
                        read_only=True,
                    ),
                ),
                (
                    "camera_frame",
                    "camera_color_optical_frame",
                    ParameterDescriptor(
                        name="camera_frame",
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of camera reference frame",
                        read_only=True,
                    ),
                ),
                (
                    "base_frame",
                    "root",
                    ParameterDescriptor(
                        name="camera_frame",
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of a robot frame fixed relative to chessboard (e.g. root frame)",
                        read_only=True,
                    ),
                ),
                (
                    "ee_frame",
                    "j2n6s200_link_6",
                    ParameterDescriptor(
                        name="camera_frame",
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of a robot frame fixed relative to camera (e.g. end effector)",
                        read_only=True,
                    ),
                ),
            ],
        )

    def info_callback(self, msg: CameraInfo) -> None:
        """
        Callback function for the RealSense's color camera_info topic.
        """
        with self.camera_info_lock:
            self.camera_info = msg

    def image_callback(self, msg: CompressedImage) -> None:
        """
        Callback function for the RealSense's color image topic.
        """
        with self.latest_image_lock:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            cv2.imshow("camera_monitor", self.latest_image)
        cv2.waitKey(1)

    def del_last_image(self):
        """
        Remove last image
        """
        self.get_logger().info("Deleting Last Transform")
        if len(self.chess_to_cam_rot) > 0:
            self.chess_to_cam_rot.pop()
            self.chess_to_cam_tran.pop()
            self.ee_to_base_rot.pop()
            self.ee_to_base_tran.pop()

    def add_image(self):
        """
        Collect the current image transform
        """
        self.get_logger().info("Saving Calibration Transform")
        with self.latest_image_lock:
            image = np.copy(self.latest_image)
        with self.camera_info_lock:
            camera_matrix = np.array(self.camera_info.k).reshape((3, 3))
            distortion = np.array(self.camera_info.d)

        # Save EE to Base Transform
        try:
            ee_transform_msg = self.buffer.lookup_transform(
                self.get_parameter("base_frame").value,
                self.get_parameter("ee_frame").value,
                self.get_clock().now(),
                Duration(seconds=1.0),
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.get_logger().warning(f"Could not get EE transform. Error: {error}")
            return

        ee_transform_mat = ros2_numpy.numpify(ee_transform_msg.transform)

        # Find Chessboard Corners
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        chessboard_dim = tuple(self.get_parameter("chessboard_dim").value)
        found, corners = cv2.findChessboardCorners(gray, chessboard_dim, None)
        if not found:
            self.get_logger().warning(
                f"Could not find chessboard ({chessboard_dim}) in image."
            )
            return

        # Subpixel adjustment
        # Args from: https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        cv2.drawChessboardCorners(image, chessboard_dim, corners, found)

        # PNP to get location
        chessboard_square_size_m = self.get_parameter("chessboard_square_size_m").value
        object_points = np.zeros((chessboard_dim[0] * chessboard_dim[1], 3))
        object_points[:, :2] = (
            np.mgrid[0 : chessboard_dim[0], 0 : chessboard_dim[1]].T.reshape(-1, 2)
            * chessboard_square_size_m
        )
        found, rodrigues, translation, _ = cv2.solvePnPRansac(
            object_points, corners, camera_matrix, distortion
        )
        if not found:
            self.get_logger().warning(f"Could not solvePnPRansac()")
        else:
            # Draw axis at chessboard origin for debugging
            axis_points = np.eye(3).reshape(-1, 3) * 3.0 * chessboard_square_size_m
            axis_image_points, _ = cv2.projectPoints(
                axis_points, rodrigues, translation, camera_matrix, distortion
            )
            origin = tuple(corners[0].ravel().astype(int))
            image = cv2.line(
                image,
                origin,
                tuple(axis_image_points[0].ravel().astype(int)),
                (0, 0, 255),  # BGR (Red == x)
                5,
            )
            image = cv2.line(
                image,
                origin,
                tuple(axis_image_points[1].ravel().astype(int)),
                (0, 255, 0),  # BGR (Green == y)
                5,
            )
            image = cv2.line(
                image,
                origin,
                tuple(axis_image_points[2].ravel().astype(int)),
                (255, 0, 0),  # BGR (Blue == z)
                5,
            )

            # Create Transformation Matrix
            chess_camera_trans_mat = np.eye(4)
            chess_camera_trans_mat[:3, :3], _ = cv2.Rodrigues(rodrigues)
            chess_camera_trans_mat[:3, 3] = translation.flatten()
            chess_camera_trans_stamped = TransformStamped()
            chess_camera_trans_stamped.transform = ros2_numpy.msgify(
                Transform, chess_camera_trans_mat
            )
            chess_camera_trans_stamped.header.stamp = self.get_clock().now().to_msg()
            chess_camera_trans_stamped.header.frame_id = self.get_parameter(
                "camera_frame"
            ).value
            chess_camera_trans_stamped.child_frame_id = self.get_parameter(
                "chess_frame"
            ).value
            self.static_transform_broadcaster.sendTransform(
                [chess_camera_trans_stamped]
            )

            # Cache Transforms
            self.chess_to_cam_rot.append(np.copy(chess_camera_trans_mat[:3, :3]))
            self.chess_to_cam_tran.append(np.copy(chess_camera_trans_mat[:3, 3:]))
            self.ee_to_base_rot.append(np.copy(ee_transform_mat[:3, :3]))
            self.ee_to_base_tran.append(np.copy(ee_transform_mat[:3, 3:]))

        cv2.imshow("3d_pose", image)
        cv2.waitKey(1)

    def solve_calib(self):
        """
        Compute Transformation Matrix
        Publish as static TF (EE -> camera)
        Publish chessboard_calib frame relative to world frame to test
        """

        if len(self.chess_to_cam_rot) < 3:
            self.get_logger().warning(
                f"Need >=3 samples, currently have {len(self.chess_to_cam_rot)}"
            )
            return

        self.get_logger().info("Solving AX=XB")

        camera_to_ee_mat = np.eye(4)
        camera_to_ee_mat[:3, :3], camera_to_ee_mat[:3, 3:] = cv2.calibrateHandEye(
            self.ee_to_base_rot,
            self.ee_to_base_tran,
            self.chess_to_cam_rot,
            self.chess_to_cam_tran,
        )

        self.get_logger().info(f"Solution: {camera_to_ee_mat}")

        camera_to_ee_stamped = TransformStamped()
        camera_to_ee_stamped.transform = ros2_numpy.msgify(Transform, camera_to_ee_mat)
        camera_to_ee_stamped.header.stamp = self.get_clock().now().to_msg()
        camera_to_ee_stamped.header.frame_id = self.get_parameter("ee_frame").value
        camera_to_ee_stamped.child_frame_id = self.get_parameter("camera_frame").value
        self.static_transform_broadcaster.sendTransform([camera_to_ee_stamped])

        # Publish debug frame
        try:
            chess_to_base_trans = self.buffer.lookup_transform(
                self.get_parameter("base_frame").value,
                self.get_parameter("chess_frame").value,
                self.get_clock().now(),
                Duration(seconds=1.0),
            )
        except (
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
            tf2.InvalidArgumentException,
            tf2.LookupException,
            tf2.TimeoutException,
            tf2.TransformException,
        ) as error:
            self.get_logger().warning(f"Could not get chess transform. Error: {error}")
            return

        chess_to_base_stamped = TransformStamped()
        chess_to_base_stamped.transform = chess_to_base_trans.transform
        chess_to_base_stamped.header.stamp = self.get_clock().now().to_msg()
        chess_to_base_stamped.header.frame_id = self.get_parameter("base_frame").value
        chess_to_base_stamped.child_frame_id = (
            self.get_parameter("chess_frame").value + "_test"
        )
        self.static_transform_broadcaster.sendTransform(
            [camera_to_ee_stamped, chess_to_base_stamped]
        )


def get_key(settings):
    """
    Read a key from stdin without writing it to terminal.
    """

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_help(node):
    """
    Print help message
    """

    node.get_logger().info(
        """
Camera Calibration Routine
---------------------------
h - Print Help
i - Print CameraInfo
Space - Take Image
Enter - Run Calibration
q - Quit
"""
    )


def main(args=None):
    """
    Run camera calibration
    """

    rclpy.init(args=args)

    camera_calibrator = CameraCalibrator()
    executor = MultiThreadedExecutor()

    # Spin in the background
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(camera_calibrator,),
        kwargs={"executor": executor},
        daemon=True,
    )
    spin_thread.start()

    print_help(camera_calibrator)
    while rclpy.ok():
        key = get_key(termios.tcgetattr(sys.stdin))
        if key == "h":
            print_help(camera_calibrator)
        elif key == "i":
            camera_calibrator.get_logger().info(str(camera_calibrator.camera_info))
        elif key == " ":
            camera_calibrator.add_image()
        elif key == "d":
            camera_calibrator.del_last_image()
        elif key == "\r" or key == "\n":
            camera_calibrator.solve_calib()
        elif key == "q":
            break

    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
