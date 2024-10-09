#!/usr/bin/env python3
"""
This module defines a node that calibrates the camera using a charucoboard.
"""

# Standard imports
import copy
from datetime import datetime
import os
import readline  # pylint: disable=unused-import
import sys
import threading
from typing import List, Optional, Tuple, Union

# Third-party imports
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SwitchController
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped, Vector3
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
from rclpy.time import Time
from rclpy.timer import Rate
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import ros2_numpy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_geometry_msgs import PoseStamped, Vector3Stamped
import tf2_py as tf2
import tf2_ros

# Local imports


# pylint: disable=invalid-name
# Although names like R_gripper2base are not snake case, they align with OpenCV
# conventions.


def print_and_flush(message: str):
    """
    Print a message and flush the output.

    Parameters
    ----------
    message : str
        The message to print.
    """
    print(message, flush=True)
    sys.stdout.flush()


def pose_to_rvec_tvec(pose: PoseStamped) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert a PoseStamped message to a rotation vector and translation vector.

    Parameters
    ----------
    pose : PoseStamped
        The pose.

    Returns
    -------
    np.ndarray
        The rotation vector.
    np.ndarray
        The translation vector.
    """
    rvec = R.from_quat(ros2_numpy.numpify(pose.pose.orientation)).as_rotvec()
    tvec = ros2_numpy.numpify(pose.pose.position)
    return rvec, tvec


class CharucoDetector:
    """
    This class returns the pose of a charucoboard in an image.
    """

    def __init__(
        self,
        n_rows: int,
        n_cols: int,
        sq_length_m: float,
        marker_length_m: float,
        predefined_dictionary: int,
    ):
        """
        Initialize the CharucoDetector.

        Parameters
        ----------
        n_rows : int
            The number of rows in the charucoboard.
        n_cols : int
            The number of columns in the charucoboard.
        sq_length_m : float
            The length of a square in the charucoboard.
        marker_length_m : float
            The length of a marker in the charucoboard.
        predefined_dictionary : int
            The predefined dictionary to use, e.g., cv2.aruco.DICT_4X4_50.
        """
        self.board = cv2.aruco.CharucoBoard_create(
            squaresX=n_cols,
            squaresY=n_rows,
            squareLength=sq_length_m,
            markerLength=marker_length_m,
            dictionary=cv2.aruco.getPredefinedDictionary(predefined_dictionary),
        )
        self.charuco_params = cv2.aruco.CharucoParameters()
        self.charuco_params.minMarkers = 2
        self.charuco_params.tryRefineMarkers = True
        self.detector_params = cv2.aruco.DetectorParameters()
        self.refine_params = cv2.aruco.RefineParameters()
        self.detector = None

    def set_camera_intrinsics(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """
        Set the camera intrinsics.

        Parameters
        ----------
        camera_matrix : np.ndarray
            The camera matrix.
        dist_coeffs : np.ndarray
            The distortion coefficients.
        """
        if self.detector is None:
            self.charuco_params.cameraMatrix = camera_matrix
            self.charuco_params.distCoeffs = dist_coeffs
            self.detector = cv2.aruco.CharucoDetector(
                self.board,
                self.charuco_params,
                self.detector_params,
                self.refine_params,
            )

    def got_camera_intrinsics(self) -> bool:
        """
        Check if the camera intrinsics have been set.

        Returns
        -------
        bool
            Whether the camera intrinsics have been set.
        """
        return self.detector is not None

    def detect(
        self, img: np.ndarray, viz: bool = False
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect the charucoboard in the image.

        Parameters
        ----------
        img : np.ndarray
            The image.
        viz : bool
            Whether to visualize the detection.

        Returns
        -------
        bool
            Whether the charucoboard was detected.
        np.ndarray
            The rotation vector for the charucoboard in the camera's frame.
        np.ndarray
            The translation vector for the charucoboard in the camera's frame.
        np.ndarray
            if viz is True, the image with the charucoboard drawn on it.

        """
        if self.detector is None:
            print_and_flush("Camera intrinsics not set.")
            return False, None, None, None

        (
            charuco_corners,
            charuco_ids,
            marker_corners,
            marker_ids,
        ) = self.detector.detectBoard(img)
        if viz:
            retval_img = copy.deepcopy(img)
            if len(marker_ids) > 0:
                retval_img = cv2.aruco.drawDetectedMarkers(
                    retval_img, marker_corners, marker_ids, (0, 255, 0)
                )
            if len(charuco_ids) >= 0:
                retval_img = cv2.aruco.drawDetectedCornersCharuco(
                    retval_img, charuco_corners, charuco_ids, (255, 0, 0)
                )
        if len(charuco_ids) >= 4:
            obj_points, img_points = self.board.matchImagePoints(
                charuco_corners, charuco_ids
            )
            succ, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self.charuco_params.cameraMatrix,
                self.charuco_params.distCoeffs,
            )
            if not succ:
                print_and_flush("Failed to solve PnP.")
                return False, None, None, retval_img
            if viz:
                cv2.drawFrameAxis(
                    retval_img,
                    self.charuco_params.cameraMatrix,
                    self.charuco_params.distCoeffs,
                    rvec,
                    tvec,
                    0.1,
                )
                return True, rvec, tvec, retval_img
            return True, rvec, tvec, None
        print_and_flush("Failed to detect the charucoboard.")
        return False, None, None, None


class CameraCalibration:
    """
    This class performs camera calibration. It allows users to add samples, which
    consist of an RGB image, a gripper2base transform, and a target2cam transform.
    It saves the data in the specified data directory, and computes the cam2gripper
    transform. It also computes the transformation error.
    """

    def __init__(
        self,
        data_dir: Optional[str] = None,
        method: int = cv2.calib3d.CALIB_HAND_EYE_TSAI,
    ):
        """
        Initialize the CameraCalibration.

        Parameters
        ----------
        data_dir : Optional[str]
            The directory to save the data.
        method : int
            The hand-eye calibration method.
        """
        # Saving data in a folder corresponding to the datetime: YYYY_MM_DD_HH_MM_SS
        self.data_dir = None
        if data_dir is not None:
            folder_name = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            self.data_dir = os.path.join(data_dir, folder_name)
            os.makedirs(self.data_dir, exist_ok=True)
            self.sample_i = 0

        # Hand-eye calibration method
        self.method = method

        # Tracking samples
        self.rgb_images = []
        self.Rs_gripper2base = []
        self.ts_gripper2base = []
        self.Rs_target2cam = []
        self.ts_target2cam = []

    def add_sample(
        self,
        rgb_img: np.ndarray,
        gripper2base: Union[PoseStamped, Tuple[np.ndarray, np.ndarray]],
        target2cam: Union[PoseStamped, Tuple[np.ndarray, np.ndarray]],
        save_data: bool = True,
    ):
        """
        Add a sample to the camera calibration.

        Parameters
        ----------
        rgb_img : np.ndarray
            The RGB image.
        gripper2base : Union[PoseStamped, Tuple[np.ndarray, np.ndarray]]
            The gripper2base transform.
        target2cam : Union[PoseStamped, Tuple[np.ndarray, np.ndarray]]
            The target2cam transform.
        save_data : bool
            Whether to save the data.
        """
        if isinstance(gripper2base, PoseStamped):
            R_gripper2base, t_gripper2base = pose_to_rvec_tvec(gripper2base)
        else:
            R_gripper2base, t_gripper2base = gripper2base
        if isinstance(target2cam, PoseStamped):
            R_target2cam, t_target2cam = pose_to_rvec_tvec(target2cam)
        else:
            R_target2cam, t_target2cam = target2cam
        self.rgb_images.append(rgb_img)
        self.Rs_gripper2base.append(R_gripper2base)
        self.ts_gripper2base.append(t_gripper2base)
        self.Rs_target2cam.append(R_target2cam)
        self.ts_target2cam.append(t_target2cam)

        # Save the data
        if save_data and self.data_dir is not None:
            self.save_sample(
                self.data_dir,
                self.sample_i,
                rgb_img,
                R_gripper2base,
                t_gripper2base,
                R_target2cam,
                t_target2cam,
            )
            self.sample_i += 1

    @staticmethod
    def save_sample(
        data_dir: str,
        sample_i: int,
        rgb_img: np.ndarray,
        R_gripper2base: np.ndarray,
        t_gripper2base: np.ndarray,
        R_target2cam: np.ndarray,
        t_target2cam: np.ndarray,
    ) -> None:
        """
        Save the sample to the data directory.

        Parameters
        ----------
        data_dir : str
            The directory to save the data.
        sample_i : int
            The sample index.
        rgb_img : np.ndarray
            The RGB image.
        R_gripper2base : np.ndarray
            The gripper2base rotation vector.
        t_gripper2base : np.ndarray
            The gripper2base translation vector.
        R_target2cam : np.ndarray
            The target2cam rotation vector.
        t_target2cam : np.ndarray
            The target2cam translation vector.
        """
        cv2.imwrite(os.path.join(data_dir, f"{sample_i}_rgb_img.png"), rgb_img)
        np.savez_compressed(
            os.path.join(data_dir, f"{sample_i}_sample.npz"),
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
        )

    @staticmethod
    def load_sample(
        data_dir: str,
        sample_i: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Load the sample from the data directory.

        Parameters
        ----------
        data_dir : str
            The directory to save the data.
        sample_i : int
            The sample index.

        Returns
        -------
        np.ndarray
            The RGB image.
        np.ndarray
            The gripper2base rotation vector.
        np.ndarray
            The gripper2base translation vector.
        np.ndarray
            The target2cam rotation vector.
        np.ndarray
            The target2cam translation vector.
        """
        rgb_img = cv2.imread(os.path.join(data_dir, f"{sample_i}_rgb_img.png"))
        data = np.load(os.path.join(data_dir, f"{sample_i}_sample.npz"))
        return (
            rgb_img,
            data["R_gripper2base"],
            data["t_gripper2base"],
            data["R_target2cam"],
            data["t_target2cam"],
        )

    def compute_calibration(
        self, save_data: bool
    ) -> Tuple[np.ndarray, np.ndarray, float, float]:
        """
        Compute the camera calibration and return the rotation and translation
        errors. For the rotation and translation errors, we compute the pose of the
        target in the base frame across all samples. In theory, this should be the
        same across all samples. We then take the pairwise translation and rotation
        distance between the samples, and average them.

        References:
        - Eq 27A and 28A here: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9581431/pdf/pone.0273261.pdf
        - The derivation of AX=XB for eye-in-hand calibration from:
          https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b

        Parameters
        ----------
        save_data : bool
            Whether to save the data.

        Returns
        -------
        np.ndarray
            The rotation matrix from the camera to the gripper.
        np.ndarray
            The translation vector from the camera to the gripper.
        float
            The rotation error.
        float
            The translation error.
        """
        # Compute the camera extrinsics calibration
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            self.Rs_gripper2base,
            self.ts_gripper2base,
            self.Rs_target2cam,
            self.ts_target2cam,
            method=self.method,
        )

        # Convert to a homogenous transform
        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper

        # Compute the transformation error
        Rs_target2base = []
        ts_target2base = []
        for i in range(
            len(self.Rs_target2cam)
        ):  # pylint: disable=consider-using-enumerate
            # Get the homogenous transform from the gripper to the base
            T_gripper2base = np.eye(4)
            T_gripper2base[:3, :3] = R.from_rotvec(self.Rs_gripper2base[i]).as_matrix()
            T_gripper2base[:3, 3] = self.ts_gripper2base[i]

            # Get the homogenous transform from the target to the camera
            T_target2cam = np.eye(4)
            T_target2cam[:3, :3] = R.from_rotvec(self.Rs_target2cam[i]).as_matrix()
            T_target2cam[:3, 3] = self.ts_target2cam[i]

            # Compute the homogenous transform from the target to the base
            T_target2base = T_target2cam @ T_cam2gripper @ T_gripper2base

            # Extract the rotation and translation
            Rs_target2base.append(R.from_matrix(T_target2base[:3, :3]))
            ts_target2base.append(T_target2base[:3, 3])

        # Compute the translation and rotation errors
        translation_errors = []
        rotation_errors = []
        for i in range(len(Rs_target2base)):
            for j in range(i + 1, len(Rs_target2base)):
                translation_errors.append(
                    np.linalg.norm(ts_target2base[i] - ts_target2base[j])
                )
                rotation_errors.append(
                    (Rs_target2base[i].inv() * Rs_target2base[j]).magnitude()
                )

        # Average the errors
        translation_error = np.mean(translation_errors)
        rotation_error = np.mean(rotation_errors)

        # Save the calibration
        if save_data and self.data_dir is not None:
            np.savez_compressed(
                os.path.join(self.data_dir, f"{self.sample_i}_calib.npz"),
                R_cam2gripper=R_cam2gripper,
                t_cam2gripper=t_cam2gripper,
            )

        return R_cam2gripper, t_cam2gripper, rotation_error, translation_error


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

    # pylint: disable=too-many-instance-attributes

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
        self.twist_pub = self.create_publisher(
            TwistStamped,
            "/jaco_arm_cartesian_controller/twist_cmd",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Create the MoveIt2 object
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=kinova.joint_names(),
            base_link_name=kinova.base_link_name(),
            end_effector_name=kinova.end_effector_name(),
            group_name="jaco_arm",
            callback_group=callback_group,
        )

        # Create the charuco detector
        self.charuco_detector = CharucoDetector(
            n_rows=self.charuco_n_rows,
            n_cols=self.charuco_n_cols,
            sq_length_m=self.charuco_sq_length_m,
            marker_length_m=self.charuco_marker_length_m,
            predefined_dictionary=self.charuco_predefined_dictionary,
        )

        # Subscribe to the camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/local/camera/aligned_depth_to_color/camera_info",
            self.camera_info_callback,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Subscribe to the RGB camera feed
        self.bridge = CvBridge()
        self.latest_img_lock = threading.Lock()
        self.latest_annotated_img = None
        self.latest_raw_img = None
        self.latest_charuco_rvec = None
        self.latest_charuco_tvec = None
        self.rgb_img_sub = self.create_subscription(
            CompressedImage,
            "/local/camera/color/image_raw/compressed",
            self.rgb_img_callback,
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Create the service to (de)activate the controller
        self.switch_controller_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
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
                -2.3149168248766614,  # j2n6s200_joint_1
                3.1444595465032634,  # j2n6s200_joint_2
                1.7332586075115999,  # j2n6s200_joint_3
                -2.3609596843308234,  # j2n6s200_joint_4
                4.43936623280362,  # j2n6s200_joint_5
                3.06866544924739,  # j2n6s200_joint_6
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
            10.0,
            ParameterDescriptor(
                name="wait_before_capture_secs",
                type=ParameterType.PARAMETER_DOUBLE,
                description="The time to wait between the end of motion and capturing the image.",
                read_only=True,
            ),
        ).value

        # Charuco board parameters
        self.charuco_n_rows = self.declare_parameter(
            "charuco_n_rows",
            6,
            ParameterDescriptor(
                name="charuco_n_rows",
                type=ParameterType.PARAMETER_INTEGER,
                description="The number of rows in the charucoboard.",
                read_only=True,
            ),
        ).value
        self.charuco_n_cols = self.declare_parameter(
            "charuco_n_cols",
            8,
            ParameterDescriptor(
                name="charuco_n_cols",
                type=ParameterType.PARAMETER_INTEGER,
                description="The number of columns in the charucoboard.",
                read_only=True,
            ),
        ).value
        self.charuco_sq_length_m = self.declare_parameter(
            "charuco_sq_length_m",
            0.025,
            ParameterDescriptor(
                name="charuco_sq_length_m",
                type=ParameterType.PARAMETER_DOUBLE,
                description="The length of a square in the charucoboard.",
                read_only=True,
            ),
        ).value
        self.charuco_marker_length_m = self.declare_parameter(
            "charuco_marker_length_m",
            0.015,
            ParameterDescriptor(
                name="charuco_marker_length_m",
                type=ParameterType.PARAMETER_DOUBLE,
                description="The length of a marker in the charucoboard.",
                read_only=True,
            ),
        ).value
        self.charuco_predefined_dictionary = self.declare_parameter(
            "charuco_predefined_dictionary",
            cv2.aruco.DICT_4X4_50,
            ParameterDescriptor(
                name="charuco_predefined_dictionary",
                type=ParameterType.PARAMETER_INTEGER,
                description=(
                    "The predefined dictionary to use. See cv::aruco::PredefinedDictionaryType for the enum: "
                    "https://docs.opencv.org/4.x/de/d67/group__objdetect__aruco.html#ga4e13135a118f497c6172311d601ce00d"
                ),
                read_only=True,
            ),
        ).value

        # Hand-eye calibration method
        self.hand_eye_calibration_method = self.declare_parameter(
            "hand_eye_calibration_method",
            cv2.calib3d.CALIB_HAND_EYE_TSAI,
            ParameterDescriptor(
                name="hand_eye_calibration_method",
                type=ParameterType.PARAMETER_INTEGER,
                description=(
                    "The hand-eye calibration method to use. See cv::calib3d::HandEyeCalibrationMethod for the enum: "
                    "https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gad10a5ef12ee3499a0774c7904a801b99"
                ),
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
            if created_rate:
                self.destroy_rate(rate)
            return retval

        # Wait for the joint states
        while self.moveit2.joint_state is None:
            if not rclpy.ok():
                self.get_logger().error(
                    "Interrupted while waiting for the joint states."
                )
                return cleanup(False)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error("Timed out while gettine the joint states.")
                return cleanup(False)
            rate.sleep()

        # Wait for the RGB image
        latest_raw_img = None
        while latest_raw_img is None:
            with self.latest_img_lock:
                latest_raw_img = self.latest_raw_img
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
            if created_rate:
                self.destroy_rate(rate)
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
            timeout_sec=self.get_remaining_time(start_time, timeout_secs),
        ):
            self.get_logger().error(
                "Failed to connect to the switch_controller service."
            )
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
        # pylint: disable=too-many-return-statements
        # Okay due to extensive error checking

        # Configuration for the timeout
        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_secs)
        created_rate = False
        if isinstance(rate, float):
            rate = self.create_rate(rate)
            created_rate = True

        def cleanup(retval: bool) -> bool:
            if created_rate:
                self.destroy_rate(rate)
            return retval

        # Plan the motion to the configuration
        future = self.moveit2.plan_async(
            joint_positions=configuration,
        )
        while not future.done():
            if not rclpy.ok():
                self.get_logger().error(
                    "Interrupted while moving to the configuration."
                )
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
        self.active_controller = (
            self.default_moveit2_controller
        )  # MoveIt2 automatically activates this controller
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
        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            or result.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            self.get_logger().error("Failed to execute the trajectory.")
            return cleanup(False)
        return cleanup(True)

    def pose_to_twist(
        self,
        pose_stamped: PoseStamped,
        max_linear_speed: float = 0.1,  # m/s
        max_angular_speed: float = 0.3,  # rad/s
        round_decimals: Optional[int] = 3,
        rate_hz: float = 10.0,
    ) -> TwistStamped:
        """
        Convert a PoseStamped message to a TwistStamped message. Essentially, it
        returns the lienar and angular velocities to execute for 1/rate_hz sec to
        move the pose's frame_id to the pose. Based on
        ada_feeding/ada_feeding/behaviors/ros/msgs.py's PoseStampedToTwistStamped behavior.
        """
        # pylint: disable=too-many-locals, too-many-arguments
        # Okay to provide considerable flexibility.

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
        if angular_distance == 0.0:
            angular_velocity = np.zeros(3)
        else:
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

    def transform_stamped_msg(
        self,
        stamped_msg: Union[PoseStamped, TwistStamped],
        target_frame: str,
        timeout_secs: float = 10.0,
    ) -> Optional[Union[PoseStamped, TwistStamped]]:
        """
        Transform the pose stamped to the target frame.
        """
        stamped_msgs = []
        if isinstance(stamped_msg, PoseStamped):
            stamped_msgs.append(stamped_msg)
        else:
            stamped_msgs.append(
                Vector3Stamped(
                    header=stamped_msg.header, vector=stamped_msg.twist.linear
                )
            )
            stamped_msgs.append(
                Vector3Stamped(
                    header=stamped_msg.header, vector=stamped_msg.twist.angular
                )
            )
        transformed_msgs = []
        for msg in stamped_msgs:
            try:
                transformed_msg = self.tf_buffer.transform(
                    msg,
                    target_frame,
                    timeout=Duration(seconds=timeout_secs),
                )
                transformed_msgs.append(transformed_msg)
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
        if isinstance(stamped_msg, PoseStamped):
            return transformed_msgs[0]
        transformed_twist = TwistStamped()
        transformed_twist.header = transformed_msgs[0].header
        transformed_twist.twist.linear = transformed_msgs[0].vector
        transformed_twist.twist.angular = transformed_msgs[1].vector
        return transformed_twist

    def move_end_effector_to_pose_cartesian(
        self,
        pose_stamped: PoseStamped,
        timeout_secs: float = 10.0,
        rate: Union[Rate, float] = 10.0,
        linear_threshold: float = 0.005,  # meters
        angular_threshold: float = 0.01,  # radians
        linear_override: Optional[Vector3] = None,
        angular_override: Optional[Vector3] = None,
    ):
        """
        Move the end-effector to the specified pose via Cartesian motion.
        """
        # pylint: disable=too-many-arguments

        # Configuration for the timeout
        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_secs)
        created_rate = False
        if isinstance(rate, float):
            rate = self.create_rate(rate)
            created_rate = True

        def cleanup(retval: bool) -> bool:
            zero_twist = TwistStamped()
            zero_twist.header.frame_id = self.moveit2.base_link_name
            self.twist_pub.publish(zero_twist)  # Stop the motion
            if created_rate:
                self.destroy_rate(rate)
            return retval

        # Activate the controller
        if not self.activate_controller(
            "jaco_arm_cartesian_controller", timeout_secs, rate
        ):
            return cleanup(False)

        # Convert pose to base link frame if it isn't already
        if pose_stamped.header.frame_id != self.moveit2.base_link_name:
            pose_stamped_base = self.transform_stamped_msg(
                pose_stamped,
                self.moveit2.base_link_name,
                self.get_remaining_time(start_time, timeout_secs),
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
            pose_stamped_ee = self.transform_stamped_msg(
                pose_stamped_base,
                self.moveit2.end_effector_name,
                self.get_remaining_time(start_time, timeout_secs),
            )
            if pose_stamped_ee is None:
                return cleanup(False)

            # Check if the pose is reached
            position_diff = np.linalg.norm(
                ros2_numpy.numpify(pose_stamped_ee.pose.position)
            )
            angular_diff = np.linalg.norm(
                R.from_quat(
                    ros2_numpy.numpify(pose_stamped_ee.pose.orientation)
                ).as_rotvec()
            )
            if position_diff < linear_threshold and angular_diff < angular_threshold:
                return cleanup(True)

            # Compute the twist required to move the end effector to the target pose
            twist_stamped = self.pose_to_twist(
                pose_stamped_ee,
                rate_hz=1.0e9
                / rate._timer.timer_period_ns,  # pylint: disable=protected-access
            )
            if linear_override is not None:
                twist_stamped.twist.linear = linear_override
            if angular_override is not None:
                twist_stamped.twist.angular = angular_override

            # Transform to the base frame
            twist_stamped_base = self.transform_stamped_msg(
                twist_stamped,
                self.moveit2.base_link_name,
                self.get_remaining_time(start_time, timeout_secs),
            )
            if twist_stamped_base is None:
                return cleanup(False)

            # Publish the twist
            self.twist_pub.publish(twist_stamped_base)

            rate.sleep()

        self.get_logger().error("Timeout while moving the end-effector to the pose.")
        return cleanup(False)

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for the camera info.

        Parameters
        ----------
        msg : CameraInfo
            The camera info message.
        """
        if not self.charuco_detector.got_camera_intrinsics():
            self.charuco_detector.set_camera_intrinsics(
                np.array(msg.K).reshape((3, 3)),
                np.array(msg.D),
            )
            self.destroy_subscription(self.camera_info_sub)

    def rgb_img_callback(self, msg: CompressedImage):
        """
        Callback for the RGB image.

        Parameters
        ----------
        msg : CompressedImage
            The compressed image message.
        """
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        succ, rvec, tvec, annotated_img = self.charuco_detector.detect(img, viz=True)
        if succ:
            with self.latest_img_lock:
                self.latest_annotated_img = annotated_img
                self.latest_raw_img = img
                self.latest_charuco_rvec = rvec
                self.latest_charuco_tvec = tvec
        else:
            with self.latest_img_lock:
                self.latest_annotated_img = img
                self.latest_raw_img = img
                self.latest_charuco_rvec = None
                self.latest_charuco_tvec = None

    def get_input(self, prompt: str) -> str:
        """
        Get input from the user.

        Parameters
        ----------
        prompt : str
            The prompt to display.

        Returns
        -------
        str
            The input from the user.
        """
        print_and_flush(prompt)
        return input()

    def run(
        self,
        rate: Union[Rate, float] = 10.0,
    ):
        """
        Run the node.
        """
        # pylint: disable=too-many-locals
        # Fine since this does the main work.

        try:
            created_rate = False
            if isinstance(rate, float):
                rate = self.create_rate(rate)
                created_rate = True
            # Wait for the user to place the robot on its tripod mount
            _ = self.get_input(
                "Place the robot on its tripod mount. Ensure there is at least 1m of empty space "
                "on all sides of the robot's base. Press Enter when done."
            )
            print_and_flush("Moving the robot...")

            # Move the robot to the starting configuration
            self.move_to_configuration(
                configuration=self.starting_arm_configuration,
                rate=rate,
            )

            # Wait for the user to place the charucoboard in front of the robot
            _ = self.get_input(
                "Place the charucoboard on the hospital table. Roll the hospital table "
                "under the robot's end-effector. Adjust the height of the tripod and hospital "
                "table so the end-effector is as close as possible to the charuboard while "
                "the charucoboard is still fully visible in the camera. Press Enter when done."
            )

            # Get the current end-effector pose in base frame
            zero_pose = PoseStamped()
            zero_pose.header.frame_id = self.moveit2.end_effector_name
            zero_pose.pose.orientation.w = 1.0
            init_ee_pose = self.transform_stamped_msg(
                copy.deepcopy(zero_pose),
                self.moveit2.base_link_name,
            )
            if init_ee_pose is None:
                return
            print_and_flush(f"Initial end-effector pose: {init_ee_pose}")

            # Capture the poses and images. TODO: Consider adding EE rotation here
            camera_calibration = CameraCalibration()  # TODO: add a data dir!
            lateral_radius = 0.15  # meters
            lateral_intervals = 4
            wait_before_capture = Duration(seconds=self.wait_before_capture_secs)
            for d_z in [0.0, 0.1, 0.2, 0.3, 0.4]:
                for lateral_i in range(-1, lateral_intervals):
                    # Get the target pose
                    if lateral_i == -1:
                        d_x = 0.0
                        d_y = 0.0
                    else:
                        theta = 2 * np.pi * lateral_i / lateral_intervals
                        d_x = lateral_radius * np.cos(theta)
                        d_y = lateral_radius * np.sin(theta)
                    target_pose = copy.deepcopy(init_ee_pose)
                    target_pose.pose.position.x += d_x
                    target_pose.pose.position.y += d_y
                    target_pose.pose.position.z += d_z
                    target_pose.header.stamp = self.get_clock().now().to_msg()

                    # Move to the target pose
                    print_and_flush(f"Moving to the target pose: {target_pose}")
                    self.move_end_effector_to_pose_cartesian(
                        target_pose,
                        rate=rate,
                        angular_override=Vector3(),
                    )

                    # Wait for the joint states to update
                    print_and_flush("Waiting...")
                    wait_start_time = self.get_clock().now()
                    while (
                        self.get_clock().now() - wait_start_time < wait_before_capture
                    ):
                        if not rclpy.ok():
                            return
                        rate.sleep()

                    # Capture the transform from the end effector to the base link
                    ee_pose_in_base_frame = self.transform_stamped_msg(
                        copy.deepcopy(zero_pose),
                        self.moveit2.base_link_name,
                    )
                    if ee_pose_in_base_frame is None:
                        self.get_logger().error(
                            "Failed to capture the transform. Skipping."
                        )
                        continue

                    # Capture the transform from the camera to the charucoboard
                    with self.latest_img_lock:
                        latest_raw_img = self.latest_raw_img
                        charuco_rvec = self.latest_charuco_rvec
                        charuco_tvec = self.latest_charuco_tvec
                    if charuco_rvec is None or charuco_tvec is None:
                        self.get_logger().error(
                            "Failed to capture the transform. Skipping."
                        )
                        continue

                    # Save the transforms
                    camera_calibration.add_sample(
                        latest_raw_img,
                        ee_pose_in_base_frame,
                        (charuco_rvec, charuco_tvec),
                    )

                # Compute the camera extrinsics calibration
                (
                    R_cam2gripper,
                    T_cam2gripper,
                    rotation_error,
                    translation_error,
                ) = camera_calibration.compute_calibration(
                    save_data=True,
                )
                print_and_flush(f"Rotation error: {rotation_error}")
                print_and_flush(f"Translation error: {translation_error}")
                print_and_flush(
                    f"R_cam2gripper: {R.from_matrix(R_cam2gripper).as_quat()}"
                )
                print_and_flush(f"T_cam2gripper: {T_cam2gripper}")

            if created_rate:
                self.destroy_rate(rate)
        except KeyboardInterrupt:
            zero_twist = TwistStamped()
            zero_twist.header.frame_id = self.moveit2.base_link_name
            self.twist_pub.publish(zero_twist)
            return

    def show_img(self, rate_hz: float = 10.0):
        """
        Show the latest RGB image.
        """
        # Configuration for the timeout
        rate = self.create_rate(rate_hz)
        try:
            cv2.namedWindow("RGB Image", cv2.WINDOW_NORMAL)
            while rclpy.ok():
                with self.latest_img_lock:
                    img = self.latest_annotated_img
                if img is not None:
                    cv2.imshow("RGB Image", img)
                    cv2.waitKey(1000 // rate_hz)
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
    """
    Initialize and execute the node.
    """
    # Initialize the node
    rclpy.init()
    node = CalibrateCameraNode()
    print_and_flush("Node created.")

    # Spin in the background, as the node initializes
    executor = MultiThreadedExecutor(num_threads=4)
    spin_thread = threading.Thread(
        target=spin,
        args=(node,),
        kwargs={"executor": executor},
        daemon=True,
    )
    spin_thread.start()

    # Run the node
    print_and_flush("Initializing node")
    initialized = node.initialize()
    show_img_thread = None
    if initialized:
        print_and_flush("Node initialized")
        # Show the image stream in the background
        show_img_thread = threading.Thread(
            target=node.show_img,
            daemon=True,
        )
        show_img_thread.start()

        # Run the node in the main thread
        try:
            node.run()
        except KeyboardInterrupt:
            pass

    # Cleanly terminate the node
    print_and_flush("Terminating node")
    node.destroy_node()
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:  # pylint: disable=protected-access
        pass
    print_and_flush("")
    if show_img_thread:
        show_img_thread.join()
    spin_thread.join()
    print_and_flush("Cleanly terminated.")


if __name__ == "__main__":
    main()
