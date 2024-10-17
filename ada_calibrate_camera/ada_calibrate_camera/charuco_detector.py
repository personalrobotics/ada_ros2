"""
This module defines the CharucoDetector class, which is used to detect a charucoboard in an image.
"""

# Standard imports
import copy
from typing import Optional, Tuple

# Third-party imports
import cv2
import numpy as np

# Local imports


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
        # pylint: disable=too-many-arguments
        # This is meant to be a flexible class, hence the many arguments.
        self.camera_matrix = None
        self.dist_coeffs = None
        self.board = cv2.aruco.CharucoBoard_create(
            squaresX=n_cols,
            squaresY=n_rows,
            squareLength=sq_length_m,
            markerLength=marker_length_m,
            dictionary=cv2.aruco.getPredefinedDictionary(predefined_dictionary),
        )
        self.detector_params = cv2.aruco.DetectorParameters_create()

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
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def got_camera_intrinsics(self) -> bool:
        """
        Check if the camera intrinsics have been set.

        Returns
        -------
        bool
            Whether the camera intrinsics have been set.
        """
        return self.camera_matrix is not None

    def detect(
        self, img: np.ndarray, viz: bool = False, verbose: bool = False
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect the charucoboard in the image.

        Parameters
        ----------
        img : np.ndarray
            The image.
        viz : bool
            Whether to visualize the detection.
        verbose : bool
            Whether to print verbose output.

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
        # pylint: disable=too-many-return-statements
        # This is meant to be a flexible function, hence the many return statements.
        if self.camera_matrix is None:
            print("Camera intrinsics not set.", flush=True)
            return False, None, None, None

        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            img,
            self.board.dictionary,
            parameters=self.detector_params,
            cameraMatrix=self.camera_matrix,
            distCoeff=self.dist_coeffs,
        )
        if marker_ids is None or len(marker_ids) == 0:
            if verbose:
                print("Failed to detect the markers.", flush=True)
            return False, None, None, None
        _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            img,
            self.board,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
        )
        if charuco_corners is None or len(charuco_corners) == 0:
            if verbose:
                print("Failed to interpolate the charucoboard corners.", flush=True)
            return False, None, None, None
        if viz:
            retval_img = copy.deepcopy(img)
            retval_img = cv2.aruco.drawDetectedMarkers(
                retval_img, marker_corners, marker_ids, (0, 255, 0)
            )
            retval_img = cv2.aruco.drawDetectedCornersCharuco(
                retval_img, charuco_corners, charuco_ids, (255, 0, 0)
            )
        if len(charuco_ids) >= 4:
            rvec = np.zeros((3, 1))
            tvec = np.zeros((3, 1))
            succ, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners,
                charuco_ids,
                self.board,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
            )
            if not succ:
                if verbose:
                    print("Failed to get charucoboard pose.", flush=True)
                return False, None, None, retval_img
            if viz:
                cv2.drawFrameAxes(
                    retval_img,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.1,
                )
                return True, rvec, tvec, retval_img
            return True, rvec, tvec, None
        if verbose:
            print("Failed to detect the charucoboard.", flush=True)
        return False, None, None, None
