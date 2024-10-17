"""
This module defines the CameraCalibration class, which allows users to add samples
consisting of an RGB image, a gripper2base transform, and a target2cam transform.
It saves the data in the specified data directory, and computes the cam2gripper
transform. It also computes the transformation error.
"""

# Standard imports
from datetime import datetime
import os
from typing import List, Optional, Tuple, Union

# Third-party imports
import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

# Local imports
from ada_calibrate_camera.helpers import pose_to_rot_trans

# pylint: disable=invalid-name
# Although names like R_gripper2base are not snake case, they align with OpenCV
# conventions.


class CameraCalibration:
    """
    This class performs camera calibration. It allows users to add samples, which
    consist of an RGB image, a gripper2base transform, and a target2cam transform.
    It saves the data in the specified data directory, and computes the cam2gripper
    transform. It also computes the transformation error.
    """

    # pylint: disable=too-many-instance-attributes
    # One over is okay.

    # pylint: disable=dangerous-default-value
    def __init__(
        self,
        data_dir: Optional[str] = None,
        methods: List[int] = [cv2.CALIB_HAND_EYE_ANDREFF],
    ):
        """
        Initialize the CameraCalibration.

        Parameters
        ----------
        data_dir : Optional[str]
            The directory to save the data.
        methods : List[int]
            The hand-eye calibration methods.
        """
        # Saving data in a folder corresponding to the datetime: YYYY_MM_DD_HH_MM_SS
        self.data_dir = None
        if data_dir is not None:
            folder_name = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            self.data_dir = os.path.join(data_dir, folder_name)
            print(f"Saving data to: {self.data_dir}", flush=True)
            os.makedirs(self.data_dir, exist_ok=True)
            self.sample_i = 0

        # Hand-eye calibration method
        self.methods = methods

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
            R_gripper2base, t_gripper2base = pose_to_rot_trans(
                gripper2base, rot_vec=False
            )
        else:
            R_gripper2base, t_gripper2base = gripper2base
        if np.prod(R_gripper2base.shape) == 3:
            R_gripper2base = R_gripper2base.reshape((3,))
        if np.prod(t_gripper2base.shape) == 3:
            t_gripper2base = t_gripper2base.reshape((3,))
        if isinstance(target2cam, PoseStamped):
            R_target2cam, t_target2cam = pose_to_rot_trans(target2cam, rot_vec=False)
        else:
            R_target2cam, t_target2cam = target2cam
        if np.prod(R_target2cam.shape) == 3:
            R_target2cam = R_target2cam.reshape((3,))
        if np.prod(t_target2cam.shape) == 3:
            t_target2cam = t_target2cam.reshape((3,))
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

    def add_samples_from_folder(
        self,
        data_dir: str,
        exclude_samples: Optional[List[int]] = None,
    ):
        """
        Add samples from a folder.

        Parameters
        ----------
        data_dir : str
            The directory to load the data from.
        exclude_samples : Optional[List[int]]
            The samples to exclude.
        """
        if not os.path.exists(data_dir):
            print(f"Data directory {data_dir} does not exist.", flush=True)
            return
        if exclude_samples is None:
            exclude_samples = []

        # Load the samples
        sample_i = -1
        while True:
            sample_i += 1
            if sample_i in exclude_samples:
                continue
            if not os.path.exists(os.path.join(data_dir, f"{sample_i}_rgb_img.png")):
                break
            (
                rgb_img,
                R_gripper2base,
                t_gripper2base,
                R_target2cam,
                t_target2cam,
            ) = self.load_sample(data_dir, sample_i)
            self.rgb_images.append(rgb_img)
            self.Rs_gripper2base.append(R_gripper2base)
            self.ts_gripper2base.append(t_gripper2base)
            self.Rs_target2cam.append(R_target2cam)
            self.ts_target2cam.append(t_target2cam)

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
        # pylint: disable=too-many-arguments
        # Necessay to get all the data for this sample.
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
        self,
        save_data: bool = False,
        verbose: bool = False,
    ) -> Tuple[
        Optional[np.ndarray],
        Optional[np.ndarray],
        Optional[float],
        Optional[float],
        Optional[int],
    ]:
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
        verbose : bool
            Whether to print the calibration results.

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
        int
            The calibration method that got the least error.
        """
        # pylint: disable=too-many-locals
        # One over is fine.

        if len(self.Rs_gripper2base) < 3:
            print("Need at least 3 samples to calibrate the camera.", flush=True)
            return None, None, None, None, None

        # Compute the camera extrinsics calibration
        best_R_cam2gripper = None
        best_t_cam2gripper = None
        best_translation_error = np.inf
        best_rotation_error = np.inf
        best_method = None
        for method in self.methods:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                self.Rs_gripper2base,
                self.ts_gripper2base,
                self.Rs_target2cam,
                self.ts_target2cam,
                method=method,
            )

            # Convert to a homogenous transform
            T_cam2gripper = np.eye(4)
            T_cam2gripper[:3, :3] = R_cam2gripper
            T_cam2gripper[:3, 3] = t_cam2gripper.reshape((3,))

            # Compute the transformation error
            Rs_target2base = []
            ts_target2base = []
            # pylint: disable=consider-using-enumerate
            for i in range(len(self.Rs_target2cam)):
                # Get the homogenous transform from the gripper to the base
                T_gripper2base = np.eye(4)
                if self.Rs_gripper2base[i].shape == (3,):
                    T_gripper2base[:3, :3] = R.from_rotvec(
                        self.Rs_gripper2base[i]
                    ).as_matrix()
                else:
                    T_gripper2base[:3, :3] = self.Rs_gripper2base[i]
                T_gripper2base[:3, 3] = self.ts_gripper2base[i]

                # Get the homogenous transform from the target to the camera
                T_target2cam = np.eye(4)
                if self.Rs_target2cam[i].shape == (3,):
                    T_target2cam[:3, :3] = R.from_rotvec(
                        self.Rs_target2cam[i]
                    ).as_matrix()
                else:
                    T_target2cam[:3, :3] = self.Rs_target2cam[i]
                T_target2cam[:3, 3] = self.ts_target2cam[i]

                # Compute the homogenous transform from the target to the base
                T_target2base = T_gripper2base @ T_cam2gripper @ T_target2cam

                # Extract the rotation and translation
                Rs_target2base.append(R.from_matrix(T_target2base[:3, :3]))
                ts_target2base.append(T_target2base[:3, 3])

            # Compute the translation and rotation errors
            translation_errors = []
            rotation_errors = []
            for i in range(
                len(Rs_target2base)
            ):  # pylint: disable=consider-using-enumerate
                for j in range(i + 1, len(Rs_target2base)):
                    translation_errors.append(
                        np.linalg.norm(ts_target2base[i] - ts_target2base[j])
                    )
                    rotation_errors.append(
                        (Rs_target2base[i].inv() * Rs_target2base[j]).magnitude()
                    )

            # Average the errors
            translation_error = np.percentile(translation_errors, 50)
            rotation_error = np.percentile(rotation_errors, 50)

            # Print the calibration results
            if verbose:
                print(f"Method: {method}", flush=True)
                print(f"Translation error: {translation_error}", flush=True)
                print(f"Rotation error: {rotation_error}", flush=True)
                print(
                    f"R_cam2gripper: {R.from_matrix(R_cam2gripper).as_euler('ZYX')}",
                    flush=True,
                )
                print(f"t_cam2gripper: {t_cam2gripper}", flush=True)

            # Save the best calibration
            if translation_error < best_translation_error:
                best_translation_error = translation_error
                best_rotation_error = rotation_error
                best_R_cam2gripper = R_cam2gripper
                best_t_cam2gripper = t_cam2gripper
                best_method = method

        # Save the calibration
        if save_data and self.data_dir is not None:
            np.savez_compressed(
                os.path.join(self.data_dir, f"{self.sample_i}_calib.npz"),
                R_cam2gripper=best_R_cam2gripper,
                t_cam2gripper=best_t_cam2gripper,
                translation_error=best_translation_error,
                rotation_error=best_rotation_error,
            )

        return (
            best_R_cam2gripper,
            best_t_cam2gripper,
            best_rotation_error,
            best_translation_error,
            best_method,
        )
