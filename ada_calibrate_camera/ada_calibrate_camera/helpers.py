# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

"""
This module contains helper functions for calibrating the camera.
"""

# Standard imports
from typing import Tuple, Optional

# Third-party imports
import numpy as np
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TwistStamped,
    Vector3,
)
from scipy.spatial.transform import Rotation as R
import ros2_numpy


def pose_to_rot_trans(
    pose: PoseStamped, rot_vec: bool = True
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Disaggregates a into its rotation and translation components.

    Parameters
    ----------
    pose : PoseStamped
        The pose.
    rot_vec : bool
        Whether to return the rotation as a rotation vector or a matrix

    Returns
    -------
    np.ndarray
        The rotation vector.
    np.ndarray
        The translation vector.
    """
    if rot_vec:
        rot = R.from_quat(ros2_numpy.numpify(pose.pose.orientation)).as_rotvec()
    else:
        rot = R.from_quat(ros2_numpy.numpify(pose.pose.orientation)).as_matrix()
    trans = ros2_numpy.numpify(pose.pose.position)
    return rot, trans


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """
    Convert a Pose message to a homogeneous transformation matrix.

    Parameters
    ----------
    pose : Pose
        The pose.

    Returns
    -------
    np.ndarray
        The homogeneous transformation matrix.
    """
    M = np.eye(4)
    M[:3, :3] = R.from_quat(ros2_numpy.numpify(pose.orientation)).as_matrix()
    M[:3, 3] = ros2_numpy.numpify(pose.position)
    return M


def transform_to_matrix(transform: Transform) -> np.ndarray:
    """
    Convert a Transform message to a homogeneous transformation matrix.

    Parameters
    ----------
    transform : Transform
        The transform.

    Returns
    -------
    np.ndarray
        The homogeneous transformation matrix.
    """
    M = np.eye(4)
    M[:3, :3] = R.from_quat(ros2_numpy.numpify(transform.rotation)).as_matrix()
    M[:3, 3] = ros2_numpy.numpify(transform.translation)
    return M


def matrix_to_pose(M: np.ndarray) -> Pose:
    """
    Convert a homogeneous transformation matrix to a Pose message.

    Parameters
    ----------
    M : np.ndarray
        The homogeneous transformation matrix.

    Returns
    -------
    Pose
        The pose.
    """
    pose = Pose()
    pose.position = ros2_numpy.msgify(
        Point,
        M[:3, 3],
    )
    pose.orientation = ros2_numpy.msgify(
        Quaternion,
        R.from_matrix(M[:3, :3]).as_quat(),
    )
    return pose


def pose_to_twist(
    pose_stamped: PoseStamped,
    max_linear_speed: float = 0.1,  # m/s
    max_angular_speed: float = 0.5,  # rad/s
    round_decimals: Optional[int] = 6,
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
