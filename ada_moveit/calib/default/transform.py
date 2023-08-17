import transformations

################################################################################
# j2n6s200_end_effector -> camera_color_optical_frame : outputted by calibration
################################################################################

# The below calibration had reprojection error 0.0293236m and 0.0154105rad. In practice, it detects
# objects in the depth camera as slightly closer (looks like ~1cm in RVIZ) than they are.
ee_to_camera_optical_1 = transformations.quaternion_matrix(
    [-0.00145183, -0.13495, 0.990841, -0.00447916]
)
ee_to_camera_optical_1[:3, 3] = [0.032072, 0.13256, -0.155098]

# The below calibration had reprojection error 0.0153083m and 0.00546327rad. In practice, it detects
# objects in the depth camera as farther (looks like ~1cm in RVIZ) than they are.
ee_to_camera_optical_2 = transformations.quaternion_matrix(
    [0.0017638, -0.134985, 0.990838, 0.00400733]
)
ee_to_camera_optical_2[:3, 3] = [0.0300509, 0.134319, -0.138905]

# Therefore, we average the two transforms together to get the ee_to_camera_optical transform
ee_to_camera_optical_quat = transformations.quaternion_slerp(
    transformations.quaternion_from_matrix(ee_to_camera_optical_1),
    transformations.quaternion_from_matrix(ee_to_camera_optical_2),
    0.5,
)
ee_to_camera_optical_trans = (
    ee_to_camera_optical_1[:3, 3] + ee_to_camera_optical_2[:3, 3]
) / 2.0
print(
    "j2n6s200_end_effector -> camera_color_optical_frame translation: %s, quaternion: %s"
    % (ee_to_camera_optical_trans, ee_to_camera_optical_quat)
)
ee_to_camera_optical = transformations.quaternion_matrix(ee_to_camera_optical_quat)
ee_to_camera_optical[:3, 3] = ee_to_camera_optical_trans

################################################################################
# j2n6s200_end_effector -> cameraMount : hardcoded into ADA URDF
#     ROS1: rosrun tf tf_echo /j2n6s200_end_effector /cameraMount
################################################################################
ee_to_camera_mount = transformations.quaternion_matrix([0.090, -0.090, 0.701, 0.701])
ee_to_camera_mount[:3, 3] = [-0.000, 0.122, -0.180]

################################################################################
# camera_link -> camera_color_optical_frame : outputted by RealSense's URDF
#     ROS1: rosrun tf tf_echo /camera_link /camera_color_optical_frame
################################################################################
camera_link_to_camera_optical = transformations.quaternion_matrix(
    [0.502, -0.501, 0.498, -0.499]
)
camera_link_to_camera_optical[:3, 3] = [-0.000, 0.015, -0.000]

################################################################################
# Compute cameraMount -> camera_link
################################################################################
camera_mount_to_camera_link = transformations.concatenate_matrices(
    transformations.inverse_matrix(ee_to_camera_mount),
    ee_to_camera_optical,
    transformations.inverse_matrix(camera_link_to_camera_optical),
)
print("camera_mount_to_camera_link Translation: ", camera_mount_to_camera_link[:3, 3])
print(
    "camera_mount_to_camera_link Euler: ",
    transformations.euler_from_matrix(camera_mount_to_camera_link, "sxyz"),
)
