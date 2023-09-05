# Default Camera Calibration Methodology

This camera calibration was done in ROS1, using [MoveIt's Hand-Eye Calibration](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html). For the sake of reproduceability, this readme documents the methodology followed to do the camera calibration.

## Calibrations

To account for the potential error in measuring the size of the Aruco tags and the space in between tags (e.g., are the tags 0.059m or 0.06m?), we ran calibration twice.
- **Calib1**: All the joint states used in this calibration, the samples measured from HandEyeCalibration, and the ROS1 launchfile with the final transform, can be found in the `calib1` folder. This calibration had a reprojection error of 0.0293236m and 0.0154105rad.
- **Calib2**: We did not save the joint states from this calibration, but all the samples and the launchfile with the final transform can be found in the `calib2` folder. This calibration had a reprojection error of 0.0153083m and 0.00549327rad.

For both, we used the solver `crigroup/Daniilidis1999`, which achieved the lowest reprojection error.

## Testing the Calibrations

We used two visual tests to test the calibration:

**Test 1**: Turning the robot to point at itself so part of its chassis is visible from the depth camera. In RVIZ, the depth cloud that the camera sees should align with the robot model. (See the below image, which includes the joint states required to re-create this configuration).

![Screenshot from 2023-08-16 17-26-50](https://github.com/personalrobotics/ada_ros2/assets/8277986/9064820f-0350-4d13-a1c9-4ce79c49155c)
(Note: my index finger was touching the actual bottom of the fork in the real-world.)

**Test 2**: Touching the fork to the plate on the table. In RVIZ, the tip of the fork should touch the bottom of the plate as detected by the depth cloud.(See the below image, which includes the joint states required to re-create this configuration). Note that errors in this test could either be due to camera miscalibration or because the actual fork does not align wiht the URDF (e.g., it got bent).

![Screenshot from 2023-08-16 17-29-47](https://github.com/personalrobotics/ada_ros2/assets/8277986/94f89a37-11f1-48ac-b886-b49b0694597d)

**Results**: The results of the above test revealed that **Calib1** detected obstacles as being slightly closer to it than they actually were, and **Calib2** detected obstacles slightly farther from it than they actually were. Visually, the distance for each looked to be 0.5-1.0cm. Therefore, **_we decided to average the transforms_**.

## Transform Calculations

The script `transform.py` contains the following calculations:
1. Averaging the transforms for **Calib1** and **Calib2** together, and printing the resulting transform from `j2n6s200_end_effector` to `camera_color_optical_frame`. This is the value in `calib_camera_pose.launch.py`.
2. Combining that transform with the fixed transforms between---(a) `camera_link` and `camera_color_optical_frame` and (b) `j2n6s200_end_effector` and `cameraMount`---in order to compute the transform from `cameraMount` to `camera_link`. This is the transform in `ada_description/urdf/ada.xacro`.

It is a standalone Python script, and can be executed with `python3 /path/to/transform.py`. It requires the following dependency: `python3 -m pip install transformations`.

## Usage

There are two ways to use this transform:
1. Run `ros2 launch ada_moveit demo.launch.py` to publish the static transform from `j2n6s200_end_effector` to `camera_color_optical_frame`, and launch the RealSense with the parameter `publish_tf:=false`. The upside of this is that the published transform will likely be more accurate, since it is directly computed from camera calibration. The downside is that no other camera frames (e.g., the depth frame) are published, so we can only use messages in `camera_color_optical_frame`.
2. Run `ros2 launch ada_moveit demo.launch.py calib:=none` to not publish a transform directly to `camera_color_optical_frame`, and launch the RealSense with the parameter `publish_tf:=true` (the default value). This uses the computed transform from `cameraMount` to `camera_link` from the URDF, as well as the RealSense's URDF. The benefit of this is that all camera frames are published, but the downside is that there are more potential sources of error (e.g., if the RealSense URDF is inaccurate, that introduces error with this approach).
