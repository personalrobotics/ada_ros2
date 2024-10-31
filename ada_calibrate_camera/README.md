# ada_camera_calibration

This file contains a nodes to do the following:
1. Calibrate ADA's eye-in-hand system's extrinsics (run every time the eye-in-hand system changes);
2. Publish the transform between ADA's end-effector and the camera (run every time the robot is used with perception).

## Calibrating the Camera's Extrinsics
1. Be in the `src` directory of your workspace.
2. `python3 src/ada_ros2/ada_calibrate_camera/calibrate_camera_start.py`
3. `screen -r calibrate`
    1. (See here for relevant [`screen`` commands](https://gist.github.com/jctosta/af918e1618682638aa82))
4. Follow the instructions on-screen. (Motions are expected to take ~6 mins and collect up to 30 samples)
5. Once it is done, verify it:
    1. Re-build your workspace.
    2. Run `ros2 launch ada_moveit demo.launch.py sim:=mock`
    3. In RVIZ, add an axis for `camera_color_optical_frame`
    4. Verify it looks like the frame is correct.

## Publishing the Transform
1. `ros2 launch ada_calibrate_camera publish_camera_extrinsics_launch.xml` (this is done by default in the `ada_moveit` launchfile).
