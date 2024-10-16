# ada_camera_calibration

This file contains a nodes to do the following:
1. Calibrate ADA's eye-in-hand system's extrinsics (run every time the eye-in-hand system changes);
2. Publish the transform between ADA's end-effector and the camera (run every time the robot is used with perception).

## Calibrating the Camera's Extrinsics
1. Be in the `src` directory of your workspace.
2. `python3 src/ada_ros2/ada_calibrate_camera/calibrate_camera_start.py`
3. `screen -r calibrate`
    1. (See here for relevant [`screen`` commands](https://gist.github.com/jctosta/af918e1618682638aa82))
4. Follow the instructions on-screen.

## Publishing the Transform
1. `ros2 launch ada_calibrate_camera publish_camera_extrinsics_launch.xml`
