# ada_imu
IMU jointstate node for determining and publishing wheelchair tilt

## Dependencies
1. Install required python packages: `python3 -m pip install pyserial`
2. Remove braille package that conflicts with serial input: `sudo apt remove brltty`

## Running imu_jointstate_publisher
1. Source your workspace: `source install/setup.bash`
2. Launch the node: `ros2 launch ada_imu ada_imu.launch.py`

## ROS Parameters
* `joint_name`: joint name under which the tilt is published
* `main_calib_vector`: IMU accelerometer reading when robot is level; used to calculate the tilt angle (see Calibration)
* `tilt_calib_vector`: IMU accelerometer reading when robot is tilted; used to establish the plane of rotation to calculate the sign of the tilt angle (see Calibration)
* `serial_port`: serial port IMU outputs to
* `velocity_thresh`: threshold to eliminate velocity noise, if the calculated velocity is less than the threshold, velocity is published as 0
* `position_smoothing_factor` and `velocity_smoothing_factor`: weights used in exponential smoothing on position and velocity respectively; floating point values between 0 and 1.
* `sim`: If `"real"`, read IMU data from the serial port. Else, assume the IMU angle is always 0.

## Calibration
1. Level the wheelchair and make sure the robot base is completely perpendicular to the floor
2. cd into `ada_imu/scripts` and run `python3 imu_read.py`
3. Copy the x, y, and z values into the `main_calib_vector` parameter in `config/imu_params.yaml`
4. Tilt the wheelchair backwards around 20-30 degrees. The exact number of degrees doesn't matter because this is only used to determine the direction or rotation, not the magnitude.
5. Run `python3 imu_read.py` again
6. Copy the x, y, and z values into the `tilt_calib_vector` parameter in `config/imu_params.yaml`

## Hardware
IMU: [OpenLog Artemis](https://learn.sparkfun.com/tutorials/openlog-artemis-hookup-guide/introduction)
