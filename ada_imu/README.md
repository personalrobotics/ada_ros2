# ada_imu
IMU jointstate node for determining and publishing wheelchair tilt

## Dependencies
1. Install required python packages: `python3 -m pip install pyserial`
2. Remove braille package that conflicts with serial input: `sudo apt remove brltty`

## Running imu_jointstate_publisher
1. Source your workspace: `source install/setup.bash`
2. Launch the node: `ros2 launch ada_imu ada_imu.launch.py`

## ROS Parameters

## Calibration
1. Level the wheelchair and make sure the robot base is completely perpendicular to the floor.
2. cd into `ada_imu/scripts` and run `python3 imu_read.py`
3. Copy the x, y, and z values into the `main_calib_vector` parameter in `config/imu_params.yaml`
4. Tilt the wheelchair backwards

## Hardware
IMU: [OpenLog Artemis](https://learn.sparkfun.com/tutorials/openlog-artemis-hookup-guide/introduction)
