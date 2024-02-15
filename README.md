# ada_ros2
ROS2 Hardware Interface and Description for the ADA Robot

## Setup

See the [`ada_feeding` top-level README for setup instructions](https://github.com/personalrobotics/ada_feeding/blob/amaln/rosdeps/README.md).

## Running ADA MoveIt
### RVIZ
1. Run `ros2 launch ada_moveit demo.launch.py sim:=mock` command from your ROS2 workspace.
2. See here for a [brief guide to using RVIZ to interact with MoveIt](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).

### Real
1. Run `ros2 launch ada_moveit demo.launch.py` command from your ROS2 workspace. If running for feeding specifically (i.e., where the watchdog dying kills the controllers) run `ros2 launch ada_moveit demo_feeding.launch.py`. Make sure the watchdog is running before you launch this node.

### MoveIt Servo

MoveIt Servo allows [real-time arm servoing](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html) in cartesian space by sending twist commands to the end effector. 

To use Servo with keyboard teleop:
1. Launch the force-torque sensor:
    1. Sim: `ros2 run ada_feeding dummy_ft_sensor.py`
    2. Real: `ros2 run forque_sensor_hardware forque_sensor_hardware`
2. Launch MoveIt:
    1. Sim:`ros2 launch ada_moveit demo.launch.py sim:=mock`
    2. Real: `ros2 launch ada_moveit demo.launch.py`
3. Re-tare the F/T sensor: `ros2 service call /wireless_ft/set_bias std_srvs/srv/SetBool "{data: true}"`
4. Enable MoveIt Servo:
    1. Switch Controllers: `ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [\"jaco_arm_servo_controller\"], deactivate_controllers: [\"jaco_arm_controller\"], start_controllers: [], stop_controllers: [], strictness: 0, start_asap: false, activate_asap: false, timeout: {sec: 0, nanosec: 0}}"`
    2. Toggle Servo On: `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"`
5. Run the keyboard teleop script: `ros2 run ada_moveit ada_keyboard_teleop.py`
6. Follow the on-screen instructions to teleoperate the robot. Note that although cartesian control avoids obstacles in the planning scene, joint control does not.
7. Toggle Servo Off: `ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger "{}"`

To create your own Servo client:
1. Follow steps 1-4 above.
2. Have your client publish Twist commands to `/servo_node/delta_twist_cmds`. Note the following:
    1. For reliable cartesian control when sending angular velocities on the real robot and `lovelace`, ensure the angular velocity is <= 0.3 rad/s in magnitude. Greater angular velocities might change the end effector's position in addition to its orientation. We believe this is because of latencies with MoveIt Servo getting the robot's joint states via the joint state publisher.
    2. Be sure to send 0-velocity Twist messages at the end to stop the robot.

## Camera Calibration

See the [README for the `default` calibration](./ada_moveit/calib/default/README.md) for details about our extrinsics calibration methdology.
