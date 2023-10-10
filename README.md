# ada_ros2
ROS2 Hardware Interface and Description for the ADA Robot

## Dependencies
1. Install the Kinova SDK for your robot by searching [here](https://www.kinovarobotics.com/resources?r=79301&s). PRL currently uses the [Gen2 SDK v1.5.1](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view). Note that although the latest version of that SDK is for Ubuntu 16.04, it still works on Ubuntu 22.04 (only for x86 systems, not ARM system).
2. Install required ROS binaries. Note that some or all of these may already be installed. `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-kinematics-interface-kdl ros-humble-ament-cmake-clang-format ros-humble-rviz2 ros-humble-moveit ros-humble-moveit-ros-perception ros-humble-ackermann-msgs ros-humble-control-toolbox ros-humble-generate-parameter-library ros-humble-generate-parameter-library-py ros-humble-moveit-ros-control-interface`
3. Install required python packages: `python3 -m pip install trimesh`
4. Configure and build your workspace:
    1. Git clone [this repo (ada_ros2)](https://github.com/personalrobotics/ada_ros2) and [`pr_ros_controllers` (branch: `main`)](https://github.com/personalrobotics/pr_ros_controllers) into your ROS2 workspace's `src` folder.
    2. Build the workspace: `colcon build`. Note that if the Kinova SDK can't be installed on your device (e.g., you are on an ARM system and only need to run sim), you can skip `ada_hardware` with this command: `colcon build --packages-skip ada_hardware`

## Running ADA MoveIt
### RVIZ
1. Run `ros2 launch ada_moveit demo.launch.py sim:=mock` command from your ROS2 workspace.
2. See here for a [brief guide to using RVIZ to interact with MoveIt](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).

### Real
1. Run `ros2 launch ada_moveit demo.launch.py` command from your ROS2 workspace. If running for feeding specifically (i.e., where the watchdog dying kills the controllers) run `ros2 launch ada_moveit demo_feeding.launch.py`. Make sure the watchdog is running before you launch this node.

## Camera Calibration

See the [README for the `default` calibration](./ada_moveit/calib/default/README.md) for details about our extrinsics calibration methdology.
