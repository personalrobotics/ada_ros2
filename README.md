# ada_ros2
ROS2 Hardware Interface and Description for the ADA Robot

## Dependencies
1. Install the Kinova SDK for your robot by searching [here](https://www.kinovarobotics.com/resources?r=79301&s). PRL currently uses the [Gen2 SDK v1.5.1](https://drive.google.com/file/d/1UEQAow0XLcVcPCeQfHK9ERBihOCclkJ9/view). Note that although the latest version of that SDK is for Ubuntu 16.04, it still works on Ubuntu 22.04 (only for x86 systems, not ARM system).
2. Install required ROS binaries. Note that some or all of these may already be installed. `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ament-cmake-clang-format ros-humble-rviz2 ros-humble-moveit`

## Running ADA MoveIt in RVIZ
1. `ros2 launch ada_moveit demo.launch.py sim:=mock`
2. See here for a [brief guide to using RVIZ to interact with MoveIt](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).
