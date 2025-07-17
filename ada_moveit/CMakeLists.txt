cmake_minimum_required(VERSION 3.22)
project(ada_moveit)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(chomp_motion_planner REQUIRED)
# find_package(moveit_planners_chomp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ada_keyboard_teleop.py
  scripts/servo_republisher.py
  scripts/unified_joint_state_publisher.py
  scripts/simulated_orientation_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
