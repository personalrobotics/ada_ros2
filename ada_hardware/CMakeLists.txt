cmake_minimum_required(VERSION 3.8)
project(ada_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ada_hardware SHARED
  src/jaco2.cpp
  src/isaacsim.cpp)

target_include_directories(ada_hardware PUBLIC include)

target_link_libraries(ada_hardware -l:Kinova.API.CommLayerUbuntu.so -l:Kinova.API.USBCommandLayerUbuntu.so)

ament_target_dependencies(ada_hardware rclcpp rclcpp_lifecycle pluginlib hardware_interface sensor_msgs)

pluginlib_export_plugin_description_file(hardware_interface ada_hardware.xml)

# Install (required for pluginlib)
install(
  DIRECTORY include/
  DESTINATION include/ada_hardware
)
install(TARGETS ada_hardware
  EXPORT export_ada_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ### Linting
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ### GMock
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_jaco2 test/test_jaco2.cpp)
  ament_target_dependencies(test_jaco2
    pluginlib
    ros2_control_test_assets
    rclcpp
    rclcpp_lifecycle
    hardware_interface
  )

  ament_add_gmock(test_isaacsim test/test_isaacsim.cpp)
  ament_target_dependencies(test_isaacsim
    pluginlib
    ros2_control_test_assets
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    sensor_msgs
  )
endif()

ament_export_dependencies(rclcpp rclcpp_lifecycle pluginlib hardware_interface)
ament_package()
