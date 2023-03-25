cmake_minimum_required(VERSION 3.8)
project(ada_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

add_library(ada_hardware SHARED
  src/jaco2.cpp)

target_include_directories(ada_hardware include)

target_link_libraries(ada_hardware -l:Kinova.API.CommLayerUbuntu.so -l:Kinova.API.USBCommandLayerUbuntu.so)

ament_target_dependencies(ada_hardware rclcpp rclcpp_lifecycle pluginlib hardware_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
