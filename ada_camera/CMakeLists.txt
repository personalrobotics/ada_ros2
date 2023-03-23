cmake_minimum_required(VERSION 3.8)
project(ada_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(realsense2-net REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

## Building
include_directories(
  include 
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ada_camera src/main.cpp)

## Realsense needs explicit linking
target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    ${realsense2-net_LIBRARY}
    ${OpenCV_LIBS}
)

ament_target_dependencies(ada_camera 
  rclcpp std_msgs realsense2 realsense2-net image_transport sensor_msgs cv_bridge image_transport)

install(TARGETS
  ada_camera
  DESTINATION lib/${PROJECT_NAME})

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
