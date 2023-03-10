cmake_minimum_required(VERSION 3.5)
project(ada_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # ament_add_pytest_test(<test_name> test/test_path.py)
endif()

ament_package()
