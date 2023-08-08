cmake_minimum_required(VERSION 2.8.3)
project(jaco_hardware)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
    )

find_package(Boost REQUIRED COMPONENTS system)
link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${libkindrv_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(jaco_hardware
#   src/${PROJECT_NAME}/jaco_hardware.cpp
# )

## Declare a cpp executable
add_executable(jaco_hardware src/main.cpp src/JacoRobot.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(jaco_hardware_node jaco_hardware_generate_messages_cpp)
set(KINOVA_ARM_LIBS ${catkin_LIBRARIES})
set(KINOVA_ARM_LIBS
    ${KINOVA_ARM_LIBS}
    -l:USBCommLayerUbuntu.so
    -l:USBCommandLayerUbuntu.so)


#==============================================================================
# Format
#

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")
include(ClangFormat)

clang_format_setup(VERSION 6.0)

if (CLANG_FORMAT_EXECUTABLE)
  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    include/JacoRobot.h src/*.c src/*.cpp)

  clang_format_add_sources(${ALL_SOURCE_FILES})

  clang_format_add_targets()
endif()

## Specify libraries to link a library or executable target against
 target_link_libraries(jaco_hardware
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${KINOVA_ARM_LIBS}
 )


