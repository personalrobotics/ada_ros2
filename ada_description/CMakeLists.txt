cmake_minimum_required(VERSION 2.8.12)
project(ada_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  transmission_interface
)

catkin_package(
)
