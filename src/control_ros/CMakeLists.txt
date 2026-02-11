cmake_minimum_required(VERSION 3.0.2)
project(control_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  autodrive_msgs
  control_core
  eigen_conversions
  geometry_msgs
  roscpp
  roslib
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS autodrive_msgs control_core eigen_conversions geometry_msgs roscpp roslib std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_node
  src/control_node.cpp
)

target_link_libraries(control_node
  ${catkin_LIBRARIES}
)
