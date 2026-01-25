cmake_minimum_required(VERSION 3.0.2)
project(localization_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  autodrive_msgs
  localization_core
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib sensor_msgs geometry_msgs nav_msgs tf2_ros pcl_ros pcl_conversions autodrive_msgs localization_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(location_node
  src/main.cpp
  src/location.cpp
)

add_executable(state_estimator_node
  src/state_estimator_node.cpp
)

target_link_libraries(location_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  localization_core
)

target_link_libraries(state_estimator_node
  ${catkin_LIBRARIES}
  localization_core
)
