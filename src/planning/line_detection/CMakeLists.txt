cmake_minimum_required(VERSION 3.0.2)
project(line_detection)

add_compile_options(-std=c++17)

# Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  common_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  roslib
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs common_msgs roslib std_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/common_msgs/msg
)

# Create executable
add_executable(${PROJECT_NAME}_node
  src/line_detection.cpp
  src/main.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
