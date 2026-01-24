cmake_minimum_required(VERSION 3.0.2)
project(vehicle_racing_num_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  autodrive_msgs
  vehicle_racing_num_core
)

catkin_package(
  CATKIN_DEPENDS roscpp autodrive_msgs vehicle_racing_num_core
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(vehicle_racing_num_node
  src/vehicle_racing_num_node.cpp
)

target_link_libraries(vehicle_racing_num_node
  ${catkin_LIBRARIES}
  vehicle_racing_num_core
)
