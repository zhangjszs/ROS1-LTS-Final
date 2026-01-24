cmake_minimum_required(VERSION 3.0.2)
project(vehicle_interface_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  autodrive_msgs
  std_msgs
  vehicle_interface_core
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp autodrive_msgs std_msgs vehicle_interface_core
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(vehicle_interface_node
  nodes/vehicle_interface_node.cpp
  src/App.cpp
  src/Node.cpp
)

target_link_libraries(vehicle_interface_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  vehicle_interface_core
)
