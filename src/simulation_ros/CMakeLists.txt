cmake_minimum_required(VERSION 3.0.2)
project(simulation_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  autodrive_msgs
  simulation_core
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf2
    tf2_ros
    autodrive_msgs
    simulation_core
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(simulation_node src/simulation_node.cpp)
target_link_libraries(simulation_node
  ${catkin_LIBRARIES}
)

# Install
install(TARGETS simulation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
