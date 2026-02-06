cmake_minimum_required(VERSION 3.0.2)
project(fsd_visualization)

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
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros autodrive_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Visualization Node
add_executable(fsd_viz_node
  src/fsd_viz_node.cpp
  src/cone_visualizer.cpp
  src/path_visualizer.cpp
  src/vehicle_visualizer.cpp
  src/triangulation_visualizer.cpp
)
target_link_libraries(fsd_viz_node ${catkin_LIBRARIES})

# Install
install(TARGETS fsd_viz_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch rviz config meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
