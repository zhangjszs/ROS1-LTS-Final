cmake_minimum_required(VERSION 3.0.2)
project(perception_ros)
add_compile_options(-std=c++17)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  nodelet
  pluginlib
  pcl_ros
  pcl_conversions
  visualization_msgs
  autodrive_msgs
  perception_core
)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs nodelet pluginlib pcl_ros pcl_conversions visualization_msgs autodrive_msgs perception_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/lidar_cluster_ros.cpp
  src/visulization_marker.cpp
  src/imu_subscriber.cpp
  src/nodelets/lidar_cluster_nodelet.cc
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  perception_core
)

add_executable(${PROJECT_NAME}_viewer nodes/pcd_viewer_node.cpp)
target_link_libraries(${PROJECT_NAME}_viewer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node nodes/lidar_cluster_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
  perception_core
)

add_executable(test_conf test/confidence.cpp)
target_link_libraries(test_conf ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(perception_ros_perf_stats_test test/test_perf_stats.cpp)
  if(TARGET perception_ros_perf_stats_test)
    target_link_libraries(perception_ros_perf_stats_test ${catkin_LIBRARIES})
  endif()
endif()
