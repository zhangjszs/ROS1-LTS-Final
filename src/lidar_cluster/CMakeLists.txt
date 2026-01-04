cmake_minimum_required(VERSION 3.0.2)
project(lidar_cluster)
add_compile_options(-std=c++11)
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
geometry_msgs
common_msgs
std_msgs
nav_msgs
nodelet
)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros geometry_msgs common_msgs nodelet
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_viewer src/pcdViewer.cpp)
target_link_libraries(${PROJECT_NAME}_viewer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

link_directories(${PCL_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/distortion_adjust.cpp  src/lidar_cluster.cpp src/utility.cpp src/ground_segmentation.cpp src/visulization_marker.cpp src/lidar_cluster_nodelet.cc src/imu_subscriber.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(test_conf test/confidence.cpp)
target_link_libraries(test_conf ${PROJECT_NAME} ${catkin_LIBRARIES})