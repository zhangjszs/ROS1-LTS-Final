cmake_minimum_required(VERSION 3.0.2)
project(planning_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  autodrive_msgs
  eigen_conversions
  fsd_common
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  roslib
  std_msgs
  tf
  planning_core
  diagnostic_msgs
)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS AND EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS autodrive_msgs eigen_conversions fsd_common geometry_msgs message_filters nav_msgs roscpp roslib std_msgs tf planning_core diagnostic_msgs
)

include_directories(
  include
  include/high_speed_tracking
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/line_detection_node.cpp
  src/skidpad_detection_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  planning_core
)

add_library(${PROJECT_NAME}_high_speed
  src/high_speed_tracking/modules/DelaunayTri.cpp
  src/high_speed_tracking/modules/Visualization.cpp
  src/high_speed_tracking/modules/WayComputer.cpp
  src/high_speed_tracking/structures/Circle.cpp
  src/high_speed_tracking/structures/Edge.cpp
  src/high_speed_tracking/structures/Node.cpp
  src/high_speed_tracking/structures/Point.cpp
  src/high_speed_tracking/structures/Trace.cpp
  src/high_speed_tracking/structures/Triangle.cpp
  src/high_speed_tracking/structures/Vector.cpp
  src/high_speed_tracking/structures/Way.cpp
  src/high_speed_tracking/utils/Failsafe.cpp
  src/high_speed_tracking/utils/KDTree.cpp
  src/high_speed_tracking/utils/Params.cpp
  src/high_speed_tracking/utils/Time.cpp
)

target_link_libraries(${PROJECT_NAME}_high_speed
  ${catkin_LIBRARIES}
)

add_executable(line_detection_node
  nodes/line_detection_node.cpp
)

target_link_libraries(line_detection_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  planning_core
)

add_executable(skidpad_detection_node
  nodes/skidpad_detection_node.cpp
)

target_link_libraries(skidpad_detection_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  planning_core
)

add_executable(high_speed_tracking_node
  src/high_speed_tracking/main.cpp
)

target_link_libraries(high_speed_tracking_node
  ${PROJECT_NAME}_high_speed
  ${catkin_LIBRARIES}
)

add_executable(planning_pipeline_node
  nodes/planning_pipeline_node.cpp
  src/planning_pipeline_node.cpp
)

target_link_libraries(planning_pipeline_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_high_speed
  ${catkin_LIBRARIES}
  planning_core
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_way_safety
    test/test_way_safety.cpp
  )
  if(TARGET test_way_safety)
    target_link_libraries(test_way_safety
      ${PROJECT_NAME}_high_speed
      ${catkin_LIBRARIES}
    )
  endif()
endif()
