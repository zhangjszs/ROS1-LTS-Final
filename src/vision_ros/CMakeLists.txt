cmake_minimum_required(VERSION 3.0.2)
project(vision_ros)
add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs cv_bridge image_transport
  autodrive_msgs fsd_common vision_core diagnostic_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
                 autodrive_msgs fsd_common vision_core diagnostic_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/vision_node.cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(vision_node src/vision_main.cpp)
target_link_libraries(vision_node ${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/vision_node_py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_backlog_runtime.test)
  add_rostest(test/test_default_launch.test)
endif()
