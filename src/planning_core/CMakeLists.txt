cmake_minimum_required(VERSION 3.0.2)
project(planning_core)

add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
if(NOT Eigen3_INCLUDE_DIRS)
  if(EIGEN3_INCLUDE_DIRS)
    set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
  elseif(EIGEN3_INCLUDE_DIR)
    set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  endif()
endif()
if(NOT Eigen3_LIBRARIES)
  set(Eigen3_LIBRARIES "")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS PCL Eigen3
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/line_detection_core.cpp
  src/skidpad_detection_core.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_test test/test_line_detection.cpp)
  catkin_add_gtest(${PROJECT_NAME}_test_speed_profile test/test_speed_profile.cpp)
  catkin_add_gtest(${PROJECT_NAME}_test_skidpad test/test_skidpad_detection_core.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  if(TARGET ${PROJECT_NAME}_test_speed_profile)
    target_link_libraries(${PROJECT_NAME}_test_speed_profile ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  if(TARGET ${PROJECT_NAME}_test_skidpad)
    target_link_libraries(${PROJECT_NAME}_test_skidpad ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()
