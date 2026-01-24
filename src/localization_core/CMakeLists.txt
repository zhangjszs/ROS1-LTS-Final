cmake_minimum_required(VERSION 3.0.2)
project(localization_core)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS AND EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES localization_core
  DEPENDS PCL Eigen3
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/imu_state_estimator.cpp
  src/location_mapper.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)
