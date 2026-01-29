cmake_minimum_required(VERSION 3.0.2)
project(perception_core)

add_compile_options(-std=c++17)

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
  LIBRARIES perception_core
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
  src/lidar_cluster_core.cpp
  src/ground_segmentation_ransac.cpp
  src/ground_segmentation_patchworkpp.cpp
  src/patchworkpp.cpp
  src/distortion_adjust.cpp
  src/utility.cpp
  src/cluster_feature_extractor.cpp
  src/confidence_scorer.cpp
  src/cone_model_fitter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)
