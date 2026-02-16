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
  src/ground_segmentation_fgs.cpp
  src/fast_ground_segmentation.cpp
  src/patchworkpp.cpp
  src/distortion_adjust_v2.cpp
  src/utility.cpp
  src/cluster_feature_extractor.cpp
  src/confidence_scorer.cpp
  src/cone_model_fitter.cpp
  src/cone_tracker.cpp
  src/topology_repair.cpp
  src/point_cloud_pool.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# ============== Benchmark ==============
# 独立的性能基准测试程序
option(BUILD_BENCHMARK "Build benchmark executables" ON)

if(BUILD_BENCHMARK)
  add_executable(ground_segmentation_benchmark
    benchmark/ground_segmentation_benchmark.cpp
  )
  target_link_libraries(ground_segmentation_benchmark
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    stdc++fs  # for std::filesystem
  )
  target_compile_features(ground_segmentation_benchmark PRIVATE cxx_std_17)
endif()

# ============== Tests ==============
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # 地面分割性能测试
  catkin_add_gtest(test_ground_segmentation_perf
    test/test_ground_segmentation_perf.cpp
  )
  if(TARGET test_ground_segmentation_perf)
    target_link_libraries(test_ground_segmentation_perf
      ${PROJECT_NAME}
      ${PCL_LIBRARIES}
    )
  endif()

  # 点云内存池测试
  catkin_add_gtest(test_point_cloud_pool
    test/test_point_cloud_pool.cpp
  )
  if(TARGET test_point_cloud_pool)
    target_link_libraries(test_point_cloud_pool
      ${PROJECT_NAME}
      ${PCL_LIBRARIES}
    )
  endif()

  # Confidence scorer 测试
  catkin_add_gtest(test_confidence_scorer
    test/test_confidence_scorer.cpp
  )
  if(TARGET test_confidence_scorer)
    target_link_libraries(test_confidence_scorer
      ${PROJECT_NAME}
      ${PCL_LIBRARIES}
    )
  endif()

  # Cone tracker 测试
  catkin_add_gtest(test_cone_tracker
    test/test_cone_tracker.cpp
  )
  if(TARGET test_cone_tracker)
    target_link_libraries(test_cone_tracker
      ${PROJECT_NAME}
      ${PCL_LIBRARIES}
    )
  endif()
endif()
