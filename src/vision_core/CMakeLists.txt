cmake_minimum_required(VERSION 3.0.2)
project(vision_core)
add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

# ONNX Runtime: optional — build without model support if not installed
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  find_path(ONNXRT_INCLUDE_DIR onnxruntime_cxx_api.h
    PATHS /usr/local/include/onnxruntime ENV ONNXRUNTIME_ROOT_DIR
    PATH_SUFFIXES include include/onnxruntime)
  find_library(ONNXRT_LIBRARY onnxruntime
    PATHS /usr/local/lib ENV ONNXRUNTIME_ROOT_DIR
    PATH_SUFFIXES lib)
endif()

if(ONNXRT_INCLUDE_DIR AND ONNXRT_LIBRARY)
  set(HAVE_ONNXRUNTIME TRUE)
  add_definitions(-DHAVE_ONNXRUNTIME)
  message(STATUS "ONNX Runtime found: ${ONNXRT_LIBRARY}")
else()
  set(HAVE_ONNXRUNTIME FALSE)
  message(STATUS "ONNX Runtime NOT found — building without model inference support")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vision_core
  DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
if(HAVE_ONNXRUNTIME)
  include_directories(${ONNXRT_INCLUDE_DIR})
endif()

add_library(${PROJECT_NAME}
  src/image_quality.cpp
  src/image_enhancer.cpp
  src/fallback_detector.cpp
  src/detection_postprocess.cpp
  src/temporal_tracker.cpp
  src/inference_backend.cpp
)
if(HAVE_ONNXRUNTIME)
  target_sources(${PROJECT_NAME} PRIVATE src/onnx_backend.cpp)
endif()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
if(HAVE_ONNXRUNTIME)
  target_link_libraries(${PROJECT_NAME} ${ONNXRT_LIBRARY})
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_image_quality test/test_image_quality.cpp)
  if(TARGET test_image_quality)
    target_link_libraries(test_image_quality ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  endif()
  catkin_add_gtest(test_fallback_detector test/test_fallback_detector.cpp)
  if(TARGET test_fallback_detector)
    target_link_libraries(test_fallback_detector ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  endif()
endif()
