cmake_minimum_required(VERSION 3.0.2)
project(control_core)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/controller_base.cpp
  src/line_controller.cpp
  src/high_controller.cpp
  src/skip_controller.cpp
  src/test_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
