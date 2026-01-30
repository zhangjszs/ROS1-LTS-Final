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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_test test/test_controller.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()
