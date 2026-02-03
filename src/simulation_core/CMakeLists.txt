cmake_minimum_required(VERSION 3.0.2)
project(simulation_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/tire_model.cpp
  src/vehicle_dynamics.cpp
  src/cone_sensor.cpp
  src/track_loader.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_tire_model test/test_tire_model.cpp)
  target_link_libraries(test_tire_model ${PROJECT_NAME})

  catkin_add_gtest(test_vehicle_dynamics test/test_vehicle_dynamics.cpp)
  target_link_libraries(test_vehicle_dynamics ${PROJECT_NAME})
endif()
