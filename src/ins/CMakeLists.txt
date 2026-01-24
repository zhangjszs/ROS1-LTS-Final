cmake_minimum_required(VERSION 3.0.2)
project(ins)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  rospy
)

add_message_files(FILES
  ASENSING.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rospy
)

catkin_install_python(PROGRAMS
  scripts/ins_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
