cmake_minimum_required(VERSION 3.0.2)

project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation  # Needed to generate custom message types
  genmsg
  std_msgs
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  )

add_message_files(  # Tell catkin which custom message files to include
  FILES
  Cone.msg
  ins_p2.msg
  ImageObjs.msg
  ImageRect.msg
  lane_point.msg
  local_planning.msg
  path_data.msg

  logging_msg.msg
  vehicle_cmd.msg
  vehicle_status.msg
  HUAT_cone.msg
  HUAT_map.msg
  HUAT_HighSpeedViz.msg
  HUAT_PathLimits.msg
  HUAT_Tracklimits.msg
  HUAT_ASENSING.msg
  HUAT_Carstate.msg
  new_ASENSING.msg
  time_and_azimuth.msg
  HUAT_ControlCommand.msg
  HUAT_VehcileCmd.msg 
  HUAT_stop.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs # Needed to generate custom messages
)
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  #${roscpp_INCLUDE_DIRS}
  #${rospy_INCLUDE_DIRS}
)
