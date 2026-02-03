cmake_minimum_required(VERSION 3.0.2)

project(autodrive_msgs)

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
  HUAT_Asensing.msg
  HUAT_AsensingStamped.msg
  HUAT_CarState.msg
  HUAT_Cone.msg
  HUAT_ConeDetections.msg
  HUAT_ConeMap.msg
  HUAT_ControlCommand.msg
  HUAT_FsdCone.msg
  HUAT_FsdMap.msg
  HUAT_HighSpeedViz.msg
  HUAT_ImageObjs.msg
  HUAT_ImageRect.msg
  HUAT_InsP2.msg
  HUAT_LanePoint.msg
  HUAT_LocalPlanning.msg
  HUAT_Logging.msg
  HUAT_MsfCone.msg
  HUAT_PathData.msg
  HUAT_PathLimits.msg
  HUAT_SimState.msg
  HUAT_Stop.msg
  HUAT_TimeAndAzimuth.msg
  HUAT_TrackLimits.msg
  HUAT_VehicleCmd.msg
  HUAT_VehicleStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs # Needed to generate custom messages
)
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  #${roscpp_INCLUDE_DIRS}
  #${rospy_INCLUDE_DIRS}
)
