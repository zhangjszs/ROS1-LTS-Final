#pragma once

#include <string>

#include <ros/ros.h>
#include <autodrive_msgs/HUAT_Asensing.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_Cone.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <localization_core/location_mapper.hpp>
#include <localization_core/types.hpp>

namespace localization_ros {

class LocationNode {
 public:
  explicit LocationNode(ros::NodeHandle &nh);

 private:
  void loadParameters();
  void imuCallback(const autodrive_msgs::HUAT_Asensing::ConstPtr &msg);
  void carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msg);
  void coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &msg);

  static localization_core::Asensing ToCore(const autodrive_msgs::HUAT_Asensing &msg);
  static localization_core::CarState ToCore(const autodrive_msgs::HUAT_CarState &msg);
  static void ToRos(const localization_core::CarState &state, autodrive_msgs::HUAT_CarState *out);
  static localization_core::ConeDetections ToCore(const autodrive_msgs::HUAT_ConeDetections &msg);
  static void ToRos(const localization_core::ConeMap &map, autodrive_msgs::HUAT_ConeMap *out);

  ros::NodeHandle nh_;

  ros::Subscriber ins_sub_;
  ros::Subscriber carstate_sub_;
  ros::Subscriber cone_sub_;

  ros::Publisher carstate_pub_;
  ros::Publisher map_pub_;
  ros::Publisher global_map_pub_;

  bool use_external_carstate_ = false;
  bool publish_visualization_ = false;
  std::string sub_topic_;

  localization_core::LocationParams params_;
  localization_core::LocationMapper mapper_;
};

}  // namespace localization_ros
