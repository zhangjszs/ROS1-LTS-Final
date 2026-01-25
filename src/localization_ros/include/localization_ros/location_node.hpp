#pragma once

#include <string>

#include <ros/ros.h>
#include <autodrive_msgs/HUAT_Asensing.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_Cone.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

#include <localization_core/location_mapper.hpp>
#include <localization_core/types.hpp>

namespace localization_ros {

class LocationNode {
 public:
  explicit LocationNode(ros::NodeHandle &nh);

 private:
  void loadParameters();
  void publishState(const localization_core::CarState &state, const ros::Time &stamp, bool publish_carstate);
  void imuCallback(const autodrive_msgs::HUAT_Asensing::ConstPtr &msg);
  void carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msg);
  void coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &msg);

  static localization_core::Asensing ToCore(const autodrive_msgs::HUAT_Asensing &msg);
  static localization_core::CarState ToCore(const autodrive_msgs::HUAT_CarState &msg);
  static void ToRos(const localization_core::CarState &state, autodrive_msgs::HUAT_CarState *out);
  static localization_core::ConeDetections ToCore(const autodrive_msgs::HUAT_ConeDetections &msg);
  static void ToRos(const localization_core::ConeMap &map, autodrive_msgs::HUAT_ConeMap *out);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber ins_sub_;
  ros::Subscriber carstate_sub_;
  ros::Subscriber cone_sub_;

  ros::Publisher carstate_pub_;
  ros::Publisher map_pub_;
  ros::Publisher global_map_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  bool use_external_carstate_ = false;
  std::string ins_topic_;
  std::string carstate_in_topic_;
  std::string cone_topic_;
  std::string carstate_out_topic_;
  std::string cone_map_topic_;
  std::string global_map_topic_;
  std::string pose_topic_;
  std::string odom_topic_;
  std::string world_frame_;
  std::string base_link_frame_;

  localization_core::LocationParams params_;
  localization_core::LocationMapper mapper_;
};

}  // namespace localization_ros
