#include <localization_ros/location_node.hpp>

#include <ros/package.h>

namespace localization_ros {

LocationNode::LocationNode(ros::NodeHandle &nh)
    : nh_(nh),
      mapper_(params_)
{
  loadParameters();

  mapper_.Configure(params_);
  mapper_.SetDataDirectory(ros::package::getPath("localization_ros"));

  if (use_external_carstate_)
  {
    carstate_sub_ = nh_.subscribe<autodrive_msgs::HUAT_CarState>("/Carstate", 10, &LocationNode::carstateCallback, this);
  }
  else
  {
    ins_sub_ = nh_.subscribe<autodrive_msgs::HUAT_Asensing>("/pbox_pub/Ins", 10, &LocationNode::imuCallback, this);
  }

  cone_sub_ = nh_.subscribe<autodrive_msgs::HUAT_ConeDetections>("/cone_position", 10, &LocationNode::coneCallback, this);

  carstate_pub_ = nh_.advertise<autodrive_msgs::HUAT_CarState>("/Carstate", 10);
  map_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeMap>("/coneMap", 10);
  global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/globalMapOnly", 10);

  (void)publish_visualization_;
  (void)sub_topic_;
}

void LocationNode::loadParameters()
{
  if (!nh_.param("length/lidarToIMUDist", params_.lidar_to_imu_dist, 1.87))
  {
    ROS_WARN_STREAM("Did not load lidarToIMUDist. Standard value is: " << params_.lidar_to_imu_dist);
  }
  if (!nh_.param("length/frontToIMUdistanceX", params_.front_to_imu_x, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceX. Standard value is: " << params_.front_to_imu_x);
  }
  if (!nh_.param("length/frontToIMUdistanceY", params_.front_to_imu_y, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceY. Standard value is " << params_.front_to_imu_y);
  }
  if (!nh_.param("length/frontToIMUdistanceZ", params_.front_to_imu_z, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceZ. Standard value is: " << params_.front_to_imu_z);
  }
  if (!nh_.param("length/rearToIMUdistanceX", params_.rear_to_imu_x, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceX. Standard value is " << params_.rear_to_imu_x);
  }
  if (!nh_.param("length/rearToIMUdistanceY", params_.rear_to_imu_y, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceY. Standard value is " << params_.rear_to_imu_y);
  }
  if (!nh_.param("length/rearToIMUdistanceZ", params_.rear_to_imu_z, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceZ. Standard value is " << params_.rear_to_imu_z);
  }

  if (!nh_.param<std::string>("filtered_topic_name", sub_topic_, "/location"))
  {
    ROS_WARN_STREAM("Did not load topic name. Standard value is: " << sub_topic_);
  }
  if (!nh_.param("use_external_carstate", use_external_carstate_, false))
  {
    ROS_WARN_STREAM("Did not load use_external_carstate. Standard value is: " << (use_external_carstate_ ? "true" : "false"));
  }
  if (!nh_.param("publish_visualization", publish_visualization_, false))
  {
    ROS_WARN_STREAM("Did not load publish_visualization. Standard value is: " << (publish_visualization_ ? "true" : "false"));
  }
}

void LocationNode::imuCallback(const autodrive_msgs::HUAT_Asensing::ConstPtr &msg)
{
  localization_core::Asensing core_msg = ToCore(*msg);
  localization_core::CarState state;

  if (mapper_.UpdateFromIns(core_msg, &state))
  {
    autodrive_msgs::HUAT_CarState out;
    ToRos(state, &out);
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "world";  // 全局 ENU 坐标系
    carstate_pub_.publish(out);
  }
}

void LocationNode::carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msg)
{
  mapper_.UpdateFromCarState(ToCore(*msg));
}

void LocationNode::coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &msg)
{
  if (!mapper_.has_carstate())
  {
    ROS_WARN("INS 数据没更新!");
    return;
  }
  if (msg->points.empty())
  {
    ROS_WARN("锥桶坐标为空!");
    return;
  }

  localization_core::ConeDetections detections = ToCore(*msg);
  localization_core::ConeMap map;
  localization_core::PointCloudPtr cloud;

  if (!mapper_.UpdateFromCones(detections, &map, &cloud))
  {
    return;
  }

  autodrive_msgs::HUAT_ConeMap out_map;
  ToRos(map, &out_map);
  out_map.header.stamp = ros::Time::now();
  out_map.header.frame_id = "world";  // 全局 ENU 坐标系
  map_pub_.publish(out_map);

  if (cloud)
  {
    sensor_msgs::PointCloud2 global_cloud_msg;
    pcl::toROSMsg(*cloud, global_cloud_msg);
    global_cloud_msg.header.frame_id = "world";  // 全局 ENU 坐标系
    global_cloud_msg.header.stamp = ros::Time::now();
    global_map_pub_.publish(global_cloud_msg);
  }
}

localization_core::Asensing LocationNode::ToCore(const autodrive_msgs::HUAT_Asensing &msg)
{
  localization_core::Asensing out;
  out.latitude = msg.latitude;
  out.longitude = msg.longitude;
  out.altitude = msg.altitude;
  out.north_velocity = msg.north_velocity;
  out.east_velocity = msg.east_velocity;
  out.ground_velocity = msg.ground_velocity;
  out.roll = msg.roll;
  out.pitch = msg.pitch;
  out.azimuth = msg.azimuth;
  out.x_angular_velocity = msg.x_angular_velocity;
  out.y_angular_velocity = msg.y_angular_velocity;
  out.z_angular_velocity = msg.z_angular_velocity;
  out.x_acc = msg.x_acc;
  out.y_acc = msg.y_acc;
  out.z_acc = msg.z_acc;
  return out;
}

localization_core::CarState LocationNode::ToCore(const autodrive_msgs::HUAT_CarState &msg)
{
  localization_core::CarState out;
  out.car_state.x = msg.car_state.x;
  out.car_state.y = msg.car_state.y;
  out.car_state.theta = msg.car_state.theta;
  out.car_state_front.x = msg.car_state_front.x;
  out.car_state_front.y = msg.car_state_front.y;
  out.car_state_front.z = msg.car_state_front.z;
  out.car_state_rear.x = msg.car_state_rear.x;
  out.car_state_rear.y = msg.car_state_rear.y;
  out.car_state_rear.z = msg.car_state_rear.z;
  out.V = msg.V;
  out.W = msg.W;
  out.A = msg.A;
  return out;
}

void LocationNode::ToRos(const localization_core::CarState &state, autodrive_msgs::HUAT_CarState *out)
{
  if (!out)
  {
    return;
  }
  out->car_state.x = state.car_state.x;
  out->car_state.y = state.car_state.y;
  out->car_state.theta = state.car_state.theta;
  out->car_state_front.x = state.car_state_front.x;
  out->car_state_front.y = state.car_state_front.y;
  out->car_state_front.z = state.car_state_front.z;
  out->car_state_rear.x = state.car_state_rear.x;
  out->car_state_rear.y = state.car_state_rear.y;
  out->car_state_rear.z = state.car_state_rear.z;
  out->V = static_cast<float>(state.V);
  out->W = static_cast<float>(state.W);
  out->A = static_cast<float>(state.A);
}

localization_core::ConeDetections LocationNode::ToCore(const autodrive_msgs::HUAT_ConeDetections &msg)
{
  localization_core::ConeDetections out;
  out.points.reserve(msg.points.size());
  for (const auto &pt : msg.points)
  {
    localization_core::Point3 p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    out.points.push_back(p);
  }
  return out;
}

void LocationNode::ToRos(const localization_core::ConeMap &map, autodrive_msgs::HUAT_ConeMap *out)
{
  if (!out)
  {
    return;
  }
  out->cone.clear();
  out->cone.reserve(map.cones.size());
  for (const auto &cone : map.cones)
  {
    autodrive_msgs::HUAT_Cone msg;
    msg.id = cone.id;
    msg.position_global.x = cone.position_global.x;
    msg.position_global.y = cone.position_global.y;
    msg.position_global.z = cone.position_global.z;
    msg.position_baseLink.x = cone.position_base_link.x;
    msg.position_baseLink.y = cone.position_base_link.y;
    msg.position_baseLink.z = cone.position_base_link.z;
    msg.confidence = cone.confidence;
    msg.type = cone.type;
    out->cone.push_back(msg);
  }
}

}  // namespace localization_ros
