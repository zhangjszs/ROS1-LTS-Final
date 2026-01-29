#include <localization_ros/location_node.hpp>

#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>

namespace localization_ros {

namespace {
template <typename T>
bool LoadParam(ros::NodeHandle &pnh, ros::NodeHandle &nh, const std::string &key, T &out, const T &default_value)
{
  if (pnh.param(key, out, default_value))
  {
    return true;
  }
  if (nh.param(key, out, default_value))
  {
    return true;
  }
  return false;
}

geometry_msgs::Quaternion YawToQuaternion(double yaw)
{
  geometry_msgs::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}
}  // namespace

LocationNode::LocationNode(ros::NodeHandle &nh)
    : nh_(nh),
      pnh_("~"),
      mapper_(params_)
{
  loadParameters();

  mapper_.Configure(params_);
  mapper_.SetDataDirectory(ros::package::getPath("localization_ros"));

  if (use_external_carstate_)
  {
    carstate_sub_ = nh_.subscribe<autodrive_msgs::HUAT_CarState>(carstate_in_topic_, 10, &LocationNode::carstateCallback, this);
  }
  else
  {
    ins_sub_ = nh_.subscribe<autodrive_msgs::HUAT_InsP2>(ins_topic_, 10, &LocationNode::imuCallback, this);
  }

  cone_sub_ = nh_.subscribe<autodrive_msgs::HUAT_ConeDetections>(cone_topic_, 10, &LocationNode::coneCallback, this);

  carstate_pub_ = nh_.advertise<autodrive_msgs::HUAT_CarState>(carstate_out_topic_, 10);
  map_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeMap>(cone_map_topic_, 10);
  global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_, 10);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

}

void LocationNode::loadParameters()
{
  if (!LoadParam(pnh_, nh_, "length/lidarToIMUDist", params_.lidar_to_imu_dist, 1.87))
  {
    ROS_WARN_STREAM("Did not load lidarToIMUDist. Standard value is: " << params_.lidar_to_imu_dist);
  }
  if (!LoadParam(pnh_, nh_, "length/frontToIMUdistanceX", params_.front_to_imu_x, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceX. Standard value is: " << params_.front_to_imu_x);
  }
  if (!LoadParam(pnh_, nh_, "length/frontToIMUdistanceY", params_.front_to_imu_y, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceY. Standard value is " << params_.front_to_imu_y);
  }
  if (!LoadParam(pnh_, nh_, "length/frontToIMUdistanceZ", params_.front_to_imu_z, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceZ. Standard value is: " << params_.front_to_imu_z);
  }
  if (!LoadParam(pnh_, nh_, "length/rearToIMUdistanceX", params_.rear_to_imu_x, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceX. Standard value is " << params_.rear_to_imu_x);
  }
  if (!LoadParam(pnh_, nh_, "length/rearToIMUdistanceY", params_.rear_to_imu_y, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceY. Standard value is " << params_.rear_to_imu_y);
  }
  if (!LoadParam(pnh_, nh_, "length/rearToIMUdistanceZ", params_.rear_to_imu_z, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceZ. Standard value is " << params_.rear_to_imu_z);
  }

  if (!LoadParam(pnh_, nh_, "topics/ins", ins_topic_, std::string("sensors/ins")))
  {
    ROS_WARN_STREAM("Did not load topics/ins. Standard value is: " << ins_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/car_state_in", carstate_in_topic_, std::string("localization/car_state")))
  {
    ROS_WARN_STREAM("Did not load topics/car_state_in. Standard value is: " << carstate_in_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/cone", cone_topic_, std::string("perception/lidar_cluster/detections")))
  {
    ROS_WARN_STREAM("Did not load topics/cone. Standard value is: " << cone_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/car_state_out", carstate_out_topic_, std::string("localization/car_state")))
  {
    ROS_WARN_STREAM("Did not load topics/car_state_out. Standard value is: " << carstate_out_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/cone_map", cone_map_topic_, std::string("localization/cone_map")))
  {
    ROS_WARN_STREAM("Did not load topics/cone_map. Standard value is: " << cone_map_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/global_map", global_map_topic_, std::string("localization/global_map")))
  {
    ROS_WARN_STREAM("Did not load topics/global_map. Standard value is: " << global_map_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/pose", pose_topic_, std::string("localization/pose")))
  {
    ROS_WARN_STREAM("Did not load topics/pose. Standard value is: " << pose_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/odom", odom_topic_, std::string("localization/odom")))
  {
    ROS_WARN_STREAM("Did not load topics/odom. Standard value is: " << odom_topic_);
  }
  if (!LoadParam(pnh_, nh_, "frames/world", world_frame_, std::string("world")))
  {
    ROS_WARN_STREAM("Did not load frames/world. Standard value is: " << world_frame_);
  }
  if (!LoadParam(pnh_, nh_, "frames/base_link", base_link_frame_, std::string("base_link")))
  {
    ROS_WARN_STREAM("Did not load frames/base_link. Standard value is: " << base_link_frame_);
  }
  if (!LoadParam(pnh_, nh_, "use_external_carstate", use_external_carstate_, false))
  {
    ROS_WARN_STREAM("Did not load use_external_carstate. Standard value is: " << (use_external_carstate_ ? "true" : "false"));
  }
}

void LocationNode::publishState(const localization_core::CarState &state, const ros::Time &stamp, bool publish_carstate)
{
  const geometry_msgs::Quaternion orientation = YawToQuaternion(state.car_state.theta);
  double v_forward = 0.0;
  double yaw_rate = 0.0;
  if (has_last_state_ && stamp == last_stamp_)
  {
    // Avoid duplicate TF publishing
    return;
  }

  if (has_last_state_)
  {
    const double dt = (stamp - last_stamp_).toSec();
    if (dt > 1e-3)
    {
      const double dx = state.car_state.x - last_state_.car_state.x;
      const double dy = state.car_state.y - last_state_.car_state.y;
      const double yaw = state.car_state.theta;
      v_forward = (std::cos(yaw) * dx + std::sin(yaw) * dy) / dt;
      const double dtheta = std::atan2(std::sin(state.car_state.theta - last_state_.car_state.theta),
                                       std::cos(state.car_state.theta - last_state_.car_state.theta));
      yaw_rate = dtheta / dt;
    }
  }

  if (publish_carstate)
  {
    autodrive_msgs::HUAT_CarState out;
    ToRos(state, &out);
    out.header.stamp = stamp;
    out.header.frame_id = world_frame_;
    carstate_pub_.publish(out);
  }

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = world_frame_;
  pose_msg.pose.position.x = state.car_state.x;
  pose_msg.pose.position.y = state.car_state.y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation = orientation;
  pose_pub_.publish(pose_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = world_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.pose.pose = pose_msg.pose;
  odom_msg.twist.twist.linear.x = v_forward;
  odom_msg.twist.twist.angular.z = yaw_rate;
  odom_pub_.publish(odom_msg);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = world_frame_;
  tf_msg.child_frame_id = base_link_frame_;
  tf_msg.transform.translation.x = state.car_state.x;
  tf_msg.transform.translation.y = state.car_state.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = orientation;
  tf_broadcaster_.sendTransform(tf_msg);

  last_state_ = state;
  last_stamp_ = stamp;
  has_last_state_ = true;
}

void LocationNode::imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr &msg)
{
  localization_core::Asensing core_msg = ToCore(*msg);
  localization_core::CarState state;

  if (mapper_.UpdateFromIns(core_msg, &state))
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    publishState(state, stamp, true);
  }
}

void LocationNode::carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msg)
{
  mapper_.UpdateFromCarState(ToCore(*msg));
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  const bool publish_carstate = carstate_out_topic_ != carstate_in_topic_;
  publishState(mapper_.car_state(), stamp, publish_carstate);
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

  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

  localization_core::ConeDetections detections = ToCore(*msg);
  localization_core::ConeMap map;
  localization_core::PointCloudPtr cloud;

  if (!mapper_.UpdateFromCones(detections, &map, &cloud))
  {
    return;
  }

  autodrive_msgs::HUAT_ConeMap out_map;
  ToRos(map, &out_map);
  out_map.header.stamp = stamp;
  out_map.header.frame_id = world_frame_;
  map_pub_.publish(out_map);

  if (cloud)
  {
    sensor_msgs::PointCloud2 global_cloud_msg;
    pcl::toROSMsg(*cloud, global_cloud_msg);
    global_cloud_msg.header.frame_id = world_frame_;
    global_cloud_msg.header.stamp = stamp;
    global_map_pub_.publish(global_cloud_msg);
  }
}

localization_core::Asensing LocationNode::ToCore(const autodrive_msgs::HUAT_InsP2 &msg)
{
  localization_core::Asensing out;

  // 位置 (WGS84)
  out.latitude = msg.Lat;
  out.longitude = msg.Lon;
  out.altitude = msg.Altitude;

  // 速度 (m/s, NED坐标系)
  // HUAT_InsP2.Vd 向下为正
  out.north_velocity = msg.Vn;
  out.east_velocity = msg.Ve;
  out.ground_velocity = msg.Vd;  // Vd(向下)

  // 姿态角 (度)
  out.roll = msg.Roll;
  out.pitch = msg.Pitch;
  out.azimuth = msg.Heading;

  // 角速度 (rad/s, FRD车体系)
  // HUAT_InsP2中已经是 rad/s，直接使用
  out.x_angular_velocity = msg.gyro_x;
  out.y_angular_velocity = msg.gyro_y;
  out.z_angular_velocity = msg.gyro_z;

  // 加速度 (m/s², FRD车体系)
  // HUAT_InsP2中已经是 m/s²，直接使用
  out.x_acc = msg.acc_x;
  out.y_acc = msg.acc_y;
  out.z_acc = msg.acc_z;

  // 质量指标
  out.status = msg.Status;
  out.nsv1 = msg.NSV1;
  out.nsv2 = msg.NSV2;
  out.age = msg.Age;

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
