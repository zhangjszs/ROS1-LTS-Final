#pragma once

#include "fsd_visualization/viz_config.hpp"

#include <ros/ros.h>

#include <deque>

#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_SimState.h>
#include <autodrive_msgs/HUAT_VehicleCmd.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/MarkerArray.h>

namespace fsd_viz {

class VehicleVisualizer {
 public:
  VehicleVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void clearTrail();

 private:
  void carStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr& msg);
  void simStateCallback(const autodrive_msgs::HUAT_SimState::ConstPtr& msg);
  void vehicleCmdCallback(const autodrive_msgs::HUAT_VehicleCmd::ConstPtr& msg);

  // 使用 geometry_msgs::Pose2D 作为参数（x, y, theta）
  visualization_msgs::Marker createBodyMarker(const geometry_msgs::Pose2D& state);
  visualization_msgs::Marker createArrowMarker(const geometry_msgs::Pose2D& state);
  std::vector<visualization_msgs::Marker> createWheelMarkers(const geometry_msgs::Pose2D& state);
  visualization_msgs::Marker createTrailMarker(const ros::Time& stamp);

  // 新增：速度矢量和转向可视化
  visualization_msgs::Marker createVelocityMarker(const geometry_msgs::Pose2D& state, double vx,
                                                  double vy);
  visualization_msgs::Marker createSteeringMarker(const geometry_msgs::Pose2D& state,
                                                  double steering_angle);
  visualization_msgs::Marker createStateTextMarker(const geometry_msgs::Pose2D& state,
                                                   double speed);

  ros::Subscriber sub_car_state_;
  ros::Subscriber sub_sim_state_;
  ros::Subscriber sub_vehicle_cmd_;
  ros::Publisher pub_markers_;

  std::string car_state_topic_;
  std::string sim_state_topic_;
  std::string vehicle_cmd_topic_;
  std::string markers_topic_;

  std::string frame_id_;
  int trail_max_size_;
  bool show_trail_;
  bool show_velocity_;
  bool show_steering_;
  bool use_mesh_;

  // Runtime color overrides (loaded from ROS parameters)
  std::array<float, 4> vehicle_body_color_;
  std::array<float, 4> vehicle_arrow_color_;
  std::array<float, 4> vehicle_wheel_color_;
  std::array<float, 4> vehicle_trail_color_;
  std::array<float, 4> vehicle_velocity_color_;
  std::array<float, 4> vehicle_steering_color_;

  // 轨迹历史
  std::deque<geometry_msgs::Point> trail_points_;

  // 仿真状态缓存
  double cached_vx_ = 0.0;
  double cached_vy_ = 0.0;
  double cached_steering_ = 0.0;
  bool has_sim_state_ = false;

  // 控制指令缓存
  uint8_t cached_cmd_steering_ = 110;  // 中位
  uint8_t cached_cmd_pedal_ = 0;
  uint8_t cached_cmd_brake_ = 0;
  bool has_vehicle_cmd_ = false;
};

}  // namespace fsd_viz
