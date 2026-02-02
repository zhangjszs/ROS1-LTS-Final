#pragma once

#include <string>

#include <ros/ros.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <perception_core/lidar_cluster_core.hpp>
#include <perception_ros/perf_stats.hpp>
#include <perception_ros/distortion_compensator.hpp>

namespace perception_ros {

class LidarClusterRos {
 public:
  LidarClusterRos(ros::NodeHandle nh, ros::NodeHandle private_nh);

  void RunOnce();

 private:
  void loadParams();
  void pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void publishOutput(const LidarClusterOutput &output);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_point_cloud_;

  ros::Publisher passthrough_pub_;
  ros::Publisher no_ground_pub_;
  ros::Publisher cones_pub_;
  ros::Publisher detections_pub_;

  LidarClusterConfig config_;
  lidar_cluster core_;
  LidarClusterOutput output_;
  PerfStats perf_stats_;

  std_msgs::Header last_header_;
  bool got_cloud_ = false;
  uint32_t last_seq_ = 0;              // P0: 最新接收的点云序列号
  uint32_t last_processed_seq_ = 0;    // P0: 最后处理的点云序列号
  double max_cloud_age_ = 0.5;         // P0: 帧间隔警告阈值 (秒)，10Hz点云正常间隔0.1s
  std::mutex seq_mutex_;               // P0: 保护序列号访问
  ros::Time last_process_stamp_;       // P0: 上次处理时间戳

  int sensor_model_ = 16;

  std::string input_topic_;
  std::string passthrough_topic_;
  std::string no_ground_topic_;
  std::string cones_topic_;
  std::string detections_topic_;

  bool perf_enabled_ = true;
  size_t perf_window_ = 300;
  size_t perf_log_every_ = 30;

  // IMU Distortion Compensation (解耦后的独立模块)
  std::unique_ptr<DistortionCompensator> compensator_;
};

}  // namespace perception_ros
