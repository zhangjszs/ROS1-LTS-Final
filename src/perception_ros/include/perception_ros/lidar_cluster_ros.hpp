#pragma once

#include <string>

#include <ros/ros.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

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

  bool visInit();
  void visForMarker(const PointType max,
                    const PointType min,
                    float euc,
                    float intensity_max,
                    float intensity_min,
                    float intensity_mean,
                    bool type);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_point_cloud_;

  ros::Publisher passthrough_pub_;
  ros::Publisher no_ground_pub_;
  ros::Publisher cones_pub_;
  ros::Publisher detections_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_pub_all_;

  LidarClusterConfig config_;
  lidar_cluster core_;
  LidarClusterOutput output_;
  PerfStats perf_stats_;

  std_msgs::Header last_header_;
  bool got_cloud_ = false;
  uint32_t last_seq_ = 0;              // P0: 最新接收的点云序列号
  uint32_t last_processed_seq_ = 0;    // P0: 最后处理的点云序列号
  double max_cloud_age_ = 0.5;         // P0: 帧间隔警告阈值 (秒)，10Hz点云正常间隔0.1s

  int vis_ = 0;
  int sensor_model_ = 16;

  std::string input_topic_;
  std::string passthrough_topic_;
  std::string no_ground_topic_;
  std::string cones_topic_;
  std::string detections_topic_;
  std::string markers_topic_;
  std::string markers_all_topic_;

  bool perf_enabled_ = true;
  size_t perf_window_ = 300;
  size_t perf_log_every_ = 30;

  // IMU Distortion Compensation (解耦后的独立模块)
  std::unique_ptr<DistortionCompensator> compensator_;

  visualization_msgs::MarkerArray marker_array_;
  visualization_msgs::MarkerArray marker_array_all_;
  visualization_msgs::Marker euc_marker_;
  visualization_msgs::Marker bbox_marker_;
  visualization_msgs::Marker intensity_max_marker_;
  geometry_msgs::Point p_;
  int marker_id_ = 0;
  bool vis_init_status_ = false;
};

}  // namespace perception_ros
