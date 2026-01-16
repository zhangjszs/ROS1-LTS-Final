#ifndef UTILITY_HPP_H // #pragma once
#define UTILITY_HPP_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_set>
#include <imu_subscriber.hpp>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Float64MultiArray.h"
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/time.h>
// #include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unistd.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <boost/thread/thread.hpp>
#include <distortion_adjust.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
// #include<
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/common/transforms.h>
#include <common_msgs/Cone.h>
#include <common_msgs/ins_p2.h>
#include "perf_stats.hpp"

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace lidar_distortion;
#define Eight_Ring 1
#define Linear_Acc 2
#define High_Speed_Tracking 3
using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;
typedef pcl::PointXYZI PointType;

class lidar_cluster
{

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_point_cloud_;
  ros::Subscriber sub_ins_;
  ros::Publisher markerPub;
  ros::Publisher markerPubAll;
  ros::Publisher pub_filtered_points_;
  ros::Publisher pub_filtered_points__;
  ros::Publisher pub_filtered_points___;
  ros::Publisher pub_filtered_points____;
  ros::Publisher cone_position;
  ros::Publisher lidarClusterPublisher_;
  ros::Publisher adjust_check_front;
  ros::Publisher adjust_check_back;
  ros::Publisher skidpad_detection;
  ros::Publisher logging_pub;

  nav_msgs::Path ros_path_;
  geometry_msgs::PoseStamped pose;
  Eigen::Matrix4f transMat_;
  Eigen::Matrix4f m;

  vector<double> tmp_comparison_min_x_first_v;
  vector<double> tmp_comparison_min_x_second_v;
  vector<double> tmp_comparison_y_first_v;
  vector<double> tmp_comparison_y_second_v;

  double tmp_comparison_min_x_first;
  double tmp_comparison_min_x_second;
  double tmp_comparison_y_first = 0;
  double tmp_comparison_y_second = 0;

  int filter_centroid = 0;
  double base_x = 1.0;
  double base_y = 0;
  bool matchFlag = true;
  std::string no_ground_topic, ground_topic, all_points_topic, input_topic;
  ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;
  std::string point_topic_;
  bool getPointClouds = false;
  bool inverse_flag; // matrix.inverse()???
  int i;
  int useED = 1;
  float ed[4] = {0};
  int marker_id = 0;
  int sensor_model_;
  double sensor_height_, clip_height_, // configuration settings
      min_distance_, max_distance_,
      z_up_, z_down_;
  int vis_;
  double min_height_, max_height_, min_area_, max_area_, max_box_altitude_; // for confidence calculation
  double accel_x_max_, accel_x_min_, accel_y_max_, accel_y_min_,
      track_x_max_, track_x_min_, track_y_max_, track_y_min_,
      skid_x_max_, skid_x_min_, skid_y_max_, skid_y_min_;
  int num_seg_ = 1;
  int road_type_ = -1;
  int num_iter_, num_lpr_;
  double th_seeds_, th_dist_;
  double euc;
  float intensity, intensity_min, intensity_max, intensity_mean;
  int intensity_count;
  std::string str_range_, str_seg_distance_;
  geometry_msgs::Point p;
  float d_, th_dist_d_;
  MatrixXf normal_;

  sensor_msgs::PointCloud2 pub_pc;
  sensor_msgs::PointCloud2 cluster_for_publish;
  bool vis_init_status;

public:
  pcl::PointCloud<PointType>::Ptr g_seeds_pc;
  pcl::PointCloud<PointType>::Ptr g_ground_pc;
  pcl::PointCloud<PointType>::Ptr g_not_ground_pc;
  pcl::PointCloud<PointType>::Ptr g_all_pc;
  pcl::PointCloud<PointType>::Ptr current_pc_ptr;
  pcl::PointCloud<PointType>::Ptr distortion_out; // distortion_Adjust
  pcl::PointCloud<PointType>::Ptr cloud_filtered;
  pcl::PointCloud<PointType>::Ptr StatisticalOutlierFilter;
  pcl::PointCloud<PointType>::Ptr skidpad_detection_pc;
  sensor_msgs::PointCloud out_pc;

  sensor_msgs::PointCloud2 in_pc;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray marker_array_all;
  visualization_msgs::Marker bbox_marker;
  visualization_msgs::Marker euc_marker, intensity_max_marker, intensity_min_marker, intensity_mean_marker;
  // DistortionAdjust distadjust;
  bool hasIMu = false;
  std::mutex lidar_mutex;
  std::deque<IMUData> unsynced_imu_;

  std::vector<double> dis_range;
  std::vector<double> seg_distances;

  // eval options
  const bool eval = false;
  std::string eval_file = "/home/adams/postsynced_msf/evaluation/lidar_cluster/lidar_cluster_single_linear.txt";
  // std::string eval_file = "/home/adams/postsynced_msf/evaluation/lidar_cluster/lidar_cluster_single_track_time_1.txt";
  std::ofstream ofs;
  int frame_count = 0;
  PerfStats perf_stats_;
  size_t last_cluster_count_ = 0;
  size_t last_cluster_pub_bytes_ = 0;

public:
  void init();

  // visualization
  void vis_for_marker(const PointType max, const PointType min, float euc, float intensity_max, float intensity_min, float intensity_mean, bool type);
  bool vis_init();

  // callback function
  void point_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

  // ground seg
  void SACSMethod(pcl::PointCloud<PointType>::Ptr &cloud_filtered, int maxIterations, float distanceThreshold, float planarThreshold);
  void SVDGroundMethod(pcl::PointCloud<PointType>::Ptr &cloud_filtered, float distanceThreshold, float seedThreshold, int Nlpr);
  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<PointType> &p_sorted);
  void ground_segmentation(const pcl::PointCloud<PointType>::Ptr &in_pc,
                           pcl::PointCloud<PointType>::Ptr &g_not_ground_pc);
  void RANSAC(const pcl::PointCloud<PointType>::Ptr &cloud_filtered,
              pcl::PointCloud<PointType>::Ptr &cloud_filtered_out, int maxIterations, float thre);
  void PROSAC(const pcl::PointCloud<PointType>::Ptr &cloud_filtered,
              pcl::PointCloud<PointType>::Ptr &cloud_filtered_out, int maxIterations, float thre);

  // filter
  void PassThrough(pcl::PointCloud<PointType>::Ptr &cloud_filtered, pcl::PointCloud<PointType>::Ptr &StatisticalOutlierFilter);
  void guidedFilter(const pcl::PointCloud<PointType>::Ptr &cloud_filtered, pcl::PointCloud<PointType>::Ptr &cloud_filtered_out, unsigned int k, float eps);
  void octreeRemovePoints(pcl::PointCloud<PointType>::Ptr &cloud, pcl::PointCloud<PointType>::Ptr &output);
  // cluster
  void EucClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<pcl::PointIndices> &cluster_indices, const double &max_cluster_dis);
  void EuclideanAdaptiveClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<std::vector<pcl::PointIndices>> &cluster_indices,
                                      std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array);
  void splitString(const std::string &in_string, std::vector<double> &out_array);
  void clusterMethod16();
  void clusterMethod32();

  void runAlgorithm();
  void publishToCheckAdjust(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out);
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<DistortionAdjust> disAdjust;
  double cloud_time;
  lidar_cluster(ros::NodeHandle node,
                ros::NodeHandle private_nh);
  ~lidar_cluster();
  void Spin();

  double getConfidence(PointType max, PointType min, Eigen::Vector4f centroid);
};
#endif
