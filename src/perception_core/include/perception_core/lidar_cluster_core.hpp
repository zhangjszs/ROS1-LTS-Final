#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <perception_core/distortion_adjust.hpp>
#include <perception_core/imu_data.hpp>

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using PointType = pcl::PointXYZI;

#define Eight_Ring 1
#define Linear_Acc 2
#define High_Speed_Tracking 3

struct ConeDetection
{
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::PointXYZ centroid;
  double confidence = 0.0;
  double distance = 0.0;
  pcl::PointCloud<PointType>::Ptr cluster;
  bool is_cone = false;
};

struct LidarClusterOutput
{
  pcl::PointCloud<PointType>::Ptr passthrough;
  pcl::PointCloud<PointType>::Ptr not_ground;
  pcl::PointCloud<PointType>::Ptr cones_cloud;
  pcl::PointCloud<PointType>::Ptr skidpad_detection;
  std::vector<ConeDetection> cones;
  std::vector<ConeDetection> non_cones;
  std::size_t input_points = 0;
  std::size_t total_clusters = 0;
  double t_pass_ms = 0.0;
  double t_ground_ms = 0.0;
  double t_cluster_ms = 0.0;
  double t_total_ms = 0.0;
};

struct LidarClusterConfig
{
  double clip_height = 2.0;
  double sensor_height = 0.135;
  double min_distance = 0.1;
  double max_distance = 100.0;
  int sensor_model = 16;
  int num_iter = 3;
  int num_lpr = 5;
  double th_seeds = 0.03;
  double th_dist = 0.03;
  int road_type = 2;
  double z_up = 0.7;
  double z_down = -1.0;
  int vis = 0;
  std::string str_range = "15,30,45,60";
  std::string str_seg_distance = "0.5,1.1,1.6,2.1,2.6";

  double min_height = -1;
  double max_height = -1;
  double min_area = -1;
  double max_area = -1;
  double max_box_altitude = 0;

  double accel_x_max = 0;
  double accel_x_min = 0;
  double accel_y_max = 0;
  double accel_y_min = 0;

  double track_x_max = 0;
  double track_x_min = 0;
  double track_y_max = 0;
  double track_y_min = 0;

  double skid_x_max = 0;
  double skid_x_min = 0;
  double skid_y_max = 0;
  double skid_y_min = 0;
};

class lidar_cluster
{
public:
  lidar_cluster();
  ~lidar_cluster();

  void Configure(const LidarClusterConfig &config);
  void SetInputCloud(const pcl::PointCloud<PointType>::ConstPtr &cloud, uint32_t seq);
  bool Process(LidarClusterOutput *output);

private:
  void init();

  // ground seg
  void SACSMethod(pcl::PointCloud<PointType>::Ptr &cloud_filtered, int maxIterations, float distanceThreshold, float planarThreshold);
  void SVDGroundMethod(pcl::PointCloud<PointType>::Ptr &cloud_filtered, float distanceThreshold, float seedThreshold, int Nlpr);
  void estimate_plane_();
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
  void clusterMethod16(LidarClusterOutput *output);
  void clusterMethod32(LidarClusterOutput *output);

  double getConfidence(PointType max, PointType min, Eigen::Vector4f centroid);

private:
  LidarClusterConfig config_;

  std::vector<double> tmp_comparison_min_x_first_v;
  std::vector<double> tmp_comparison_min_x_second_v;
  std::vector<double> tmp_comparison_y_first_v;
  std::vector<double> tmp_comparison_y_second_v;

  double tmp_comparison_min_x_first;
  double tmp_comparison_min_x_second;
  double tmp_comparison_y_first = 0;
  double tmp_comparison_y_second = 0;

  int filter_centroid = 0;
  double base_x = 1.0;
  double base_y = 0;
  bool matchFlag = true;

  bool getPointClouds = false;
  bool inverse_flag = false;
  int i;
  int useED = 1;
  float ed[4] = {0};
  int marker_id = 0;
  int sensor_model_ = 16;
  double sensor_height_ = 0.135;
  double clip_height_ = 2.0; // configuration settings
  double min_distance_ = 0.1;
  double max_distance_ = 100.0;
  double z_up_ = 0.7;
  double z_down_ = -1.0;
  int vis_ = 0;
  double min_height_ = -1;
  double max_height_ = -1;
  double min_area_ = -1;
  double max_area_ = -1;
  double max_box_altitude_ = 0;
  double accel_x_max_ = 0;
  double accel_x_min_ = 0;
  double accel_y_max_ = 0;
  double accel_y_min_ = 0;
  double track_x_max_ = 0;
  double track_x_min_ = 0;
  double track_y_max_ = 0;
  double track_y_min_ = 0;
  double skid_x_max_ = 0;
  double skid_x_min_ = 0;
  double skid_y_max_ = 0;
  double skid_y_min_ = 0;
  int num_seg_ = 1;
  int road_type_ = -1;
  int num_iter_ = 0;
  int num_lpr_ = 0;
  double th_seeds_ = 0.0;
  double th_dist_ = 0.0;
  double euc = 0.0;
  float intensity = 0.0f;
  float intensity_min = 0.0f;
  float intensity_max = 0.0f;
  float intensity_mean = 0.0f;
  int intensity_count = 0;
  std::string str_range_;
  std::string str_seg_distance_;
  float d_ = 0.0f;
  float th_dist_d_ = 0.0f;
  MatrixXf normal_;

  bool vis_init_status = false;

  pcl::PointCloud<PointType>::Ptr g_seeds_pc;
  pcl::PointCloud<PointType>::Ptr g_ground_pc;
  pcl::PointCloud<PointType>::Ptr g_not_ground_pc;
  pcl::PointCloud<PointType>::Ptr g_all_pc;
  pcl::PointCloud<PointType>::Ptr current_pc_ptr;
  pcl::PointCloud<PointType>::Ptr distortion_out; // distortion_Adjust
  pcl::PointCloud<PointType>::Ptr cloud_filtered;
  pcl::PointCloud<PointType>::Ptr StatisticalOutlierFilter;
  pcl::PointCloud<PointType>::Ptr skidpad_detection_pc;

  std::mutex lidar_mutex;
  std::deque<IMUData> unsynced_imu_;

  std::vector<double> dis_range;
  std::vector<double> seg_distances;

  const bool eval = false;
  std::string eval_file = "/home/adams/postsynced_msf/evaluation/lidar_cluster/lidar_cluster_single_linear.txt";
  std::ofstream ofs;
  int frame_count = 0;
  double cloud_time = 0.0;
  std::size_t last_cluster_count_ = 0;
};
