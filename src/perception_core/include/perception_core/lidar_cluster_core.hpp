#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <memory>
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
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <perception_core/patchworkpp.hpp>
#include <perception_core/cluster_feature_extractor.hpp>
#include <perception_core/confidence_scorer.hpp>

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using PointType = pcl::PointXYZI;

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
  struct ConfidenceConfig
  {
    double min_height = 0.15;
    double max_height = 0.5;
    double min_area = 0.01;
    double max_area = 0.15;
    double max_box_altitude = 0.5;
    double min_aspect_ratio = 1.5;
    double min_verticality = 0.8;
    double min_density_near = 50.0;
    double min_density_far = 10.0;
    double distance_threshold = 5.0;
    double min_intensity_mean = 30.0;
    double weight_size = 0.3;
    double weight_shape = 0.25;
    double weight_density = 0.2;
    double weight_intensity = 0.15;
    double weight_position = 0.1;
    bool enable_model_fitting = true;
    double model_fit_bonus = 0.2;
    double model_fit_penalty = 0.15;
  };

  struct RansacConfig
  {
    int num_iter = 3;
    int num_lpr = 5;
    double th_seeds = 0.03;
    double th_dist = 0.03;
  };

  struct PatchworkppConfig
  {
    bool enable_rnr = true;
    bool enable_rvpf = true;
    bool enable_tgr = true;

    int num_iter = 3;
    int num_lpr = 20;
    int num_min_pts = 10;
    int num_zones = 4;
    int num_rings_of_interest = 4;

    double rnr_ver_angle_thr = -15.0;
    double rnr_intensity_thr = 0.2;

    double th_seeds = 0.125;
    double th_dist = 0.125;
    double th_seeds_v = 0.25;
    double th_dist_v = 0.1;
    double max_range = 80.0;
    double min_range = 2.7;
    double uprightness_thr = 0.707;
    double adaptive_seed_selection_margin = -1.2;

    std::vector<int> num_sectors_each_zone = {16, 32, 54, 32};
    std::vector<int> num_rings_each_zone = {2, 4, 4, 4};

    int max_flatness_storage = 1000;
    int max_elevation_storage = 1000;

    std::vector<double> elevation_thr = {0, 0, 0, 0};
    std::vector<double> flatness_thr = {0, 0, 0, 0};
  };

  struct RoiRange
  {
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;
  };

  struct RoiConfig
  {
    std::string mode = "track";  // skidpad | accel | track | custom
    bool use_point_clip = false; // only for skidpad
    double z_min = -0.2;
    double z_max = 1.0;
    RoiRange skidpad{0.0, 10.0, -3.0, 3.0};
    RoiRange accel{0.0, 100.0, -3.0, 3.0};
    RoiRange track{0.0, 20.0, -3.0, 3.0};
    RoiRange custom{0.0, 20.0, -3.0, 3.0};
  };

  struct FilterConfig
  {
    struct SorConfig
    {
      bool enable = false;
      int mean_k = 50;
      double stddev_mul = 1.0;
    };

    struct VoxelConfig
    {
      bool enable = true;
      double leaf_size = 0.05;
    };

    struct AdaptiveVoxelConfig
    {
      bool enable = false;
      double leaf_size = 0.1;
      int density_thr = 50;
    };

    // 距离自适应体素滤波：近处大体素，远处小体素
    struct DistanceAdaptiveVoxelConfig
    {
      bool enable = false;
      double near_leaf = 0.08;       // 近距离体素大小 (m)
      double far_leaf = 0.03;        // 远距离体素大小 (m)，0表示不降采样
      double dist_threshold = 10.0;  // 距离分界点 (m)
    };

    // 强度滤波：去除低强度噪声点
    struct IntensityConfig
    {
      bool enable = false;
      float min_intensity = 5.0;     // 最小强度阈值
    };

    SorConfig sor;
    VoxelConfig voxel;
    AdaptiveVoxelConfig adaptive_voxel;
    DistanceAdaptiveVoxelConfig distance_adaptive_voxel;
    IntensityConfig intensity;
    bool use_cropbox = true;  // 用CropBox替代多次PassThrough
  };

  // 聚类配置
  struct ClusterConfig
  {
    // 聚类方法选择
    std::string method = "euclidean";  // euclidean | dbscan

    // 距离分段（支持可变数量）
    std::vector<double> distance_segments = {5.0, 10.0, 15.0, 20.0};  // 分段距离阈值
    std::vector<double> cluster_tolerance = {0.15, 0.3, 0.5, 0.6, 0.6};  // 各段聚类距离

    // 自适应聚类大小（根据距离调整）
    struct AdaptiveClusterSize
    {
      bool enable = true;
      // 近距离参数（< distance_segments[0]）
      int near_min_size = 3;      // 近距离最小聚类点数（过滤噪声）
      int near_max_size = 100;    // 近距离最大聚类点数（锥桶点多）
      // 远距离参数（> distance_segments.back()）
      int far_min_size = 1;       // 远距离最小聚类点数（接受稀疏锥桶）
      int far_max_size = 30;      // 远距离最大聚类点数
      // 中间距离线性插值
    };

    AdaptiveClusterSize adaptive_size;

    // 固定聚类大小（adaptive_size.enable=false时使用）
    int min_cluster_size = 1;
    int max_cluster_size = 50;

    // DBSCAN 参数
    struct DbscanConfig
    {
      double eps = 0.3;           // 邻域半径
      int min_pts = 3;            // 核心点最小邻居数
      bool adaptive_eps = true;   // 是否根据距离自适应调整eps
      double eps_near = 0.15;     // 近距离eps
      double eps_far = 0.5;       // 远距离eps
    };
    DbscanConfig dbscan;

    // VLP-16 专用参数（sensor_model=16时使用）
    struct Vlp16Config
    {
      double cluster_tolerance = 0.3;  // 聚类距离阈值
      double max_bbox_x = 0.4;         // 锥桶bbox最大X尺寸
      double max_bbox_y = 0.4;         // 锥桶bbox最大Y尺寸
      double max_bbox_z = 0.5;         // 锥桶bbox最大Z尺寸
    };
    Vlp16Config vlp16;

    // point_clip 参数（skidpad模式使用）
    struct PointClipConfig
    {
      double min_distance = 1.0;   // 最小距离
      double max_distance = 15.0;  // 最大距离
    };
    PointClipConfig point_clip;
  };

  double sensor_height = 0.135;
  int sensor_model = 16;
  int road_type = 2;
  std::string ground_method = "ransac";
  double z_up = 0.7;
  double z_down = -1.0;
  int vis = 0;
  std::string str_range = "15,30,45,60";
  std::string str_seg_distance = "0.5,1.1,1.6,2.1,2.6";
  ConfidenceConfig confidence;
  RansacConfig ransac;
  PatchworkppConfig patchworkpp;
  RoiConfig roi;
  FilterConfig filters;
  ClusterConfig cluster;

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

  lidar_cluster(const lidar_cluster&) = delete;
  lidar_cluster& operator=(const lidar_cluster&) = delete;

  void Configure(const LidarClusterConfig &config);
  void SetInputCloud(const pcl::PointCloud<PointType>::ConstPtr &cloud, uint32_t seq);
  bool Process(LidarClusterOutput *output);

private:
  void init();

  // ground seg
  void estimate_plane_();
  void extract_initial_seeds_(const pcl::PointCloud<PointType> &p_sorted);
  void ground_segmentation_dispatch_(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                     pcl::PointCloud<PointType>::Ptr &g_not_ground_pc);
  void ground_segmentation_ransac_(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                   pcl::PointCloud<PointType>::Ptr &g_not_ground_pc);
  void ground_segmentation_patchworkpp_(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                        pcl::PointCloud<PointType>::Ptr &g_not_ground_pc);

  // filter
  void PassThrough(pcl::PointCloud<PointType>::Ptr &cloud_filtered);

  // cluster
  void EucClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<pcl::PointIndices> &cluster_indices, const double &max_cluster_dis);
  void EucClusterMethodAdaptive(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<pcl::PointIndices> &cluster_indices,
                                const double &max_cluster_dis, int min_cluster_size, int max_cluster_size);
  void getAdaptiveClusterSize(double distance, int &min_size, int &max_size) const;
  void EuclideanAdaptiveClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<std::vector<pcl::PointIndices>> &cluster_indices,
                                      std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array);

  // DBSCAN clustering
  void DbscanClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<pcl::PointIndices> &cluster_indices,
                           double eps, int min_pts, int max_cluster_size);
  void DbscanAdaptiveClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<std::vector<pcl::PointIndices>> &cluster_indices,
                                   std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array);
  double getAdaptiveEps(double distance) const;
  void splitString(const std::string &in_string, std::vector<double> &out_array);
  void clusterMethod16(LidarClusterOutput *output);
  void clusterMethod32(LidarClusterOutput *output);

  double getConfidence(PointType max, PointType min, Eigen::Vector4f centroid);
  double getConfidenceNew(const pcl::PointCloud<PointType>::Ptr& cluster);
  double getConfidenceWithFitting(const pcl::PointCloud<PointType>::Ptr& cluster);

private:
  LidarClusterConfig config_;

  bool getPointClouds = false;
  int sensor_model_ = 16;
  double sensor_height_ = 0.135;
  // configuration settings
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
  int road_type_ = -1;
  int num_iter_ = 0;
  int num_lpr_ = 0;
  double th_seeds_ = 0.0;
  double th_dist_ = 0.0;
  std::string ground_method_ = "ransac";
  std::string str_range_;
  std::string str_seg_distance_;
  float d_ = 0.0f;
  float th_dist_d_ = 0.0f;
  MatrixXf normal_;

  pcl::PointCloud<PointType>::Ptr g_seeds_pc;
  pcl::PointCloud<PointType>::Ptr g_ground_pc;
  pcl::PointCloud<PointType>::Ptr g_not_ground_pc;
  pcl::PointCloud<PointType>::Ptr current_pc_ptr;
  pcl::PointCloud<PointType>::Ptr cloud_filtered;

  std::mutex lidar_mutex;

  std::vector<double> dis_range;
  std::vector<double> seg_distances;

  int frame_count = 0;
  std::size_t last_cluster_count_ = 0;

  LidarClusterConfig::RoiConfig roi_;
  LidarClusterConfig::FilterConfig filters_;
  LidarClusterConfig::ClusterConfig cluster_;
  std::string roi_mode_;
  bool roi_use_point_clip_ = false;

  patchwork::Params patchwork_params_;
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;

  perception::ClusterFeatureExtractor feature_extractor_;
  perception::ConfidenceScorer confidence_scorer_;
};
