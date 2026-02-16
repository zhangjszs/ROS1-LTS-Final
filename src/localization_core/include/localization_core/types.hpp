#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization_core {

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct CarState {
  Pose2 car_state;
  Point3 car_state_front;
  Point3 car_state_rear;
  double V = 0.0;   // 纵向速度 [m/s]
  double W = 0.0;   // 偏航角速度 [rad/s] (deprecated, use Wz)
  double A = 0.0;   // 加速度 [m/s^2] (deprecated, use Ax)

  // FSSIM风格扩展状态
  double Vy = 0.0;  // 横向速度 [m/s]
  double Wz = 0.0;  // 偏航角速度 [rad/s]
  double Ax = 0.0;  // 纵向加速度 [m/s^2]
  double Ay = 0.0;  // 横向加速度 [m/s^2]
};

struct Asensing {
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double north_velocity = 0.0;
  double east_velocity = 0.0;
  double ground_velocity = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double azimuth = 0.0;
  double x_angular_velocity = 0.0;
  double y_angular_velocity = 0.0;
  double z_angular_velocity = 0.0;
  double x_acc = 0.0;
  double y_acc = 0.0;
  double z_acc = 0.0;
  std::uint8_t status = 0;
  std::uint8_t nsv1 = 0;
  std::uint8_t nsv2 = 0;
  std::uint8_t age = 0;
};

struct Cone {
  Point3 position_base_link;
  Point3 position_global;
  std::uint32_t id = 0;
  std::uint32_t confidence = 0;  // 置信度，scaled 0–1000 (0.0–1.0 * 1000)
  std::uint32_t type = 4;  // 0=BLUE,1=YELLOW,2=ORANGE_SMALL,3=ORANGE_BIG,4=NONE,5=RED
};

struct ConeMap {
  std::vector<Cone> cones;
};

struct ConeDetection {
  Point3 point;
  Point3 bbox_min;
  Point3 bbox_max;
  double confidence = 0.0;
  double distance = 0.0;
  std::uint8_t color_type = 4;  // 0=BLUE,1=YELLOW,2=ORANGE_SMALL,3=ORANGE_BIG,4=NONE,5=RED
};

struct ConeDetections {
  std::vector<ConeDetection> detections;
};

struct ConeAssociationDebug {
  Point3 detection_base;
  Point3 detection_global;
  Point3 matched_global;
  std::uint32_t matched_id = 0;
  double distance = 0.0;
  bool merged = false;
};

struct MapUpdateStats {
  int input_count = 0;
  int bbox_reject_count = 0;
  int geometry_reject_count = 0;
  int conf_add_reject_count = 0;
  int conf_merge_reject_count = 0;
  int frozen_reject_count = 0;
  int merged_count = 0;
  int inserted_count = 0;
  int local_output_count = 0;
  int map_size = 0;
  int update_seq = 0;
  bool map_frozen = false;
  double mean_match_distance = 0.0;
  double max_match_distance = 0.0;
  std::vector<Cone> inlier_cones;
  std::vector<Cone> outlier_cones;
  std::vector<ConeAssociationDebug> associations;
};

struct LocationParams {
  double lidar_to_imu_dist = 1.87;
  double front_to_imu_x = 0.0;
  double front_to_imu_y = 0.0;
  double front_to_imu_z = 0.0;
  double rear_to_imu_x = 0.0;
  double rear_to_imu_y = 0.0;
  double rear_to_imu_z = 0.0;
  std::uint8_t min_ins_status = 2;
  std::uint8_t min_satellite_count = 8;
  std::uint8_t max_diff_age = 30;

  // 锥桶合并与地图管理
  double merge_distance = 2.5;       // 锥桶合并距离阈值 (m)
  int max_map_size = 500;            // 地图最大锥桶数量
  int min_obs_to_keep = 2;           // 清理时保留的最小观测次数
  double local_cone_range = 50.0;    // 输出局部锥桶的最大距离 (m)

  // 入图过滤参数
  double min_confidence_to_add = 0.3;    // 新锥桶入图最低 confidence
  double min_confidence_to_merge = 0.15; // 合并到已有锥桶的最低 confidence
  double max_cone_height = 0.75;         // 锥桶最大高度 (m)
  double max_cone_width = 0.5;           // 锥桶最大宽度 (m)
  double min_cone_height = 0.03;         // 锥桶最小高度 (m)

  // 赛道模式
  std::string map_mode = "track";  // "accel" | "skidpad" | "track"

  // 几何约束参数（由模式预设覆盖）
  double cone_y_max = 0.0;              // base_link Y 轴最大偏移，0=不启用
  double expected_cone_spacing = 0.0;    // 预期锥桶间距，0=不启用
  double track_width = 3.0;             // 赛道宽度 (m)

  // Skidpad 圆弧验证
  bool enable_circle_validation = false;
  double circle_radius = 15.25;          // 圆半径 (m)
  double circle_center_dist = 18.25;     // 两圆心距离 (m)
  double circle_tolerance = 2.0;         // 圆弧验证容差 (m)

  // 缺锥补偿（Missing Cone Fallback）
  struct MissingConeFallback {
    bool enabled = false;
    double max_interpolation_distance = 8.0;  // 最大插值距离 (m)
    double expected_spacing = 5.0;            // 预期锥桶间距 (m)
    double min_confidence_for_interpolation = 0.15;
    int max_consecutive_missing = 3;
  };
  MissingConeFallback missing_cone_fallback;

  // 短路径抑制（Short Path Suppression）
  struct ShortPathSuppression {
    bool enabled = false;
    double min_path_length = 3.0;   // 最小有效路径长度 (m)
    int min_cone_count = 3;         // 最小锥桶数量
    bool reject_single_cone_paths = true;
  };
  ShortPathSuppression short_path_suppression;

  struct MapFreeze {
    bool enabled = false;
    int freeze_after_frames = 300;         // 处理多少帧锥桶后冻结
    int freeze_after_cones = 120;          // 地图锥桶数量达到该阈值后冻结
    bool allow_merge_when_frozen = true;   // 冻结后是否允许更新已有锥桶
    bool allow_new_cones_when_frozen = false;  // 冻结后是否允许新增锥桶
  };
  MapFreeze map_freeze;
};

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

}  // namespace localization_core
