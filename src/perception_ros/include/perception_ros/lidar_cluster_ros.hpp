#pragma once

#include <string>
#include <mutex>

#include <ros/ros.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include <perception_core/lidar_cluster_core.hpp>
#include <perception_ros/perf_stats.hpp>
#include <perception_ros/distortion_compensator_v2.hpp>
#include <fsd_common/diagnostics_helper.hpp>
#include <fsd_common/topic_contract.hpp>

namespace perception_ros {

/// Perception health level (mirrors diagnostic_msgs levels).
enum class HealthLevel : uint8_t {
  NORMAL = 0,         // diagnostic_msgs::DiagnosticStatus::OK
  LOW_DETECTION = 1,  // diagnostic_msgs::DiagnosticStatus::WARN
  NO_DETECTION = 2    // diagnostic_msgs::DiagnosticStatus::ERROR
};

class LidarClusterRos {
 public:
  LidarClusterRos(ros::NodeHandle nh, ros::NodeHandle private_nh);

  void RunOnce();
  bool IsLegacyPollMode() const;
  double LegacyPollHz() const;

 private:
  void loadParams();
  void applyModePreset();
  void pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void publishOutput(const LidarClusterOutput &output);
  void runOnceLocked();
  void updateGroundWatchdogLocked(double t_ground_ms);
  void updateHealthState(int n_detections, double t_total_ms);
  void publishDiagnostics(const LidarClusterOutput &output, int n_published);
  void publishDebugMarkers(const LidarClusterOutput &output);
  void validateStamp(std_msgs::Header &header);

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
  ros::Time last_cloud_stamp_;         // P0: 上一帧点云时间戳（用于丢帧检测）

  int sensor_model_ = 16;

  std::string input_topic_;
  std::string passthrough_topic_;
  std::string no_ground_topic_;
  std::string cones_topic_;
  std::string detections_topic_;

  bool perf_enabled_ = true;
  size_t perf_window_ = 300;
  size_t perf_log_every_ = 30;
  std::string pipeline_mode_ = "event_driven";  // event_driven | legacy_poll
  double legacy_poll_hz_ = 10.0;
  bool input_guard_enable_ = true;              // G4: 输入边界防御总开关
  int input_guard_max_points_ = 500000;         // G4: 点云最大允许点数
  bool input_guard_filter_invalid_points_ = true;  // G4: 过滤NaN/Inf点
  bool force_fgs_fast_path_ = true;             // T30-2: 锁定FGS快路径
  bool ground_watchdog_enable_ = true;          // T30-2: 地面分割看门狗
  double ground_watchdog_warn_ms_ = 8.0;        // T30-2: t_ground_ms告警阈值
  int ground_watchdog_warn_frames_ = 5;         // T30-2: 连续超阈值帧数
  int ground_watchdog_overrun_count_ = 0;

  // 性能优化选项
  bool use_point_cloud_pool_ = false;  // 是否启用点云内存池
  bool enable_cone_size_typing_ = true;   // 基于几何尺寸推断 ORANGE_SMALL / ORANGE_BIG
  double big_cone_height_threshold_ = 0.45;
  double big_cone_area_threshold_ = 0.08;

  // 零拷贝优化：复用发布消息缓冲区
  sensor_msgs::PointCloud2 pub_pc_msg_;       // 复用的PointCloud2消息
  sensor_msgs::PointCloud2 pub_cones_msg_;    // 复用的锥桶点云消息

  // IMU Distortion Compensation V2 (支持time字段、预积分、外参)
  std::unique_ptr<DistortionCompensatorV2> compensator_;

  // ── Diagnostics & Health State ──────────────────────────────────
  fsd_common::DiagnosticsHelper diag_helper_;
  HealthLevel health_level_ = HealthLevel::NORMAL;
  int consecutive_zero_detections_ = 0;
  int consecutive_low_detections_ = 0;
  double diag_rate_hz_ = 2.0;                // 诊断发布频率
  int diag_low_detection_threshold_ = 2;     // 低检测数阈值
  int diag_zero_frames_to_warn_ = 3;         // 连续零检测帧数 → WARN
  int diag_zero_frames_to_error_ = 8;        // 连续零检测帧数 → ERROR
  int diag_low_frames_to_warn_ = 5;          // 连续低检测帧数 → WARN
  int diag_recovery_frames_ = 3;             // 恢复所需连续正常帧数
  int consecutive_normal_frames_ = 0;        // 连续正常帧计数（用于恢复）
  double diag_latency_warn_ms_ = 15.0;       // 延迟告警阈值
  double stamp_max_drift_sec_ = 1.0;          // stamp 与接收时间最大允许偏差 [s]

  // ── Debug visualization ───────────────────────────────────────
  bool debug_publish_markers_ = false;         // G6: 调试标记发布开关
  ros::Publisher debug_marker_pub_;            // G6: bbox marker publisher
};

}  // namespace perception_ros
