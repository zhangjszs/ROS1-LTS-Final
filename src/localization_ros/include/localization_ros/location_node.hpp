#pragma once

#include <deque>
#include <memory>
#include <string>
#include <cstdint>
#include <unordered_set>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <autodrive_msgs/HUAT_InsP2.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_Cone.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>

#include <localization_core/location_mapper.hpp>
#include <localization_core/factor_graph_optimizer.hpp>
#include <localization_core/types.hpp>
#include <localization_ros/localization_perf_stats.hpp>
#include <autodrive_msgs/topic_contract.hpp>
#include <autodrive_msgs/diagnostics_helper.hpp>

namespace localization_ros {

class LocationNode {
 public:
  explicit LocationNode(ros::NodeHandle &nh);

 private:
  void loadParameters();
  void publishState(const localization_core::CarState &state, const ros::Time &stamp, bool publish_carstate);
  void imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr &msg);
  void carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msg);
  void coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &msg);
  void publishDiagnostics(const diagnostic_msgs::DiagnosticArray &diag_arr);
  void publishEntryHealth(const std::string &source, const ros::Time &stamp, bool force = false);
  void updateHeadingSanity(const autodrive_msgs::HUAT_InsP2 &msg, const ros::Time &stamp);
  void updateConeMapLifecycleStats(const autodrive_msgs::HUAT_ConeMap &map_msg);
  void publishMapperDebugTopics(const localization_core::MapUpdateStats &stats, const ros::Time &stamp);
  bool evaluateDriftAndRecover(const localization_core::MapUpdateStats &stats, const ros::Time &stamp);
  bool handleSaveMap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool handleLoadMap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  static localization_core::Asensing ToCore(const autodrive_msgs::HUAT_InsP2 &msg);
  static localization_core::CarState ToCore(const autodrive_msgs::HUAT_CarState &msg);
  static void ToRos(const localization_core::CarState &state, autodrive_msgs::HUAT_CarState *out);
  static localization_core::ConeDetections ToCore(const autodrive_msgs::HUAT_ConeDetections &msg);
  static void ToRos(const localization_core::ConeMap &map, autodrive_msgs::HUAT_ConeMap *out);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber ins_sub_;
  ros::Subscriber carstate_sub_;
  ros::Subscriber cone_sub_;

  ros::Publisher carstate_pub_;
  ros::Publisher map_pub_;
  ros::Publisher global_map_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher status_pub_;
  ros::Publisher debug_inlier_map_pub_;
  ros::Publisher debug_outlier_map_pub_;
  ros::Publisher debug_local_map_pub_;
  ros::Publisher debug_match_pairs_pub_;
  ros::Publisher debug_relocalization_pub_;
  autodrive_msgs::DiagnosticsHelper diag_helper_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  bool use_external_carstate_ = false;
  std::string ins_topic_;
  std::string carstate_in_topic_;
  std::string cone_topic_;
  std::string carstate_out_topic_;
  std::string cone_map_topic_;
  std::string global_map_topic_;
  std::string pose_topic_;
  std::string odom_topic_;
  std::string world_frame_;
  std::string base_link_frame_;
  std::string status_topic_;
  double diagnostics_rate_hz_ = 1.0;

  bool has_last_state_ = false;
  localization_core::CarState last_state_;
  ros::Time last_stamp_;

  localization_core::LocationParams params_;
  localization_core::LocationMapper mapper_;

  // Factor graph backend (shadow mode)
  std::string backend_;  // "mapper" (default) or "factor_graph"
  std::unique_ptr<localization_core::FactorGraphOptimizer> fg_optimizer_;
  localization_core::FactorGraphConfig fg_config_;
  double fg_start_time_ = -1.0;
  // B21: Relocalization success rate tracking
  localization_core::AnomalyState fg_last_anomaly_state_ = localization_core::AnomalyState::TRACKING;
  int fg_reloc_attempt_count_ = 0;
  int fg_reloc_success_count_ = 0;
  double fg_reloc_total_ms_ = 0.0;
  ros::WallTime fg_reloc_start_time_;
  void feedFactorGraph(const autodrive_msgs::HUAT_InsP2 &msg);
  void feedFactorGraphCones(const autodrive_msgs::HUAT_ConeDetections &msg);
  void updateMapperStateMachine(bool frame_good);
  bool shouldProcessConeFusion();
  int carStateQualityLevel() const;
  std::string carStateQualityLabel() const;
  static const char *mapperStateName(int state);

  // INS 状态环形缓冲区，用于检测-状态时间戳对齐
  struct StampedIns {
    ros::Time stamp;
    localization_core::Asensing data;
  };
  static constexpr size_t kInsBufferSize = 200;  // ~2s @ 100Hz
  std::deque<StampedIns> ins_buffer_;
  bool interpolateIns(const ros::Time &target, localization_core::Asensing &out) const;

  // Performance statistics
  LocPerfStats perf_stats_;
  bool perf_enabled_ = true;
  size_t perf_window_ = 300;
  size_t perf_log_every_ = 30;

  struct MapperRecoveryConfig {
    bool enabled = true;
    int fail_frames_to_degraded = 3;
    int fail_frames_to_ins_only = 8;
    int success_frames_to_tracking = 3;
    int recovery_cooldown_frames = 20;
    int recovery_probe_interval_frames = 3;
    int min_local_cones_for_tracking = 3;
    double min_match_ratio_for_tracking = 0.25;
    bool publish_stale_map_on_failure = true;
  };
  MapperRecoveryConfig mapper_recovery_cfg_;

  enum class MapperRuntimeState : std::uint8_t {
    TRACKING = 0,
    DEGRADED = 1,
    INS_ONLY = 2,
  };
  MapperRuntimeState mapper_state_ = MapperRuntimeState::TRACKING;
  std::uint64_t mapper_drop_count_ = 0;
  int mapper_consecutive_failures_ = 0;
  int mapper_consecutive_successes_ = 0;
  int mapper_ins_only_frames_ = 0;
  // B20: Degraded mode performance counters
  int mapper_degraded_entry_count_ = 0;
  int mapper_ins_only_entry_count_ = 0;
  int mapper_recovery_count_ = 0;
  int mapper_degraded_total_frames_ = 0;
  int mapper_ins_only_total_frames_ = 0;
  bool last_cone_update_success_ = false;
  bool last_cone_frame_good_ = false;
  int last_input_cone_count_ = 0;
  int last_local_cone_count_ = 0;
  double last_cone_match_ratio_ = 0.0;
  bool has_last_map_msg_ = false;
  autodrive_msgs::HUAT_ConeMap last_map_msg_;

  struct TfMonitorConfig {
    double max_delay_sec = 0.10;
    double max_future_sec = 0.02;
    double max_gap_sec = 0.20;
  };
  TfMonitorConfig tf_monitor_cfg_;
  double tf_last_lag_sec_ = 0.0;
  double tf_max_lag_sec_ = 0.0;
  std::uint64_t tf_delay_exceed_count_ = 0;
  std::uint64_t tf_future_stamp_count_ = 0;
  std::uint64_t tf_stamp_regression_count_ = 0;
  std::uint64_t tf_gap_exceed_count_ = 0;

  struct HeadingSanityConfig {
    bool enabled = true;
    double abs_max_deg = 360.0;
    double max_step_deg = 45.0;
    double max_rate_deg_per_sec = 180.0;
    double max_heading_gyro_residual_rad_s = 0.35;
  };
  HeadingSanityConfig heading_sanity_cfg_;
  bool has_last_heading_ = false;
  double last_heading_deg_ = 0.0;
  double last_heading_rate_rad_s_ = 0.0;
  ros::Time last_heading_stamp_;
  std::uint64_t heading_range_violation_count_ = 0;
  std::uint64_t heading_jump_violation_count_ = 0;
  std::uint64_t heading_rate_violation_count_ = 0;
  std::uint64_t heading_gyro_mismatch_count_ = 0;

  // B9: INS timestamp rollback protection
  bool has_last_ins_stamp_ = false;
  ros::Time last_ins_stamp_;
  std::uint64_t ins_stamp_rollback_count_ = 0;

  std::unordered_set<std::uint32_t> seen_cone_ids_;
  std::uint32_t cone_max_id_seen_ = 0;
  int cone_last_map_size_ = 0;
  std::uint64_t cone_new_id_count_ = 0;
  std::uint64_t cone_non_monotonic_new_id_count_ = 0;
  std::uint64_t cone_zero_id_count_ = 0;
  std::uint64_t cone_stale_publish_count_ = 0;

  struct DebugTopicsConfig {
    bool enabled = true;
    std::string inlier_map_topic = "localization/debug/inlier_map";
    std::string outlier_map_topic = "localization/debug/outlier_map";
    std::string local_map_topic = "localization/debug/local_map";
    std::string match_pairs_topic = "localization/debug/match_pairs";
    std::string relocalization_topic = "localization/debug/relocalization";
    int max_pairs = 20;
  };
  DebugTopicsConfig debug_topics_cfg_;
  localization_core::MapUpdateStats last_map_stats_;
  bool has_last_map_stats_ = false;

  struct DriftMonitorConfig {
    bool enabled = true;
    double min_match_ratio = 0.15;
    double max_mean_match_distance = 2.0;
    int min_local_cones = 2;
    int bad_frames_to_trigger = 15;
    int cooldown_frames = 50;
    bool trigger_only_when_frozen = true;
    std::string action = "rollback";  // rollback | reset | freeze
  };
  DriftMonitorConfig drift_monitor_cfg_;
  int drift_bad_frames_ = 0;
  int drift_cooldown_frames_ = 0;
  std::uint64_t drift_event_count_ = 0;
  std::uint64_t drift_rollback_count_ = 0;
  std::uint64_t drift_reset_count_ = 0;
  std::uint64_t drift_freeze_count_ = 0;
  bool drift_last_frame_bad_ = false;

  struct MapPersistenceConfig {
    bool enabled = true;
    std::string save_path = "/tmp/localization_map.csv";
    std::string version_tag = "v1";
    std::string save_service = "localization/map/save";
    std::string load_service = "localization/map/load";
  };
  MapPersistenceConfig map_persistence_cfg_;
  std::uint64_t map_save_count_ = 0;
  std::uint64_t map_load_count_ = 0;
  std::uint64_t map_save_fail_count_ = 0;
  std::uint64_t map_load_fail_count_ = 0;
  double map_last_save_time_sec_ = 0.0;
  double map_last_load_time_sec_ = 0.0;

  // Heading init logging (Item 3)
  bool heading_init_logged_ = false;
};

}  // namespace localization_ros
