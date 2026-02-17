#ifndef PLANNING_ROS_PLANNING_PIPELINE_NODE_HPP_
#define PLANNING_ROS_PLANNING_PIPELINE_NODE_HPP_

#include <memory>
#include <string>

#include <autodrive_msgs/HUAT_ConeMap.h>
#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/HUAT_Stop.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <autodrive_msgs/topic_contract.hpp>
#include <autodrive_msgs/diagnostics_helper.hpp>

#include "planning_ros/line_detection_node.hpp"
#include "planning_ros/skidpad_detection_node.hpp"

#include "high_speed_tracking/modules/WayComputer.hpp"
#include "high_speed_tracking/modules/Visualization.hpp"
#include "high_speed_tracking/modules/DelaunayTri.hpp"
#include "high_speed_tracking/utils/Params.hpp"
#include "high_speed_tracking/utils/PerfStats.hpp"

namespace planning_ros
{

/**
 * @brief Unified planning pipeline node (Facade).
 *
 * Constructs exactly one backend based on the `mission` parameter:
 *   - "line" / "acceleration"  → LineDetectionNode
 *   - "skidpad"                → SkidpadDetectionNode
 *   - "high_speed" / "trackdrive" / "autocross" → High-speed tracking
 */
class PlanningPipelineNode
{
public:
  PlanningPipelineNode(ros::NodeHandle &nh, const std::string &mission);

  /// Run one iteration. Returns true when the mission is finished and the node should shut down.
  bool SpinOnce();

private:
  // --- Backend initializers ---
  void InitLine(ros::NodeHandle &nh);
  void InitSkidpad(ros::NodeHandle &nh);
  void InitHighSpeed(ros::NodeHandle &nh);

  // --- High-speed specific ---
  void HighSpeedConeCallback(const autodrive_msgs::HUAT_ConeMap::ConstPtr &data);
  bool HighSpeedFinishCheck();
  void PublishDiagnostics(const diagnostic_msgs::DiagnosticArray &diag_arr);
  void PublishEntryHealth(const ros::Time &stamp, bool force = false);

  std::string mission_;

  // Line / Skidpad: reuse existing node classes
  std::unique_ptr<LineDetectionNode> line_node_;
  std::unique_ptr<SkidpadDetectionNode> skidpad_node_;

  // High-speed: members migrated from main.cpp globals
  std::unique_ptr<Params> hs_params_;
  std::unique_ptr<WayComputer> hs_way_computer_;
  Visualization *hs_viz_ = nullptr;
  PerfStats hs_perf_;

  ros::Publisher pathlimits_pub_;
  ros::Publisher hs_stop_pub_;
  autodrive_msgs::DiagnosticsHelper diag_helper_;
  ros::Subscriber hs_cone_sub_;
  ros::Subscriber hs_pose_sub_;

  // Lap counting / loop state (from main.cpp globals)
  bool wasLoopClosed_ = false;
  bool fullPathPublishedOnce_ = false;
  int finishGraceCount_ = 0;
  int finishGraceFrames_ = 300;
  bool waitFullBeforeStop_ = true;
  bool enableLoopFallbackByLapCounter_ = true;
  int loopFallbackMinLaps_ = 1;
  bool loopFallbackWasActive_ = false;
  int interTimes_ = 0;
  bool inter_ = false;

  // Finish zone parameters
  double finish_zone_x_threshold_ = 1.0;
  double finish_zone_y_threshold_ = 1.0;

  // Input validation
  ros::Time last_cone_stamp_;

  // PerfStats config
  bool perf_enabled_ = true;
  int perf_window_ = 300;
  int perf_log_every_ = 30;
  bool enable_internal_viz_side_channel_ = false;
  double diagnostics_rate_hz_ = 1.0;

  // Debug file helpers
  bool debug_save_way_files_ = false;
  void TxtClear();
  void DoWayMsg(const autodrive_msgs::HUAT_PathLimits &msgs);
  void DoWayFullMsg(const autodrive_msgs::HUAT_PathLimits &msgs);
};

} // namespace planning_ros

#endif // PLANNING_ROS_PLANNING_PIPELINE_NODE_HPP_
