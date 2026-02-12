#include "planning_ros/planning_pipeline_node.hpp"

#include <ros/package.h>
#include <ros/serialization.h>
#include <sys/stat.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

#include "planning_ros/contract_utils.hpp"

namespace planning_ros
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

PlanningPipelineNode::PlanningPipelineNode(ros::NodeHandle &nh, const std::string &mission)
  : mission_(mission)
{
  ros::NodeHandle pnh("~");
  pnh.param("diagnostics_rate_hz", diagnostics_rate_hz_, 1.0);
  pnh.param<std::string>("diagnostics_topic", diagnostics_topic_, "planning/diagnostics");
  pnh.param("publish_global_diagnostics", publish_global_diagnostics_, true);
  pnh.param<std::string>("global_diagnostics_topic", global_diagnostics_topic_, "/diagnostics");
  pnh.param("enable_internal_viz_side_channel", enable_internal_viz_side_channel_, false);
  if (diagnostics_rate_hz_ <= 0.0)
  {
    diagnostics_rate_hz_ = 1.0;
  }

  diag_pub_local_ = nh.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 1, true);
  if (publish_global_diagnostics_)
  {
    diag_pub_global_ = nh.advertise<diagnostic_msgs::DiagnosticArray>(global_diagnostics_topic_, 1);
  }

  if (mission_ == "line" || mission_ == "acceleration")
  {
    InitLine(nh);
  }
  else if (mission_ == "skidpad")
  {
    InitSkidpad(nh);
  }
  else
  {
    // Default: high_speed / trackdrive / autocross
    InitHighSpeed(nh);
  }
  ROS_INFO("[PlanningPipeline] Initialized with mission='%s'", mission_.c_str());
  PublishEntryHealth(ros::Time::now(), true);
}

// ---------------------------------------------------------------------------
// Backend initializers
// ---------------------------------------------------------------------------

void PlanningPipelineNode::InitLine(ros::NodeHandle &nh)
{
  // Override topic names to publish to unified topic
  ros::NodeHandle pnh("~");
  std::string output_topic;
  pnh.param<std::string>("output_pathlimits_topic", output_topic, "planning/pathlimits");
  pnh.setParam("topics/pathlimits", output_topic);
  
  line_node_ = std::make_unique<LineDetectionNode>(nh);
}

void PlanningPipelineNode::InitSkidpad(ros::NodeHandle &nh)
{
  // Override topic names to publish to unified topic
  ros::NodeHandle pnh("~");
  std::string output_topic;
  pnh.param<std::string>("output_pathlimits_topic", output_topic, "planning/pathlimits");
  pnh.setParam("topics/pathlimits", output_topic);
  
  skidpad_node_ = std::make_unique<SkidpadDetectionNode>(nh);
}

void PlanningPipelineNode::InitHighSpeed(ros::NodeHandle &nh)
{
  ros::NodeHandle pnh("~");

  pnh.param("wait_full_before_stop", waitFullBeforeStop_, true);
  pnh.param("finish_grace_frames", finishGraceFrames_, 300);
  pnh.param("enable_loop_fallback_by_lap_counter", enableLoopFallbackByLapCounter_, true);
  pnh.param("loop_fallback_min_laps", loopFallbackMinLaps_, 1);
  pnh.param("perf_stats_enable", perf_enabled_, true);
  pnh.param("perf_stats_window", perf_window_, 300);
  pnh.param("perf_stats_log_every", perf_log_every_, 30);

  hs_perf_.Configure("planning_pipeline/high_speed", perf_enabled_,
                      static_cast<size_t>(perf_window_),
                      static_cast<size_t>(perf_log_every_));

  hs_params_ = std::make_unique<Params>(&nh);
  hs_way_computer_ = std::make_unique<WayComputer>(hs_params_->wayComputer);

  if (enable_internal_viz_side_channel_)
  {
    hs_viz_ = &Visualization::getInstance();
    hs_viz_->init(&nh, hs_params_->visualization);
    ROS_WARN("[planning_pipeline/high_speed] Internal visualization side-channel enabled for compatibility.");
  }
  else
  {
    hs_viz_ = nullptr;
    ROS_INFO("[planning_pipeline/high_speed] Internal visualization side-channel disabled (decoupled mode).");
  }

  debug_save_way_files_ = hs_params_->main.debug_save_way_files;
  if (debug_save_way_files_)
  {
    TxtClear();
  }

  hs_cone_sub_ = nh.subscribe(hs_params_->main.input_cones_topic, 1,
                               &PlanningPipelineNode::HighSpeedConeCallback, this);
  hs_pose_sub_ = nh.subscribe(hs_params_->main.input_pose_topic, 1,
                               &WayComputer::stateCallback, hs_way_computer_.get());

  std::string pathlimits_topic;
  pnh.param<std::string>("output_pathlimits_topic", pathlimits_topic,
                          "planning/pathlimits");
  pathlimits_pub_ = nh.advertise<autodrive_msgs::HUAT_PathLimits>(pathlimits_topic, 1);
  hs_stop_pub_ = nh.advertise<autodrive_msgs::HUAT_Stop>(hs_params_->main.stop_topic, 1);
}

// ---------------------------------------------------------------------------
// SpinOnce
// ---------------------------------------------------------------------------

void PlanningPipelineNode::PublishDiagnostics(const diagnostic_msgs::DiagnosticArray &diag_arr)
{
  diag_pub_local_.publish(diag_arr);
  if (publish_global_diagnostics_)
  {
    diag_pub_global_.publish(diag_arr);
  }
}

void PlanningPipelineNode::PublishEntryHealth(const ros::Time &stamp, bool force)
{
  const ros::Time now = ros::Time::now();
  if (!force && last_entry_diag_pub_.isValid())
  {
    const double min_interval = 1.0 / diagnostics_rate_hz_;
    if ((now - last_entry_diag_pub_).toSec() < min_interval)
    {
      return;
    }
  }
  last_entry_diag_pub_ = now;

  diagnostic_msgs::DiagnosticArray diag_arr;
  diag_arr.header.stamp = stamp.isZero() ? now : stamp;

  diagnostic_msgs::DiagnosticStatus ds;
  ds.name = "planning_entry_health";
  ds.hardware_id = "planning_ros/planning_pipeline";
  ds.level = diagnostic_msgs::DiagnosticStatus::OK;
  ds.message = "OK";

  auto kv = [](const std::string &k, const std::string &v) {
    diagnostic_msgs::KeyValue pair;
    pair.key = k;
    pair.value = v;
    return pair;
  };

  ds.values.push_back(kv("mission", mission_));

  if (mission_ == "line" || mission_ == "acceleration")
  {
    ds.values.push_back(kv("backend", "line_detection"));
    ds.values.push_back(kv("local_tf_valid", "n/a"));
    ds.values.push_back(kv("lap_mode", "n/a"));
    ds.values.push_back(kv("internal_viz_side_channel", "false"));
  }
  else if (mission_ == "skidpad")
  {
    ds.values.push_back(kv("backend", "skidpad_detection"));
    ds.values.push_back(kv("local_tf_valid", "n/a"));
    ds.values.push_back(kv("lap_mode", "n/a"));
    ds.values.push_back(kv("internal_viz_side_channel", "false"));
  }
  else
  {
    const bool local_tf_valid = hs_way_computer_ && hs_way_computer_->isLocalTfValid();
    if (!local_tf_valid)
    {
      ds.level = diagnostic_msgs::DiagnosticStatus::WARN;
      ds.message = "WAITING_CARSTATE";
    }

    std::string lap_mode = "MAP_BUILD_SAFE";
    if (hs_way_computer_ && hs_way_computer_->lapMode() == WayComputer::LapMode::FAST_LAP)
    {
      lap_mode = "FAST_LAP";
    }

    ds.values.push_back(kv("backend", "high_speed_tracking"));
    ds.values.push_back(kv("local_tf_valid", local_tf_valid ? "true" : "false"));
    ds.values.push_back(kv("lap_mode", lap_mode));
    ds.values.push_back(kv("loop_closed", (hs_way_computer_ && hs_way_computer_->isLoopClosed()) ? "true" : "false"));
    ds.values.push_back(kv("loop_fallback_active", loopFallbackWasActive_ ? "true" : "false"));
    ds.values.push_back(kv("inter_times", std::to_string(interTimes_)));
    ds.values.push_back(kv("internal_viz_side_channel", enable_internal_viz_side_channel_ ? "true" : "false"));
  }

  diag_arr.status.push_back(ds);
  PublishDiagnostics(diag_arr);
}

bool PlanningPipelineNode::SpinOnce()
{
  PublishEntryHealth(ros::Time::now(), false);

  if (mission_ == "line" || mission_ == "acceleration")
  {
    line_node_->RunOnce();
    return line_node_->IsFinished();
  }
  if (mission_ == "skidpad")
  {
    skidpad_node_->RunOnce();
    return false;
  }
  // high_speed: callback-driven, SpinOnce only returns false
  return false;
}

// ---------------------------------------------------------------------------
// High-speed cone callback (migrated from main.cpp::callback_ccat)
// ---------------------------------------------------------------------------

void PlanningPipelineNode::HighSpeedConeCallback(
    const autodrive_msgs::HUAT_ConeMap::ConstPtr &data)
{
  const ros::Time diag_stamp = contract::NormalizeInputStamp(data->header.stamp);

  if (!hs_params_ || !hs_way_computer_ || !hs_way_computer_->isLocalTfValid())
  {
    PublishEntryHealth(diag_stamp, false);
    ROS_WARN("[planning_pipeline/high_speed] CarState not received or wayComputer invalid.");
    return;
  }
  if (data->cone.empty())
  {
    PublishEntryHealth(diag_stamp, false);
    ROS_WARN("[planning_pipeline/high_speed] Empty cone set.");
    return;
  }

  ros::WallTime total_start = ros::WallTime::now();
  size_t bytes_pub = 0;

  // Filter cones by confidence
  std::vector<Node> nodes;
  nodes.reserve(data->cone.size());
  for (const autodrive_msgs::HUAT_Cone &c : data->cone)
  {
    if (contract::DecodeConeConfidenceScore(c.confidence) >= hs_params_->main.min_cone_confidence)
    {
      nodes.emplace_back(c);
    }
  }
  if (nodes.empty())
  {
    ROS_WARN_THROTTLE(1.0,
        "[planning_pipeline/high_speed] No cones passed confidence filter (min=%.3f).",
        hs_params_->main.min_cone_confidence);
    return;
  }

  for (const Node &n : nodes)
  {
    n.updateLocal(hs_way_computer_->getLocalTf());
  }

  // Delaunay triangulation
  ros::WallTime delaunay_start = ros::WallTime::now();
  TriangleSet triangles = DelaunayTri::compute(nodes);
  ros::WallDuration delaunay_dur = ros::WallTime::now() - delaunay_start;

  // Way computation
  ros::WallTime way_start = ros::WallTime::now();
  hs_way_computer_->update(triangles, diag_stamp);
  ros::WallDuration way_dur = ros::WallTime::now() - way_start;

  const bool finish_reached = HighSpeedFinishCheck();

  if (hs_viz_)
  {
    hs_viz_->setTimestamp(diag_stamp);
    hs_viz_->visualize(hs_way_computer_->lastFilteredTriangulation());
    hs_viz_->visualize(hs_way_computer_->lastFilteredEdges());
    hs_viz_->visualize(hs_way_computer_->wayForVisualization());
  }
  // Loop closure logic
  const bool loop_closed_raw = hs_way_computer_->isLoopClosed();
  const bool loop_closed_fallback =
      enableLoopFallbackByLapCounter_ && (interTimes_ >= std::max(1, loopFallbackMinLaps_));
  const bool loop_fallback_active_now = (!loop_closed_raw && loop_closed_fallback);

  if (loop_fallback_active_now && !loopFallbackWasActive_)
  {
    ROS_WARN("[planning_pipeline/high_speed] Loop-closure fallback activated (interTimes=%d, threshold=%d).",
             interTimes_, std::max(1, loopFallbackMinLaps_));
  }
  else if (!loop_fallback_active_now && loopFallbackWasActive_)
  {
    ROS_INFO("[planning_pipeline/high_speed] Loop-closure fallback cleared.");
  }
  loopFallbackWasActive_ = loop_fallback_active_now;
  const bool loop_closed_for_publish = loop_closed_raw || loop_closed_fallback;

  if (loop_closed_for_publish)
  {
    if (!wasLoopClosed_)
    {
      ROS_INFO("[planning_pipeline/high_speed] Enter FAST_LAP: publishing full path.");
    }
    autodrive_msgs::HUAT_PathLimits full_msg =
        hs_way_computer_->getPathLimitsGlobal(hs_params_->main.the_mode_of_full_path);
    if (debug_save_way_files_) DoWayFullMsg(full_msg);
    pathlimits_pub_.publish(full_msg);
    bytes_pub += ros::serialization::serializationLength(full_msg);
    fullPathPublishedOnce_ = true;
    finishGraceCount_ = 0;
    wasLoopClosed_ = true;
    if (debug_save_way_files_)
    {
      std::string loopDir = hs_params_->main.package_path + "/loops";
      mkdir(loopDir.c_str(), 0777);
      hs_way_computer_->writeWayToFile(loopDir + "/loop.unay");
    }
    if (hs_params_->main.shutdown_on_loop_closure)
    {
      ros::shutdown();
      return;
    }
  }
  else
  {
    if (wasLoopClosed_)
    {
      ROS_WARN_THROTTLE(1.0, "[planning_pipeline/high_speed] FAST_LAP lost. Fallback to SAFE_LAP.");
    }
    autodrive_msgs::HUAT_PathLimits partial_msg =
        hs_way_computer_->getPathLimitsGlobal(hs_params_->main.the_mode_of_partial_path);
    if (debug_save_way_files_) DoWayMsg(partial_msg);
    pathlimits_pub_.publish(partial_msg);
    bytes_pub += ros::serialization::serializationLength(partial_msg);
    wasLoopClosed_ = false;
  }

  // Publish planning diagnostics
  {
    diagnostic_msgs::DiagnosticArray diag_arr;
    diag_arr.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus ds;
    ds.name = "planning/high_speed";
    ds.hardware_id = "planning";
    ds.level = diagnostic_msgs::DiagnosticStatus::OK;

    const char* mode_str = loop_closed_for_publish ? "FAST_LAP" : "MAP_BUILD_SAFE";
    ds.message = mode_str;

    auto kv = [](const std::string &k, const std::string &v) {
      diagnostic_msgs::KeyValue pair;
      pair.key = k;
      pair.value = v;
      return pair;
    };
    ds.values.push_back(kv("lap_mode", mode_str));
    ds.values.push_back(kv("loop_closed", loop_closed_raw ? "true" : "false"));
    ds.values.push_back(kv("loop_fallback_active", loop_fallback_active_now ? "true" : "false"));
    ds.values.push_back(kv("inter_times", std::to_string(interTimes_)));

    diag_arr.status.push_back(ds);
    PublishDiagnostics(diag_arr);
  }
  PublishEntryHealth(diag_stamp, false);

  // Finish handling
  if (finish_reached)
  {
    if (waitFullBeforeStop_ && !fullPathPublishedOnce_ &&
        finishGraceCount_ < std::max(0, finishGraceFrames_))
    {
      ++finishGraceCount_;
      ROS_WARN_THROTTLE(1.0,
          "[planning_pipeline/high_speed] Finish reached but full path not published. "
          "Delaying stop (%d/%d).",
          finishGraceCount_, finishGraceFrames_);
    }
    else
    {
      autodrive_msgs::HUAT_Stop msg;
      msg.stop = true;
      hs_stop_pub_.publish(msg);
      bytes_pub += ros::serialization::serializationLength(msg);
      ROS_INFO("[planning_pipeline/high_speed] Finished.");
      ros::shutdown();
      return;
    }
  }

  // PerfStats
  ros::WallDuration total_dur = ros::WallTime::now() - total_start;
  PerfSample sample;
  sample.t_delaunay_ms = delaunay_dur.toSec() * 1000.0;
  sample.t_way_ms = way_dur.toSec() * 1000.0;
  sample.t_total_ms = total_dur.toSec() * 1000.0;
  sample.n_points = static_cast<double>(nodes.size());
  sample.n_triangles = static_cast<double>(hs_way_computer_->lastTriangleCount());
  sample.n_edges = static_cast<double>(hs_way_computer_->lastEdgeCount());
  sample.bytes_pub = static_cast<double>(bytes_pub);
  hs_perf_.Add(sample);
}

// ---------------------------------------------------------------------------
// High-speed finish check (migrated from main.cpp::finish())
// ---------------------------------------------------------------------------

bool PlanningPipelineNode::HighSpeedFinishCheck()
{
  if (!inter_ &&
      std::abs(hs_way_computer_->getCarState().car_state.x) < 1 &&
      std::abs(hs_way_computer_->getCarState().car_state.y) < 1)
  {
    inter_ = true;
    ROS_INFO("[planning_pipeline/high_speed] Entered finish zone.");
  }
  if (std::abs(hs_way_computer_->getCarState().car_state.y) < 1 &&
      inter_ && hs_way_computer_->getCarState().car_state.x > 1)
  {
    inter_ = false;
    interTimes_++;
    ROS_INFO("[planning_pipeline/high_speed] Starting lap %d.", interTimes_);
  }
  return interTimes_ > hs_params_->main.number_of_stopped_turns;
}

// ---------------------------------------------------------------------------
// Debug file helpers (migrated from main.cpp)
// ---------------------------------------------------------------------------

void PlanningPipelineNode::TxtClear()
{
  std::string dir = ros::package::getPath("planning_ros") + "/testData";
  mkdir(dir.c_str(), 0777);
  std::ofstream f1(dir + "/wayPartial.txt", std::ios::out | std::ios::trunc);
  f1.close();
  std::ofstream f2(dir + "/wayFull.txt", std::ios::out | std::ios::trunc);
  f2.close();
}

void PlanningPipelineNode::DoWayMsg(const autodrive_msgs::HUAT_PathLimits &msgs)
{
  std::string path = ros::package::getPath("planning_ros") + "/testData/wayPartial.txt";
  std::ofstream f(path.c_str(), std::ios_base::app);
  if (f.fail() || msgs.path.empty()) return;
  std::stringstream ss;
  for (size_t i = 0; i < msgs.path.size(); i++)
    ss << msgs.path[i].x << "\t" << msgs.path[i].y << "\t" << std::endl;
  f << ss.str();
}

void PlanningPipelineNode::DoWayFullMsg(const autodrive_msgs::HUAT_PathLimits &msgs)
{
  std::string path = ros::package::getPath("planning_ros") + "/testData/wayFull.txt";
  std::ofstream f(path.c_str(), std::ios_base::app);
  if (f.fail() || msgs.path.empty()) return;
  std::stringstream ss;
  for (size_t i = 0; i < msgs.path.size(); i++)
    ss << msgs.path[i].x << "\t" << msgs.path[i].y << "\t" << std::endl;
  f << ss.str();
  f << "************************************\n";
}

} // namespace planning_ros
