#pragma once

#include <cmath>
#include <sstream>
#include <string>

#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/topic_contract.hpp>
#include <fsd_common/contract_utils.hpp>
#include <ros/ros.h>

namespace planning_ros {
namespace contract {

// Delegated to fsd_common::contract
using fsd_common::contract::NormalizeInputStamp;
using fsd_common::contract::NormalizeFrameId;
using fsd_common::contract::DecodeConeConfidenceScore;
using fsd_common::contract::EncodeConeConfidenceScaled;

// B9: Timestamp semantics
// - msg.header.stamp  = sensor/input timestamp (from upstream message, normalized)
// - msg.stamp         = publish wall-clock time (ros::Time::now() at publish)
// - msg.tracklimits.stamp = same as msg.stamp (co-published)
inline void FinalizePathLimitsMessage(autodrive_msgs::HUAT_PathLimits &msg,
                                      const ros::Time &input_stamp,
                                      const std::string &frame_id = autodrive_msgs::frame_contract::kWorld)
{
  msg.header.stamp = NormalizeInputStamp(input_stamp);
  msg.header.frame_id = NormalizeFrameId(frame_id, autodrive_msgs::frame_contract::kWorld);
  msg.stamp = ros::Time::now();
  msg.tracklimits.stamp = msg.stamp;
}

// B1: PathLimits array length invariant enforcement
// INVARIANT: len(path) == len(target_speeds) == len(curvatures)
// This function ensures the invariant by resizing arrays to match path length.
inline void EnforcePathDynamicsShape(autodrive_msgs::HUAT_PathLimits &msg)
{
  const size_t n = msg.path.size();

  // Resize to match path length (fills with 0.0 if extending)
  msg.curvatures.resize(n, 0.0);
  msg.target_speeds.resize(n, 0.0);
}

// B1: Validate PathLimits array length invariant
// Returns true if valid, false if invariant violated
inline bool ValidatePathDynamicsShape(const autodrive_msgs::HUAT_PathLimits &msg, std::string *error = nullptr)
{
  const size_t path_len = msg.path.size();
  const size_t speeds_len = msg.target_speeds.size();
  const size_t curvatures_len = msg.curvatures.size();

  if (path_len != speeds_len || path_len != curvatures_len)
  {
    if (error)
    {
      std::ostringstream oss;
      oss << "PathLimits array length mismatch: path=" << path_len
          << ", target_speeds=" << speeds_len
          << ", curvatures=" << curvatures_len;
      *error = oss.str();
    }
    return false;
  }

  return true;
}

// B5: Validate path quality (curvature, length)
// Returns true if valid, false if quality issues detected
inline bool ValidatePathQuality(const autodrive_msgs::HUAT_PathLimits &msg,
                                std::string *warning = nullptr,
                                int *curvature_violations = nullptr,
                                double *max_curvature = nullptr)
{
  bool has_issues = false;
  std::ostringstream oss;

  // Check path length
  const size_t path_len = msg.path.size();
  constexpr size_t kMinPathLength = 5; // Minimum 5 points for lookahead
  if (path_len < kMinPathLength)
  {
    oss << "Path too short: " << path_len << " points (min: " << kMinPathLength << "); ";
    has_issues = true;
  }

  // Check curvature limit (vehicle physical constraint)
  constexpr double kMaxCurvature = 0.222; // 1/m, corresponds to min turning radius ~4.5m
  int violations = 0;
  double max_curv = 0.0;

  for (size_t i = 0; i < msg.curvatures.size(); ++i)
  {
    const double abs_curv = std::abs(msg.curvatures[i]);
    if (abs_curv > max_curv)
    {
      max_curv = abs_curv;
    }
    if (abs_curv > kMaxCurvature)
    {
      ++violations;
    }
  }

  if (violations > 0)
  {
    oss << "Curvature violations: " << violations << " points exceed " << kMaxCurvature
        << " 1/m (max: " << max_curv << " 1/m); ";
    has_issues = true;
  }

  if (curvature_violations)
  {
    *curvature_violations = violations;
  }
  if (max_curvature)
  {
    *max_curvature = max_curv;
  }

  if (warning && has_issues)
  {
    *warning = oss.str();
  }

  return !has_issues;
}

}  // namespace contract
}  // namespace planning_ros
