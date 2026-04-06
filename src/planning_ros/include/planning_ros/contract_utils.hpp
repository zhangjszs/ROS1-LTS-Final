#pragma once

#include <ros/ros.h>

#include <cmath>
#include <sstream>
#include <string>

#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/HUAT_PathLimitsV2.h>
#include <autodrive_msgs/topic_contract.hpp>
#include <fsd_common/contract_utils.hpp>
#include <nav_msgs/Path.h>

namespace planning_ros {
namespace contract {

// Delegated to fsd_common::contract
using fsd_common::contract::DecodeConeConfidenceScore;
using fsd_common::contract::EncodeConeConfidenceScaled;
using fsd_common::contract::NormalizeFrameId;
using fsd_common::contract::NormalizeInputStamp;

// B9: Timestamp semantics
// - msg.header.stamp  = sensor/input timestamp (from upstream message, normalized)
// - msg.stamp         = publish wall-clock time (ros::Time::now() at publish)
// - msg.tracklimits.stamp = same as msg.stamp (co-published)
inline void FinalizePathLimitsMessage(
    autodrive_msgs::HUAT_PathLimits& msg, const ros::Time& input_stamp,
    const std::string& frame_id = autodrive_msgs::frame_contract::kWorld) {
  msg.header.stamp = NormalizeInputStamp(input_stamp);
  msg.header.frame_id = NormalizeFrameId(frame_id, autodrive_msgs::frame_contract::kWorld);
  msg.stamp = ros::Time::now();
  msg.tracklimits.stamp = msg.stamp;
}

// B1: PathLimits array length invariant enforcement
// INVARIANT: len(path) == len(target_speeds) == len(curvatures)
// This function ensures the invariant by resizing arrays to match path length.
inline void EnforcePathDynamicsShape(autodrive_msgs::HUAT_PathLimits& msg) {
  const size_t n = msg.path.size();

  // Resize to match path length (fills with 0.0 if extending)
  msg.curvatures.resize(n, 0.0);
  msg.target_speeds.resize(n, 0.0);
}

// B1: Validate PathLimits array length invariant
// Returns true if valid, false if invariant violated
inline bool ValidatePathDynamicsShape(const autodrive_msgs::HUAT_PathLimits& msg,
                                      std::string* error = nullptr) {
  const size_t path_len = msg.path.size();
  const size_t speeds_len = msg.target_speeds.size();
  const size_t curvatures_len = msg.curvatures.size();

  if (path_len != speeds_len || path_len != curvatures_len) {
    if (error) {
      std::ostringstream oss;
      oss << "PathLimits array length mismatch: path=" << path_len
          << ", target_speeds=" << speeds_len << ", curvatures=" << curvatures_len;
      *error = oss.str();
    }
    return false;
  }

  return true;
}

// B5: Validate path quality (curvature, length)
// Returns true if valid, false if quality issues detected
// min_violations: minimum number of violating points before flagging (default 5)
// warn_scale: tolerance multiplier on kMaxCurvature for warn gate (default 1.08)
inline bool ValidatePathQuality(const autodrive_msgs::HUAT_PathLimits& msg,
                                std::string* warning = nullptr, int* curvature_violations = nullptr,
                                double* max_curvature = nullptr, int min_violations = 5,
                                double warn_scale = 1.08) {
  bool has_issues = false;
  std::ostringstream oss;

  // Check path length
  const size_t path_len = msg.path.size();
  constexpr size_t kMinPathLength = 5;  // Minimum 5 points for lookahead
  if (path_len < kMinPathLength) {
    oss << "Path too short: " << path_len << " points (min: " << kMinPathLength << "); ";
    has_issues = true;
  }

  // Check curvature limit (vehicle physical constraint)
  constexpr double kMaxCurvature = 0.222;  // 1/m, corresponds to min turning radius ~4.5m
  const double kWarnCurvature = kMaxCurvature * std::max(1.0, warn_scale);
  int violations = 0;
  double max_curv = 0.0;

  for (size_t i = 0; i < msg.curvatures.size(); ++i) {
    const double abs_curv = std::abs(msg.curvatures[i]);
    if (abs_curv > max_curv) {
      max_curv = abs_curv;
    }
    if (abs_curv > kWarnCurvature) {
      ++violations;
    }
  }

  if (violations >= std::max(1, min_violations)) {
    oss << "Curvature violations: " << violations << " points exceed " << kMaxCurvature
        << " 1/m (max: " << max_curv << " 1/m); ";
    has_issues = true;
  }

  if (curvature_violations) {
    *curvature_violations = violations;
  }
  if (max_curvature) {
    *max_curvature = max_curv;
  }

  if (warning && has_issues) {
    *warning = oss.str();
  }

  return !has_issues;
}

// ==================== V2 Message Utilities ====================

/**
 * @brief Convert V1 PathLimits to V2 format
 * @param v1 Input V1 message
 * @param v2 Output V2 message (will be populated)
 * @param mode Optional mode override (defaults to MODE_TRACK)
 */
inline void ConvertPathLimitsV1ToV2(
    const autodrive_msgs::HUAT_PathLimits& v1, autodrive_msgs::HUAT_PathLimitsV2& v2,
    uint8_t mode = autodrive_msgs::HUAT_PathLimitsV2::MODE_UNKNOWN) {
  v2.mode = mode;
  v2.replan = v1.replan;
  v2.quality_level = 2;  // Default to medium quality

  // Copy path points
  const size_t n = v1.path.size();
  v2.path = v1.path;

  // Initialize V2-specific arrays
  v2.s.resize(n, 0.0);
  v2.yaw.resize(n, 0.0);
  v2.curvatures = v1.curvatures;
  v2.target_speeds = v1.target_speeds;
  v2.target_accels.resize(n, 0.0);
  v2.time_to_point.resize(n, 0.0);

  // Compute arc length (s) and yaw
  if (n > 0) {
    v2.s[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
      const double dx = v2.path[i].x - v2.path[i - 1].x;
      const double dy = v2.path[i].y - v2.path[i - 1].y;
      v2.s[i] = v2.s[i - 1] + std::hypot(dx, dy);

      // Compute yaw from path tangent
      v2.yaw[i - 1] = std::atan2(dy, dx);
    }
    // Set last yaw same as second-to-last
    if (n > 1) {
      v2.yaw[n - 1] = v2.yaw[n - 2];
    }
  }

  // Compute max available speed
  v2.max_available_speed = 0.0;
  for (double v : v2.target_speeds) {
    if (v > v2.max_available_speed) {
      v2.max_available_speed = v;
    }
  }

  // Default values for other fields
  v2.lookahead_distance = 5.0;
  v2.track_width = 3.0;
  v2.finish_line_detected = false;
}

/**
 * @brief Finalize V2 PathLimits message with timestamps and frame info
 * @param msg V2 message to finalize (modified in place)
 * @param input_stamp Input data timestamp
 * @param frame_id Frame ID (defaults to "world")
 */
inline void FinalizePathLimitsV2Message(
    autodrive_msgs::HUAT_PathLimitsV2& msg, const ros::Time& input_stamp,
    const std::string& frame_id = autodrive_msgs::frame_contract::kWorld) {
  msg.header.stamp = NormalizeInputStamp(input_stamp);
  msg.header.frame_id = NormalizeFrameId(frame_id, autodrive_msgs::frame_contract::kWorld);
  msg.stamp = ros::Time::now();
}

/**
 * @brief Validate V2 PathLimits message array shape consistency
 * @param msg V2 message to validate
 * @param error Optional error message output
 * @return true if all arrays have consistent length, false otherwise
 */
inline bool ValidatePathLimitsV2Shape(const autodrive_msgs::HUAT_PathLimitsV2& msg,
                                      std::string* error = nullptr) {
  const size_t path_len = msg.path.size();
  const size_t s_len = msg.s.size();
  const size_t yaw_len = msg.yaw.size();
  const size_t curv_len = msg.curvatures.size();
  const size_t speeds_len = msg.target_speeds.size();
  const size_t accels_len = msg.target_accels.size();
  const size_t time_len = msg.time_to_point.size();

  bool valid = true;
  std::ostringstream oss;

  if (path_len != s_len) {
    oss << "s array length mismatch: path=" << path_len << ", s=" << s_len << "; ";
    valid = false;
  }
  if (path_len != yaw_len) {
    oss << "yaw array length mismatch: path=" << path_len << ", yaw=" << yaw_len << "; ";
    valid = false;
  }
  if (path_len != curv_len) {
    oss << "curvatures array length mismatch: path=" << path_len << ", curvatures=" << curv_len
        << "; ";
    valid = false;
  }
  if (path_len != speeds_len) {
    oss << "target_speeds array length mismatch: path=" << path_len
        << ", target_speeds=" << speeds_len << "; ";
    valid = false;
  }
  if (path_len != accels_len) {
    oss << "target_accels array length mismatch: path=" << path_len
        << ", target_accels=" << accels_len << "; ";
    valid = false;
  }
  if (path_len != time_len) {
    oss << "time_to_point array length mismatch: path=" << path_len
        << ", time_to_point=" << time_len << "; ";
    valid = false;
  }

  if (!valid && error) {
    *error = oss.str();
  }

  return valid;
}

}  // namespace contract
}  // namespace planning_ros
