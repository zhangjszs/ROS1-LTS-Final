#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <autodrive_msgs/HUAT_PathLimits.h>
#include <ros/ros.h>

namespace planning_ros {
namespace contract {

inline ros::Time NormalizeInputStamp(const ros::Time &stamp)
{
  return stamp.isZero() ? ros::Time::now() : stamp;
}

inline std::string NormalizeFrameId(const std::string &frame_id, const std::string &fallback = "world")
{
  return frame_id.empty() ? fallback : frame_id;
}

inline void FinalizePathLimitsMessage(autodrive_msgs::HUAT_PathLimits &msg,
                                      const ros::Time &input_stamp,
                                      const std::string &frame_id = "world")
{
  msg.header.stamp = NormalizeInputStamp(input_stamp);
  msg.header.frame_id = NormalizeFrameId(frame_id, "world");
  msg.stamp = ros::Time::now();
  msg.tracklimits.stamp = msg.stamp;
}

inline void EnforcePathDynamicsShape(autodrive_msgs::HUAT_PathLimits &msg)
{
  const size_t n = msg.path.size();
  msg.curvatures.resize(n, 0.0);
  msg.target_speeds.resize(n, 0.0);
}

inline double DecodeConeConfidenceScore(std::uint32_t confidence_scaled)
{
  const double score = static_cast<double>(confidence_scaled) / 1000.0;
  return std::max(0.0, std::min(1.0, score));
}

inline std::uint32_t EncodeConeConfidenceScaled(double confidence_score)
{
  const double clamped = std::max(0.0, std::min(1.0, confidence_score));
  return static_cast<std::uint32_t>(std::lround(clamped * 1000.0));
}

}  // namespace contract
}  // namespace planning_ros
