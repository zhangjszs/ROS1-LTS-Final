#pragma once

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

inline void EnforcePathDynamicsShape(autodrive_msgs::HUAT_PathLimits &msg)
{
  const size_t n = msg.path.size();
  msg.curvatures.resize(n, 0.0);
  msg.target_speeds.resize(n, 0.0);
}

}  // namespace contract
}  // namespace planning_ros
