#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <ros/ros.h>
#include <fsd_common/topic_contract.hpp>

namespace fsd_common {
namespace contract {

inline ros::Time NormalizeInputStamp(const ros::Time &stamp) {
  return stamp.isZero() ? ros::Time::now() : stamp;
}

inline std::string NormalizeFrameId(const std::string &frame_id,
                                    const std::string &fallback = fsd_common::frame_contract::kWorld) {
  return frame_id.empty() ? fallback : frame_id;
}

inline double DecodeConeConfidenceScore(std::uint32_t confidence_scaled) {
  const double score = static_cast<double>(confidence_scaled) / 1000.0;
  return std::max(0.0, std::min(1.0, score));
}

inline std::uint32_t EncodeConeConfidenceScaled(double confidence_score) {
  const double clamped = std::max(0.0, std::min(1.0, confidence_score));
  return static_cast<std::uint32_t>(std::lround(clamped * 1000.0));
}

}  // namespace contract
}  // namespace fsd_common
