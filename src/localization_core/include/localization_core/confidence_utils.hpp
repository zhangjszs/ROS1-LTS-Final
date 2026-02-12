#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace localization_core {
namespace confidence {

inline std::uint32_t EncodeScaled(double score)
{
  const double clamped = std::max(0.0, std::min(1.0, score));
  return static_cast<std::uint32_t>(std::lround(clamped * 1000.0));
}

inline double DecodeScaled(std::uint32_t scaled)
{
  const double score = static_cast<double>(scaled) / 1000.0;
  return std::max(0.0, std::min(1.0, score));
}

}  // namespace confidence
}  // namespace localization_core
