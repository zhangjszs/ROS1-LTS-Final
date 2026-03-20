#pragma once

#include <cstdint>

namespace perception_ros {

// Unified cone type constants:
// 0=BLUE, 1=YELLOW_SMALL, 2=YELLOW_BIG, 3=RED, 4=NONE
constexpr std::uint8_t kConeBlue = 0;
constexpr std::uint8_t kConeYellowSmall = 1;
constexpr std::uint8_t kConeYellowBig = 2;
constexpr std::uint8_t kConeRed = 3;
constexpr std::uint8_t kConeNone = 4;

// In fusion fallback paths (no/late vision sync), keep definitive boundary colors
// and map ambiguous colors by lateral sign to preserve LEFT=RED / RIGHT=BLUE semantics.
inline std::uint8_t FusionFallbackSemanticColor(std::uint8_t raw_color, float lateral_y) {
  if (raw_color == kConeBlue || raw_color == kConeRed) {
    return raw_color;
  }
  return lateral_y > 0.0f ? kConeRed : kConeBlue;
}

}  // namespace perception_ros
