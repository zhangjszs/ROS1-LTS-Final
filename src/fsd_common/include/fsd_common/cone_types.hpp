#ifndef FSD_COMMON_CONE_TYPES_HPP_
#define FSD_COMMON_CONE_TYPES_HPP_

#include <cstdint>

namespace fsd_common {

/// Canonical cone type constants used across all packages.
/// These values match the HUAT_Cone message definition.
struct ConeType {
  static inline constexpr std::uint8_t kBlue = 0;
  static inline constexpr std::uint8_t kYellowSmall = 1;
  static inline constexpr std::uint8_t kYellowBig = 2;
  static inline constexpr std::uint8_t kRed = 3;
  static inline constexpr std::uint8_t kNone = 4;

  /// Check if a cone type is a valid colored cone (not NONE)
  static constexpr bool IsColored(std::uint8_t type) {
    return type == kBlue || type == kRed || type == kYellowSmall || type == kYellowBig;
  }

  /// Check if a cone type is a boundary cone (blue or red)
  static constexpr bool IsBoundary(std::uint8_t type) {
    return type == kBlue || type == kRed;
  }

  /// Check if a cone type is yellow (small or big)
  static constexpr bool IsYellow(std::uint8_t type) {
    return type == kYellowSmall || type == kYellowBig;
  }
};

// Legacy aliases for backward compatibility
constexpr std::uint8_t kConeBlue = ConeType::kBlue;
constexpr std::uint8_t kConeYellowSmall = ConeType::kYellowSmall;
constexpr std::uint8_t kConeYellowBig = ConeType::kYellowBig;
constexpr std::uint8_t kConeRed = ConeType::kRed;
constexpr std::uint8_t kConeNone = ConeType::kNone;

// Legacy aliases used by localization_core (mapped to new values)
constexpr std::uint8_t kConeYellow = ConeType::kYellowSmall;
constexpr std::uint8_t kConeOrangeSmall = ConeType::kYellowSmall;
constexpr std::uint8_t kConeOrangeBig = ConeType::kYellowBig;

}  // namespace fsd_common

#endif  // FSD_COMMON_CONE_TYPES_HPP_
