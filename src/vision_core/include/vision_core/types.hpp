#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace vision_core {

// Color type enum — mirrors autodrive_msgs/HUAT_ConeDetections color_types
// Updated: Removed ORANGE types, replaced with YELLOW_SMALL/YELLOW_BIG
// Note: Vision outputs unified YELLOW (1), LiDAR classifies into YELLOW_SMALL/YELLOW_BIG by size
enum ConeColorType : uint8_t {
  BLUE = 0,          // Blue cone
  YELLOW = 1,        // Unified yellow cone (vision output)
  YELLOW_SMALL = 1,  // Small yellow cone (alias for compatibility)
  YELLOW_BIG = 2,    // Big yellow cone
  RED = 3,           // Red cone
  NONE = 4           // Unknown/None
};

struct Detection {
  float x, y, w, h;    // bbox center + size (pixels)
  float confidence;    // 0.0 ~ 1.0
  uint8_t class_id;    // model class index (0-4)
  uint8_t color_type;  // mapped ConeColorType
};

enum class ImageQuality : uint8_t { GOOD = 0, DEGRADED = 1, POOR = 2, UNUSABLE = 3 };

struct QualityMetrics {
  float blur_score;
  float brightness;
  float contrast;
  float overexposure_ratio;
  float underexposure_ratio;
  ImageQuality overall;
};

// Model class_id (0-2) → ConeColorType mapping
// Vision model: 0=blue, 1=yellow, 2=red (unified yellow, no size distinction)
// Note: Vision outputs unified YELLOW (1), size classification done by LiDAR
inline ConeColorType modelClassToColorType(uint8_t cls) {
  constexpr ConeColorType kMap[] = {BLUE, YELLOW, RED};
  return (cls < 3) ? kMap[cls] : NONE;
}

// Optional model-specific remap: class_id -> ConeColorType enum value.
// Example for classes [red, blue, yellow_small]: [3, 0, 1].
// Note: Max valid enum value is now 4 (NONE), previously was 5 (RED at old value).
inline ConeColorType modelClassToColorType(uint8_t cls,
                                           const std::vector<uint8_t>& class_to_color_map) {
  if (!class_to_color_map.empty()) {
    if (cls < class_to_color_map.size()) {
      const auto mapped = class_to_color_map[cls];
      // Validate mapped value is within valid enum range (0-4)
      return (mapped <= static_cast<uint8_t>(NONE)) ? static_cast<ConeColorType>(mapped) : NONE;
    }
    return NONE;
  }
  return modelClassToColorType(cls);
}

}  // namespace vision_core
