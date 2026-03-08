#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace vision_core {

// Color type enum — mirrors autodrive_msgs/HUAT_ConeDetections color_types
enum ConeColorType : uint8_t {
  BLUE = 0,
  YELLOW = 1,
  ORANGE_SMALL = 2,
  ORANGE_BIG = 3,
  NONE = 4,
  RED = 5
};

struct Detection {
  float x, y, w, h;        // bbox center + size (pixels)
  float confidence;         // 0.0 ~ 1.0
  uint8_t class_id;         // model class index (0-4)
  uint8_t color_type;       // mapped ConeColorType
};

enum class ImageQuality : uint8_t {
  GOOD = 0,
  DEGRADED = 1,
  POOR = 2,
  UNUSABLE = 3
};

struct QualityMetrics {
  float blur_score;
  float brightness;
  float contrast;
  float overexposure_ratio;
  float underexposure_ratio;
  ImageQuality overall;
};

// Model class_id (0-4) → ConeColorType mapping
// model: 0=blue, 1=yellow, 2=orange_small, 3=orange_big, 4=red
inline ConeColorType modelClassToColorType(uint8_t cls) {
  constexpr ConeColorType kMap[] = {BLUE, YELLOW, ORANGE_SMALL, ORANGE_BIG, RED};
  return (cls < 5) ? kMap[cls] : NONE;
}

// Optional model-specific remap: class_id -> ConeColorType enum value.
// Example for classes [red, blue, yellow]: [5, 0, 1].
inline ConeColorType modelClassToColorType(
    uint8_t cls, const std::vector<uint8_t>& class_to_color_map) {
  if (!class_to_color_map.empty()) {
    if (cls < class_to_color_map.size()) {
      const auto mapped = class_to_color_map[cls];
      return (mapped <= RED) ? static_cast<ConeColorType>(mapped) : NONE;
    }
    return NONE;
  }
  return modelClassToColorType(cls);
}

}  // namespace vision_core
