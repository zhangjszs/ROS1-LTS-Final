#pragma once
#include "vision_core/types.hpp"

#include <array>
#include <vector>

#include <opencv2/core.hpp>

namespace vision_core {

struct HsvRange {
  cv::Scalar lower;
  cv::Scalar upper;
};

struct FallbackConfig {
  // Vision outputs unified YELLOW, size classification done by LiDAR
  // Note: YELLOW_SMALL and YELLOW_BIG use same HSV range, LiDAR distinguishes by size
  HsvRange blue = {{100, 100, 70}, {130, 255, 255}};      // Blue: higher S/V threshold
  HsvRange yellow = {{20, 100, 80}, {35, 255, 255}};      // Unified yellow (vision only)
  HsvRange red_low = {{0, 120, 80}, {5, 255, 255}};       // Red low hue (0-5)
  HsvRange red_high = {{170, 120, 80}, {180, 255, 255}};  // Red high hue (170-180)
  double min_area = 200.0;
  double max_area = 50000.0;
  float min_aspect = 0.3f;
  float max_aspect = 1.5f;
  float min_fill_ratio = 0.35f;
};

class FallbackDetector {
 public:
  explicit FallbackDetector(const FallbackConfig& config = {});
  std::vector<Detection> detect(const cv::Mat& bgr) const;

 private:
  void detectByColor(const cv::Mat& hsv, std::vector<Detection>& out, const HsvRange& range,
                     uint8_t color_type) const;
  FallbackConfig config_;
};

}  // namespace vision_core
