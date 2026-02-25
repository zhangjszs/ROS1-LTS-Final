#pragma once
#include "vision_core/types.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace vision_core {

struct HsvRange {
  cv::Scalar lower;
  cv::Scalar upper;
};

struct FallbackConfig {
  HsvRange blue   = {{100, 80, 50},  {130, 255, 255}};
  HsvRange yellow = {{15, 80, 50},   {45, 255, 255}};
  HsvRange orange = {{5, 100, 100},  {20, 255, 255}};
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
  void detectByColor(const cv::Mat& hsv, std::vector<Detection>& out,
                     const HsvRange& range, uint8_t color_type) const;
  FallbackConfig config_;
};

}  // namespace vision_core
