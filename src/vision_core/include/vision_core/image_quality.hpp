#pragma once
#include "vision_core/types.hpp"

#include <opencv2/core.hpp>

namespace vision_core {

struct QualityThresholds {
  float blur_good = 200.0f;
  float blur_degraded = 100.0f;
  float blur_poor = 50.0f;
  float brightness_low = 40.0f;
  float brightness_high = 220.0f;
  float brightness_very_low = 15.0f;
  float brightness_very_high = 250.0f;
  float overexposure_limit = 0.3f;
  float underexposure_limit = 0.3f;
  float overexposure_unusable = 0.5f;
  float underexposure_unusable = 0.5f;
};

class ImageQualityAssessor {
 public:
  explicit ImageQualityAssessor(const QualityThresholds& thresholds = {});
  QualityMetrics assess(const cv::Mat& bgr_image) const;

 private:
  QualityThresholds thresholds_;
};

}  // namespace vision_core
