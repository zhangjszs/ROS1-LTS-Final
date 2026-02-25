#pragma once
#include "vision_core/types.hpp"
#include <opencv2/core.hpp>

namespace vision_core {

struct EnhancerConfig {
  bool auto_clahe = true;
  float clahe_clip_limit = 2.0f;
  int clahe_grid_size = 8;
  bool auto_gamma = true;
  bool denoise_on_poor = true;
  bool sharpen_on_blur = true;
};

class ImageEnhancer {
public:
  explicit ImageEnhancer(const EnhancerConfig& config = {});
  cv::Mat enhance(const cv::Mat& bgr, ImageQuality quality) const;
private:
  void applyCLAHE(cv::Mat& bgr) const;
  void applyGamma(cv::Mat& bgr, float brightness) const;
  void applyDenoise(cv::Mat& bgr) const;
  void applySharpen(cv::Mat& bgr) const;
  EnhancerConfig config_;
};

}  // namespace vision_core
