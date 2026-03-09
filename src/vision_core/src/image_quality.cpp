#include "vision_core/image_quality.hpp"

#include <opencv2/imgproc.hpp>

namespace vision_core {

ImageQualityAssessor::ImageQualityAssessor(const QualityThresholds& t) : thresholds_(t) {}

QualityMetrics ImageQualityAssessor::assess(const cv::Mat& bgr) const {
  QualityMetrics m{};
  if (bgr.empty()) {
    m.overall = ImageQuality::UNUSABLE;
    return m;
  }

  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  // Blur: Laplacian variance
  cv::Mat lap;
  cv::Laplacian(gray, lap, CV_64F);
  cv::Scalar mu, sigma;
  cv::meanStdDev(lap, mu, sigma);
  m.blur_score = static_cast<float>(sigma.val[0] * sigma.val[0]);

  // Brightness + contrast
  cv::meanStdDev(gray, mu, sigma);
  m.brightness = static_cast<float>(mu.val[0]);
  m.contrast = static_cast<float>(sigma.val[0]);

  // Over/under exposure ratios
  const float total = static_cast<float>(gray.total());
  m.overexposure_ratio = cv::countNonZero(gray > 240) / total;
  m.underexposure_ratio = cv::countNonZero(gray < 15) / total;

  // Classification
  const auto& t = thresholds_;
  if (m.blur_score < t.blur_poor || m.overexposure_ratio > t.overexposure_unusable ||
      m.underexposure_ratio > t.underexposure_unusable || m.brightness < t.brightness_very_low ||
      m.brightness > t.brightness_very_high) {
    m.overall = ImageQuality::UNUSABLE;
  } else if (m.blur_score < t.blur_degraded || m.brightness < t.brightness_low ||
             m.brightness > t.brightness_high || m.overexposure_ratio > t.overexposure_limit) {
    m.overall = ImageQuality::POOR;
  } else if (m.blur_score < t.blur_good || m.contrast < 30.0f) {
    m.overall = ImageQuality::DEGRADED;
  } else {
    m.overall = ImageQuality::GOOD;
  }
  return m;
}

}  // namespace vision_core
