#include "vision_core/fallback_detector.hpp"

#include <opencv2/imgproc.hpp>

namespace vision_core {

FallbackDetector::FallbackDetector(const FallbackConfig& c) : config_(c) {}

std::vector<Detection> FallbackDetector::detect(const cv::Mat& bgr) const {
  if (bgr.empty())
    return {};
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  std::vector<Detection> results;
  // Vision outputs unified YELLOW (1), size classification done by LiDAR
  // Cone types: BLUE=0, YELLOW=1, RED=3, NONE=4
  // Red is detected in two hue ranges (wraps around 0/180)
  detectByColor(hsv, results, config_.blue, ConeColorType::BLUE);
  detectByColor(hsv, results, config_.yellow, ConeColorType::YELLOW);
  detectByColor(hsv, results, config_.red_low, ConeColorType::RED);
  detectByColor(hsv, results, config_.red_high, ConeColorType::RED);
  return results;
}

void FallbackDetector::detectByColor(const cv::Mat& hsv, std::vector<Detection>& out,
                                     const HsvRange& range, uint8_t color_type) const {
  cv::Mat mask;
  cv::inRange(hsv, range.lower, range.upper, mask);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area < config_.min_area || area > config_.max_area)
      continue;

    cv::Rect bbox = cv::boundingRect(contour);
    float aspect = static_cast<float>(bbox.width) / bbox.height;
    if (aspect < config_.min_aspect || aspect > config_.max_aspect)
      continue;

    float fill = static_cast<float>(area) / (bbox.width * bbox.height);
    if (fill < config_.min_fill_ratio)
      continue;

    Detection d;
    d.x = bbox.x + bbox.width * 0.5f;
    d.y = bbox.y + bbox.height * 0.5f;
    d.w = static_cast<float>(bbox.width);
    d.h = static_cast<float>(bbox.height);
    d.class_id = color_type;
    d.color_type = color_type;
    d.confidence = 0.3f * fill + 0.2f;
    out.push_back(d);
  }
}

}  // namespace vision_core
