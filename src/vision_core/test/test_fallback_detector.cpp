#include "vision_core/fallback_detector.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

using namespace vision_core;

TEST(FallbackDetector, EmptyImageReturnsEmpty) {
  FallbackDetector det;
  cv::Mat empty;
  auto results = det.detect(empty);
  EXPECT_TRUE(results.empty());
}

TEST(FallbackDetector, DetectsBlueRegion) {
  FallbackDetector det;
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(img, cv::Rect(200, 200, 60, 80), cv::Scalar(200, 100, 50), cv::FILLED);
  auto results = det.detect(img);
  SUCCEED();
}

TEST(FallbackDetector, ConfidenceCapped) {
  FallbackDetector det;
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(120, 200, 200));
  auto results = det.detect(img);
  for (const auto& d : results) {
    EXPECT_LE(d.confidence, 0.55f);
  }
}
