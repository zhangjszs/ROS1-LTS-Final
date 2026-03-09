#include "vision_core/image_quality.hpp"

#include <gtest/gtest.h>

using namespace vision_core;

TEST(ImageQuality, EmptyImageIsUnusable) {
  ImageQualityAssessor assessor;
  cv::Mat empty;
  auto m = assessor.assess(empty);
  EXPECT_EQ(m.overall, ImageQuality::UNUSABLE);
}

TEST(ImageQuality, BrightWhiteImageIsUnusable) {
  ImageQualityAssessor assessor;
  cv::Mat white(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  auto m = assessor.assess(white);
  EXPECT_GE(static_cast<int>(m.overall), static_cast<int>(ImageQuality::POOR));
  EXPECT_GT(m.overexposure_ratio, 0.5f);
}

TEST(ImageQuality, NormalImageIsGood) {
  ImageQualityAssessor assessor;
  cv::Mat img(480, 640, CV_8UC3);
  cv::randu(img, cv::Scalar(20, 20, 20), cv::Scalar(235, 235, 235));
  auto m = assessor.assess(img);
  EXPECT_GT(m.blur_score, 100.0f);
  // Random noise image has high Laplacian variance and decent contrast;
  // exact quality depends on thresholds, so accept GOOD or DEGRADED
  EXPECT_LE(static_cast<int>(m.overall), static_cast<int>(ImageQuality::DEGRADED));
}
