#include <gtest/gtest.h>
#include "vision_core/image_quality.hpp"

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
  cv::randu(img, cv::Scalar(60, 60, 60), cv::Scalar(200, 200, 200));
  auto m = assessor.assess(img);
  EXPECT_EQ(m.overall, ImageQuality::GOOD);
  EXPECT_GT(m.blur_score, 200.0f);
}
