#include "vision_core/image_enhancer.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

namespace vision_core {

ImageEnhancer::ImageEnhancer(const EnhancerConfig& c) : config_(c) {}

cv::Mat ImageEnhancer::enhance(const cv::Mat& bgr, ImageQuality quality) const {
  if (quality == ImageQuality::GOOD || quality == ImageQuality::UNUSABLE)
    return bgr.clone();

  cv::Mat out = bgr.clone();
  if (config_.auto_clahe)
    applyCLAHE(out);

  cv::Mat gray;
  cv::cvtColor(out, gray, cv::COLOR_BGR2GRAY);
  float brightness = static_cast<float>(cv::mean(gray).val[0]);
  if (config_.auto_gamma)
    applyGamma(out, brightness);

  if (quality == ImageQuality::POOR) {
    if (config_.denoise_on_poor)
      applyDenoise(out);
    if (config_.sharpen_on_blur)
      applySharpen(out);
  }
  return out;
}

void ImageEnhancer::applyCLAHE(cv::Mat& bgr) const {
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(lab, channels);
  auto clahe = cv::createCLAHE(config_.clahe_clip_limit,
                               cv::Size(config_.clahe_grid_size, config_.clahe_grid_size));
  clahe->apply(channels[0], channels[0]);
  cv::merge(channels, lab);
  cv::cvtColor(lab, bgr, cv::COLOR_Lab2BGR);
}

void ImageEnhancer::applyGamma(cv::Mat& bgr, float brightness) const {
  float gamma = 1.0f;
  if (brightness < 80.0f)
    gamma = 0.6f;
  else if (brightness > 180.0f)
    gamma = 1.5f;
  else
    return;

  cv::Mat lut(1, 256, CV_8U);
  for (int i = 0; i < 256; ++i)
    lut.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(std::pow(i / 255.0, gamma) * 255.0);
  cv::LUT(bgr, lut, bgr);
}

void ImageEnhancer::applyDenoise(cv::Mat& bgr) const {
  cv::Mat small, denoised;
  cv::resize(bgr, small, cv::Size(), 0.5, 0.5);
  cv::bilateralFilter(small, denoised, 5, 50, 50);
  cv::resize(denoised, bgr, bgr.size());
}

void ImageEnhancer::applySharpen(cv::Mat& bgr) const {
  cv::Mat blurred;
  cv::GaussianBlur(bgr, blurred, cv::Size(0, 0), 2.0);
  cv::addWeighted(bgr, 1.5, blurred, -0.5, 0, bgr);
}

}  // namespace vision_core
