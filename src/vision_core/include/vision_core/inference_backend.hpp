#pragma once
#include "vision_core/types.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace vision_core {

struct InferenceConfig {
  std::string model_path;
  int input_width = 640;
  int input_height = 640;
  float conf_threshold = 0.5f;
  float nms_threshold = 0.45f;
  bool use_fp16 = true;
  int num_threads = 2;
  int num_classes = 5;
};

class InferenceBackend {
public:
  virtual ~InferenceBackend() = default;
  virtual bool initialize(const InferenceConfig& config) = 0;
  virtual std::vector<Detection> detect(const cv::Mat& bgr) = 0;
  virtual std::string backendName() const = 0;
  virtual bool isReady() const = 0;
};

std::unique_ptr<InferenceBackend> createBackend(const std::string& type);

}  // namespace vision_core
