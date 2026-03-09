#pragma once
#include "vision_core/inference_backend.hpp"

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace vision_core {

class OnnxBackend : public InferenceBackend {
 public:
  OnnxBackend();
  ~OnnxBackend() override;
  bool initialize(const InferenceConfig& config) override;
  std::vector<Detection> detect(const cv::Mat& bgr) override;
  std::string backendName() const override { return "onnx"; }
  bool isReady() const override { return ready_; }

 private:
  cv::Mat preprocess(const cv::Mat& bgr) const;
  std::vector<Detection> postprocess(const float* output, int num_proposals, int num_classes) const;

  InferenceConfig config_;
  bool ready_ = false;

#ifdef HAVE_ONNXRUNTIME
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "vision"};
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo mem_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
#endif
};

}  // namespace vision_core
