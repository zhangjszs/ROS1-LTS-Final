#include "vision_core/onnx_backend.hpp"
#include "vision_core/detection_postprocess.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>

namespace vision_core {

std::unique_ptr<InferenceBackend> createBackend(const std::string& type) {
#ifdef HAVE_ONNXRUNTIME
  if (type == "onnx") return std::make_unique<OnnxBackend>();
#endif
  (void)type;
  return nullptr;
}

OnnxBackend::OnnxBackend() = default;
OnnxBackend::~OnnxBackend() = default;

bool OnnxBackend::initialize(const InferenceConfig& config) {
#ifdef HAVE_ONNXRUNTIME
  config_ = config;
  std::ifstream f(config_.model_path);
  if (!f.good()) {
    ready_ = false;
    return false;
  }

  try {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(config_.num_threads);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(
        env_, config_.model_path.c_str(), opts);
    ready_ = true;
    return true;
  } catch (const Ort::Exception& e) {
    ready_ = false;
    return false;
  }
#else
  (void)config;
  ready_ = false;
  return false;
#endif
}

cv::Mat OnnxBackend::preprocess(const cv::Mat& bgr) const {
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(config_.input_width, config_.input_height));
  cv::Mat blob;
  cv::dnn::blobFromImage(resized, blob, 1.0 / 255.0,
      cv::Size(config_.input_width, config_.input_height),
      cv::Scalar(), true, false, CV_32F);
  return blob;
}

std::vector<Detection> OnnxBackend::detect(const cv::Mat& bgr) {
#ifdef HAVE_ONNXRUNTIME
  if (!ready_) return {};

  cv::Mat blob = preprocess(bgr);
  std::array<int64_t, 4> input_shape = {
      1, 3, config_.input_height, config_.input_width};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info_, blob.ptr<float>(), blob.total(),
      input_shape.data(), input_shape.size());

  const char* input_names[] = {"images"};
  const char* output_names[] = {"output0"};
  auto outputs = session_->Run(Ort::RunOptions{nullptr},
      input_names, &input_tensor, 1, output_names, 1);

  auto& out_tensor = outputs[0];
  auto shape = out_tensor.GetTensorTypeAndShapeInfo().GetShape();
  const int cols = static_cast<int>(shape[2]);
  const float* data = out_tensor.GetTensorData<float>();

  auto raw = postprocess(data, cols);

  const float sx = static_cast<float>(bgr.cols) / config_.input_width;
  const float sy = static_cast<float>(bgr.rows) / config_.input_height;
  for (auto& d : raw) {
    d.x *= sx; d.w *= sx;
    d.y *= sy; d.h *= sy;
  }
  return raw;
#else
  (void)bgr;
  return {};
#endif
}

std::vector<Detection> OnnxBackend::postprocess(
    const float* data, int num_proposals) const {
  const int nc = config_.num_classes;
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int i = 0; i < num_proposals; ++i) {
    float cx = data[0 * num_proposals + i];
    float cy = data[1 * num_proposals + i];
    float w  = data[2 * num_proposals + i];
    float h  = data[3 * num_proposals + i];

    float max_score = 0.0f;
    int max_cls = 0;
    for (int c = 0; c < nc; ++c) {
      float s = data[(4 + c) * num_proposals + i];
      if (s > max_score) { max_score = s; max_cls = c; }
    }

    if (max_score < config_.conf_threshold) continue;

    boxes.emplace_back(
        static_cast<int>(cx - w / 2), static_cast<int>(cy - h / 2),
        static_cast<int>(w), static_cast<int>(h));
    scores.push_back(max_score);
    class_ids.push_back(max_cls);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, config_.conf_threshold,
                     config_.nms_threshold, indices);

  std::vector<Detection> dets;
  for (int idx : indices) {
    Detection d;
    d.x = boxes[idx].x + boxes[idx].width * 0.5f;
    d.y = boxes[idx].y + boxes[idx].height * 0.5f;
    d.w = static_cast<float>(boxes[idx].width);
    d.h = static_cast<float>(boxes[idx].height);
    d.confidence = scores[idx];
    d.class_id = static_cast<uint8_t>(class_ids[idx]);
    d.color_type = static_cast<uint8_t>(modelClassToColorType(d.class_id));
    dets.push_back(d);
  }
  return dets;
}

}  // namespace vision_core
