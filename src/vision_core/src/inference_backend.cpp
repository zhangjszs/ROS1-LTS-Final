#include "vision_core/inference_backend.hpp"

#ifdef HAVE_ONNXRUNTIME
#include "vision_core/onnx_backend.hpp"
#endif

namespace vision_core {

std::unique_ptr<InferenceBackend> createBackend(const std::string& type) {
#ifdef HAVE_ONNXRUNTIME
  if (type == "onnx") return std::make_unique<OnnxBackend>();
#endif
  (void)type;
  return nullptr;
}

}  // namespace vision_core
