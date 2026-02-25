#include "vision_core/detection_postprocess.hpp"
#include <algorithm>

namespace vision_core {

std::vector<Detection> filterTopK(std::vector<Detection> dets, int max_det) {
  if (static_cast<int>(dets.size()) <= max_det) return dets;
  std::partial_sort(dets.begin(), dets.begin() + max_det, dets.end(),
      [](const Detection& a, const Detection& b) {
        return a.confidence > b.confidence;
      });
  dets.resize(max_det);
  return dets;
}

}  // namespace vision_core
