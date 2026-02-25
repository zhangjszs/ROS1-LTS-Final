#pragma once
#include "vision_core/types.hpp"
#include <vector>

namespace vision_core {

std::vector<Detection> filterTopK(std::vector<Detection> dets, int max_det);

}  // namespace vision_core
