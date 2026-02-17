#pragma once

#include <fsd_common/perf_stats_base.hpp>

namespace planning_ros {

struct SkidpadPerfSample {
  double t_total_ms = 0.0;
  double n_cones = 0.0;
  double path_size = 0.0;
};

class SkidpadPerfStats : public fsd_common::perf::PerfStatsBase<SkidpadPerfSample> {
 public:
  SkidpadPerfStats() {
    SetDescriptors({
        {"t_total_ms", [](const SkidpadPerfSample &s) { return s.t_total_ms; }},
        {"n_cones", [](const SkidpadPerfSample &s) { return s.n_cones; }},
        {"path_size", [](const SkidpadPerfSample &s) { return s.path_size; }},
    });
  }
};

}  // namespace planning_ros
