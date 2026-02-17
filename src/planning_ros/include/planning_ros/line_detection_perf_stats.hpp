#pragma once

#include <fsd_common/perf_stats_base.hpp>

namespace planning_ros {

struct LinePerfSample {
  double t_total_ms = 0.0;
  double n_cones = 0.0;
  double path_size = 0.0;
};

class LinePerfStats : public fsd_common::perf::PerfStatsBase<LinePerfSample> {
 public:
  LinePerfStats() {
    SetDescriptors({
        {"t_total_ms", [](const LinePerfSample &s) { return s.t_total_ms; }},
        {"n_cones", [](const LinePerfSample &s) { return s.n_cones; }},
        {"path_size", [](const LinePerfSample &s) { return s.path_size; }},
    });
  }
};

}  // namespace planning_ros
