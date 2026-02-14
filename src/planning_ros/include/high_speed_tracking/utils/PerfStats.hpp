#pragma once

#include <fsd_common/perf_stats_base.hpp>

namespace planning_ros {

struct PerfSample {
  double t_delaunay_ms = 0.0;
  double t_way_ms = 0.0;
  double t_total_ms = 0.0;
  double n_points = 0.0;
  double n_triangles = 0.0;
  double n_edges = 0.0;
  double bytes_pub = 0.0;
};

class PerfStats : public fsd_common::perf::PerfStatsBase<PerfSample> {
 public:
  PerfStats() {
    SetDescriptors({
        {"t_delaunay_ms", [](const PerfSample &s) { return s.t_delaunay_ms; }},
        {"t_way_ms", [](const PerfSample &s) { return s.t_way_ms; }},
        {"t_total_ms", [](const PerfSample &s) { return s.t_total_ms; }},
        {"N", [](const PerfSample &s) { return s.n_points; }},
        {"T", [](const PerfSample &s) { return s.n_triangles; }},
        {"E", [](const PerfSample &s) { return s.n_edges; }},
        {"bytes", [](const PerfSample &s) { return s.bytes_pub; }},
    });
  }
};

}  // namespace planning_ros

// Backward-compatible global aliases.
using PerfSample = planning_ros::PerfSample;
using PerfStats = planning_ros::PerfStats;
