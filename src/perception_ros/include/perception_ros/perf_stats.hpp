#pragma once

#include <fsd_common/perf_stats_base.hpp>

namespace perception_ros {

struct PerfSample {
  double t_pass_ms = 0.0;
  double t_ground_ms = 0.0;
  double t_cluster_ms = 0.0;
  double t_total_ms = 0.0;
  double n_points = 0.0;
  double n_clusters = 0.0;
  double n_detections = 0.0;
  double bytes_pub = 0.0;
};

class PerfStats : public fsd_common::perf::PerfStatsBase<PerfSample> {
 public:
  using Base = fsd_common::perf::PerfStatsBase<PerfSample>;
  using MetricStats = fsd_common::perf::MetricStats;

  struct Snapshot {
    MetricStats t_pass_ms;
    MetricStats t_ground_ms;
    MetricStats t_cluster_ms;
    MetricStats t_total_ms;
    MetricStats n_points;
    MetricStats n_clusters;
    MetricStats n_detections;
    MetricStats bytes_pub;
  };

  PerfStats() {
    SetDescriptors({
        {"t_pass_ms", [](const PerfSample &s) { return s.t_pass_ms; }},
        {"t_ground_ms", [](const PerfSample &s) { return s.t_ground_ms; }},
        {"t_cluster_ms", [](const PerfSample &s) { return s.t_cluster_ms; }},
        {"t_total_ms", [](const PerfSample &s) { return s.t_total_ms; }},
        {"N", [](const PerfSample &s) { return s.n_points; }},
        {"K", [](const PerfSample &s) { return s.n_clusters; }},
        {"D", [](const PerfSample &s) { return s.n_detections; }},
        {"bytes", [](const PerfSample &s) { return s.bytes_pub; }},
    });
  }

  Snapshot SnapshotStats() const {
    auto snap_vec = Base::SnapshotStats();
    Snapshot snap;
    snap.t_pass_ms = snap_vec[0].second;
    snap.t_ground_ms = snap_vec[1].second;
    snap.t_cluster_ms = snap_vec[2].second;
    snap.t_total_ms = snap_vec[3].second;
    snap.n_points = snap_vec[4].second;
    snap.n_clusters = snap_vec[5].second;
    snap.n_detections = snap_vec[6].second;
    snap.bytes_pub = snap_vec[7].second;
    return snap;
  }
};

}  // namespace perception_ros

// Backward-compatible global aliases.
using PerfSample = perception_ros::PerfSample;
using PerfStats = perception_ros::PerfStats;
