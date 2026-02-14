#pragma once

#include <fsd_common/perf_stats_base.hpp>

namespace localization_ros {

struct LocPerfSample {
  double t_opt_ms = 0.0;        // iSAM2 optimization time
  double t_preint_ms = 0.0;     // IMU preintegration accumulation time
  double t_assoc_ms = 0.0;      // data association time
  double t_prune_ms = 0.0;      // landmark pruning time
  double t_total_ms = 0.0;      // total keyframe processing time
  double landmark_count = 0.0;
  double chi2_normalized = 0.0;
  double cone_match_ratio = 0.0;
  double gnss_availability = 0.0;  // 1.0 if GNSS used this frame, 0.0 otherwise
};

class LocPerfStats : public fsd_common::perf::PerfStatsBase<LocPerfSample> {
 public:
  using Base = fsd_common::perf::PerfStatsBase<LocPerfSample>;
  using MetricStats = fsd_common::perf::MetricStats;

  struct Snapshot {
    MetricStats t_opt_ms;
    MetricStats t_preint_ms;
    MetricStats t_assoc_ms;
    MetricStats t_prune_ms;
    MetricStats t_total_ms;
    MetricStats landmark_count;
    MetricStats chi2_normalized;
    MetricStats cone_match_ratio;
    MetricStats gnss_availability;
  };

  LocPerfStats() {
    SetDescriptors({
        {"t_opt_ms", [](const LocPerfSample &s) { return s.t_opt_ms; }},
        {"t_preint_ms", [](const LocPerfSample &s) { return s.t_preint_ms; }},
        {"t_assoc_ms", [](const LocPerfSample &s) { return s.t_assoc_ms; }},
        {"t_prune_ms", [](const LocPerfSample &s) { return s.t_prune_ms; }},
        {"t_total_ms", [](const LocPerfSample &s) { return s.t_total_ms; }},
        {"LM", [](const LocPerfSample &s) { return s.landmark_count; }},
        {"chi2", [](const LocPerfSample &s) { return s.chi2_normalized; }},
        {"match", [](const LocPerfSample &s) { return s.cone_match_ratio; }},
        {"gnss", [](const LocPerfSample &s) { return s.gnss_availability; }},
    });
  }

  Snapshot SnapshotStats() const {
    auto snap_vec = Base::SnapshotStats();
    Snapshot snap;
    snap.t_opt_ms = snap_vec[0].second;
    snap.t_preint_ms = snap_vec[1].second;
    snap.t_assoc_ms = snap_vec[2].second;
    snap.t_prune_ms = snap_vec[3].second;
    snap.t_total_ms = snap_vec[4].second;
    snap.landmark_count = snap_vec[5].second;
    snap.chi2_normalized = snap_vec[6].second;
    snap.cone_match_ratio = snap_vec[7].second;
    snap.gnss_availability = snap_vec[8].second;
    return snap;
  }
};

}  // namespace localization_ros
