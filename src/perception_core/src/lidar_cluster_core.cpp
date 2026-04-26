#include "perception_core/lidar_cluster_core.hpp"

#include <iomanip>
#include <sstream>

namespace {

template <class Type>
std::string num2Str(const Type value, unsigned int precision) {
  std::ostringstream out;
  if (precision > 0) {
    out.precision(precision);
  }
  out << value;
  return out.str();
}

// Task 23D: helper to count cones by far-range distance bands
// Buckets for 20-80m diagnostic histogram. Distances <=20m and >80m are
// intentionally excluded from far-band counting (they are near-range and
// extreme-range respectively, tracked by other metrics).
inline void countFarBands(const ConeDetection& det, int& b20_30, int& b30_40, int& b40_50,
                          int& b50_60, int& b60_80) {
  if (det.distance > 20.0 && det.distance <= 30.0)
    ++b20_30;
  else if (det.distance > 30.0 && det.distance <= 40.0)
    ++b30_40;
  else if (det.distance > 40.0 && det.distance <= 50.0)
    ++b40_50;
  else if (det.distance > 50.0 && det.distance <= 60.0)
    ++b50_60;
  else if (det.distance > 60.0 && det.distance <= 80.0)
    ++b60_80;
}

}  // namespace

lidar_cluster::lidar_cluster() {
  init();
}

lidar_cluster::~lidar_cluster() = default;

void lidar_cluster::SetInputCloud(const pcl::PointCloud<PointType>::ConstPtr& cloud, uint32_t seq) {
  if (!cloud || !current_pc_ptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(lidar_mutex);
  *current_pc_ptr = *cloud;
  getPointClouds = true;
  frame_count = static_cast<int>(seq);
}

void lidar_cluster::SetInputCloud(pcl::PointCloud<PointType>::Ptr&& cloud, uint32_t seq) {
  if (!cloud) {
    return;
  }
  std::lock_guard<std::mutex> lock(lidar_mutex);
  // 零拷贝：直接接管点云所有权，避免深拷贝
  current_pc_ptr = std::move(cloud);
  getPointClouds = true;
  frame_count = static_cast<int>(seq);
}

void lidar_cluster::SetEgoMotion(const perception::EgoMotion& ego) {
  ego_motion_ = ego;
}

bool lidar_cluster::Process(LidarClusterOutput* output) {
  if (!output) {
    return false;
  }
  if (!getPointClouds || !current_pc_ptr) {
    return false;
  }

  // Task 23D: reset post-confidence funnel diagnostics
  output->postconf_20_30 = output->postconf_30_40 = output->postconf_40_50 =
      output->postconf_50_60 = output->postconf_60_80 = 0;
  output->after_dedup_20_30 = output->after_dedup_30_40 = output->after_dedup_40_50 =
      output->after_dedup_50_60 = output->after_dedup_60_80 = 0;
  output->after_tracker_20_30 = output->after_tracker_30_40 = output->after_tracker_40_50 =
      output->after_tracker_50_60 = output->after_tracker_60_80 = 0;
  output->after_topology_20_30 = output->after_topology_30_40 = output->after_topology_40_50 =
      output->after_topology_50_60 = output->after_topology_60_80 = 0;
  output->tracker_confirmed_20_30 = output->tracker_confirmed_30_40 =
      output->tracker_confirmed_40_50 = output->tracker_confirmed_50_60 =
          output->tracker_confirmed_60_80 = 0;
  output->topo_interpolated_20_30 = output->topo_interpolated_30_40 =
      output->topo_interpolated_40_50 = output->topo_interpolated_50_60 =
          output->topo_interpolated_60_80 = 0;

  std::unique_lock<std::mutex> lock(lidar_mutex);
  const size_t input_points = current_pc_ptr->points.size();

  // ===== 新流水线 =====
  // ① ROI裁剪 + 强度滤波（保留原始点云密度）
  auto startTimePassThrough = std::chrono::steady_clock::now();
  PassThroughROI(current_pc_ptr, output);
  cloud_filtered = current_pc_ptr;
  auto endTimePassThrough = std::chrono::steady_clock::now();
  auto elapsedTimePassThrough = std::chrono::duration_cast<std::chrono::microseconds>(
      endTimePassThrough - startTimePassThrough);

  output->passthrough = cloud_filtered;
  output->roi_points = cloud_filtered->points.size();
  output->roi_dropped = input_points - output->roi_points;

  // ② 地面分割（在原始密度点云上做，避免体素质心污染 min_z）
  auto startTimeSeg = std::chrono::steady_clock::now();
  const size_t pre_ground_size = cloud_filtered->points.size();
  ground_segmentation_dispatch_(cloud_filtered, g_not_ground_pc);
  auto endTimeSeg = std::chrono::steady_clock::now();
  auto elapsedTimeSeg =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeSeg - startTimeSeg);
  output->ground_removed = pre_ground_size - g_not_ground_pc->points.size();

  // ③ 降采样 + SOR（仅对非地面点，减少聚类计算量）
  PostGroundFilter(g_not_ground_pc, output);

  // ④ 多帧累积（远处点累积多帧提升检测率）
  AccumulateFrames(g_not_ground_pc);

  output->not_ground = g_not_ground_pc;

  auto startTimeCluster = std::chrono::steady_clock::now();
  if (sensor_model_ == 16) {
    clusterMethod16(output);
  } else {
    clusterMethod32(output);
  }
  auto endTimeCluster = std::chrono::steady_clock::now();
  auto elapsedTimeCluster =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeCluster - startTimeCluster);

  const std::size_t cluster_count = last_cluster_count_;
  lock.unlock();

  // Task 23D: count post-confidence candidates (funnel entry)
  for (const auto& cone : output->cones) {
    countFarBands(cone, output->postconf_20_30, output->postconf_30_40, output->postconf_40_50,
                  output->postconf_50_60, output->postconf_60_80);
  }

  // ⑤b Proximity deduplication: remove stacked/duplicate cones
  if (config_.dedup.enable && !output->cones.empty()) {
    deduplicateDetections(output->cones, output->cones_cloud);
  }

  // Task 23D: count after dedup
  for (const auto& cone : output->cones) {
    countFarBands(cone, output->after_dedup_20_30, output->after_dedup_30_40,
                  output->after_dedup_40_50, output->after_dedup_50_60, output->after_dedup_60_80);
  }

  // ⑤ Cone tracker: temporal consistency filtering
  if (tracker_enabled_ && !output->cones.empty()) {
    auto now = std::chrono::steady_clock::now();
    double dt = 0.1;  // default 10Hz
    if (last_frame_time_ >= 0.0) {
      double now_sec = std::chrono::duration<double>(now.time_since_epoch()).count();
      dt = now_sec - last_frame_time_;
      if (dt <= 0.0 || dt > 2.0)
        dt = 0.1;
    }
    last_frame_time_ = std::chrono::duration<double>(now.time_since_epoch()).count();

    // Convert ConeDetections to tracker Detections
    std::vector<perception::ConeTracker::Detection> tracker_dets;
    tracker_dets.reserve(output->cones.size());
    for (const auto& cone : output->cones) {
      perception::ConeTracker::Detection td;
      td.x = cone.centroid.x;
      td.y = cone.centroid.y;
      td.z = cone.centroid.z;
      td.confidence = cone.confidence;
      tracker_dets.push_back(td);
    }

    cone_tracker_.update(tracker_dets, dt, ego_motion_);

    // Get confirmed tracks and match back to original detections
    auto confirmed = config_.tracker.only_output_confirmed ? cone_tracker_.getConfirmedCones()
                                                           : cone_tracker_.getAllTracks();

    // Build a map from tracker detection index to original cone index
    // by matching confirmed track positions back to closest original detection
    std::vector<ConeDetection> filtered_cones;
    filtered_cones.reserve(confirmed.size());
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

    for (const auto& track : confirmed) {
      // Find closest original detection
      int best_idx = -1;
      double best_dist = config_.tracker.association_threshold;
      for (size_t j = 0; j < output->cones.size(); ++j) {
        double dx = track.x - output->cones[j].centroid.x;
        double dy = track.y - output->cones[j].centroid.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < best_dist) {
          best_dist = d;
          best_idx = static_cast<int>(j);
        }
      }
      if (best_idx >= 0) {
        ConeDetection det = output->cones[best_idx];
        // Boost confidence for confirmed tracks
        det.confidence = std::min(1.0, det.confidence + config_.tracker.confirmed_confidence_boost);
        det.track_id = track.id;  // G12: propagate tracker ID
        filtered_cones.push_back(std::move(det));
        if (output->cones[best_idx].cluster) {
          filtered_cloud->points.insert(filtered_cloud->points.end(),
                                        output->cones[best_idx].cluster->points.begin(),
                                        output->cones[best_idx].cluster->points.end());
        }
      }
    }

    output->cones = std::move(filtered_cones);
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    output->cones_cloud = filtered_cloud;

    // Populate tracker diagnostics
    auto all_tracks = cone_tracker_.getAllTracks();
    output->tracker_tentative = 0;
    output->tracker_confirmed = 0;
    for (const auto& t : all_tracks) {
      if (t.confirmed) {
        output->tracker_confirmed++;
      } else {
        output->tracker_tentative++;
      }
    }
    // deleted = input detections - confirmed output (approximate)
    output->tracker_deleted = static_cast<int>(tracker_dets.size()) - output->tracker_confirmed;
    if (output->tracker_deleted < 0)
      output->tracker_deleted = 0;
  }

  // Task 23D: count after tracker + confirmed tracks by band
  for (const auto& cone : output->cones) {
    countFarBands(cone, output->after_tracker_20_30, output->after_tracker_30_40,
                  output->after_tracker_40_50, output->after_tracker_50_60,
                  output->after_tracker_60_80);
  }
  // Count confirmed tracks that were matched back (from filtered_cones before assignment to output)
  // Note: we already counted after_tracker from output->cones, which IS the confirmed set.
  // So tracker_confirmed_* = after_tracker_* when tracker is enabled.
  output->tracker_confirmed_20_30 = output->after_tracker_20_30;
  output->tracker_confirmed_30_40 = output->after_tracker_30_40;
  output->tracker_confirmed_40_50 = output->after_tracker_40_50;
  output->tracker_confirmed_50_60 = output->after_tracker_50_60;
  output->tracker_confirmed_60_80 = output->after_tracker_60_80;

  // ⑥ Topology repair: fill gaps and remove outliers
  if (config_.topology.enable && !output->cones.empty()) {
    // Convert to TopologyCone
    std::vector<perception::TopologyCone> topo_cones;
    topo_cones.reserve(output->cones.size());
    for (size_t i = 0; i < output->cones.size(); ++i) {
      perception::TopologyCone tc;
      tc.x = output->cones[i].centroid.x;
      tc.y = output->cones[i].centroid.y;
      tc.z = output->cones[i].centroid.z;
      tc.confidence = output->cones[i].confidence;
      tc.is_interpolated = false;
      tc.original_index = static_cast<int>(i);
      topo_cones.push_back(tc);
    }

    auto repaired = topology_repair_.repair(topo_cones);

    // Rebuild output: keep original detections for non-interpolated,
    // create synthetic detections for interpolated cones
    std::vector<ConeDetection> new_cones;
    new_cones.reserve(repaired.size());
    pcl::PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>);

    for (const auto& rc : repaired) {
      if (!rc.is_interpolated && rc.original_index >= 0 &&
          rc.original_index < static_cast<int>(output->cones.size())) {
        new_cones.push_back(output->cones[rc.original_index]);
        if (output->cones[rc.original_index].cluster) {
          new_cloud->points.insert(new_cloud->points.end(),
                                   output->cones[rc.original_index].cluster->points.begin(),
                                   output->cones[rc.original_index].cluster->points.end());
        }
      } else if (rc.is_interpolated) {
        // Create synthetic detection for interpolated cone
        ConeDetection det;
        det.centroid = pcl::PointXYZ(static_cast<float>(rc.x), static_cast<float>(rc.y),
                                     static_cast<float>(rc.z));
        det.min = det.centroid;
        det.max = det.centroid;
        det.confidence = rc.confidence;
        det.distance = std::sqrt(rc.x * rc.x + rc.y * rc.y);
        det.is_cone = true;
        new_cones.push_back(std::move(det));
      }
    }

    output->cones = std::move(new_cones);
    new_cloud->width = new_cloud->points.size();
    new_cloud->height = 1;
    new_cloud->is_dense = true;
    output->cones_cloud = new_cloud;
  }

  // Task 23D: count after topology + interpolated cones by band
  for (const auto& cone : output->cones) {
    countFarBands(cone, output->after_topology_20_30, output->after_topology_30_40,
                  output->after_topology_40_50, output->after_topology_50_60,
                  output->after_topology_60_80);
  }
  // Interpolated count = after_topology - (non-interpolated kept from before topology)
  // Approximate: topo_interpolated = after_topology - after_tracker (if topology enabled)
  if (config_.topology.enable) {
    output->topo_interpolated_20_30 =
        std::max(0, output->after_topology_20_30 - output->after_tracker_20_30);
    output->topo_interpolated_30_40 =
        std::max(0, output->after_topology_30_40 - output->after_tracker_30_40);
    output->topo_interpolated_40_50 =
        std::max(0, output->after_topology_40_50 - output->after_tracker_40_50);
    output->topo_interpolated_50_60 =
        std::max(0, output->after_topology_50_60 - output->after_tracker_50_60);
    output->topo_interpolated_60_80 =
        std::max(0, output->after_topology_60_80 - output->after_tracker_60_80);
  }

  auto endTimeTotal = std::chrono::steady_clock::now();
  auto elapsedTimeTotal =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeTotal - startTimePassThrough);

  output->t_pass_ms = static_cast<double>(elapsedTimePassThrough.count()) / 1000.0;
  output->t_ground_ms = static_cast<double>(elapsedTimeSeg.count()) / 1000.0;
  output->t_cluster_ms = static_cast<double>(elapsedTimeCluster.count()) / 1000.0;
  output->t_total_ms = static_cast<double>(elapsedTimeTotal.count()) / 1000.0;
  output->input_points = input_points;
  output->total_clusters = cluster_count;

  // Count far clusters (>30m) after all post-processing
  output->clusters_far = 0;
  for (const auto& cone : output->cones) {
    if (cone.distance > 30.0) {
      ++output->clusters_far;
    }
  }

  return true;
}
