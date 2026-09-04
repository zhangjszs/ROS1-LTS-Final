#!/usr/bin/env python3
import argparse
import json
import math
import time
from collections import defaultdict

import psutil
from diagnostic_msgs.msg import DiagnosticArray

import rospy
from sensor_msgs.msg import PointCloud2

from autodrive_msgs.msg import HUAT_CarState, HUAT_ConeDetections, HUAT_PathLimits, HUAT_VehicleCmd


class TrackDriveBenchmark:
    def __init__(self, duration_sec=60, output_path=None):
        self.duration = duration_sec
        self.output_path = output_path
        self.wall_start = None
        self.cpu_process = psutil.Process()

        self.perc_frame_count = 0
        self.perc_cones_per_frame = []
        self.perc_confidences = []
        self.perc_distances = []
        self.perc_band_counts = defaultdict(list)

        self.velo_timestamps = []
        self.det_timestamps = []

        self.plan_frame_count = 0
        self.plan_path_lengths = []
        self.plan_curvatures = []
        self.plan_short_path_count = 0

        self.ctrl_frame_count = 0
        self.ctrl_deltas = []
        self.ctrl_delta_rates = []
        self.ctrl_speeds = []
        self.last_delta = None
        self.last_delta_time = None

        self.sys_velo_count = 0
        self.sys_det_count = 0
        self.sys_plan_count = 0
        self.sys_cmd_count = 0
        self.sys_cpu_samples = []
        self.sys_mem_samples_mb = []

        # --- Diagnostics aggregation ---
        self.diag_frames = 0
        self.diag_input_points = []
        self.diag_roi_points = []
        self.diag_roi_dropped = []
        self.diag_intensity_dropped = []
        self.diag_ground_removed = []
        self.diag_obstacle_dropped = []
        self.diag_clusters_total = []
        self.diag_clusters_far = []
        self.diag_n_detections = []
        self.diag_n_near = []
        self.diag_n_mid = []
        self.diag_n_far = []
        self.diag_conf_near = []
        self.diag_conf_mid = []
        self.diag_conf_far = []
        # Rejection reasons
        self.diag_rejected_by_roi = []
        self.diag_rejected_by_confidence = []
        self.diag_rejected_by_semantic = []
        self.diag_rejected_by_tracker = []
        # Confidence components
        self.diag_size_scores = []
        self.diag_shape_scores = []
        self.diag_density_scores = []
        self.diag_intensity_scores = []
        self.diag_position_scores = []
        self.diag_semantic_scores = []
        self.diag_scored_count = []
        # Model fitting
        self.diag_fitting_calls = []
        self.diag_fitting_skipped = []
        self.diag_fitting_success = []
        self.diag_fitting_fail = []
        # Tracker
        self.diag_tracker_tentative = []
        self.diag_tracker_confirmed = []
        self.diag_tracker_deleted = []
        self.diag_semantic_kills = []
        # Task 23C: far-range near-threshold diagnostics
        self.diag_far_candidates_total = []
        self.diag_far_accepted = []
        self.diag_far_rejected_by_confidence = []
        self.diag_far_conf_lt_025 = []
        self.diag_far_conf_025_035 = []
        self.diag_far_conf_035_040 = []
        self.diag_far_conf_040_045 = []
        self.diag_far_conf_045_050 = []
        self.diag_far_conf_gt_050 = []
        self.diag_far_avg_conf_rejected = []
        # Task 23D: post-confidence publication funnel
        self.diag_postconf_20_30 = []
        self.diag_postconf_30_40 = []
        self.diag_postconf_40_50 = []
        self.diag_postconf_50_60 = []
        self.diag_postconf_60_80 = []
        self.diag_after_dedup_20_30 = []
        self.diag_after_dedup_30_40 = []
        self.diag_after_dedup_40_50 = []
        self.diag_after_dedup_50_60 = []
        self.diag_after_dedup_60_80 = []
        self.diag_after_tracker_20_30 = []
        self.diag_after_tracker_30_40 = []
        self.diag_after_tracker_40_50 = []
        self.diag_after_tracker_50_60 = []
        self.diag_after_tracker_60_80 = []
        self.diag_after_topology_20_30 = []
        self.diag_after_topology_30_40 = []
        self.diag_after_topology_40_50 = []
        self.diag_after_topology_50_60 = []
        self.diag_after_topology_60_80 = []
        self.diag_tracker_confirmed_20_30 = []
        self.diag_tracker_confirmed_30_40 = []
        self.diag_tracker_confirmed_40_50 = []
        self.diag_tracker_confirmed_50_60 = []
        self.diag_tracker_confirmed_60_80 = []
        self.diag_topo_interpolated_20_30 = []
        self.diag_topo_interpolated_30_40 = []
        self.diag_topo_interpolated_40_50 = []
        self.diag_topo_interpolated_50_60 = []
        self.diag_topo_interpolated_60_80 = []
        # Performance (direct per-frame latencies from output.t_*_ms)
        self.diag_t_total_ms = []
        self.diag_t_ground_ms = []
        self.diag_t_cluster_ms = []
        self.diag_t_pass_ms = []

        rospy.init_node("trackdrive_benchmark", anonymous=True)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.on_velo)
        rospy.Subscriber(
            "/perception/lidar_cluster/detections", HUAT_ConeDetections, self.on_detection
        )
        rospy.Subscriber("/planning/pathlimits", HUAT_PathLimits, self.on_path)
        rospy.Subscriber("/vehicle/cmd", HUAT_VehicleCmd, self.on_cmd)
        rospy.Subscriber("/localization/car_state", HUAT_CarState, self.on_car_state)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.on_diagnostics)

    def _parse_diag_float(self, values, key):
        for kv in values:
            if kv.key == key:
                try:
                    return float(kv.value)
                except ValueError:
                    return None
        return None

    def _parse_diag_int(self, values, key):
        for kv in values:
            if kv.key == key:
                try:
                    return int(kv.value)
                except ValueError:
                    return None
        return None

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name != "perception_lidar":
                continue
            self.diag_frames += 1
            v = status.values
            # Pipeline funnel
            ip = self._parse_diag_int(v, "n_input_points")
            if ip is not None:
                self.diag_input_points.append(ip)
            rp = self._parse_diag_int(v, "n_clusters")
            if rp is not None:
                self.diag_clusters_total.append(rp)
            nd = self._parse_diag_int(v, "n_detections")
            if nd is not None:
                self.diag_n_detections.append(nd)
            # Stage stats
            for key, lst in [
                ("stage_roi_points", self.diag_roi_points),
                ("stage_roi_dropped", self.diag_roi_dropped),
                ("stage_intensity_dropped", self.diag_intensity_dropped),
                ("stage_ground_removed", self.diag_ground_removed),
                ("stage_obstacle_dropped", self.diag_obstacle_dropped),
                ("stage_clusters_total", self.diag_clusters_total),
                ("stage_clusters_far", self.diag_clusters_far),
            ]:
                val = self._parse_diag_int(v, key)
                if val is not None:
                    lst.append(val)
            # Distance bands
            for key, lst in [
                ("n_near", self.diag_n_near),
                ("n_mid", self.diag_n_mid),
                ("n_far", self.diag_n_far),
            ]:
                val = self._parse_diag_int(v, key)
                if val is not None:
                    lst.append(val)
            # Confidence per band
            for key, lst in [
                ("conf_near", self.diag_conf_near),
                ("conf_mid", self.diag_conf_mid),
                ("conf_far", self.diag_conf_far),
            ]:
                val = self._parse_diag_float(v, key)
                if val is not None:
                    lst.append(val)
            # Rejection reasons
            for key, lst in [
                ("rejected_by_roi", self.diag_rejected_by_roi),
                ("rejected_by_confidence", self.diag_rejected_by_confidence),
                ("rejected_by_semantic", self.diag_rejected_by_semantic),
                ("rejected_by_tracker", self.diag_rejected_by_tracker),
            ]:
                val = self._parse_diag_int(v, key)
                if val is not None:
                    lst.append(val)
            # Component scores
            for key, lst in [
                ("avg_size_score", self.diag_size_scores),
                ("avg_shape_score", self.diag_shape_scores),
                ("avg_density_score", self.diag_density_scores),
                ("avg_intensity_score", self.diag_intensity_scores),
                ("avg_position_score", self.diag_position_scores),
                ("avg_semantic_score", self.diag_semantic_scores),
            ]:
                val = self._parse_diag_float(v, key)
                if val is not None:
                    lst.append(val)
            sc = self._parse_diag_int(v, "scored_count")
            if sc is not None:
                self.diag_scored_count.append(sc)
            # Model fitting
            for key, lst in [
                ("fitting_calls_total", self.diag_fitting_calls),
                ("fitting_skipped", self.diag_fitting_skipped),
                ("fitting_success", self.diag_fitting_success),
                ("fitting_fail", self.diag_fitting_fail),
            ]:
                val = self._parse_diag_int(v, key)
                if val is not None:
                    lst.append(val)
            # Tracker
            for key, lst in [
                ("tracker_tentative", self.diag_tracker_tentative),
                ("tracker_confirmed", self.diag_tracker_confirmed),
                ("tracker_deleted", self.diag_tracker_deleted),
                ("semantic_kills", self.diag_semantic_kills),
            ]:
                val = self._parse_diag_int(v, key)
                if val is not None:
                    lst.append(val)
            # Task 23C: far-range near-threshold diagnostics
            for key, lst in [
                ("far_candidates_total", self.diag_far_candidates_total),
                ("far_accepted", self.diag_far_accepted),
                ("far_rejected_by_confidence", self.diag_far_rejected_by_confidence),
                ("far_conf_lt_025", self.diag_far_conf_lt_025),
                ("far_conf_025_035", self.diag_far_conf_025_035),
                ("far_conf_035_040", self.diag_far_conf_035_040),
                ("far_conf_040_045", self.diag_far_conf_040_045),
                ("far_conf_045_050", self.diag_far_conf_045_050),
                ("far_conf_gt_050", self.diag_far_conf_gt_050),
            ]:
                val = self._parse_diag_int(v, key)
                if val is not None:
                    lst.append(val)
            acr = self._parse_diag_float(v, "far_avg_conf_rejected")
            if acr is not None:
                self.diag_far_avg_conf_rejected.append(acr)
            # Task 23D: post-confidence publication funnel
            for key, lst in [
                ("postconf_20_30", self.diag_postconf_20_30),
                ("postconf_30_40", self.diag_postconf_30_40),
                ("postconf_40_50", self.diag_postconf_40_50),
                ("postconf_50_60", self.diag_postconf_50_60),
                ("postconf_60_80", self.diag_postconf_60_80),
                ("after_dedup_20_30", self.diag_after_dedup_20_30),
                ("after_dedup_30_40", self.diag_after_dedup_30_40),
                ("after_dedup_40_50", self.diag_after_dedup_40_50),
                ("after_dedup_50_60", self.diag_after_dedup_50_60),
                ("after_dedup_60_80", self.diag_after_dedup_60_80),
                ("after_tracker_20_30", self.diag_after_tracker_20_30),
                ("after_tracker_30_40", self.diag_after_tracker_30_40),
                ("after_tracker_40_50", self.diag_after_tracker_40_50),
                ("after_tracker_50_60", self.diag_after_tracker_50_60),
                ("after_tracker_60_80", self.diag_after_tracker_60_80),
                ("after_topology_20_30", self.diag_after_topology_20_30),
                ("after_topology_30_40", self.diag_after_topology_30_40),
                ("after_topology_40_50", self.diag_after_topology_40_50),
                ("after_topology_50_60", self.diag_after_topology_50_60),
                ("after_topology_60_80", self.diag_after_topology_60_80),
                ("tracker_confirmed_20_30", self.diag_tracker_confirmed_20_30),
                ("tracker_confirmed_30_40", self.diag_tracker_confirmed_30_40),
                ("tracker_confirmed_40_50", self.diag_tracker_confirmed_40_50),
                ("tracker_confirmed_50_60", self.diag_tracker_confirmed_50_60),
                ("tracker_confirmed_60_80", self.diag_tracker_confirmed_60_80),
                ("topo_interpolated_20_30", self.diag_topo_interpolated_20_30),
                ("topo_interpolated_30_40", self.diag_topo_interpolated_30_40),
                ("topo_interpolated_40_50", self.diag_topo_interpolated_40_50),
                ("topo_interpolated_50_60", self.diag_topo_interpolated_50_60),
                ("topo_interpolated_60_80", self.diag_topo_interpolated_60_80),
            ]:
                val = self._parse_diag_int(v, key)
                if val is not None:
                    lst.append(val)
            # Performance (direct per-frame latencies)
            for key, lst in [
                ("t_total_ms", self.diag_t_total_ms),
                ("t_ground_ms", self.diag_t_ground_ms),
                ("t_cluster_ms", self.diag_t_cluster_ms),
                ("t_pass_ms", self.diag_t_pass_ms),
            ]:
                val = self._parse_diag_float(v, key)
                if val is not None:
                    lst.append(val)

    def on_velo(self, msg):
        self.sys_velo_count += 1
        self.velo_timestamps.append(time.time())

    def on_detection(self, msg):
        self.sys_det_count += 1
        self.perc_frame_count += 1
        self.det_timestamps.append(time.time())
        n = len(msg.points)
        self.perc_cones_per_frame.append(n)
        frame_bands = defaultdict(int)
        confs = msg.confidence if hasattr(msg, "confidence") and msg.confidence else [0.5] * n
        for point, conf in zip(msg.points, confs):
            dist = math.hypot(point.x, point.y)
            self.perc_distances.append(dist)
            self.perc_confidences.append(conf)
            band = self._dist_band(dist)
            frame_bands[band] += 1
        for band, count in frame_bands.items():
            self.perc_band_counts[band].append(count)

    def on_path(self, msg):
        self.sys_plan_count += 1
        self.plan_frame_count += 1
        path_len = len(msg.path) if hasattr(msg, "path") else 0
        if path_len > 0:
            self.plan_path_lengths.append(path_len)
        if hasattr(msg, "curvatures") and msg.curvatures:
            self.plan_curvatures.extend(msg.curvatures)
        if path_len < 13:
            self.plan_short_path_count += 1

    def on_cmd(self, msg):
        self.sys_cmd_count += 1
        self.ctrl_frame_count += 1
        delta = (msg.steering - 110) * 0.1 * math.pi / 180.0
        self.ctrl_deltas.append(delta)
        now = time.time()
        if self.last_delta is not None and self.last_delta_time is not None:
            dt = now - self.last_delta_time
            if dt > 0:
                self.ctrl_delta_rates.append(abs(delta - self.last_delta) / dt)
        self.last_delta = delta
        self.last_delta_time = now

    def on_car_state(self, msg):
        self.ctrl_speeds.append(msg.V)

    def _dist_band(self, dist):
        if dist < 10:
            return "0-10m"
        elif dist < 20:
            return "10-20m"
        elif dist < 30:
            return "20-30m"
        elif dist < 40:
            return "30-40m"
        elif dist < 50:
            return "40-50m"
        else:
            return ">50m"

    @staticmethod
    def _mean(arr):
        return sum(arr) / len(arr) if arr else 0

    @staticmethod
    def _std(arr):
        if len(arr) < 2:
            return 0
        m = TrackDriveBenchmark._mean(arr)
        return math.sqrt(sum((x - m) ** 2 for x in arr) / len(arr))

    @staticmethod
    def _percent(part, total):
        return (part / total * 100) if total > 0 else 0

    @staticmethod
    def _sum(arr):
        return sum(arr) if arr else 0

    @staticmethod
    def _percentile(arr, p):
        """Compute percentile p (0-100) for a list of floats."""
        if not arr:
            return 0.0
        s = sorted(arr)
        k = (len(s) - 1) * p / 100.0
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return s[int(k)]
        d0 = s[int(f)] * (c - k)
        d1 = s[int(c)] * (k - f)
        return d0 + d1

    def run(self):
        self.wall_start = time.time()
        print(f"[Benchmark] Starting {self.duration}s collection...")
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if time.time() - self.wall_start >= self.duration:
                break
            try:
                self.sys_cpu_samples.append(self.cpu_process.cpu_percent())
                self.sys_mem_samples_mb.append(self.cpu_process.memory_info().rss / (1024 * 1024))
            except Exception:
                pass
            rate.sleep()
        self._finalize()

    def _finalize(self):
        elapsed = max(1, time.time() - self.wall_start)
        total_cones = len(self.perc_distances)
        report = {
            "meta": {
                "duration_sec": round(elapsed, 1),
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            },
            "perception": {
                "frames": self.perc_frame_count,
                "mean_cones_per_frame": round(self._mean(self.perc_cones_per_frame), 2),
                "std_cones_per_frame": round(self._std(self.perc_cones_per_frame), 2),
                "mean_confidence": round(self._mean(self.perc_confidences), 3),
                "mean_distance_m": round(self._mean(self.perc_distances), 2),
                "bands": {},
            },
            "planning": {
                "frames": self.plan_frame_count,
                "mean_path_points": round(self._mean(self.plan_path_lengths), 2),
                "short_path_rate_pct": round(
                    self._percent(self.plan_short_path_count, self.plan_frame_count), 2
                ),
                "curvature_std": round(self._std(self.plan_curvatures), 4),
            },
            "control": {
                "frames": self.ctrl_frame_count,
                "delta_std_rad": round(self._std(self.ctrl_deltas), 4),
                "mean_delta_rate_rad_s": round(self._mean(self.ctrl_delta_rates), 3),
                "max_delta_rate_rad_s": round(
                    max(self.ctrl_delta_rates) if self.ctrl_delta_rates else 0, 3
                ),
                "mean_speed_m_s": round(self._mean(self.ctrl_speeds), 2),
            },
            "system": {
                "velo_hz": round(self.sys_velo_count / elapsed, 2),
                "det_hz": round(self.sys_det_count / elapsed, 2),
                "plan_hz": round(self.sys_plan_count / elapsed, 2),
                "cmd_hz": round(self.sys_cmd_count / elapsed, 2),
                "mean_cpu_pct": round(self._mean(self.sys_cpu_samples), 1),
                "mean_mem_mb": round(self._mean(self.sys_mem_samples_mb), 1),
            },
        }
        for band in ["0-10m", "10-20m", "20-30m", "30-40m", "40-50m", ">50m"]:
            band_list = self.perc_band_counts.get(band, [])
            total_in_band = sum(band_list)
            report["perception"]["bands"][band] = {
                "mean_per_frame": round(total_in_band / self.perc_frame_count, 2)
                if self.perc_frame_count > 0
                else 0,
                "pct_of_total": round(self._percent(total_in_band, total_cones), 2)
                if total_cones
                else 0,
            }

        # Compute actual input / detection Hz from inter-message intervals
        def _compute_hz(timestamps):
            if len(timestamps) < 2:
                return 0.0
            intervals = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
            mean_interval = sum(intervals) / len(intervals)
            return 1.0 / mean_interval if mean_interval > 0 else 0.0

        report["system"]["input_pointcloud_hz"] = round(_compute_hz(self.velo_timestamps), 2)
        report["system"]["detection_hz"] = round(_compute_hz(self.det_timestamps), 2)

        # --- Diagnostics section ---
        diag = {}
        diag["diag_frames_received"] = self.diag_frames
        if self.diag_frames > 0:
            diag["pipeline_funnel"] = {
                "input_points_mean": round(self._mean(self.diag_input_points), 1)
                if self.diag_input_points
                else 0,
                "roi_points_mean": round(self._mean(self.diag_roi_points), 1)
                if self.diag_roi_points
                else 0,
                "roi_dropped_mean": round(self._mean(self.diag_roi_dropped), 1)
                if self.diag_roi_dropped
                else 0,
                "intensity_dropped_mean": round(self._mean(self.diag_intensity_dropped), 1)
                if self.diag_intensity_dropped
                else 0,
                "ground_removed_mean": round(self._mean(self.diag_ground_removed), 1)
                if self.diag_ground_removed
                else 0,
                "obstacle_dropped_mean": round(self._mean(self.diag_obstacle_dropped), 1)
                if self.diag_obstacle_dropped
                else 0,
                "clusters_total_mean": round(self._mean(self.diag_clusters_total), 2)
                if self.diag_clusters_total
                else 0,
                "clusters_far_mean": round(self._mean(self.diag_clusters_far), 2)
                if self.diag_clusters_far
                else 0,
            }
            diag["distance_bands"] = {
                "near_mean": round(self._mean(self.diag_n_near), 2) if self.diag_n_near else 0,
                "mid_mean": round(self._mean(self.diag_n_mid), 2) if self.diag_n_mid else 0,
                "far_mean": round(self._mean(self.diag_n_far), 2) if self.diag_n_far else 0,
                "conf_near_mean": round(self._mean(self.diag_conf_near), 3)
                if self.diag_conf_near
                else 0,
                "conf_mid_mean": round(self._mean(self.diag_conf_mid), 3)
                if self.diag_conf_mid
                else 0,
                "conf_far_mean": round(self._mean(self.diag_conf_far), 3)
                if self.diag_conf_far
                else 0,
            }
            total_rejected = (
                self._sum(self.diag_rejected_by_roi)
                + self._sum(self.diag_rejected_by_confidence)
                + self._sum(self.diag_rejected_by_semantic)
                + self._sum(self.diag_rejected_by_tracker)
            )
            diag["rejection_reasons"] = {
                "total": total_rejected,
                "by_roi": self._sum(self.diag_rejected_by_roi),
                "by_confidence": self._sum(self.diag_rejected_by_confidence),
                "by_semantic": self._sum(self.diag_rejected_by_semantic),
                "by_tracker": self._sum(self.diag_rejected_by_tracker),
                "pct_roi": round(
                    self._percent(self._sum(self.diag_rejected_by_roi), total_rejected), 2
                )
                if total_rejected
                else 0,
                "pct_confidence": round(
                    self._percent(self._sum(self.diag_rejected_by_confidence), total_rejected), 2
                )
                if total_rejected
                else 0,
                "pct_semantic": round(
                    self._percent(self._sum(self.diag_rejected_by_semantic), total_rejected), 2
                )
                if total_rejected
                else 0,
                "pct_tracker": round(
                    self._percent(self._sum(self.diag_rejected_by_tracker), total_rejected), 2
                )
                if total_rejected
                else 0,
            }
            diag["confidence_components"] = {
                "size": round(self._mean(self.diag_size_scores), 3) if self.diag_size_scores else 0,
                "shape": round(self._mean(self.diag_shape_scores), 3)
                if self.diag_shape_scores
                else 0,
                "density": round(self._mean(self.diag_density_scores), 3)
                if self.diag_density_scores
                else 0,
                "intensity": round(self._mean(self.diag_intensity_scores), 3)
                if self.diag_intensity_scores
                else 0,
                "position": round(self._mean(self.diag_position_scores), 3)
                if self.diag_position_scores
                else 0,
                "semantic": round(self._mean(self.diag_semantic_scores), 3)
                if self.diag_semantic_scores
                else 0,
                "scored_count_mean": round(self._mean(self.diag_scored_count), 1)
                if self.diag_scored_count
                else 0,
            }
            # Task 23C: far-range near-threshold analysis
            total_far = self._sum(self.diag_far_candidates_total)
            total_far_rejected = self._sum(self.diag_far_rejected_by_confidence)
            diag["far_threshold_analysis"] = {
                "candidates_total": total_far,
                "accepted": self._sum(self.diag_far_accepted),
                "rejected_by_confidence": total_far_rejected,
                "avg_conf_rejected": round(self._mean(self.diag_far_avg_conf_rejected), 3)
                if self.diag_far_avg_conf_rejected
                else 0,
                "histogram": {
                    "lt_025": self._sum(self.diag_far_conf_lt_025),
                    "025_035": self._sum(self.diag_far_conf_025_035),
                    "035_040": self._sum(self.diag_far_conf_035_040),
                    "040_045": self._sum(self.diag_far_conf_040_045),
                    "045_050": self._sum(self.diag_far_conf_045_050),
                    "gt_050": self._sum(self.diag_far_conf_gt_050),
                },
                "pct_accepted": round(
                    self._percent(self._sum(self.diag_far_accepted), total_far), 2
                )
                if total_far
                else 0,
                "pct_rejected": round(self._percent(total_far_rejected, total_far), 2)
                if total_far
                else 0,
            }
            diag["model_fitting"] = {
                "calls_total": self._sum(self.diag_fitting_calls),
                "skipped": self._sum(self.diag_fitting_skipped),
                "success": self._sum(self.diag_fitting_success),
                "fail": self._sum(self.diag_fitting_fail),
                "success_rate_pct": round(
                    self._percent(
                        self._sum(self.diag_fitting_success), self._sum(self.diag_fitting_calls)
                    ),
                    2,
                )
                if self.diag_fitting_calls
                else 0,
            }
            diag["tracker"] = {
                "tentative_mean": round(self._mean(self.diag_tracker_tentative), 2)
                if self.diag_tracker_tentative
                else 0,
                "confirmed_mean": round(self._mean(self.diag_tracker_confirmed), 2)
                if self.diag_tracker_confirmed
                else 0,
                "deleted_mean": round(self._mean(self.diag_tracker_deleted), 2)
                if self.diag_tracker_deleted
                else 0,
                "semantic_kills": self._sum(self.diag_semantic_kills),
            }
            # Task 23D: post-confidence publication funnel
            def _sum_band(arr):
                return self._sum(arr) if arr else 0

            diag["post_confidence_funnel"] = {
                "postconf": {
                    "20_30": _sum_band(self.diag_postconf_20_30),
                    "30_40": _sum_band(self.diag_postconf_30_40),
                    "40_50": _sum_band(self.diag_postconf_40_50),
                    "50_60": _sum_band(self.diag_postconf_50_60),
                    "60_80": _sum_band(self.diag_postconf_60_80),
                    "total_20_80": _sum_band(self.diag_postconf_20_30)
                    + _sum_band(self.diag_postconf_30_40)
                    + _sum_band(self.diag_postconf_40_50)
                    + _sum_band(self.diag_postconf_50_60)
                    + _sum_band(self.diag_postconf_60_80),
                },
                "after_dedup": {
                    "20_30": _sum_band(self.diag_after_dedup_20_30),
                    "30_40": _sum_band(self.diag_after_dedup_30_40),
                    "40_50": _sum_band(self.diag_after_dedup_40_50),
                    "50_60": _sum_band(self.diag_after_dedup_50_60),
                    "60_80": _sum_band(self.diag_after_dedup_60_80),
                    "total_20_80": _sum_band(self.diag_after_dedup_20_30)
                    + _sum_band(self.diag_after_dedup_30_40)
                    + _sum_band(self.diag_after_dedup_40_50)
                    + _sum_band(self.diag_after_dedup_50_60)
                    + _sum_band(self.diag_after_dedup_60_80),
                },
                "after_tracker": {
                    "20_30": _sum_band(self.diag_after_tracker_20_30),
                    "30_40": _sum_band(self.diag_after_tracker_30_40),
                    "40_50": _sum_band(self.diag_after_tracker_40_50),
                    "50_60": _sum_band(self.diag_after_tracker_50_60),
                    "60_80": _sum_band(self.diag_after_tracker_60_80),
                    "total_20_80": _sum_band(self.diag_after_tracker_20_30)
                    + _sum_band(self.diag_after_tracker_30_40)
                    + _sum_band(self.diag_after_tracker_40_50)
                    + _sum_band(self.diag_after_tracker_50_60)
                    + _sum_band(self.diag_after_tracker_60_80),
                },
                "after_topology": {
                    "20_30": _sum_band(self.diag_after_topology_20_30),
                    "30_40": _sum_band(self.diag_after_topology_30_40),
                    "40_50": _sum_band(self.diag_after_topology_40_50),
                    "50_60": _sum_band(self.diag_after_topology_50_60),
                    "60_80": _sum_band(self.diag_after_topology_60_80),
                    "total_20_80": _sum_band(self.diag_after_topology_20_30)
                    + _sum_band(self.diag_after_topology_30_40)
                    + _sum_band(self.diag_after_topology_40_50)
                    + _sum_band(self.diag_after_topology_50_60)
                    + _sum_band(self.diag_after_topology_60_80),
                },
                "loss_dedup": {
                    "20_30": _sum_band(self.diag_postconf_20_30)
                    - _sum_band(self.diag_after_dedup_20_30),
                    "30_40": _sum_band(self.diag_postconf_30_40)
                    - _sum_band(self.diag_after_dedup_30_40),
                    "40_50": _sum_band(self.diag_postconf_40_50)
                    - _sum_band(self.diag_after_dedup_40_50),
                    "50_60": _sum_band(self.diag_postconf_50_60)
                    - _sum_band(self.diag_after_dedup_50_60),
                    "60_80": _sum_band(self.diag_postconf_60_80)
                    - _sum_band(self.diag_after_dedup_60_80),
                    "total_20_80": (
                        _sum_band(self.diag_postconf_20_30)
                        + _sum_band(self.diag_postconf_30_40)
                        + _sum_band(self.diag_postconf_40_50)
                        + _sum_band(self.diag_postconf_50_60)
                        + _sum_band(self.diag_postconf_60_80)
                    )
                    - (
                        _sum_band(self.diag_after_dedup_20_30)
                        + _sum_band(self.diag_after_dedup_30_40)
                        + _sum_band(self.diag_after_dedup_40_50)
                        + _sum_band(self.diag_after_dedup_50_60)
                        + _sum_band(self.diag_after_dedup_60_80)
                    ),
                },
                "loss_tracker": {
                    "20_30": _sum_band(self.diag_after_dedup_20_30)
                    - _sum_band(self.diag_after_tracker_20_30),
                    "30_40": _sum_band(self.diag_after_dedup_30_40)
                    - _sum_band(self.diag_after_tracker_30_40),
                    "40_50": _sum_band(self.diag_after_dedup_40_50)
                    - _sum_band(self.diag_after_tracker_40_50),
                    "50_60": _sum_band(self.diag_after_dedup_50_60)
                    - _sum_band(self.diag_after_tracker_50_60),
                    "60_80": _sum_band(self.diag_after_dedup_60_80)
                    - _sum_band(self.diag_after_tracker_60_80),
                    "total_20_80": (
                        _sum_band(self.diag_after_dedup_20_30)
                        + _sum_band(self.diag_after_dedup_30_40)
                        + _sum_band(self.diag_after_dedup_40_50)
                        + _sum_band(self.diag_after_dedup_50_60)
                        + _sum_band(self.diag_after_dedup_60_80)
                    )
                    - (
                        _sum_band(self.diag_after_tracker_20_30)
                        + _sum_band(self.diag_after_tracker_30_40)
                        + _sum_band(self.diag_after_tracker_40_50)
                        + _sum_band(self.diag_after_tracker_50_60)
                        + _sum_band(self.diag_after_tracker_60_80)
                    ),
                },
                "loss_topology": {
                    "20_30": _sum_band(self.diag_after_tracker_20_30)
                    - _sum_band(self.diag_after_topology_20_30),
                    "30_40": _sum_band(self.diag_after_tracker_30_40)
                    - _sum_band(self.diag_after_topology_30_40),
                    "40_50": _sum_band(self.diag_after_tracker_40_50)
                    - _sum_band(self.diag_after_topology_40_50),
                    "50_60": _sum_band(self.diag_after_tracker_50_60)
                    - _sum_band(self.diag_after_topology_50_60),
                    "60_80": _sum_band(self.diag_after_tracker_60_80)
                    - _sum_band(self.diag_after_topology_60_80),
                    "total_20_80": (
                        _sum_band(self.diag_after_tracker_20_30)
                        + _sum_band(self.diag_after_tracker_30_40)
                        + _sum_band(self.diag_after_tracker_40_50)
                        + _sum_band(self.diag_after_tracker_50_60)
                        + _sum_band(self.diag_after_tracker_60_80)
                    )
                    - (
                        _sum_band(self.diag_after_topology_20_30)
                        + _sum_band(self.diag_after_topology_30_40)
                        + _sum_band(self.diag_after_topology_40_50)
                        + _sum_band(self.diag_after_topology_50_60)
                        + _sum_band(self.diag_after_topology_60_80)
                    ),
                },
                "tracker_confirmed": {
                    "20_30": _sum_band(self.diag_tracker_confirmed_20_30),
                    "30_40": _sum_band(self.diag_tracker_confirmed_30_40),
                    "40_50": _sum_band(self.diag_tracker_confirmed_40_50),
                    "50_60": _sum_band(self.diag_tracker_confirmed_50_60),
                    "60_80": _sum_band(self.diag_tracker_confirmed_60_80),
                },
                "topology_interpolated": {
                    "20_30": _sum_band(self.diag_topo_interpolated_20_30),
                    "30_40": _sum_band(self.diag_topo_interpolated_30_40),
                    "40_50": _sum_band(self.diag_topo_interpolated_40_50),
                    "50_60": _sum_band(self.diag_topo_interpolated_50_60),
                    "60_80": _sum_band(self.diag_topo_interpolated_60_80),
                },
            }
            diag["performance"] = {
                "total_p50_ms": round(self._percentile(self.diag_t_total_ms, 50), 2)
                if self.diag_t_total_ms
                else 0,
                "total_p95_ms": round(self._percentile(self.diag_t_total_ms, 95), 2)
                if self.diag_t_total_ms
                else 0,
                "ground_p95_ms": round(self._percentile(self.diag_t_ground_ms, 95), 2)
                if self.diag_t_ground_ms
                else 0,
                "cluster_p95_ms": round(self._percentile(self.diag_t_cluster_ms, 95), 2)
                if self.diag_t_cluster_ms
                else 0,
            }
        report["diagnostics"] = diag

        # --- Markdown report generation ---
        md_lines = self._build_markdown(report)
        md_text = "\n".join(md_lines)

        print("\n" + "=" * 60)
        print("TRACKDRIVE BENCHMARK RESULTS")
        print("=" * 60)
        print(json.dumps(report, indent=2))
        print("=" * 60)
        if self.output_path:
            with open(self.output_path, "w") as f:
                json.dump(report, f, indent=2)
            print(f"\nJSON saved to: {self.output_path}")
            md_path = self.output_path.replace(".json", ".md")
            with open(md_path, "w") as f:
                f.write(md_text)
            print(f"Markdown saved to: {md_path}")
        return report

    def _build_markdown(self, report):
        p = report.get("perception", {})
        pl = report.get("planning", {})
        c = report.get("control", {})
        s = report.get("system", {})
        d = report.get("diagnostics", {})
        lines = [
            "# Perception Evaluation Report",
            f"",
            f"**Duration**: {report['meta']['duration_sec']}s  ",
            f"**Timestamp**: {report['meta']['timestamp']}  ",
            f"",
            "## Pipeline Funnel",
            "",
            "| Stage | Mean per frame |",
            "|-------|----------------|",
        ]
        funnel = d.get("pipeline_funnel", {})
        lines.append(f"| Input points | {funnel.get('input_points_mean', 0):.1f} |")
        lines.append(f"| ROI points | {funnel.get('roi_points_mean', 0):.1f} |")
        lines.append(f"| ROI dropped | {funnel.get('roi_dropped_mean', 0):.1f} |")
        lines.append(f"| Intensity dropped | {funnel.get('intensity_dropped_mean', 0):.1f} |")
        lines.append(f"| Ground removed | {funnel.get('ground_removed_mean', 0):.1f} |")
        lines.append(f"| Obstacle dropped | {funnel.get('obstacle_dropped_mean', 0):.1f} |")
        lines.append(f"| Clusters total | {funnel.get('clusters_total_mean', 0):.2f} |")
        lines.append(f"| Clusters far (>30m) | {funnel.get('clusters_far_mean', 0):.2f} |")
        lines.append(f"| **Detections published** | **{p.get('mean_cones_per_frame', 0):.2f}** |")
        lines.append("")
        lines.append("## Per-Distance-Band Detection Counts")
        lines.append("")
        lines.append("| Band | Mean/frame | % of total |")
        lines.append("|------|------------|------------|")
        for band in ["0-10m", "10-20m", "20-30m", "30-40m", "40-50m", ">50m"]:
            b = p.get("bands", {}).get(band, {})
            lines.append(
                f"| {band} | {b.get('mean_per_frame', 0):.2f} | {b.get('pct_of_total', 0):.2f}% |"
            )
        lines.append("")
        lines.append("## Confidence Component Averages")
        lines.append("")
        comp = d.get("confidence_components", {})
        lines.append(f"- Size: {comp.get('size', 0):.3f}")
        lines.append(f"- Shape: {comp.get('shape', 0):.3f}")
        lines.append(f"- Density: {comp.get('density', 0):.3f}")
        lines.append(f"- Intensity: {comp.get('intensity', 0):.3f}")
        lines.append(f"- Position: {comp.get('position', 0):.3f}")
        lines.append(f"- Semantic: {comp.get('semantic', 0):.3f}")
        lines.append(f"- Scored count (mean): {comp.get('scored_count_mean', 0):.1f}")
        lines.append("")
        lines.append("## Rejection Reason Breakdown")
        lines.append("")
        rej = d.get("rejection_reasons", {})
        lines.append(f"- Total rejected: {rej.get('total', 0)}")
        lines.append(f"- By ROI: {rej.get('by_roi', 0)} ({rej.get('pct_roi', 0):.1f}%)")
        lines.append(
            f"- By confidence: {rej.get('by_confidence', 0)} ({rej.get('pct_confidence', 0):.1f}%)"
        )
        lines.append(
            f"- By semantic: {rej.get('by_semantic', 0)} ({rej.get('pct_semantic', 0):.1f}%)"
        )
        lines.append(f"- By tracker: {rej.get('by_tracker', 0)} ({rej.get('pct_tracker', 0):.1f}%)")
        lines.append("")
        # Task 23C: far-range near-threshold analysis markdown
        fta = d.get("far_threshold_analysis", {})
        if fta.get("candidates_total", 0) > 0:
            lines.append("## Far-Range (20-50m) Near-Threshold Analysis")
            lines.append("")
            hist = fta.get("histogram", {})
            lines.append(f"- Total candidates: {fta.get('candidates_total', 0)}")
            lines.append(
                f"- Accepted: {fta.get('accepted', 0)} ({fta.get('pct_accepted', 0):.1f}%)"
            )
            lines.append(
                f"- Rejected by confidence: {fta.get('rejected_by_confidence', 0)} ({fta.get('pct_rejected', 0):.1f}%)"
            )
            lines.append(f"- Avg confidence of rejected: {fta.get('avg_conf_rejected', 0):.3f}")
            lines.append("")
            lines.append("### Confidence Distribution (all far-range candidates)")
            lines.append("")
            lines.append("| Bucket | Count | % of total |")
            lines.append("|--------|-------|------------|")
            total = fta.get("candidates_total", 1)
            lines.append(
                f"| <0.25 | {hist.get('lt_025', 0)} | {hist.get('lt_025', 0)/total*100:.1f}% |"
            )
            lines.append(
                f"| 0.25-0.35 | {hist.get('025_035', 0)} | {hist.get('025_035', 0)/total*100:.1f}% |"
            )
            lines.append(
                f"| 0.35-0.40 | {hist.get('035_040', 0)} | {hist.get('035_040', 0)/total*100:.1f}% |"
            )
            lines.append(
                f"| 0.40-0.45 | {hist.get('040_045', 0)} | {hist.get('040_045', 0)/total*100:.1f}% |"
            )
            lines.append(
                f"| 0.45-0.50 | {hist.get('045_050', 0)} | {hist.get('045_050', 0)/total*100:.1f}% |"
            )
            lines.append(
                f"| >0.50 | {hist.get('gt_050', 0)} | {hist.get('gt_050', 0)/total*100:.1f}% |"
            )
            lines.append("")
        lines.append("## Model Fitting Stats")
        lines.append("")
        mf = d.get("model_fitting", {})
        lines.append(f"- Calls total: {mf.get('calls_total', 0)}")
        lines.append(f"- Skipped: {mf.get('skipped', 0)}")
        lines.append(f"- Success: {mf.get('success', 0)}")
        lines.append(f"- Fail: {mf.get('fail', 0)}")
        lines.append(f"- Success rate: {mf.get('success_rate_pct', 0):.1f}%")
        lines.append("")
        lines.append("## Tracker Stats")
        lines.append("")
        tr = d.get("tracker", {})
        lines.append(f"- Tentative (mean): {tr.get('tentative_mean', 0):.2f}")
        lines.append(f"- Confirmed (mean): {tr.get('confirmed_mean', 0):.2f}")
        lines.append(f"- Deleted (mean): {tr.get('deleted_mean', 0):.2f}")
        lines.append(f"- Semantic kills: {tr.get('semantic_kills', 0)}")
        lines.append("")
        # Task 23D: post-confidence publication funnel markdown
        pcf = d.get("post_confidence_funnel", {})
        if pcf:
            lines.append("## Post-Confidence Publication Funnel (20-80m)")
            lines.append("")
            lines.append("| Stage | 20-30m | 30-40m | 40-50m | 50-60m | 60-80m | Total |")
            lines.append("|-------|--------|--------|--------|--------|--------|-------|")
            postconf = pcf.get("postconf", {})
            after_dedup = pcf.get("after_dedup", {})
            after_tracker = pcf.get("after_tracker", {})
            after_topology = pcf.get("after_topology", {})
            loss_dedup = pcf.get("loss_dedup", {})
            loss_tracker = pcf.get("loss_tracker", {})
            loss_topology = pcf.get("loss_topology", {})
            topo_interp = pcf.get("topology_interpolated", {})
            lines.append(
                f"| Post-confidence | {postconf.get('20_30', 0)} | {postconf.get('30_40', 0)} | {postconf.get('40_50', 0)} | {postconf.get('50_60', 0)} | {postconf.get('60_80', 0)} | {postconf.get('total_20_80', 0)} |"
            )
            lines.append(
                f"| After dedup | {after_dedup.get('20_30', 0)} | {after_dedup.get('30_40', 0)} | {after_dedup.get('40_50', 0)} | {after_dedup.get('50_60', 0)} | {after_dedup.get('60_80', 0)} | {after_dedup.get('total_20_80', 0)} |"
            )
            lines.append(
                f"| Loss (dedup) | {loss_dedup.get('20_30', 0)} | {loss_dedup.get('30_40', 0)} | {loss_dedup.get('40_50', 0)} | {loss_dedup.get('50_60', 0)} | {loss_dedup.get('60_80', 0)} | {loss_dedup.get('total_20_80', 0)} |"
            )
            lines.append(
                f"| After tracker | {after_tracker.get('20_30', 0)} | {after_tracker.get('30_40', 0)} | {after_tracker.get('40_50', 0)} | {after_tracker.get('50_60', 0)} | {after_tracker.get('60_80', 0)} | {after_tracker.get('total_20_80', 0)} |"
            )
            lines.append(
                f"| Loss (tracker) | {loss_tracker.get('20_30', 0)} | {loss_tracker.get('30_40', 0)} | {loss_tracker.get('40_50', 0)} | {loss_tracker.get('50_60', 0)} | {loss_tracker.get('60_80', 0)} | {loss_tracker.get('total_20_80', 0)} |"
            )
            lines.append(
                f"| After topology | {after_topology.get('20_30', 0)} | {after_topology.get('30_40', 0)} | {after_topology.get('40_50', 0)} | {after_topology.get('50_60', 0)} | {after_topology.get('60_80', 0)} | {after_topology.get('total_20_80', 0)} |"
            )
            lines.append(
                f"| Loss (topology) | {loss_topology.get('20_30', 0)} | {loss_topology.get('30_40', 0)} | {loss_topology.get('40_50', 0)} | {loss_topology.get('50_60', 0)} | {loss_topology.get('60_80', 0)} | {loss_topology.get('total_20_80', 0)} |"
            )
            lines.append(
                f"| Topo interpolated | {topo_interp.get('20_30', 0)} | {topo_interp.get('30_40', 0)} | {topo_interp.get('40_50', 0)} | {topo_interp.get('50_60', 0)} | {topo_interp.get('60_80', 0)} | {topo_interp.get('20_30', 0)+topo_interp.get('30_40', 0)+topo_interp.get('40_50', 0)+topo_interp.get('50_60', 0)+topo_interp.get('60_80', 0)} |"
            )
            lines.append("")
        lines.append("## Performance")
        lines.append("")
        perf = d.get("performance", {})
        lines.append(f"- Total latency p50: {perf.get('total_p50_ms', 0):.2f} ms")
        lines.append(f"- Total latency p95: {perf.get('total_p95_ms', 0):.2f} ms")
        lines.append(f"- Ground seg p95: {perf.get('ground_p95_ms', 0):.2f} ms")
        lines.append(f"- Clustering p95: {perf.get('cluster_p95_ms', 0):.2f} ms")
        lines.append("")
        lines.append("## Planning & Control Summary")
        lines.append("")
        lines.append(f"- Planning frames: {pl.get('frames', 0)}")
        lines.append(f"- Mean path points: {pl.get('mean_path_points', 0):.2f}")
        lines.append(f"- Short path rate: {pl.get('short_path_rate_pct', 0):.2f}%")
        lines.append(f"- Control frames: {c.get('frames', 0)}")
        lines.append(f"- Delta std: {c.get('delta_std_rad', 0):.4f} rad")
        lines.append(f"- Mean speed: {c.get('mean_speed_m_s', 0):.2f} m/s")
        lines.append("")
        lines.append("## System")
        lines.append("")
        lines.append(f"- Velo Hz: {s.get('velo_hz', 0):.2f}")
        lines.append(f"- Det Hz: {s.get('det_hz', 0):.2f}")
        lines.append(f"- Input pointcloud Hz: {s.get('input_pointcloud_hz', 0):.2f}")
        lines.append(f"- Detection Hz: {s.get('detection_hz', 0):.2f}")
        lines.append(f"- Plan Hz: {s.get('plan_hz', 0):.2f}")
        lines.append(f"- Cmd Hz: {s.get('cmd_hz', 0):.2f}")
        lines.append(f"- Mean CPU: {s.get('mean_cpu_pct', 0):.1f}%")
        lines.append(f"- Mean Mem: {s.get('mean_mem_mb', 0):.1f} MB")
        lines.append("")
        return lines


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=60)
    parser.add_argument("--output", type=str, default=None)
    args = parser.parse_args()
    output_path = args.output
    if output_path is None:
        ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
        output_path = f"/tmp/benchmark_result_{ts}.json"  # nosec B108
    TrackDriveBenchmark(duration_sec=args.duration, output_path=output_path).run()
