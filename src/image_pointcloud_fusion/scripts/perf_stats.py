#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import deque
import io
import numpy as np
import rospy

METRIC_KEYS = [
    "t_pass_ms",
    "t_ground_ms",
    "t_cluster_ms",
    "t_delaunay_ms",
    "t_way_ms",
    "t_total_ms",
    "N",
    "K",
    "T",
    "E",
    "D",
    "bytes",
]


class PerfStats:
    def __init__(self, name, window_size=300, log_every=30, enabled=True):
        self.name = name
        self.window_size = max(1, int(window_size))
        self.log_every = max(1, int(log_every))
        self.enabled = bool(enabled)
        self.samples = deque(maxlen=self.window_size)
        self.since_log = 0

    def add(self, sample):
        if not self.enabled:
            return
        normalized = {k: float(sample.get(k, 0.0)) for k in METRIC_KEYS}
        self.samples.append(normalized)
        self.since_log += 1
        if len(self.samples) >= self.window_size and self.since_log >= self.log_every:
            self._log_stats()
            self.since_log = 0

    def _compute_stats(self, values):
        if not values:
            return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}
        arr = np.asarray(values, dtype=np.float64)
        return {
            "mean": float(np.mean(arr)),
            "p50": float(np.percentile(arr, 50)),
            "p95": float(np.percentile(arr, 95)),
            "p99": float(np.percentile(arr, 99)),
            "max": float(np.max(arr)),
        }

    def _log_stats(self):
        stats_by_key = {}
        for key in METRIC_KEYS:
            values = [s[key] for s in self.samples]
            stats_by_key[key] = self._compute_stats(values)

        parts = [f"[perf] node={self.name} window={len(self.samples)}"]
        for key in METRIC_KEYS:
            s = stats_by_key[key]
            parts.append(
                f"{key}{{mean={s['mean']:.3f},p50={s['p50']:.3f},p95={s['p95']:.3f},p99={s['p99']:.3f},max={s['max']:.3f}}}"
            )
        rospy.loginfo(" ".join(parts))


def estimate_message_bytes(msg):
    if hasattr(msg, "data"):
        return len(msg.data)
    buff = io.BytesIO()
    msg.serialize(buff)
    return buff.tell()
