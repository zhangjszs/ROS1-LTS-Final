#!/usr/bin/env python3
import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "vision_node_py.py"
SPEC = importlib.util.spec_from_file_location("vision_node_py", SCRIPT_PATH)
VISION_NODE_PY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VISION_NODE_PY)


class TestBacklogRuntime(unittest.TestCase):
    @staticmethod
    def _header(stamp_sec):
        return SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: float(stamp_sec)))

    def _make_stub_node(self, newer_pending, fallback_enabled=True):
        node = VISION_NODE_PY.VisionNodePy.__new__(VISION_NODE_PY.VisionNodePy)
        node.frame_count = 0
        node.state = VISION_NODE_PY.STATE_NORMAL
        node.backend_ready = True
        node.backend_name = "onnx"
        node.fallback_enabled = fallback_enabled
        node.tracker_enabled = False
        node.publish_debug_image = False
        node.skip_heavy_if_newer_pending = True
        node.skipped_postprocess_newer_pending = 0
        node.skipped_inference_newer_pending = 0
        node.last_fallback_ms = 0.0
        node.last_tracker_ms = 0.0
        node.last_stage_timing = None
        node.max_detections = 20
        node.require_model = True
        node.model_path = "/tmp/model.onnx"
        node.stale_frame_age_sec = 0.5
        node.latest_frame_buffer = SimpleNamespace(
            has_pending=lambda: newer_pending,
            replaced_total=0,
            stale_total=0,
            pending_depth=lambda: int(newer_pending),
            last_stale_age_ms=0.0,
        )
        node._assess_quality = lambda _bgr: {
            "overall": VISION_NODE_PY.QUALITY_GOOD,
            "blur_score": 1000.0,
            "brightness": 128.0,
        }
        node._update_state = lambda _quality: None
        node._enhance_image = lambda bgr, _quality: bgr
        node._detect_fallback_hsv = lambda _bgr: ["fallback"]
        node._fuse_detections = lambda model, fallback, _quality, _state: fallback if fallback else model
        node._filter_topk = lambda detections, _max_det: detections
        node._publish_detections = lambda *_args, **_kwargs: None
        node._publish_diagnostics = lambda *_args, **_kwargs: None
        node._current_ros_time_sec = lambda: 10.5
        return node

    def test_latest_frame_buffer_overwrites_older_pending_frame(self):
        buffer = VISION_NODE_PY.LatestFrameBuffer(stale_after_sec=0.5)

        older = VISION_NODE_PY.PendingImageFrame(msg="older", receive_mono=1.0)
        newer = VISION_NODE_PY.PendingImageFrame(msg="newer", receive_mono=1.1)

        buffer.store(older)
        buffer.store(newer)

        picked = buffer.take_latest(now_mono=1.15)

        self.assertEqual("newer", picked.msg)
        self.assertEqual(1, buffer.replaced_total)
        self.assertEqual(0, buffer.stale_total)

    def test_latest_frame_buffer_drops_stale_frame(self):
        buffer = VISION_NODE_PY.LatestFrameBuffer(stale_after_sec=0.5)
        stale = VISION_NODE_PY.PendingImageFrame(msg="stale", receive_mono=1.0)

        buffer.store(stale)

        picked = buffer.take_latest(now_mono=1.7)

        self.assertIsNone(picked)
        self.assertEqual(1, buffer.stale_total)

    def test_stage_timing_snapshot_reports_expected_durations(self):
        timing = VISION_NODE_PY.FrameStageTiming(header_stamp_sec=10.0, receive_mono=100.0)

        timing.mark_picked(100.05)
        timing.mark_preprocess_done(100.09)
        timing.mark_inference_done(100.21)
        timing.mark_postprocess_done(100.24)
        timing.mark_publish_done(100.27, publish_ros_sec=10.31)

        snapshot = timing.snapshot()

        self.assertAlmostEqual(50.0, snapshot["receive_to_pick_ms"], places=3)
        self.assertAlmostEqual(40.0, snapshot["preprocess_ms"], places=3)
        self.assertAlmostEqual(120.0, snapshot["inference_stage_ms"], places=3)
        self.assertAlmostEqual(30.0, snapshot["postprocess_ms"], places=3)
        self.assertAlmostEqual(30.0, snapshot["publish_ms"], places=3)
        self.assertAlmostEqual(220.0, snapshot["total_processing_ms"], places=3)
        self.assertAlmostEqual(310.0, snapshot["end_to_end_publish_lag_ms"], places=3)

    def test_process_frame_skips_inference_when_newer_frame_pending(self):
        node = self._make_stub_node(newer_pending=True)
        bgr = np.zeros((32, 32, 3), dtype=np.uint8)

        def fail_detect_onnx(*_args, **_kwargs):
            raise AssertionError("_detect_onnx should not be called when a newer frame is pending")

        node._detect_onnx = fail_detect_onnx

        node._process_frame(bgr, self._header(10.0), receive_mono=100.0, pick_mono=100.01)

        self.assertEqual(1, node.skipped_inference_newer_pending)

    def test_process_frame_runs_inference_when_no_newer_frame_pending(self):
        node = self._make_stub_node(newer_pending=False)
        bgr = np.zeros((32, 32, 3), dtype=np.uint8)
        calls = {"detect_onnx": 0}

        def fake_detect_onnx(*_args, **_kwargs):
            calls["detect_onnx"] += 1
            return ["model"]

        node._detect_onnx = fake_detect_onnx

        node._process_frame(bgr, self._header(10.0), receive_mono=100.0, pick_mono=100.01)

        self.assertEqual(1, calls["detect_onnx"])
        self.assertEqual(0, node.skipped_inference_newer_pending)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("vision_ros", "test_backlog_runtime", TestBacklogRuntime)
