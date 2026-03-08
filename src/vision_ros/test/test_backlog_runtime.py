#!/usr/bin/env python3
import importlib.util
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "vision_node_py.py"
SPEC = importlib.util.spec_from_file_location("vision_node_py", SCRIPT_PATH)
VISION_NODE_PY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VISION_NODE_PY)


class TestBacklogRuntime(unittest.TestCase):
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


if __name__ == "__main__":
    import rostest

    rostest.rosrun("vision_ros", "test_backlog_runtime", TestBacklogRuntime)
