#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPT_DIR)

from perf_stats import PerfStats, estimate_message_bytes


class DummyDataMsg:
    def __init__(self, data):
        self.data = data


class DummySerializeMsg:
    def __init__(self, payload):
        self.payload = payload

    def serialize(self, buff):
        buff.write(self.payload)


class PerfStatsTests(unittest.TestCase):
    def test_add_disabled(self):
        stats = PerfStats("test", window_size=3, log_every=1000, enabled=False)
        stats.add({"t_total_ms": 1.0})
        self.assertEqual(len(stats.samples), 0)

    def test_window_rolls(self):
        stats = PerfStats("test", window_size=3, log_every=1000, enabled=True)
        for i in range(4):
            stats.add({"t_total_ms": float(i + 1), "N": float((i + 1) * 10)})
        self.assertEqual(len(stats.samples), 3)
        values = [s["t_total_ms"] for s in stats.samples]
        self.assertEqual(sorted(values), [2.0, 3.0, 4.0])

    def test_normalizes_missing_keys(self):
        stats = PerfStats("test", window_size=3, log_every=1000, enabled=True)
        stats.add({"t_total_ms": 5.0})
        self.assertEqual(len(stats.samples), 1)
        sample = stats.samples[0]
        self.assertEqual(sample["t_total_ms"], 5.0)
        self.assertEqual(sample["N"], 0.0)
        self.assertEqual(sample["bytes"], 0.0)


class EstimateMessageBytesTests(unittest.TestCase):
    def test_estimate_bytes_data(self):
        msg = DummyDataMsg(b"abcd")
        self.assertEqual(estimate_message_bytes(msg), 4)

    def test_estimate_bytes_serialize(self):
        msg = DummySerializeMsg(b"xyz")
        self.assertEqual(estimate_message_bytes(msg), 3)


if __name__ == "__main__":
    unittest.main()
