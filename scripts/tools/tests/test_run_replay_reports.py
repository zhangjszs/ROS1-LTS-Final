#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest import mock

SCRIPT_PATH = Path(__file__).resolve().parent.parent / "run_replay_reports.py"
SPEC = importlib.util.spec_from_file_location("run_replay_reports", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class _DummyProc:
    def wait(self, timeout=None):
        return 0

    def kill(self):
        return None

    def send_signal(self, sig):
        return None


class RunReplayReportsTest(unittest.TestCase):
    def test_run_mission_replay_kills_ros_procs_before_launch(self):
        mission = {
            "name": "trackdrive",
            "launch": "trackdrive.launch",
            "bag": "/tmp/fake.bag",  # nosec B108 - test fixture only
            "image_topic": "/camera/image_raw",
        }
        events = []

        def fake_kill():
            events.append("kill")

        def fake_popen(*args, **kwargs):
            events.append("popen")
            return _DummyProc()

        with tempfile.TemporaryDirectory() as outdir, mock.patch.object(
            MODULE, "_kill_ros_procs", side_effect=fake_kill
        ), mock.patch.object(MODULE.subprocess, "Popen", side_effect=fake_popen), mock.patch.object(
            MODULE.time, "sleep", return_value=None
        ), mock.patch.object(
            MODULE.os.path, "exists", return_value=True
        ), mock.patch.object(
            MODULE.os.path, "getsize", return_value=1024
        ):
            bag_path = MODULE.run_mission_replay(mission, outdir)

        self.assertEqual("kill", events[0], events)
        self.assertEqual(2, events.count("popen"), events)
        self.assertTrue(bag_path.endswith("trackdrive_combined.bag"))


if __name__ == "__main__":
    unittest.main()
