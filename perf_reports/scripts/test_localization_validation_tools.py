#!/usr/bin/env python3
import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parent
COMPARE_BACKENDS_PATH = SCRIPT_DIR / "compare_backends.py"
CONE_GT_ANNOTATOR_PATH = SCRIPT_DIR / "cone_gt_annotator.py"
LOCALIZATION_CI_PATH = (
    SCRIPT_DIR.parent.parent / ".github" / "workflows" / "localization_regression_ci.yaml"
)
FG_MAINLINE_REPLAY_SCRIPT = (
    SCRIPT_DIR.parent.parent / "scripts" / "validation" / "validate_localization_fg_mainline_replay.sh"
)


def _load_module_with_stubbed_ros(path: Path, module_name: str):
    fake_rospy = types.ModuleType("rospy")
    fake_rospy.Subscriber = lambda *args, **kwargs: None
    fake_rospy.init_node = lambda *args, **kwargs: None
    fake_rospy.spin = lambda: None
    fake_rospy.sleep = lambda *_args, **_kwargs: None
    fake_rospy.is_shutdown = lambda: False

    fake_time = types.SimpleNamespace(to_sec=lambda: 0.0)
    fake_rospy.Time = types.SimpleNamespace(now=lambda: fake_time)

    fake_autodrive_msgs = types.ModuleType("autodrive_msgs")
    fake_autodrive_msgs_msg = types.ModuleType("autodrive_msgs.msg")
    fake_autodrive_msgs_msg.HUAT_CarState = object
    fake_autodrive_msgs.msg = fake_autodrive_msgs_msg

    fake_matplotlib = types.ModuleType("matplotlib")
    fake_pyplot = types.ModuleType("matplotlib.pyplot")
    fake_matplotlib.pyplot = fake_pyplot

    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(
        sys.modules,
        {
            "rospy": fake_rospy,
            "autodrive_msgs": fake_autodrive_msgs,
            "autodrive_msgs.msg": fake_autodrive_msgs_msg,
            "matplotlib": fake_matplotlib,
            "matplotlib.pyplot": fake_pyplot,
        },
        clear=False,
    ):
        spec.loader.exec_module(module)
    return module


class LocalizationValidationToolsTest(unittest.TestCase):
    _compare_module = None

    @classmethod
    def _load_compare_module(cls):
        if cls._compare_module is None:
            cls._compare_module = _load_module_with_stubbed_ros(
                COMPARE_BACKENDS_PATH, "compare_backends"
            )
        return cls._compare_module

    def test_compare_backends_cli_accepts_duration_and_output(self):
        module = self._load_compare_module()
        args = module.parse_args(
            [
                "--duration",
                "60",
                "--output",
                "fg_validation_results.csv",
                "--mapper-topic",
                "/localization/mapper/car_state",
                "--fg-topic",
                "/localization/fg/car_state",
                "--max-sync-diff",
                "0.2",
            ]
        )
        self.assertEqual(60.0, args.duration)
        self.assertEqual("fg_validation_results.csv", args.output)
        self.assertEqual("/localization/mapper/car_state", args.mapper_topic)
        self.assertEqual("/localization/fg/car_state", args.fg_topic)
        self.assertAlmostEqual(0.2, args.max_sync_diff)

    def test_compare_backends_csv_header_is_unit_safe(self):
        module = self._load_compare_module()
        self.assertEqual(
            "timestamp,pos_error_m,heading_error_deg,velocity_error_mps,time_diff_s",
            module.build_csv_header(),
        )

    def test_cone_gt_annotator_script_exists_and_parses_args(self):
        self.assertTrue(CONE_GT_ANNOTATOR_PATH.exists())
        module = _load_module_with_stubbed_ros(CONE_GT_ANNOTATOR_PATH, "cone_gt_annotator")
        bag_path = str(Path(tempfile.gettempdir()) / "track.bag")
        output_path = str(Path(tempfile.gettempdir()) / "track_gt.csv")
        args = module.parse_args(
            [
                "--bag",
                bag_path,
                "--output",
                output_path,
            ]
        )
        self.assertEqual(bag_path, args.bag)
        self.assertEqual(output_path, args.output)

    def test_localization_ci_workflow_exists_and_references_tooling(self):
        self.assertTrue(LOCALIZATION_CI_PATH.exists())
        content = LOCALIZATION_CI_PATH.read_text(encoding="utf-8")
        self.assertIn("compare_backends.py", content)
        self.assertIn("inject_scenario.py", content)
        self.assertIn("localization_health_check.sh", content)
        self.assertIn("localization_log_check.sh", content)

    def test_fg_mainline_replay_script_exists_and_has_core_checks(self):
        self.assertTrue(FG_MAINLINE_REPLAY_SCRIPT.exists())
        content = FG_MAINLINE_REPLAY_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("backend:=factor_graph", content)
        self.assertIn("publish_dual_backends:=true", content)
        self.assertIn("fg_shadow_mode:=false", content)
        self.assertIn("/localization/fg/car_state", content)
        self.assertIn("/localization/mapper/car_state", content)
        self.assertIn("active_backend_source", content)


if __name__ == "__main__":
    unittest.main()
