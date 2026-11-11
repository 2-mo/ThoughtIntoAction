"""Device setup: validation rules, kernel profile compilation and the API contract."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from fastapi import HTTPException

from embodied.applications.console.core import devices
from embodied.applications.console.core.devices import SetupError, validate_setup


def dual_document(**overrides):
    document = {
        "version": 1,
        "robots": [
            {"id": "arm1", "vendor": "arx", "model": "x5", "side": "left", "can_interface": "can1"},
            {"id": "arm2", "vendor": "arx", "model": "x5", "side": "right", "can_interface": "can0"},
        ],
        "cameras": [
            {"id": "wrist_rgb", "serial": "111", "model": "D435i", "mount": "wrist", "arm": "arm1"},
            {"id": "wrist_second_rgb", "serial": "222", "model": "D405", "mount": "wrist", "arm": "arm2"},
            {"id": "exterior_rgb", "serial": "333", "model": "D405", "mount": "fixed"},
        ],
    }
    document.update(overrides)
    return document


class SetupValidationTests(unittest.TestCase):
    def test_dual_document_is_normalized(self):
        document = validate_setup(dual_document())
        self.assertEqual([robot["side"] for robot in document["robots"]], ["left", "right"])
        self.assertEqual([camera["arm"] for camera in document["cameras"]], ["arm1", "arm2", None])
        self.assertEqual(document["cameras"][2]["mount"], "fixed")

    def test_single_arm_document_binds_the_wrist_automatically(self):
        document = validate_setup({
            "version": 1,
            "robots": [{"id": "arm1", "vendor": "arx", "model": "x5", "side": "single", "can_interface": "can0"}],
            "cameras": [{"id": "wrist_rgb", "serial": "111", "mount": "wrist"}],
        })
        self.assertEqual(document["robots"][0]["side"], "single")
        self.assertEqual(document["cameras"][0]["arm"], "arm1")

    def test_unassigned_serials_are_allowed_but_duplicates_are_not(self):
        validate_setup(dual_document(cameras=[
            {"id": "wrist_rgb", "serial": "", "mount": "fixed"},
            {"id": "exterior_rgb", "serial": "", "mount": "fixed"},
        ]))
        with self.assertRaisesRegex(SetupError, "序列号重复"):
            validate_setup(dual_document(cameras=[
                {"id": "wrist_rgb", "serial": "9", "mount": "fixed"},
                {"id": "exterior_rgb", "serial": "9", "mount": "fixed"},
            ]))

    def test_camera_ids_must_end_with_rgb_and_be_unique(self):
        with self.assertRaisesRegex(SetupError, "形如"):
            validate_setup(dual_document(cameras=[{"id": "wrist", "serial": "1", "mount": "fixed"}]))
        with self.assertRaisesRegex(SetupError, "相机 ID 重复"):
            validate_setup(dual_document(cameras=[
                {"id": "wrist_rgb", "serial": "1", "mount": "fixed"},
                {"id": "wrist_rgb", "serial": "2", "mount": "fixed"},
            ]))

    def test_wrist_camera_needs_a_configured_arm_and_one_per_arm(self):
        with self.assertRaisesRegex(SetupError, "绑定"):
            validate_setup(dual_document(cameras=[
                {"id": "wrist_rgb", "serial": "1", "mount": "wrist", "arm": "arm9"},
            ]))
        with self.assertRaisesRegex(SetupError, "只能有一台腕部相机"):
            validate_setup(dual_document(cameras=[
                {"id": "wrist_rgb", "serial": "1", "mount": "wrist", "arm": "arm1"},
                {"id": "wrist_second_rgb", "serial": "2", "mount": "wrist", "arm": "arm1"},
            ]))

    def test_can_interfaces_and_sides_must_be_distinct(self):
        with self.assertRaisesRegex(SetupError, "CAN 接口重复"):
            validate_setup(dual_document(robots=[
                {"id": "arm1", "vendor": "arx", "model": "x5", "side": "left", "can_interface": "can0"},
                {"id": "arm2", "vendor": "arx", "model": "x5", "side": "right", "can_interface": "can0"},
            ]))
        with self.assertRaisesRegex(SetupError, "重复的臂侧"):
            validate_setup(dual_document(robots=[
                {"id": "arm1", "vendor": "arx", "model": "x5", "side": "left", "can_interface": "can0"},
                {"id": "arm2", "vendor": "arx", "model": "x5", "side": "left", "can_interface": "can1"},
            ]))

    def test_unimplemented_vendor_is_refused_with_a_conflict(self):
        with self.assertRaises(SetupError) as raised:
            validate_setup(dual_document(robots=[
                {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "left", "can_interface": "can0"},
            ]))
        self.assertEqual(raised.exception.status, 409)


class SetupCompileTests(unittest.TestCase):
    def test_compile_writes_robot_and_workbench_profiles(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)
            with patch.object(devices, "GENERATED_DIR", target):
                compiled = devices.compile_setup(validate_setup(dual_document()))
                robot = json.loads((target / "arx5-dual.json").read_text())
                workbench = json.loads((target / "real-dual-quest.json").read_text())
            self.assertEqual(robot["profile"], "arx5-dual")
            self.assertEqual([arm["id"] for arm in robot["arms"]], ["left", "right"])
            self.assertEqual([arm["can_interface"] for arm in robot["arms"]], ["can1", "can0"])
            self.assertEqual(workbench["config"], str(target / "arx5-dual.json"))
            self.assertEqual(devices._portable(devices.PROJECT_ROOT / "outputs/runtime/setup/arx5-dual.json"),
                             "outputs/runtime/setup/arx5-dual.json")
            self.assertTrue(compiled["workbench_name"].endswith("real-dual-quest.json"))

    def test_single_arm_compiles_the_single_profile(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)
            with patch.object(devices, "GENERATED_DIR", target):
                devices.compile_setup(validate_setup({
                    "version": 1,
                    "robots": [{"id": "arm1", "vendor": "arx", "model": "x5", "side": "single",
                                "can_interface": "can0"}],
                    "cameras": [{"id": "wrist_rgb", "serial": "1", "mount": "wrist"}],
                }))
                robot = json.loads((target / "arx5.json").read_text())
            self.assertEqual(robot, {"profile": "arx5", "can_interface": "can0"})


class SetupStorageTests(unittest.TestCase):
    def test_save_then_load_round_trips(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "devices.json"
            with patch("embodied.core.device_setup.DEVICES_PATH", path):
                saved = devices.save_setup(dual_document())
                self.assertEqual(devices.load_setup(), saved)

    def test_camera_entries_match_the_legacy_config_shape(self):
        entries = devices.camera_entries(validate_setup(dual_document()))
        self.assertEqual(entries["wrist_rgb"]["mount"], "wrist")
        self.assertEqual(entries["wrist_rgb"]["arm"], "arm1")
        self.assertEqual(entries["exterior_rgb"]["driver"], "realsense")
        self.assertEqual(entries["exterior_rgb"]["width"], 640)


if __name__ == "__main__":
    unittest.main()
