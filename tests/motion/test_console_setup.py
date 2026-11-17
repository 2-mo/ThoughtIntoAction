"""Device setup: validation rules, kernel profile compilation and the API contract."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from fastapi import HTTPException

from embodied.applications.console.core import devices, setup_library
from embodied.applications.console.core.devices import SetupError, validate_setup


def realman_document(**overrides):
    document = {
        "version": 1,
        "robots": [{"id": "arm1", "vendor": "realman", "model": "rm65", "side": "single",
                    "host": "192.168.1.19"}],
        "cameras": [{"id": "wrist_rgb", "serial": "1", "mount": "wrist", "arm": "arm1"}],
    }
    document.update(overrides)
    return document


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

    def test_realman_is_addressed_by_host_not_can(self):
        document = validate_setup(realman_document())
        self.assertEqual(document["robots"][0]["host"], "192.168.1.19")
        self.assertIsNone(document["robots"][0]["can_interface"])

    def test_realman_rejects_a_bad_host(self):
        with self.assertRaisesRegex(SetupError, "IP 不合法"):
            validate_setup(realman_document(robots=[
                {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "single", "host": "not-an-ip"},
            ]))

    def test_realman_dual_needs_two_distinct_hosts(self):
        document = validate_setup(realman_document(robots=[
            {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "left", "host": "192.168.1.19"},
            {"id": "arm2", "vendor": "realman", "model": "rm65", "side": "right", "host": "192.168.1.18"},
        ]))
        self.assertEqual([robot["host"] for robot in document["robots"]],
                         ["192.168.1.19", "192.168.1.18"])
        with self.assertRaisesRegex(SetupError, "IP 重复"):
            validate_setup(realman_document(robots=[
                {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "left", "host": "192.168.1.18"},
                {"id": "arm2", "vendor": "realman", "model": "rm65", "side": "right", "host": "192.168.1.18"},
            ]))


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
            policy = json.loads((target / "real-dual-manual.json").read_text())
            self.assertEqual(policy["source"], "cli")
            self.assertEqual(policy["config"], str(target / "arx5-dual.json"))
            self.assertEqual(compiled["workbench"]["policy"], str(target / "real-dual-manual.json"))

    def test_realman_compiles_a_connection_and_both_workbenches(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)
            with patch.object(devices, "GENERATED_DIR", target):
                compiled = devices.compile_setup(validate_setup(realman_document()))
                robot = json.loads((target / "realman-arm1.json").read_text())
                connection = json.loads((target / "realman-arm1-connection.json").read_text())
                collect = json.loads((target / "realman-left.json").read_text())
                policy = json.loads((target / "realman-manual.json").read_text())
            self.assertEqual(robot["profile"], "realman.hardware")
            self.assertEqual(robot["control_hz"], 50)
            self.assertTrue(robot["connection_path"].endswith("realman-arm1-connection.json"))
            self.assertEqual(connection["host"], "192.168.1.19")
            self.assertEqual(connection["port"], 8080)
            self.assertEqual(connection["expected_tool"], "Lq")
            self.assertEqual(collect["config"], str(target / "realman-arm1.json"))
            self.assertEqual(collect["vr_config"], str(target / "realman-collect-vr.json"))
            self.assertEqual(collect["native"]["device"], "quest.owned")
            vr = json.loads((target / "realman-collect-vr.json").read_text())
            self.assertEqual(vr["controller"], "right")
            self.assertIsNone(vr["input_prefix"])
            self.assertEqual(policy["source"], "cli")
            self.assertEqual(compiled["workbench"]["policy"], str(target / "realman-manual.json"))

    def test_realman_dual_compiles_a_pair(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)
            with patch.object(devices, "GENERATED_DIR", target):
                compiled = devices.compile_setup(validate_setup(realman_document(robots=[
                    {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "left",
                     "host": "192.168.1.19"},
                    {"id": "arm2", "vendor": "realman", "model": "rm65", "side": "right",
                     "host": "192.168.1.18"},
                ])))
                left = json.loads((target / "realman-arm1-connection.json").read_text())
                right = json.loads((target / "realman-arm2-connection.json").read_text())
                left_bench = json.loads((target / "realman-left.json").read_text())
                right_bench = json.loads((target / "realman-right.json").read_text())
            self.assertEqual((left["host"], left["expected_tool"]), ("192.168.1.19", "Lq"))
            self.assertEqual((right["host"], right["expected_tool"]), ("192.168.1.18", "work"))
            self.assertEqual(left_bench["native"]["device"], "quest.shared")
            self.assertTrue(left_bench["config"].endswith("realman-arm1.json"))
            self.assertTrue(right_bench["config"].endswith("realman-arm2.json"))
            self.assertEqual(set(compiled["pair"]), {"left", "right"})

    def test_each_arm_uses_its_own_tool_frame_template(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)
            with patch.object(devices, "GENERATED_DIR", target):
                devices.compile_setup(validate_setup(realman_document(robots=[
                    {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "single",
                     "host": "192.168.1.18"},
                ])))
                connection = json.loads((target / "realman-arm1-connection.json").read_text())
            self.assertEqual(connection["host"], "192.168.1.18")
            self.assertEqual(connection["expected_tool"], "work")

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


class SetupLibraryTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = Path(self.folder.name)
        self.patches = [
            patch.object(setup_library, "LIBRARY_DIR", self.root / "setups"),
            patch.object(setup_library, "ACTIVE_PATH", self.root / "active.json"),
        ]
        for entry in self.patches:
            entry.start()
            self.addCleanup(entry.stop)
        self.addCleanup(self.folder.cleanup)

    def test_save_list_load_and_delete(self):
        entry = setup_library.save("ARX 双臂 · 三相机", dual_document())
        self.assertEqual(entry["name"], "ARX 双臂 · 三相机")
        listed = setup_library.entries()
        self.assertEqual([item["slug"] for item in listed], [entry["slug"]])
        self.assertEqual(listed[0]["robots"], 2)
        loaded = setup_library.load(entry["slug"])
        self.assertEqual(loaded["setup"]["cameras"][0]["id"], "wrist_rgb")
        setup_library.delete(entry["slug"])
        self.assertEqual(setup_library.entries(), [])

    def test_names_get_distinct_slugs_and_are_validated(self):
        first = setup_library.save("双臂", dual_document())
        second = setup_library.save("双臂", dual_document())
        self.assertNotEqual(first["slug"], second["slug"])
        with self.assertRaises(SetupError):
            setup_library.save("", dual_document())
        with self.assertRaises(SetupError):
            setup_library.save("x", {"version": 1, "robots": [], "cameras": []})

    def test_active_survives_and_is_cleared_with_its_entry(self):
        entry = setup_library.save("当前配置", dual_document())
        setup_library.set_active(entry["slug"])
        self.assertEqual(setup_library.active(), entry["slug"])
        setup_library.delete(entry["slug"])
        self.assertIsNone(setup_library.active())


if __name__ == "__main__":
    unittest.main()
