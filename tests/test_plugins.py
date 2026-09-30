"""Plugin composition must preserve offline boundaries and release owned resources."""

from contextlib import ExitStack
from pathlib import Path
import tempfile
import os
from arm_service_fixture import ServiceFixture
import threading
import unittest
from unittest.mock import patch, Mock

import numpy as np

from robot_core.model import RobotModel
from robot_core.plugins import Registry
from robot_control.backends import factories
from quest_teleop import cli
from quest_teleop.inputs import IdleInput
from robot_control.cli import main as simulate

ROOT = Path(__file__).resolve().parents[1]


class ProbeInput(IdleInput):
    instances = []
    fail = False

    def __init__(self):
        if self.fail:
            raise RuntimeError("input plugin failed")
        self.closed = False
        self.instances.append(self)

    def close(self):
        self.closed = True


class PluginTests(unittest.TestCase):
    def setUp(self):
        self.resources = ExitStack()
        self.addCleanup(self.resources.close)
        root = Path(self.resources.enter_context(tempfile.TemporaryDirectory()))
        (root / "configs").mkdir()
        content = (ROOT / "configs/plugins.toml").read_text()
        content += '\n[input.probe]\nfactory="test_plugins:ProbeInput"\nsimulation_only=true\n'
        (root / "configs/plugins.toml").write_text(content)
        self.resources.enter_context(patch("robot_core.plugins.PROJECT_ROOT", root))
        ProbeInput.instances = []
        ProbeInput.fail = False
        self.service = ServiceFixture()
        self.resources.callback(self.service.close)
        self.resources.enter_context(patch.dict(os.environ, {"QUEST_TELEOP_CONFIG": str(self.service.config)}))

    def test_hardware_rejected_before_factory_import(self):
        registry = Registry()
        with patch("robot_core.plugins.importlib.import_module") as load:
            with self.assertRaisesRegex(ValueError, "cannot load hardware"):
                registry.load("arm", "stanford", allow_hardware=False)
            with self.assertRaisesRegex(ValueError, "Unknown viewer"):
                registry.load("viewer", "typo")
            load.assert_not_called()
    def test_simulation_rejects_actuation_and_real_recording(self):
        for input_name in ("terminal", "idle"):
            for flag in ("--actuate", "--enable-session", "--can-port"):
                with self.subTest(input=input_name, flag=flag), self.assertRaises(SystemExit) as error:
                    simulate(["--input", input_name, flag])
                self.assertEqual(error.exception.code, 2)
        self.assertIsNone(self.service.service.owner)

    def test_invalid_simulation_combinations_fail_before_components_are_created(self):
        for args in (["--input", "idle", "--replay", "missing-plan.json"],
                     ["--input", "idle", "--eef", "link4"],
                     ["--input", "idle", "--scale-factor", "nan"],
                     ["--max-joint-speed-rad-s", "nan"],
                     ["--control-rate-hz", "0"]):
            with self.subTest(args=args), self.assertRaises(SystemExit) as error:
                simulate(args)
            self.assertEqual(error.exception.code, 2)
        with self.assertRaisesRegex(ValueError, "Unknown input"):
            simulate(["--input", "typo"])
        self.assertIsNone(self.service.service.owner)
        self.assertFalse(ProbeInput.instances)
        self.assertIsNone(self.service.service.owner)

    def test_headless_pose_input_validates_without_creating_components(self):
        simulate(["--input", "probe", "--headless"])
        self.assertIsNone(self.service.service.owner)
        self.assertFalse(ProbeInput.instances)
        self.assertIsNone(self.service.service.owner)

    def test_control_inputs_use_same_service_and_release_only_the_lease(self):
        initial = self.service.arm.get_joint_positions()
        with patch("builtins.input", return_value="exit"):
            simulate([])
        def run(controller):
            controller._update_ik()
            controller._send_command()
            self.assertTrue(controller._should_keep_running())
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            simulate(["--input", "probe"])
        self.assertIsNone(self.service.service.owner)
        self.assertFalse(self.service.arm.closed)
        self.assertTrue(ProbeInput.instances[0].closed)
        np.testing.assert_allclose(initial[:6], self.service.arm.get_joint_positions()[:6])

    def test_input_failure_does_not_acquire_or_close_service(self):
        ProbeInput.fail = True
        with self.assertRaisesRegex(RuntimeError, "input plugin failed"):
            simulate(["--input", "probe"])
        self.assertIsNone(self.service.service.owner)
        self.assertFalse(self.service.arm.closed)

    def test_invalid_preset_closes_input_before_acquiring_control(self):
        with self.assertRaisesRegex(ValueError, "outside X5 joint limits"):
            simulate(["--input", "probe", "--home-joints", *(["99"] * 6)])
        self.assertTrue(ProbeInput.instances[0].closed)
        self.assertIsNone(self.service.service.owner)
        self.assertFalse(self.service.arm.closed)

    def test_experimental_arm_actuation_is_rejected_before_creation(self):
        from quest_teleop.experimental.arx2025 import main
        with self.assertRaisesRegex(RuntimeError, "disabled"):
            main(actuate=True)
        self.assertFalse(ProbeInput.instances)

    def test_recording_identifies_selected_pose_input(self):
        from quest_teleop.controller import X5TeleopController
        controller = object.__new__(X5TeleopController)
        controller._session = Mock()
        arm = Mock()
        arm.get_recording_snapshot.return_value = {}
        controller.arm_controllers = {"right_arm": arm}
        controller.active = {"right_arm": True}
        controller._session_control = {"input_plugin": "another_pose_source"}
        controller._input_name = "another_pose_source"
        controller.rotation_frame = "base"
        controller._control_enabled = True
        # This focused fixture bypasses __init__; initialize published state too.
        controller._table_plane = Mock(minimum_z=None)
        controller.R_headset_world = np.eye(3)
        controller._ik_rate = Mock()
        controller._ik_rate.snapshot.return_value = {}
        controller._rotation_alignment = {}
        controller.effector_task = {}
        controller._publish_runtime()
        self.assertEqual(controller._session.publish.call_args.args[0]["source"], "another_pose_source")

    def test_simulated_backend_bounds_targets_and_returns_detached_feedback(self):
        robot = RobotModel()
        arm = factories.simulated(settings={"dynamics_urdf_path": str(robot.urdf), "gripper_width": .082,
            "control_rate_hz": 50, "max_joint_speed_rad_s": .2, "joint_soft_limit_margin_rad": .05})
        old = arm.get_joint_positions()
        arm.set_joint_positions(np.full(6, 100.))
        actual = arm.get_joint_positions()
        self.assertLessEqual(np.max(np.abs(actual[:6] - old[:6])), .2 / 50 + 1e-12)
        actual[:] = 99
        self.assertFalse(np.any(arm.get_joint_positions() == 99))
        arm.set_catch_pos(100.)
        self.assertEqual(arm.get_joint_positions()[6], .082)
        for invalid in ([0.], [float("nan")] * 6):
            with self.assertRaises(ValueError):
                arm.set_joint_positions(invalid)
        arm.close()
        with self.assertRaisesRegex(RuntimeError, "closed"):
            arm.set_joint_positions(old[:6])

    def test_web_plugin_releases_its_server(self):
        scene = Registry().create("viewer", "web", robot=RobotModel())
        try:
            scene.update(np.zeros(6), np.zeros(6))
            self.assertTrue(scene._thread.is_alive())
        finally:
            scene.close()
        self.assertFalse(scene._thread.is_alive())
        self.assertEqual(scene._server.fileno(), -1)
        scene.close()
