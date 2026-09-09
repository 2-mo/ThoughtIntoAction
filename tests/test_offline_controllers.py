"""Exercise actual Placo/model setup with virtual feedback and fake XR input."""

from contextlib import ExitStack
import os
from arm_service_fixture import ServiceFixture
import hashlib
import inspect
from pathlib import Path
import threading
import unittest
from unittest.mock import patch, Mock
import xml.etree.ElementTree as ET

import arx5_interface as arx5
import numpy as np
from robot_control.backends.simulated import SimulatedX5
from robot_control.hardware import OfficialX5HardwareAdapter
from robot_core.model import RobotModel
from quest_teleop import cli
from quest_teleop import controller as x5_controller
from quest_teleop import presets as preset_logic
from quest_teleop.base_controller import DEFAULT_X5_MANIPULATOR_CONFIG, X5TeleopBase
import tyro
from robot_core.project import X5_2025_URDF


class FakeXR:
    grip = 0.0
    a = False
    b = False
    trigger = 0.0
    stick = False
    axes = (0., 0.)

    def get_key_value_by_name(self, name):
        return {"right_grip": self.grip, "right_trigger": self.trigger}.get(name, 0.0)

    def get_button_state_by_name(self, name):
        return {"A": self.a, "B": self.b, "right_axis_click": self.stick}.get(name, False)

    def get_joystick_state(self, name):
        return self.axes

    def get_motion_tracker_data(self):
        return {}

    def get_pose_by_name(self, name):
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    def close(self):
        pass


class OfflineControllerTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.hardware = self.stack.enter_context(patch.object(arx5, "Arx5JointController", side_effect=AssertionError("Offline tests must not open CAN")))
        self.service = ServiceFixture(hardware=True)
        self.stack.callback(self.service.close)
        self.stack.enter_context(patch.dict(os.environ, {"QUEST_TELEOP_CONFIG": str(self.service.config)}))
        self.xr = FakeXR()
        self.stack.enter_context(patch("quest_teleop.inputs.quest", return_value=self.xr))

    def test_explicit_headset_pause_preserves_controller_and_reanchors_after_fresh_frames(self):
        def run(controller):
            self.xr.get_timestamp_ns = Mock(return_value=100)
            controller._start_preset("HOME", controller._home_joints)
            self.assertIsNotNone(controller._preset_target)
            controller.set_input_paused(True)
            self.assertTrue(controller._input_paused)
            self.assertFalse(controller._input_ready)
            self.assertIsNone(controller._preset_target)
            self.assertIsNone(self.service.service.owner)
            self.assertFalse(controller._closed)
            controller.set_input_paused(False)
            self.assertFalse(controller._resume_input_ready())
            self.xr.get_timestamp_ns.return_value = 101
            self.xr.grip = 1.
            self.assertFalse(controller._resume_input_ready())
            self.xr.grip = 0.
            self.xr.a = False
            self.xr.get_timestamp_ns.return_value = 102
            self.assertTrue(controller._resume_input_ready())
            self.assertFalse(controller._resume_pending)
            self.assertIsNone(controller.ref_controller_xyz["right_arm"])
            np.testing.assert_allclose(controller.placo_robot.state.q[controller.placo_arm_joint_slice["right_arm"]],
                                       controller.arm_controllers["right_arm"].get_joint_positions()[:6])
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_reaching_joint_limit_keeps_control_and_reverse_motion_recovers(self):
        def run(controller):
            arm = controller.arm_controllers["right_arm"]
            lower = np.asarray(arm.robot_config.joint_pos_min)
            upper = np.asarray(arm.robot_config.joint_pos_max)
            slot = controller.placo_arm_joint_slice["right_arm"]
            controller.active["right_arm"] = True
            with patch.object(arm, "set_joint_positions", wraps=arm.set_joint_positions) as send:
                for command in (lambda: X5TeleopBase._send_command(controller), controller._send_command):
                    beyond = (lower + upper) / 2
                    beyond[1] = lower[1] - .25
                    beyond[2] = upper[2] + .3
                    controller.placo_robot.state.q[slot] = beyond
                    command()
                    clipped = np.clip(beyond, lower, upper)
                    np.testing.assert_array_equal(send.call_args.args[0], clipped)
                    self.assertEqual(controller._ik_limit_report["joints"], [2, 3])
                    self.assertFalse(controller._stop_event.is_set())
                    self.assertTrue(controller.active["right_arm"])
                    token = arm.token
                    restored = (lower + upper) / 2
                    controller.placo_robot.state.q[slot] = restored
                    command()
                    np.testing.assert_array_equal(send.call_args.args[0], restored)
                    self.assertEqual(arm.token, token)
                    self.assertEqual(controller._ik_limit_report["joints"], [])
                    self.assertFalse(controller._closed)
            controller.placo_robot.state.q[slot] = np.full(6, np.nan)
            with self.assertRaisesRegex(ValueError, "invalid joint data"):
                controller._bounded_ik_command("right_arm")
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_a_without_task_moves_to_grasp_and_grip_cancels(self):
        def run(controller):
            self.assertFalse(controller._collection.task_active)
            self.assertEqual(controller._collection_status()["state"], "IDLE")
            def step(t, a=False, grip=0.):
                self.xr.a, self.xr.grip = a, grip
                with patch.object(preset_logic.time, "monotonic", return_value=t):
                    controller._pre_ik_update()
            step(0)
            step(.1, a=True, grip=1.); step(.14, a=True, grip=1.)
            step(.2, grip=1.); step(.24, grip=1.)
            self.assertIsNone(controller._preset_target)
            step(.3, a=True); step(.34, a=True); step(.4); step(.44)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            np.testing.assert_allclose(controller._preset_target, controller._grasp_ready_joints)
            self.assertEqual(controller._collection_status()["state"], "POSITIONING")
            step(.5, grip=1.)
            self.assertIsNone(controller._preset_target)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_collection_edges_map_once_and_home_waits_for_grip_release(self):
        def run(controller):
            actions = controller._collection
            controller._collection = Mock()
            controller._collection.busy = False
            controller._collection.task_active = True
            controller._collection.recording = False
            controller._collection.take_home.return_value = False
            def step(t, a=False, b=False, grip=0., stick=False):
                self.xr.a, self.xr.b, self.xr.grip, self.xr.stick = a, b, grip, stick
                with patch.object(preset_logic.time, "monotonic", return_value=t):
                    controller._pre_ik_update()
            step(0)
            step(.1, a=True); step(.14, a=True); step(.2); step(.24)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            controller._collection.submit.assert_not_called()
            controller._finish_preset("GRASP_READY")
            step(.25)
            controller._collection.submit.assert_called_once_with("start_episode")
            step(.3, b=True); step(.34, b=True); step(.4); step(.44)
            step(.5, stick=True); step(.54, stick=True); step(1.51, stick=True)
            step(1.6, stick=True); step(1.7); step(1.74)
            self.assertEqual([call.args[0] for call in controller._collection.submit.call_args_list],
                             ["start_episode", "stop_episode", "end_task"])
            self.assertIsNone(controller._preset_target)
            self.xr.axes = (-1., 0.)
            step(2.)
            self.assertEqual(controller._collection.submit.call_args.args[0], "start_task")
            step(3.01); step(4.)
            self.xr.axes = (1., 0.)
            step(4.1); step(5.2)  # No neutral: must not trigger reset.
            self.xr.axes = (0., 0.)
            step(5.3)
            self.xr.axes = (1., 0.)
            step(5.4); step(6.41); step(7.)
            self.xr.axes = (0., 0.)
            self.assertEqual([call.args[0] for call in controller._collection.submit.call_args_list],
                             ["start_episode", "stop_episode", "end_task", "start_task", "pause_task"])
            step(7.1, b=True); step(7.15, b=True); step(8.2, b=True)
            step(8.3); step(8.35)
            self.assertEqual(controller._collection.submit.call_args.args[0], "home")
            self.assertEqual([c.args[0] for c in controller._collection.submit.call_args_list].count("stop_episode"), 1)
            controller._collection.take_home.return_value = True
            step(8.4, grip=1.)
            self.assertIsNone(controller._preset_target)
            step(1.9)
            self.assertEqual(controller._preset_name, "HOME")
            controller._collection.take_home.return_value = False
            step(2., grip=1.)
            self.assertIsNone(controller._preset_target)
            controller._collection = actions
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_pregrasp_recording_request_cancels_and_recording_a_is_noop(self):
        def run(controller):
            actions = controller._collection
            controller._collection = Mock()
            controller._collection.busy = False
            controller._collection.task_active = True
            controller._collection.recording = False
            controller._collection.take_home.return_value = False
            def step(t, a=False, grip=0.):
                self.xr.a, self.xr.grip = a, grip
                with patch.object(preset_logic.time, "monotonic", return_value=t):
                    controller._pre_ik_update()
            step(0); step(.1, True); step(.14, True); step(.2); step(.24)
            self.assertTrue(controller._record_on_arrival)
            step(.3, grip=1.)
            controller._finish_preset("GRASP_READY")
            step(.4)
            controller._collection.submit.assert_not_called()
            self.assertFalse(controller._start_recording_ready)
            controller._collection.recording = True
            step(.5, True); step(.54, True); step(.6); step(.64)
            self.assertIsNone(controller._preset_target)
            controller._collection.submit.assert_not_called()
            controller._collection = actions
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_model_assets_match_manifest(self):
        import json
        manifest = json.loads((X5_2025_URDF.parent / "provenance.json").read_text())
        for name, expected in manifest["sha256"].items():
            self.assertEqual(hashlib.sha256((X5_2025_URDF.parent / name).read_bytes()).hexdigest(), expected)
        for mesh in ET.parse(X5_2025_URDF).iter("mesh"):
            self.assertTrue((X5_2025_URDF.parent / mesh.attrib["filename"]).is_file())

    def test_simulation_main_initializes_and_steps_without_hardware(self):
        completed = []
        def run(controller):
            controller._update_robot_state()
            controller._update_gripper_target()
            controller._update_ik()
            controller._send_command()
            values = controller.arm_controllers["right_arm"].get_joint_positions()
            self.assertEqual(values.shape, (7,))
            self.assertTrue(np.all(np.isfinite(values)))
            controller._shutdown_robot()
            completed.append(True)
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            from robot_control.cli import main as simulate
            virtual = ServiceFixture()
            self.stack.callback(virtual.close)
            simulate(["--service-socket", str(virtual.path), "--input", "idle", "--control-mode", "position"])
        self.assertEqual(completed, [True])
        self.hardware.assert_not_called()

    def test_home_motion_keeps_grip_cancellation(self):
        completed = []
        def run(controller):
            controller._start_preset("HOME", controller._home_joints)
            self.assertEqual(controller._preset_name, "HOME")
            controller._send_command()
            self.xr.grip = 1.0
            controller._pre_ik_update()
            self.assertIsNone(controller._preset_target)
            controller._shutdown_robot()
            completed.append(True)
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)
        self.assertEqual(completed, [True])
        self.hardware.assert_not_called()

    def test_joint_diagnostic_constructor_accepts_its_existing_arguments(self):
        inspect.signature(OfficialX5HardwareAdapter).bind(
            can_port="unused", actuate=False, preview_time_s=0.05,
            gripper_open_readout=-3.4, gripper_width=0.082,
            control_rate_hz=50, max_joint_speed_rad_s=0.05,
            joint_soft_limit_margin_rad=0.05,
        )

    def test_cli_overrides_profile_values_and_can_disable_actuation(self):
        with patch.object(cli, "PresetSingleX5Controller") as factory:
            tyro.cli(cli.presets_main, args=[
                "--scale-factor", "0.5", "--scale-factor", "0.1",
                "--actuate", "--no-actuate",
            ])
        self.assertEqual(factory.call_args.kwargs["scale_factor"], 0.1)
        self.assertFalse(factory.call_args.kwargs["actuate"])
        self.hardware.assert_not_called()

    def test_runtime_hook_records_preset_and_grip_cancellation_sources(self):
        packets = []
        def run(controller):
            self.assertTrue(controller._enable_session)
            controller._session = Mock()
            controller._session.publish.side_effect = packets.append
            controller._start_preset("HOME", controller._home_joints)
            controller._send_command()
            self.xr.grip = 1.
            controller._pre_ik_update()
            controller.active["right_arm"] = True
            controller._send_command()
            controller._session = None
            controller._shutdown_robot()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(enable_session=True)
        self.assertEqual([p["source"] for p in packets], ["preset", "quest"])
        self.assertEqual(packets[0]["preset_name"], "HOME")
        self.assertIsNone(packets[1]["preset_name"])
        self.hardware.assert_not_called()

    def test_run_failure_still_damps_and_closes_xr(self):
        controller = object.__new__(x5_controller.X5TeleopController)
        controller._enable_session = False
        controller._session = None
        controller.xr_client = Mock()
        controller.close = Mock()
        with patch.object(x5_controller.X5TeleopBase, "run", side_effect=RuntimeError("control thread failed")):
            with self.assertRaisesRegex(RuntimeError, "control thread failed"):
                controller.run()
        controller.close.assert_called_once()

    def test_x5_bridge_preserves_feedback_grip_gate_and_trigger_commands(self):
        controller = object.__new__(x5_controller.X5TeleopController)
        arm = Mock()
        model = RobotModel()
        arm.robot_config.joint_pos_min = model.lower
        arm.robot_config.joint_pos_max = model.upper
        arm.get_joint_positions.return_value = np.array([.1, .2, .3, .4, .5, .6, .04])
        controller.arm_controllers = {"right_arm": arm}
        controller.placo_robot = Mock()
        controller.placo_robot.state.q = np.zeros(8)
        controller.placo_arm_joint_slice = {"right_arm": slice(2, 8)}
        controller.manipulator_config = DEFAULT_X5_MANIPULATOR_CONFIG
        controller.gripper_pos_target = {"right_arm": {"joint7": .082}}
        controller.active = {"right_arm": False}
        controller.xr_client = self.xr
        X5TeleopBase._update_robot_state(controller)
        np.testing.assert_allclose(controller.placo_robot.state.q[2:], arm.get_joint_positions()[:6])
        self.xr.trigger = 1.0
        controller._update_gripper_target()
        X5TeleopBase._send_command(controller)
        arm.set_joint_positions.assert_not_called()
        arm.set_catch_pos.assert_called_with(0.0)
        self.xr.trigger = 0.0
        controller._update_gripper_target()
        controller.active["right_arm"] = True
        desired = np.arange(6, dtype=float) / 10
        controller.placo_robot.state.q[2:] = desired
        X5TeleopBase._send_command(controller)
        controller.placo_robot.state.q[2:] = -1
        np.testing.assert_array_equal(arm.set_joint_positions.call_args.args[0], desired)
        arm.set_catch_pos.assert_called_with(.082)


if __name__ == "__main__":
    unittest.main()
