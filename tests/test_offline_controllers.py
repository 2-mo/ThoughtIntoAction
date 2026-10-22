"""Exercise actual Placo/model setup with virtual feedback and fake XR input."""

from contextlib import ExitStack
from types import SimpleNamespace
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
from legacy.robot_control.backends.simulated import SimulatedX5
from legacy.robot_control.hardware import OfficialX5HardwareAdapter
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from legacy.quest_teleop import cli
from legacy.quest_teleop import controller as x5_controller
from legacy.quest_teleop import presets as preset_logic
from legacy.quest_teleop.base_controller import DEFAULT_X5_MANIPULATOR_CONFIG, X5TeleopBase
import tyro
from embodied.devices.robots.arx5.paths import X5_2025_URDF


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
        self.stack.enter_context(patch("legacy.quest_teleop.inputs.quest", return_value=self.xr))

    def test_tool_rotation_option_reaches_controller_and_recording_metadata(self):
        def run(controller):
            self.assertEqual(controller.rotation_frame, "tool")
            self.assertEqual(controller._session_control["rotation_frame"], "tool")
            self.assertEqual(controller._session_control["operator_yaw_deg"], 90.)
            self.assertEqual(controller._session_control["controller_to_tool_rotation"],
                             [[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False, operator_yaw_deg=90., rotation_frame="tool")

    def test_aligned_reference_is_recorded_and_pause_requires_fresh_alignment(self):
        import meshcat.transformations as tf
        def run(controller):
            self.assertEqual(controller.rotation_frame, "aligned")
            self.assertEqual(controller._session_control["rotation_frame"], "aligned")
            self.assertEqual(controller._session_control["position_frame"], "aligned_horizontal")
            self.assertEqual(controller._session_control["controller_to_tool_rotation"],
                             [[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
            grasp = X5PinocchioModel().fk(preset_logic.DEFAULT_GRASP_READY_JOINTS)
            controller.ref_ee_quat["right_arm"] = tf.quaternion_from_matrix(grasp)
            controller._process_xr_pose([0, 0, 0, 0, 0, 0, 1], "right_arm")
            basis = controller._rotation_alignment["right_arm"]
            position_basis = controller._position_alignment["right_arm"]
            session = Mock()
            controller._session = session
            controller._publish_runtime()
            packet = session.publish.call_args.args[0]
            np.testing.assert_allclose(packet["rotation_alignment"]["right_arm"], basis)
            np.testing.assert_allclose(packet["position_alignment"]["right_arm"], position_basis)
            # Presets and ordinary clutch resets preserve the mapping.
            controller._reset_teleop_reference()
            np.testing.assert_allclose(controller._rotation_alignment["right_arm"], basis)
            np.testing.assert_allclose(controller._position_alignment["right_arm"], position_basis)
            controller.set_input_paused(True)
            self.assertIsNone(controller._rotation_alignment["right_arm"])
            self.assertIsNone(controller._position_alignment["right_arm"])
            np.testing.assert_allclose(packet["rotation_alignment"]["right_arm"], basis)
            np.testing.assert_allclose(packet["position_alignment"]["right_arm"], position_basis)
            controller._publish_runtime()
            self.assertIsNone(session.publish.call_args.args[0]["rotation_alignment"]["right_arm"])
            self.assertIsNone(session.publish.call_args.args[0]["position_alignment"]["right_arm"])
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False, operator_yaw_deg=90., rotation_frame="aligned")

    def test_button_feedback_uses_control_timers(self):
        def run(controller):
            controller._b_press.started = 10.
            controller._task_end_press.started = 10.25
            controller._activation_gate.down_since = 10.1
            with patch.object(preset_logic.time, "monotonic", return_value=10.5):
                feedback = controller.button_feedback()
            self.assertAlmostEqual(feedback["B"]["progress"], .5)
            self.assertAlmostEqual(feedback["R3"]["progress"], .25)
            self.assertAlmostEqual(feedback["down"]["progress"], .4)
            controller._reset_buttons()
            self.assertEqual(controller.button_feedback()["B"]["progress"], 0.)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_xy_replay_buttons_separate_web_review_from_gated_robot_motion(self):
        def run(controller):
            buttons = {}
            self.xr.get_button_state_by_name = lambda name: buttons.get(name, False)
            controller._collection.submit = Mock(return_value=True)
            controller._prepare_robot_replay = Mock()
            def press(key, start):
                for delta, value in [(0., False), (.1, True), (.15, True), (.2, False), (.25, False)]:
                    buttons[key] = value
                    with patch.object(preset_logic.time, "monotonic", return_value=start + delta):
                        controller._control_gate_ready()
            press("X", 0)
            controller._collection.submit.assert_called_once_with("replay_previous")
            press("Y", 1)
            self.assertIn("启用控制", controller.button_feedback()["Y"])
            controller._prepare_robot_replay.assert_not_called()
            controller._control_enabled = True
            press("Y", 2)
            controller._prepare_robot_replay.assert_called_once()
            self.xr.grip = 1.
            press("Y", 3)
            self.assertIn("Grip", controller.button_feedback()["Y"])
            controller._prepare_robot_replay.assert_called_once()
            self.xr.grip = 0.
            controller._robot_replay = Mock()
            with patch.object(controller.arm_controllers["right_arm"], "stop_control") as stop:
                press("Y", 4)
                self.assertIsNone(controller._robot_replay)
                stop.assert_called_once()
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_table_floor_clamps_cartesian_target_without_changing_xy_or_rotation(self):
        def run(controller):
            from embodied.core.safety import TablePlane
            controller._table_plane = TablePlane(0., 0.)
            controller._control_enabled = True
            self.xr.grip = 1.
            self.xr.get_pose_by_name = Mock(return_value=[0., 0., 0., 0., 0., 0., 1.])
            controller._update_ik()
            initial = controller.effector_task["right_arm"].T_world_frame.copy()
            self.xr.get_pose_by_name.return_value = [0., -1., -.01, 0., 0., 0., 1.]
            controller._update_ik()
            target = controller.effector_task["right_arm"].T_world_frame
            self.assertAlmostEqual(target[2, 3], 0.)
            np.testing.assert_allclose(target[:2, 3], initial[:2, 3] + [.01, 0], atol=1e-8)
            np.testing.assert_allclose(target[:3, :3], initial[:3, :3], atol=1e-8)
            self.xr.get_pose_by_name.return_value = [0., .01, -.01, 0., 0., 0., 1.]
            controller._update_ik()
            self.assertGreater(controller.effector_task["right_arm"].T_world_frame[2, 3], 0.)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False, scale_factor=1.,
                             max_eef_linear_speed_m_s=0.)

    def test_legacy_wrist_rotation_and_fingertip_floor(self):
        from embodied.core.safety import TablePlane
        def run(controller):
            self.assertEqual(controller.manipulator_config["right_arm"]["link_name"], "link6")
            self.xr.grip = 1.
            self.xr.get_pose_by_name = Mock(return_value=[0., 0., 0., 0., 0., 0., 1.])
            controller._update_ik()
            initial = controller.effector_task["right_arm"].T_world_frame.copy()
            # Pure controller rotation must leave the wrist target fixed.
            self.xr.get_pose_by_name.return_value = [0., 0., 0., 0., .2, 0., np.sqrt(.96)]
            controller._update_ik()
            target = controller.effector_task["right_arm"].T_world_frame
            np.testing.assert_allclose(target[:3, 3], initial[:3, 3], atol=1e-9)
            self.assertGreater(np.linalg.norm(target[:3, :3] - initial[:3, :3]), .1)
            # Downward tool: the wrist must stay 145 mm above the table,
            # since the protected point is still the fingertips.
            controller.ref_ee_quat["right_arm"] = np.array([1., 0., 0., 0.])
            adjusted = controller._clamp_tcp_target(
                "right_arm", np.zeros(3), np.array([.2, .1, -.3]),
                np.array([0., np.pi / 2, 0.]), TablePlane(0., 0.))
            np.testing.assert_allclose(adjusted, [.2, .1, .145], atol=1e-9)
            # Pointing up: the wrist origin itself is not the floor constraint.
            adjusted = controller._clamp_tcp_target(
                "right_arm", np.zeros(3), np.array([.2, .1, -.3]),
                np.array([0., -np.pi / 2, 0.]), TablePlane(0., 0.))
            np.testing.assert_allclose(adjusted, [.2, .1, -.145], atol=1e-9)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False, control_link="link6", scale_factor=.5, preset_speed_rad_s=0.,
                             max_eef_linear_speed_m_s=0.)

    def test_quick_base_alignment_updates_both_pose_maps_and_clears_grip_anchor(self):
        from legacy.quest_teleop.frames import R_CONTROLLER_TO_TOOL, operator_yaw_rotation
        def run(controller):
            self.xr.get_timestamp_ns = Mock(return_value=1000)
            controller._session = Mock(id="quick-test", recording=None)
            controller._control_enabled = False
            controller._input_freshness = Mock(ready=Mock(return_value=True))
            controller.ref_controller_xyz["right_arm"] = np.ones(3)
            basis = operator_yaw_rotation(90) @ R_CONTROLLER_TO_TOOL
            controller.set_frame_calibration(basis, "quick-test")
            np.testing.assert_allclose(controller.R_headset_world, basis)
            np.testing.assert_allclose(controller._session_control["quest_to_base_rotation"], basis)
            self.assertIsNone(controller.ref_controller_xyz["right_arm"])
            controller._control_enabled = True
            with self.assertRaises(RuntimeError):
                controller.set_frame_calibration(R_CONTROLLER_TO_TOOL, "quick-test")
            controller._session = None
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False, rotation_frame="base", control_link="link6")

    def test_calibrated_tcp_targets_decouple_rotation_from_translation(self):
        import meshcat.transformations as tf
        from legacy.quest_teleop.frames import R_CONTROLLER_TO_TOOL
        def run(controller):
            self.assertEqual(controller.manipulator_config["right_arm"]["link_name"], "eef_link")
            controller._session = Mock(id="calibration-test", recording=None)
            with patch.object(controller._activation_gate, "update", return_value=True):
                self.assertFalse(controller._control_gate_ready())
                self.assertFalse(controller._control_enabled)
            controller.set_frame_calibration(R_CONTROLLER_TO_TOOL, "calibration-test")
            controller._control_enabled = True
            with self.assertRaises(RuntimeError):
                controller.set_frame_calibration(R_CONTROLLER_TO_TOOL, "calibration-test")
            self.xr.grip = 1.
            anchor = tf.quaternion_from_euler(.4, -.2, .7)
            self.xr.get_pose_by_name = Mock(return_value=[.2, .3, .4, *anchor[[1, 2, 3, 0]]])
            initial = controller.placo_robot.get_T_world_frame("eef_link").copy()
            controller._update_ik()
            np.testing.assert_allclose(controller.effector_task["right_arm"].T_world_frame, initial, atol=1e-8)
            current = tf.quaternion_multiply(tf.quaternion_about_axis(.03, [1, 0, 0]), anchor)
            self.xr.get_pose_by_name.return_value = [.2, .3, .4, *current[[1, 2, 3, 0]]]
            controller._update_ik()
            target = controller.effector_task["right_arm"].T_world_frame.copy()
            np.testing.assert_allclose(target[:3, 3], initial[:3, 3], atol=1e-10)
            expected = tf.rotation_matrix(.03, [0, -1, 0])[:3, :3] @ initial[:3, :3]
            np.testing.assert_allclose(target[:3, :3], expected, atol=1e-8)
            self.xr.get_pose_by_name.return_value[2] -= .01
            controller._update_ik()
            moved = controller.effector_task["right_arm"].T_world_frame
            np.testing.assert_allclose(moved[:3, 3], initial[:3, 3] + [.01, 0, 0], atol=1e-8)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False, rotation_frame="calibrated", scale_factor=1., max_eef_linear_speed_m_s=0)

    def test_startup_gate_blocks_commands_and_presets_until_down_hold(self):
        def run(controller):
            self.assertFalse(controller._control_enabled)
            arm = controller.arm_controllers["right_arm"]
            with patch.object(arm, "set_catch_pos", wraps=arm.set_catch_pos) as gripper, \
                 patch.object(arm, "set_joint_positions", wraps=arm.set_joint_positions) as joints:
                self.xr.a = self.xr.b = True
                self.xr.grip = self.xr.trigger = 1.
                controller._pre_ik_update()
                controller._send_command()
                self.assertIsNone(controller._preset_target)
                self.assertIsNone(self.service.service.owner)
                gripper.assert_not_called()
                joints.assert_not_called()
                self.xr.a = self.xr.b = False
                self.xr.grip = self.xr.trigger = 0.
                for now, axes in [(0, (0, 0)), (.25, (0, -1)), (1.24, (0, -1)),
                                  (1.25, (0, -1))]:
                    self.xr.axes = axes
                    update = controller._activation_gate.update
                    with patch.object(controller._activation_gate, "update",
                                      side_effect=lambda axes, _now, released: update(axes, now, released=released)):
                        self.assertFalse(controller._control_gate_ready())
                self.assertTrue(controller._control_enabled)
                self.assertIsNone(controller.ref_controller_xyz["right_arm"])
                self.assertFalse(controller.active["right_arm"])
                self.assertIsNone(self.service.service.owner)
                gripper.assert_not_called()
                joints.assert_not_called()
                controller._send_command()
                self.assertIsNotNone(self.service.service.owner)
                controller._start_preset("HOME", controller._home_joints)
                gripper.reset_mock()
                joints.reset_mock()
                for now, axes in [(2, (0, 0)), (2.25, (0, -1)), (3.24, (0, -1)),
                                  (3.25, (0, -1))]:
                    self.xr.axes = axes
                    update = controller._activation_gate.update
                    with patch.object(controller._activation_gate, "update",
                                      side_effect=lambda axes, _now, released: update(axes, now, released=released)):
                        controller._control_gate_ready()
                self.assertFalse(controller._control_enabled)
                self.assertIsNone(controller._preset_target)
                self.assertIsNone(self.service.service.owner)
                controller._send_command()
                gripper.assert_not_called()
                joints.assert_not_called()
                self.assertFalse(controller._collection_status()["control_enabled"])
                for now, axes in [(4, (0, 0)), (4.25, (0, -1)), (5.24, (0, -1)),
                                  (5.25, (0, -1))]:
                    self.xr.axes = axes
                    update = controller._activation_gate.update
                    with patch.object(controller._activation_gate, "update",
                                      side_effect=lambda axes, _now, released: update(axes, now, released=released)):
                        controller._control_gate_ready()
                self.assertTrue(controller._control_enabled)
                self.assertIsNone(controller.ref_controller_xyz["right_arm"])
                controller.set_input_paused(True)
                self.assertFalse(controller._control_enabled)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_explicit_headset_pause_preserves_controller_and_reanchors_after_fresh_frames(self):
        def run(controller):
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
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
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
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

    def test_b_without_task_moves_to_grasp_and_grip_cancels(self):
        def run(controller):
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
            self.assertFalse(controller._collection.task_active)
            self.assertEqual(controller._collection_status()["state"], "IDLE")
            def step(t, b=False, grip=0.):
                self.xr.b, self.xr.grip = b, grip
                with patch.object(preset_logic, "time", SimpleNamespace(monotonic=lambda: t)):
                    controller._pre_ik_update()
            step(0)
            step(.1, b=True, grip=1.); step(.14, b=True, grip=1.)
            step(.2, grip=1.); step(.24, grip=1.)
            self.assertIsNone(controller._preset_target)
            step(.3, b=True); step(.34, b=True); step(.4); step(.44)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            np.testing.assert_allclose(controller._preset_target, controller._grasp_ready_joints)
            self.assertEqual(controller._collection_status()["state"], "POSITIONING")
            step(.5, grip=1.)
            self.assertIsNone(controller._preset_target)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_a_requires_task_and_does_not_queue_a_press_from_idle(self):
        def run(controller):
            controller._control_enabled = True
            actions = controller._collection
            controller._collection = Mock(busy=False, recording=False, error=None, task_active=False)
            controller._collection.take_home.return_value = False
            def step(t, a=False):
                self.xr.a = a
                with patch.object(preset_logic, "time", SimpleNamespace(monotonic=lambda: t)):
                    controller._pre_ik_update()
            step(0); step(.1, True); step(.14, True); step(.2); step(.24)
            self.assertIsNone(controller._preset_target)
            self.assertIsNone(controller._episode_position_pending)
            controller._collection.submit.assert_not_called()
            # A held before task activation must be released and pressed again.
            step(.3, True); step(.34, True)
            controller._collection.task_active = True
            step(.4); step(.44)
            self.assertIsNone(controller._preset_target)
            step(.5, True); step(.54, True); step(.6); step(.64)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            # Ending/pausing during positioning must not start recording on arrival.
            controller._collection.task_active = False
            controller._finish_preset("GRASP_READY")
            controller._collection.submit.assert_not_called()
            controller._collection = actions
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_collection_edges_map_once_and_home_waits_for_grip_release(self):
        def run(controller):
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
            actions = controller._collection
            controller._collection = Mock()
            controller._collection.busy = False
            controller._collection.task_active = True
            controller._collection.error = None
            controller._collection.recording = False
            controller._collection.take_home.return_value = False
            def step(t, a=False, b=False, grip=0., stick=False):
                self.xr.a, self.xr.b, self.xr.grip, self.xr.stick = a, b, grip, stick
                with patch.object(preset_logic, "time", SimpleNamespace(monotonic=lambda: t)):
                    controller._pre_ik_update()
            step(0)
            step(.1, a=True); step(.14, a=True); step(.2); step(.24)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            controller._collection.submit.assert_not_called()
            controller._finish_preset("GRASP_READY")
            controller._collection.submit.assert_called_once_with("toggle_episode")
            controller._collection.recording = True
            step(.3, a=True); step(.34, a=True); step(.4); step(.44)
            controller._collection.recording = False
            step(.5, stick=True); step(.54, stick=True); step(1.51, stick=True)
            step(1.6, stick=True); step(1.7); step(1.74)
            self.assertEqual([call.args[0] for call in controller._collection.submit.call_args_list],
                             ["toggle_episode", "toggle_episode", "end_task"])
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
                             ["toggle_episode", "toggle_episode", "end_task", "start_task", "pause_task"])
            step(7.1, b=True); step(7.15, b=True); step(8.2, b=True)
            step(8.3); step(8.35)
            self.assertEqual(controller._collection.submit.call_args.args[0], "home")
            self.assertEqual([c.args[0] for c in controller._collection.submit.call_args_list].count("toggle_episode"), 2)
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

    def test_a_waits_for_save_and_grip_and_cancellation_prevents_autorecord(self):
        def run(controller):
            controller._control_enabled = True
            actions = controller._collection
            controller._collection = Mock(busy=False, recording=False, error=None, task_active=True)
            controller._collection.take_home.return_value = False
            def step(t, a=False, grip=0.):
                self.xr.a, self.xr.grip = a, grip
                with patch.object(preset_logic, "time", SimpleNamespace(monotonic=lambda: t)):
                    controller._pre_ik_update()
            step(0)
            step(.1, True, 1.); step(.15, True, 1.); step(.2, grip=1.); step(.25, grip=1.)
            self.assertIsNone(controller._preset_target)
            controller._collection.submit.assert_not_called()
            step(.3)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            step(.4, grip=1.)
            controller._finish_preset("GRASP_READY")
            controller._collection.submit.assert_not_called()
            controller._collection.recording = True
            step(.5); step(.6, True); step(.65, True); step(.7); step(.75)
            controller._collection.submit.assert_called_once_with("toggle_episode")
            self.assertIsNone(controller._preset_target)
            controller._collection.busy = True
            controller._collection.recording = False
            step(.8)
            self.assertIsNone(controller._preset_target)
            controller._collection.busy = False
            step(.9)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            controller._finish_preset("GRASP_READY")
            controller._collection.submit.assert_called_once_with("toggle_episode")
            controller._collection = actions
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_b_pregrasp_never_autorecords_and_a_saves_during_recording(self):
        def run(controller):
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
            actions = controller._collection
            controller._collection = Mock()
            controller._collection.busy = False
            controller._collection.task_active = True
            controller._collection.error = None
            controller._collection.recording = False
            controller._collection.take_home.return_value = False
            def step(t, a=False, b=False, grip=0.):
                self.xr.a, self.xr.b, self.xr.grip = a, b, grip
                with patch.object(preset_logic, "time", SimpleNamespace(monotonic=lambda: t)):
                    controller._pre_ik_update()
            step(0); step(.1, b=True); step(.14, b=True); step(.2); step(.24)
            self.assertEqual(controller._preset_name, "GRASP_READY")
            controller._finish_preset("GRASP_READY")
            step(.3)
            controller._collection.submit.assert_not_called()
            controller._collection.recording = True
            step(.4, b=True); step(.44, b=True); step(.5); step(.54)
            self.assertIsNone(controller._preset_target)
            controller._collection.submit.assert_not_called()
            step(.6, a=True, grip=1.); step(.64, a=True, grip=1.); step(.7, grip=1.); step(.74, grip=1.)
            controller._collection.submit.assert_called_once_with("toggle_episode")
            controller._collection = actions
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False)

    def test_cli_operator_yaw_reaches_controller_and_recording_metadata(self):
        from legacy.quest_teleop.frames import operator_yaw_rotation
        from legacy.quest_teleop.base_controller import R_HEADSET_TO_WORLD
        def run(controller):
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
            np.testing.assert_allclose(controller.R_headset_world,
                                       operator_yaw_rotation(90) @ R_HEADSET_TO_WORLD)
            self.assertEqual(controller._session_control["operator_yaw_deg"], 90)
            limiter = controller._cartesian_limits["right_arm"]
            self.assertEqual(limiter.linear_speed, .3)
            self.assertEqual(limiter.angular_speed, 1.5)
            self.assertEqual(controller._session_control["max_eef_linear_speed_m_s"], .3)
            self.assertEqual(controller._session_control["max_eef_angular_speed_rad_s"], 1.5)
            np.testing.assert_allclose(controller._session_control["quest_to_base_rotation"],
                                       controller.R_headset_world)
            controller.close()
        with patch.object(cli.PresetSingleX5Controller, "run", run):
            cli.presets_main(actuate=False, operator_yaw_deg=90,
                             max_eef_linear_speed_m_s=.3, max_eef_angular_speed_rad_s=1.5)

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
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
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
            from legacy.robot_control.cli import main as simulate
            virtual = ServiceFixture()
            self.stack.callback(virtual.close)
            simulate(["--service-socket", str(virtual.path), "--input", "idle", "--control-mode", "position"])
        self.assertEqual(completed, [True])
        self.hardware.assert_not_called()

    def test_home_motion_keeps_grip_cancellation(self):
        completed = []
        def run(controller):
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
            controller._start_preset("HOME", controller._home_joints)
            self.assertEqual(controller._preset_name, "HOME")
            self.assertEqual(controller._preset_rates["HOME"], (.3, .5))
            controller._send_command()
            self.assertEqual(controller.arm_controllers["right_arm"]._pending_profile, "home")
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
            controller._control_enabled = True  # Existing tests exercise already-enabled controls.
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
        model = X5PinocchioModel()
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
