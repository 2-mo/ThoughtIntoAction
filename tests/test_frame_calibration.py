"""Measured frame fitting and TCP pivot checks; no Quest or arm commands."""
import unittest
import json
from pathlib import Path
import tempfile
from unittest.mock import patch
import numpy as np
import meshcat.transformations as tf

from legacy.quest_teleop.frame_calibration import fit_directions, validate_rotation, standby_snapshot, capture, main
from legacy.quest_teleop.frames import R_CONTROLLER_TO_TOOL, operator_yaw_rotation
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from legacy.quest_teleop.presets import DEFAULT_GRASP_READY_JOINTS


class FrameCalibrationTests(unittest.TestCase):
    def test_quick_heading_maps_forward_left_and_up_for_any_yaw(self):
        from legacy.quest_teleop.frame_calibration import heading_rotation
        for yaw in (-179, -90, 0, 45, 179):
            q = tf.quaternion_about_axis(np.radians(yaw), [0, 1, 0])
            rotation = tf.quaternion_matrix(q)[:3, :3]
            basis = heading_rotation(q[[1, 2, 3, 0]])
            np.testing.assert_allclose(basis @ (rotation @ [0, 0, -1]), [1, 0, 0], atol=1e-10)
            np.testing.assert_allclose(basis @ (rotation @ [-1, 0, 0]), [0, 1, 0], atol=1e-10)
            np.testing.assert_allclose(basis @ [0, 1, 0], [0, 0, 1], atol=1e-10)
        for q in ([0, 0, 0, 0], [float("nan"), 0, 0, 1],
                  [np.sqrt(.5), 0, 0, np.sqrt(.5)]):
            with self.assertRaises(ValueError):
                heading_rotation(q)

    def test_quick_heading_save_and_explicit_reuse(self):
        from legacy.quest_teleop.frames import R_CONTROLLER_TO_TOOL
        with tempfile.TemporaryDirectory() as directory:
            config = {"runtime": {"socket": "unused"}, "paths": {"log_dir": directory}}
            with patch("embodied.core.project.load_config", return_value=config), \
                 patch("legacy.quest_teleop.frame_calibration.standby_snapshot", return_value=({"session_id": "a", "rotation_frame": "base"}, np.zeros(3), 1)), \
                 patch("legacy.quest_teleop.frame_calibration.capture_heading", return_value=R_CONTROLLER_TO_TOOL) as capture_quick, \
                 patch("builtins.input", return_value=""), \
                 patch("legacy.quest_teleop.frame_calibration.request") as apply:
                main([])
                main(["--reuse"])
                self.assertEqual(capture_quick.call_count, 1)
                self.assertEqual(apply.call_count, 2)
                np.testing.assert_allclose(apply.call_args.kwargs["rotation"], R_CONTROLLER_TO_TOOL)

    def test_measurements_recover_heading_without_controller_or_gripper_orientation(self):
        for yaw in (-170, -90, 0, 57, 90):
            expected = operator_yaw_rotation(yaw) @ R_CONTROLLER_TO_TOOL
            rotation, errors = fit_directions(expected.T @ [.18, 0, 0], expected.T @ [0, .14, 0])
            np.testing.assert_allclose(rotation, expected, atol=1e-10)
            np.testing.assert_allclose(errors, 0, atol=1e-5)

    def test_bad_movements_and_reflections_are_rejected(self):
        for x, y in (([0, 0, -.02], [-.15, 0, 0]), ([0, .15, -.15], [-.15, 0, 0]),
                     ([0, 0, -.15], [.15, 0, 0]), ([0, 0, -.15], [0, 0, -.2]),
                     ([float('nan'), 0, -.15], [-.15, 0, 0])):
            with self.subTest(x=x, y=y), self.assertRaises(ValueError):
                fit_directions(x, y)
        for bad in (np.eye(3), np.zeros((3, 3)), np.diag([1, 1, -1]), [[1, 0, 0]]):
            with self.assertRaises(ValueError):
                validate_rotation(bad)

    def test_small_direction_error_is_fitted_without_vertical_tilt(self):
        rotation, errors = fit_directions([.01, .002, -.18], [-.15, -.002, 0])
        np.testing.assert_allclose(rotation @ [0, 1, 0], [0, 0, 1], atol=1e-10)
        self.assertLess(max(errors), 2.)

    def test_sampling_rejects_active_stale_or_restarted_session(self):
        status = {"session_id": "a", "state_age_s": 0., "recording": None,
                  "state": {"control_enabled": True}}
        with patch("legacy.quest_teleop.frame_calibration.request", return_value=status):
            with self.assertRaises(RuntimeError):
                standby_snapshot("unused")
            status["state"]["control_enabled"] = False
            with self.assertRaises(RuntimeError):
                standby_snapshot("unused")
            with self.assertRaises(RuntimeError):
                standby_snapshot("unused", "b")
        with patch("legacy.quest_teleop.frame_calibration.standby_snapshot", return_value=({}, np.zeros(3), 1)), \
             patch("legacy.quest_teleop.frame_calibration.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "时间戳"):
                capture("unused", "a")

    def test_interactive_calibration_saves_measurements_and_applies_to_same_session(self):
        points = [np.zeros(3), np.array([0, 0, -.18]), np.ones(3), np.array([.85, 1, 1])]
        with tempfile.TemporaryDirectory() as directory:
            config = {"runtime": {"socket": "unused"}, "paths": {"log_dir": directory}}
            with patch("embodied.core.project.load_config", return_value=config), \
                 patch("legacy.quest_teleop.frame_calibration.standby_snapshot", return_value=({"session_id": "a", "rotation_frame": "calibrated"}, points[0], 1)), \
                 patch("legacy.quest_teleop.frame_calibration.capture", side_effect=points), \
                 patch("builtins.input", return_value=""), \
                 patch("legacy.quest_teleop.frame_calibration.request") as apply:
                main([])
            payload = apply.call_args.kwargs
            self.assertEqual(payload["session_id"], "a")
            self.assertEqual(apply.call_args.args[1], "input.calibrate")
            np.testing.assert_allclose(payload["rotation"], R_CONTROLLER_TO_TOOL, atol=1e-10)
            saved = json.loads(next(Path(directory).glob("frame-calibration-*.json")).read_text())
            np.testing.assert_allclose(saved["positions_quest_m"], points)

    def test_tip_rotation_keeps_tcp_fixed_and_compensates_wrist_position(self):
        tcp, wrist = X5PinocchioModel(eef="eef_link"), X5PinocchioModel(eef="link6")
        q = np.array(DEFAULT_GRASP_READY_JOINTS)
        initial = tcp.fk(q)
        wrist_initial = wrist.fk(q)
        np.testing.assert_allclose(initial[:3, 3] - wrist_initial[:3, 3],
                                   wrist_initial[:3, :3] @ [.145, 0, 0], atol=1e-10)
        for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            for angle in (-.03, .03):
                target = initial.copy()
                target[:3, :3] = tf.rotation_matrix(angle, axis)[:3, :3] @ initial[:3, :3]
                solution = tcp.ik(target, q)
                actual = tcp.fk(solution)
                np.testing.assert_allclose(actual[:3, 3], initial[:3, 3], atol=1e-4)
                np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=1e-3)
                if axis[2] == 0:
                    self.assertGreater(np.linalg.norm(wrist.fk(solution)[:3, 3] - wrist_initial[:3, 3]), .002)
