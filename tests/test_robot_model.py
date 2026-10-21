"""The robot model, and the legacy arm stack (trajectory, replay, terminal control, SDK adapter).

The legacy tests are deleted with src/legacy (todo K-10).
"""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from embodied.core.model import RobotModel
from embodied.core.transforms import pose_error
from legacy.robot_control.trajectory import trajectory


class ModelTests(unittest.TestCase):
    def test_qp_boundary_roundoff_is_normalized_but_real_violations_fail(self):
        robot = RobotModel()
        q = robot.lower.copy()
        q[1] = np.nextafter(q[1], -np.inf)
        q[2] -= 4e-9
        original = q.copy()
        normalized = robot.joints(q)
        np.testing.assert_array_equal(normalized, robot.lower)
        np.testing.assert_array_equal(q, original)
        upper = robot.upper.copy()
        upper[0] += 4e-9
        np.testing.assert_array_equal(robot.joints(upper), robot.upper)
        for delta in (-1e-6, -0.01):
            bad = robot.lower.copy()
            bad[1] += delta
            with self.assertRaises(ValueError):
                robot.joints(bad)
        with self.assertRaises(ValueError):
            robot.joints([float("nan")] * 6)

    def setUp(self):
        self.robot = RobotModel()
        self.q = np.array([0., 1.5, 1.45, -1.2, .1, .2])

    def test_limits_and_fk_ik_roundtrip(self):
        self.assertEqual(self.robot.lower[0], -3.14)
        self.assertEqual(self.robot.upper[-1], 2.)
        target = self.robot.fk(self.q + [0.02, -.01, .02, .01, -.02, .01])
        solution = self.robot.ik(target, self.q)
        distance, angle = pose_error(target, self.robot.fk(solution))
        self.assertLess(distance, 1e-4)
        self.assertLess(angle, 0.06)
        with self.assertRaises(ValueError):
            self.robot.joints([0, 0, 0, 0, 0, 2.1])
        with self.assertRaises(ValueError):
            RobotModel(eef="not_a_frame")

    def test_quintic_has_bounded_speed_acceleration_and_exact_endpoints(self):
        end = self.q + [.2, -.2, .1, .1, -.2, .15]
        times, qs = trajectory(self.robot, self.q, end)
        np.testing.assert_allclose(qs[0], self.q)
        np.testing.assert_allclose(qs[-1], end)
        velocity = np.diff(qs, axis=0) / np.diff(times)[:, None]
        acceleration = np.diff(velocity, axis=0) / np.diff(times)[:-1, None]
        self.assertLessEqual(np.max(np.abs(velocity)), .15 + 1e-9)
        self.assertLessEqual(np.max(np.abs(acceleration)), .3 + 1e-9)
        for speed in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                trajectory(self.robot, self.q, end, speed=speed)

    def test_replay_starts_at_plan_reference_not_a_default_preset(self):
        from legacy.robot_control.cli import replay_points
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "plan.json"
            path.write_text(json.dumps({"robot": self.robot.metadata(), "center": self.q.tolist(),
                                        "waypoints": [{"q": (self.q + .01).tolist()}]}))
            qs = replay_points(path, self.robot)
            np.testing.assert_array_equal(qs[0], self.q)
            np.testing.assert_allclose(qs[1], self.q + .01)

    def test_shared_can_lease_rejects_second_owner_and_releases(self):
        from embodied.contracts.device_lease import DeviceLease
        first = DeviceLease("offline-test-can-only")
        try:
            with self.assertRaisesRegex(RuntimeError, "owned"):
                DeviceLease("offline-test-can-only")
        finally:
            first.close()
        second = DeviceLease("offline-test-can-only")
        second.close()

    def test_terminal_control_does_not_start_a_scene(self):
        root = Path(__file__).resolve().parents[1]
        from arm_service_fixture import ServiceFixture
        service = ServiceFixture()
        self.addCleanup(service.close)
        result = subprocess.run([str(root / "run.sh"), "control-arm", "--service-socket", str(service.path)], input="exit\n",
                                text=True, capture_output=True, cwd="/tmp", timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("Scene (", result.stdout)
        self.assertNotIn("Robot state publisher:", result.stdout)
        self.assertFalse(service.arm.closed)


class SharedBackendTests(unittest.TestCase):
    def test_arm_and_gripper_submit_once_with_zero_extra_torque(self):
        import arx5_interface as arx5
        from legacy.robot_control.hardware import OfficialX5HardwareAdapter
        state = arx5.JointState(6)
        state.pos()[:] = [0, 1.5, 1.45, -1.2, 0, 0]
        state.gripper_pos = .04
        native = MagicMock()
        native.get_joint_state.return_value = state
        with patch.object(arx5, "Arx5JointController", return_value=native):
            backend = OfficialX5HardwareAdapter(
                can_port="offline-test-submit", actuate=False, preview_time_s=.03,
                gripper_open_readout=-3.4, gripper_width=.082, control_rate_hz=50,
                max_joint_speed_rad_s=.15, joint_soft_limit_margin_rad=.05, gripper_force_hold=False,
                gripper_open_limit_m=.077)
            try:
                backend.actuate = True
                backend.set_joint_positions(state.pos().copy() + .01)
                native.set_joint_cmd.assert_not_called()
                backend.set_catch_pos(.04)
                native.set_joint_cmd.assert_called_once()
                command = native.set_joint_cmd.call_args.args[0]
                np.testing.assert_array_equal(command.torque(), np.zeros(6))
                self.assertEqual(command.gripper_pos, .04)
                native.get_timestamp.return_value = 10.
                backend.set_motion_limit(0, max_command_lead_rad=0, preview_time_s=.01, joint_soft_limit_margin_rad=0)
                np.testing.assert_allclose(backend._max_step_rad, np.asarray(backend.robot_config.joint_vel_max) / 50)
                backend.set_joint_positions(state.pos().copy() + .3)
                backend.set_catch_pos(.04)
                self.assertGreater(np.max(native.set_joint_cmd.call_args.args[0].pos() - state.pos()), .05)
                self.assertAlmostEqual(native.set_joint_cmd.call_args.args[0].timestamp, 10.01)
                backend.set_motion_limit(100, max_command_lead_rad=.15, preview_time_s=.02, joint_soft_limit_margin_rad=.05)
                np.testing.assert_allclose(backend._max_step_rad, np.asarray(backend.robot_config.joint_vel_max) / 50)
                backend.set_catch_pos(.04)
                self.assertAlmostEqual(native.set_joint_cmd.call_args.args[0].timestamp, 10.02)
                backend.set_catch_pos(.082)
                self.assertAlmostEqual(native.set_joint_cmd.call_args.args[0].gripper_pos, .077)
                self.assertAlmostEqual(backend.robot_config.gripper_width, .082)
                self.assertAlmostEqual(backend.robot_config.gripper_open_readout, -3.4)
                backend.set_catch_pos(.02)
                self.assertAlmostEqual(native.set_joint_cmd.call_args.args[0].gripper_pos, .02)
            finally:
                backend.close()
        native.set_to_damping.assert_called()

    def test_partial_initialization_failure_damps_and_releases_lease(self):
        import arx5_interface as arx5
        from legacy.robot_control.hardware import OfficialX5HardwareAdapter
        from embodied.contracts.device_lease import DeviceLease
        native = MagicMock()
        native.get_joint_state.side_effect = RuntimeError("feedback failed")
        with patch.object(arx5, "Arx5JointController", return_value=native):
            with self.assertRaisesRegex(RuntimeError, "feedback failed"):
                OfficialX5HardwareAdapter(
                    can_port="offline-test-initialization", actuate=False, preview_time_s=.03,
                    gripper_open_readout=-3.4, gripper_width=.082, control_rate_hz=50,
                    max_joint_speed_rad_s=.15, joint_soft_limit_margin_rad=.05)
        native.set_to_damping.assert_called()
        lease = DeviceLease("offline-test-initialization")
        lease.close()


if __name__ == "__main__":
    unittest.main()
