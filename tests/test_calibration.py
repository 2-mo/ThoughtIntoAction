"""Synthetic truth, bad data and fake-device tests; never open physical hardware."""

import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from robot_core.model import RobotModel
from robot_core.transforms import from_rt, inverse, pose_error, transform
from robot_control.trajectory import trajectory
from robot_control.motion import move_and_settle
from handeye_calibration.motion import execute_plan, make_plan, validate_plan
from handeye_calibration.session import Session
from handeye_calibration.solver import solve
from handeye_calibration.target import MarkerDetector, marker_svg, normalize_pixels, square_pose, target_spec


def synthetic():
    rng = np.random.default_rng(3)
    def pose():
        return from_rt(cv2.Rodrigues(rng.uniform(-0.8, 0.8, 3))[0], rng.uniform(-0.3, 0.3, 3))
    x, z = pose(), pose()
    rows = []
    for i in range(24):
        a = pose()
        c = inverse(x) @ inverse(a) @ z
        rows.append({"id": i * 3 + 7, "split": "fit" if i < 18 else "validation",
                     "T_base_eef": a.tolist(), "T_camera_target": c.tolist()})
    return rows, x, z


class SolverTests(unittest.TestCase):
    def test_recovers_transform_direction_and_non_contiguous_ids(self):
        rows, x, z = synthetic()
        result = solve(rows)
        self.assertEqual(result["status"], "validated")
        np.testing.assert_allclose(result["T_eef_camera"], x, atol=1e-8)
        np.testing.assert_allclose(result["T_base_target"], z, atol=1e-8)
        self.assertEqual(result["inlier_ids"], [row["id"] for row in rows[:18]])
        self.assertFalse(result["validation"]["refitted"])

    def test_noise_and_gross_outlier_are_handled(self):
        rows, x, _ = synthetic()
        rng = np.random.default_rng(5)
        for row in rows:
            noise = from_rt(cv2.Rodrigues(rng.normal(0, 0.0005, 3))[0], rng.normal(0, 0.0002, 3))
            row["T_camera_target"] = (noise @ row["T_camera_target"]).tolist()
        rows[3]["T_camera_target"][0][3] += 0.15
        result = solve(rows)
        self.assertEqual(result["excluded_ids"], [rows[3]["id"]])
        self.assertEqual(result["status"], "validated")
        distance, angle = pose_error(result["T_eef_camera"], x)
        self.assertLess(distance, 0.002)
        self.assertLess(angle, 0.2)

    def test_held_out_failure_cannot_be_trimmed_or_refitted(self):
        rows, x, _ = synthetic()
        rows[-1]["T_camera_target"][0][3] += 0.1
        result = solve(rows)
        np.testing.assert_allclose(result["T_eef_camera"], x, atol=1e-8)
        self.assertEqual(result["status"], "candidate")
        self.assertEqual(result["validation"]["count"], 6)
        self.assertGreater(result["validation"]["translation_max_m"], 0.05)

    def test_no_validation_is_candidate(self):
        rows, _, _ = synthetic()
        self.assertEqual(solve(rows[:18])["status"], "candidate")

    def test_rejects_duplicates_nonrigid_nan_and_single_axis(self):
        rows, _, _ = synthetic()
        with self.assertRaisesRegex(ValueError, "Duplicate sample IDs"):
            solve(rows + rows[:1])
        rows[-1]["T_base_eef"] = rows[0]["T_base_eef"]
        with self.assertRaisesRegex(ValueError, "near-identical"):
            solve(rows)
        for bad in (np.full((4, 4), np.nan), np.diag([2, 1, 1, 1]), np.diag([-1, 1, 1, 1])):
            with self.assertRaises(ValueError):
                transform(bad)
        for i, row in enumerate(rows):
            row["T_base_eef"] = from_rt(cv2.Rodrigues(np.array([0., 0., i * 0.08]))[0], [i * 0.01, 0, 0]).tolist()
        with self.assertRaisesRegex(ValueError, "diversity"):
            solve(rows)


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
        from robot_control.cli import replay_points
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "plan.json"
            path.write_text(json.dumps({"robot": self.robot.metadata(), "center": self.q.tolist(),
                                        "waypoints": [{"q": (self.q + .01).tolist()}]}))
            qs = replay_points(path, self.robot)
            np.testing.assert_array_equal(qs[0], self.q)
            np.testing.assert_allclose(qs[1], self.q + .01)

    def test_shared_can_lease_rejects_second_owner_and_releases(self):
        from robot_core.device_lease import DeviceLease
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


class TargetTests(unittest.TestCase):
    intrinsics = {"width": 640, "height": 480, "matrix": [[600., 0, 320], [0, 610, 240], [0, 0, 1]],
                  "distortion_model": "distortion.none", "coefficients": [0.] * 5}

    def test_pnp_recovers_optical_target_pose(self):
        h = .05
        points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
        rv, tv = np.array([2.8, .3, .2]), np.array([.01, -.01, .5])
        pixels = cv2.projectPoints(points, rv, tv, np.asarray(self.intrinsics["matrix"]), None)[0]
        actual, error = square_pose(pixels, self.intrinsics, .1)
        np.testing.assert_allclose(actual, from_rt(cv2.Rodrigues(rv)[0], tv), atol=1e-7)
        self.assertLess(error, 1e-6)

    def test_inverse_brown_matches_native_deprojection(self):
        import pyrealsense2 as rs
        intr = rs.intrinsics()
        intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy = 640, 480, 600, 610, 320, 240
        intr.model, intr.coeffs = rs.distortion.inverse_brown_conrady, [.01, -.02, .001, -.001, .005]
        description = {**self.intrinsics, "distortion_model": str(intr.model), "coefficients": list(intr.coeffs)}
        pixels = np.array([[10., 30.], [320., 240.], [600., 400.]])
        expected = [rs.rs2_deproject_pixel_to_point(intr, x.tolist(), 1.)[:2] for x in pixels]
        np.testing.assert_allclose(normalize_pixels(pixels, description), expected)
        description["distortion_model"] = "distortion.modified_brown_conrady"
        with self.assertRaisesRegex(ValueError, "Unsupported"):
            normalize_pixels(pixels, description)

    def test_marker_has_measurable_black_square_and_detects(self):
        svg = marker_svg(target_spec(.1))
        self.assertIn('width="133.333333mm"', svg)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 120)
        objects = np.array([[-.05, .05, 0], [.05, .05, 0], [.05, -.05, 0], [-.05, -.05, 0]])
        pixels = cv2.projectPoints(objects, np.array([2.8, .3, .2]), np.array([.01, -.01, .5]),
                                  np.asarray(self.intrinsics["matrix"]), None)[0].reshape(4, 2)
        warp = cv2.getPerspectiveTransform(np.float32([[0, 0], [119, 0], [119, 119], [0, 119]]),
                                           pixels.astype(np.float32))
        canvas = cv2.warpPerspective(marker, warp, (640, 480), borderValue=255)
        detector = MarkerDetector(target_spec(.1), self.intrinsics)
        result = detector.detect(canvas)
        self.assertGreater(result["T_camera_target"][2, 3], .4)
        with self.assertRaisesRegex(ValueError, "not visible"):
            detector.detect(np.full((480, 640), 255, np.uint8))


class FakeArm:
    def __init__(self, q):
        self.q = np.asarray(q).copy()
        self.enabled = False
        self.damped = False
        self.calls = 0

    def snapshot(self):
        self.calls += 1
        return {"q": self.q.tolist(), "dq": [0.] * 6, "host_monotonic_ns": self.calls * 1_000_000,
                "sdk_timestamp": self.calls * .001}

    def enable(self):
        self.enabled = True

    def command(self, q):
        self.q = np.asarray(q).copy()

    def damp(self):
        self.damped = True


class MotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = RobotModel()
        cls.q = np.array([0., 1.5, 1.45, -1.2, 0., 0.])
        cls.plan = make_plan(cls.robot, cls.q, [[-1, 1], [-1, 1], [-1, 1]])

    def test_plan_model_workspace_and_reference_gates(self):
        self.assertEqual(validate_plan(self.plan, self.robot)["waypoints"], 26)
        bad = copy.deepcopy(self.plan)
        bad["robot"]["eef_frame"] = "eef_link"
        with self.assertRaises(ValueError):
            validate_plan(bad, self.robot)
        bad = copy.deepcopy(self.plan)
        bad["eef_bounds_m"] = [[0, .001]] * 3
        with self.assertRaisesRegex(ValueError, "workspace"):
            validate_plan(bad, self.robot)
        arm = FakeArm(self.q + .1)
        with self.assertRaisesRegex(ValueError, "reference"):
            execute_plan(arm, self.robot, self.plan, lambda *args: None)
        self.assertFalse(arm.enabled)
        self.assertTrue(arm.damped)

    def test_interrupt_and_camera_failure_always_damp_without_return_motion(self):
        for error in (KeyboardInterrupt(), RuntimeError("camera disconnected")):
            arm = FakeArm(self.q)
            moves = []
            def move(arm, robot, target, settings, **kwargs):
                moves.append(target)
            def capture(*args):
                raise error
            with self.assertRaises(type(error)):
                execute_plan(arm, self.robot, self.plan, capture, move=move)
            self.assertTrue(arm.damped)
            self.assertEqual(len(moves), 1)

    def test_motion_tracks_and_settles_using_virtual_time(self):
        arm = FakeArm(self.q)
        clock = [0.]
        def sleep(seconds):
            clock[0] += seconds
        target = self.q + [.02, .01, -.01, .02, .01, -.02]
        move_and_settle(arm, self.robot, target, self.plan["motion"],
                        clock=lambda: clock[0], sleep=sleep)
        np.testing.assert_allclose(arm.q, target)

    def test_stationary_burst_saves_images_and_brackets_once_per_pose(self):
        class Camera:
            sequence = 0
            def read(self):
                self.sequence += 1
                return {"image": np.zeros((32, 32, 3), dtype=np.uint8), "frame_number": self.sequence,
                        "host_received_monotonic_ns": self.sequence * 1_000_000,
                        "device_timestamp_ms": self.sequence * 33.3}
        class Detector:
            def detect(self, image):
                return {"T_camera_target": np.eye(4), "reprojection_error_undistorted_px": .1,
                        "corners_px": [[0, 0]] * 4}
        with tempfile.TemporaryDirectory() as directory:
            session = Session(Path(directory) / "session", self.robot, {}, target_spec(.1))
            arm = FakeArm(self.q)
            sample = session.capture(Camera(), arm, Detector(), "fit")
            self.assertEqual(len(sample["frames"]), 20)
            self.assertTrue((session.directory / sample["frames"][0]["image"]).is_file())
            self.assertIn("robot_before", sample["frames"][0])
            with self.assertRaisesRegex(ValueError, "already captured"):
                session.capture(Camera(), arm, Detector(), "validation")
            session.finish("complete")
            self.assertEqual(json.loads((session.directory / "session.json").read_text())["status"], "complete")

    def test_offline_commands_do_not_import_arm_xr_or_camera_drivers(self):
        from handeye_calibration.cli import main
        import builtins
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.startswith(("arx5_interface", "pyrealsense2", "xrobotoolkit")):
                raise AssertionError(f"Offline command imported {name}")
            return original(name, *args, **kwargs)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "plan.json"
            path.write_text(json.dumps(self.plan))
            with patch("builtins.__import__", side_effect=guarded):
                main(["auto", "--plan", str(path)])


class SharedBackendTests(unittest.TestCase):
    def test_arm_and_gripper_submit_once_with_zero_extra_torque(self):
        import arx5_interface as arx5
        from robot_control.hardware import OfficialX5HardwareAdapter
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
        from robot_control.hardware import OfficialX5HardwareAdapter
        from robot_core.device_lease import DeviceLease
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
