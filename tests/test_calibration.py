"""Hand-eye calibration: solver for both mounts, target, plan, stationary capture, results, CLI."""
import builtins
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, Mock, patch

import cv2
import numpy as np

from embodied.applications import calibration
from embodied.applications.calibration.arm import Refused
from embodied.applications.calibration.plan import (MAX_CONSECUTIVE_MISSES, kernel_model, make_plan,
                                                   make_realman_plan, run_plan, target_visible, validate_plan)
from embodied.applications.calibration.session import Session, pose_matrix
from embodied.applications.calibration.solver import solve
from embodied.applications.calibration.target import MarkerDetector, marker_svg, normalize_pixels, square_pose, target_spec
from embodied.core.transforms import from_rt, inverse, pose_error, transform


def synthetic(mount="wrist"):
    """Rows for A X C = Z: wrist A = T_base_eef, fixed A = inv(T_base_eef)."""
    rng = np.random.default_rng(3)
    def pose():
        return from_rt(cv2.Rodrigues(rng.uniform(-0.8, 0.8, 3))[0], rng.uniform(-0.3, 0.3, 3))
    x, z = pose(), pose()
    rows = []
    for i in range(24):
        robot = pose()
        a = robot if mount == "wrist" else inverse(robot)
        c = inverse(x) @ inverse(a) @ z
        rows.append({"id": i * 3 + 7, "split": "fit" if i < 18 else "validation",
                     "T_base_eef": robot.tolist(), "T_camera_target": c.tolist()})
    return rows, x, z


class SolverTests(unittest.TestCase):
    def test_recovers_both_mounts_and_non_contiguous_ids(self):
        for mount, x_name, z_name in (("wrist", "T_eef_camera", "T_base_target"), ("fixed", "T_base_camera", "T_eef_target")):
            with self.subTest(mount=mount):
                rows, x, z = synthetic(mount)
                result = solve(rows, mount)
                self.assertEqual((result["status"], result["mount"]), ("validated", mount))
                np.testing.assert_allclose(result[x_name], x, atol=1e-8)
                np.testing.assert_allclose(result[z_name], z, atol=1e-8)
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

    def test_joint_fit_recovers_mount_under_small_motion_and_orientation_noise(self):
        for mount, name in (("wrist", "T_eef_camera"), ("fixed", "T_base_camera")):
            with self.subTest(mount=mount):
                rng = np.random.default_rng(17)
                x = from_rt(cv2.Rodrigues(np.array([.1, .05, -.1]))[0], [.04, .02, .10])
                z = from_rt(cv2.Rodrigues(np.array([2.8, .1, .1]))[0], [.05, -.05, .9])
                rows = []
                for i in range(24):
                    span = .2 if i < 18 else .35
                    robot = from_rt(cv2.Rodrigues(rng.uniform(-span, span, 3))[0],
                                    np.array([.1, -.1, .3]) + rng.uniform(-.05, .05, 3))
                    a = robot if mount == 'wrist' else inverse(robot)
                    c = inverse(x) @ inverse(a) @ z
                    c[:3, :3] = cv2.Rodrigues(rng.normal(0, .008, 3))[0] @ c[:3, :3]
                    c[:3, 3] += rng.normal(0, .0003, 3)
                    rows.append({'id': i, 'split': 'fit' if i < 18 else 'validation',
                                 'T_base_eef': robot.tolist(), 'T_camera_target': c.tolist()})
                result = solve(rows, mount)
                self.assertEqual(result['status'], 'validated')
                self.assertEqual(result['refinement'], 'joint_se3_least_squares')
                distance, angle = pose_error(result[name], x)
                self.assertLess(distance, .003)
                self.assertLess(angle, .2)
                self.assertEqual(result['inlier_ids'], list(range(18)))
                # A validation error must not affect the mounting estimate or model selection.
                rows[-1]['T_camera_target'][0][3] += .1
                rejected = solve(rows, mount)
                self.assertEqual(rejected['status'], 'candidate')
                np.testing.assert_allclose(rejected[name], result[name], atol=1e-10)

    def test_joint_fit_does_not_hide_tenfold_translation_scale_error(self):
        rows, _, _ = synthetic()
        for row in rows:
            for axis in range(3):
                row['T_camera_target'][axis][3] *= 10
        with self.assertRaisesRegex(ValueError, r'No fit.*best consensus'):
            solve(rows)

    def test_rejects_duplicates_nonrigid_nan_single_axis_and_unknown_mount(self):
        rows, _, _ = synthetic()
        with self.assertRaisesRegex(ValueError, "Mount"):
            solve(rows, "ceiling")
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
    """Snapshots and moves of CalibrationArm without a kernel; `refuse` holds joint targets to decline."""

    def __init__(self, model, q, refuse=()):
        self.model, self.q, self.refuse = model, np.asarray(q, dtype=float), [np.asarray(r) for r in refuse]
        self.moves, self.calls = [], 0

    def snapshot(self):
        self.calls += 1
        tcp = self.model.fk(self.q)
        w = np.sqrt(max(0., 1 + np.trace(tcp[:3, :3]))) / 2
        x, y, z = (tcp[2, 1] - tcp[1, 2]) / (4 * w), (tcp[0, 2] - tcp[2, 0]) / (4 * w), (tcp[1, 0] - tcp[0, 1]) / (4 * w)
        return {"q": self.q.tolist(), "dq": [0.] * 6, "pose": [*tcp[:3, 3], x, y, z, w],
                "host_monotonic_ns": self.calls * 1_000_000}

    def move(self, q, speed):
        if any(np.allclose(q, r) for r in self.refuse):
            raise Refused("TCP path reaches the table plane")
        self.moves.append(np.asarray(q, dtype=float))
        self.q = np.asarray(q, dtype=float)

    def hold(self):
        pass


class PlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = kernel_model()
        cls.center = np.array([0., 1.0, 1.0, -0.9, 0., 0.])
        cls.plan = make_plan(cls.model, cls.center)

    def test_plan_is_diverse_within_limits_and_tied_to_the_model(self):
        summary = validate_plan(self.plan, self.model)
        self.assertEqual(summary["waypoints"], 26)
        self.assertLessEqual(summary["max_offset_rad"], .25 + 1e-9)
        self.assertEqual([row["split"] for row in self.plan["waypoints"]].count("validation"), 6)
        other = {**self.plan, "robot": {**self.plan["robot"], "eef_frame": "link6"}}
        with self.assertRaisesRegex(ValueError, "robot model"):
            validate_plan(other, self.model)
        with self.assertRaisesRegex(ValueError, "step"):
            make_plan(self.model, self.center, step=.6)

    def test_refused_and_unseen_poses_are_skipped_then_returns_to_centre(self):
        arm = FakeArm(self.model, self.center, refuse=[self.plan["waypoints"][0]["q"]])
        seen, log = [], []
        def sample(split, index):
            if index == 1:
                raise ValueError("Marker not visible")
            seen.append(index)
        taken = run_plan(arm, self.plan, sample, log=log.append)
        self.assertEqual(taken, 24)
        self.assertNotIn(0, seen)
        self.assertTrue(any("refused" in line for line in log) and any("not visible" in line for line in log))
        np.testing.assert_allclose(arm.q, self.center)

    def test_repeated_misses_stop_in_place(self):
        arm = FakeArm(self.model, self.center)
        def sample(split, index):
            raise ValueError("Marker not visible")
        with self.assertRaisesRegex(RuntimeError, "in a row"):
            run_plan(arm, self.plan, sample, log=lambda text: None)
        self.assertEqual(len(arm.moves), MAX_CONSECUTIVE_MISSES)  # no return move after the abort

    def test_kernel_faults_propagate_without_moving_back(self):
        arm = FakeArm(self.model, self.center)
        def sample(split, index):
            raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            run_plan(arm, self.plan, sample)
        self.assertEqual(len(arm.moves), 1)


class RealmanPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from embodied.core.project import PROJECT_ROOT
        cls.model = kernel_model(PROJECT_ROOT / 'assets/robots/realman/runtime/rm65.urdf')
        cls.center = np.array([.11, -.39, 1.13, -.13, 1.81, -.18])
        cls.camera = {**TargetTests.intrinsics, 'serial': 'synthetic-rm65'}
        cls.x = from_rt(cv2.Rodrigues(np.array([0., 0., -.5]))[0], [-.08, -.03, -.11])
        cls.board = cls.model.fk(cls.center) @ cls.x @ from_rt(cv2.Rodrigues(np.array([2.9, .1, 0]))[0], [0, 0, .5])
        cls.view = {'camera_id': 'wrist_rgb', 'camera': cls.camera, 'target': target_spec(.1),
                    'T_eef_camera': cls.x.tolist(), 'T_base_target': cls.board.tolist()}
        cls.plan = make_realman_plan(cls.model, cls.center, view=cls.view)

    def test_preset_has_separate_diverse_splits_and_visible_paths(self):
        report = validate_plan(self.plan, self.model)
        self.assertEqual(report['waypoints'], 26)
        self.assertLessEqual(report['max_offset_rad'], .25 + 1e-9)
        self.assertEqual([row['split'] for row in self.plan['waypoints']].count('validation'), 6)
        self.assertEqual(len({tuple(row['q']) for row in self.plan['waypoints']}), 26)
        self.assertTrue(all(np.count_nonzero(np.abs(np.asarray(row['q']) - self.center) > 1e-10) <= 2
                            for row in self.plan['waypoints']))
        for split in ('fit', 'validation'):
            self.assertGreaterEqual(report[split]['translation_span_m'], .04)
            self.assertGreaterEqual(report[split]['rotation_span_deg'], 20)
            self.assertGreaterEqual(report[split]['rotation_axis_rank'], 2)
        self.assertTrue(target_visible(self.model.fk(self.center), self.view))
        moved = {**self.view, 'T_base_target': from_rt(np.eye(3), [10., 10., 10.]).tolist()}
        self.assertFalse(target_visible(self.model.fk(self.center), moved))
        with self.assertRaisesRegex(ValueError, 'image margin'):
            validate_plan({**self.plan, 'view': moved}, self.model)
        with self.assertRaisesRegex(ValueError, 'Not enough visible'):
            make_realman_plan(self.model, self.center, view=moved)

    def test_reference_candidate_is_used_only_for_planning_and_heldout_is_untouched(self):
        from embodied.applications.calibration.cli import main
        rows = []
        for i, row in enumerate(self.plan['waypoints']):
            rows.append({'id': i, 'q': row['q'], 'split': row['split'],
                         'T_camera_target': (inverse(self.model.fk(row['q']) @ self.x) @ self.board).tolist()
                         if row['split'] == 'fit' else 'held-out values must not be read'})
        with tempfile.TemporaryDirectory() as folder:
            session = Path(folder)
            (session / 'session.json').write_text(json.dumps({
                'kind': 'hand_eye_samples', 'camera_id': 'wrist_rgb', 'mount': 'wrist',
                'camera': self.camera, 'target': target_spec(.1), 'plan': self.plan, 'samples': rows}))
            result = {'status': 'candidate', 'T_eef_camera': self.x.tolist(), 'inlier_ids': list(range(20))}
            (session / 'result.json').write_text(json.dumps(result))
            output = session / 'preset.json'
            with patch('embodied.applications.calibration.save') as save, patch('builtins.print'):
                main(['plan', str(session), '--output', str(output)])
            plan = json.loads(output.read_text())
            self.assertEqual(plan['view']['source_status'], 'candidate')
            self.assertEqual(plan['view']['purpose'], 'approximate view planning only; not an adopted calibration')
            np.testing.assert_allclose(plan['view']['T_base_target'], self.board, atol=1e-12)
            self.assertEqual(json.loads((session / 'result.json').read_text()), result)
            save.assert_not_called()
            with patch('builtins.print'), self.assertRaises(FileExistsError):
                main(['plan', str(session), '--output', str(output)])

    def test_simulation_preview_rejects_hardware_before_acquiring_control(self):
        from embodied.applications.calibration.cli import main
        client = MagicMock()
        client.identity = {'backend': 'real'}
        with patch('embodied.contracts.runtime.control.ControlClient', return_value=client), \
                patch('embodied.applications.calibration.arm.CalibrationArm') as arm, \
                self.assertRaisesRegex(ValueError, 'simulation runtime'):
            main(['preview', '/missing-plan.json', '--run', '/fake-runtime'])
        arm.assert_not_called()

    def test_preview_records_actual_poses_and_fails_if_a_point_is_refused(self):
        from embodied.applications.calibration.cli import main
        client = MagicMock()
        client.identity = {'backend': 'sim'}
        client.parameters = {'model': {'xml_path': str(self.model.urdf.with_suffix('.xml'))}}
        for refuse in (False, True):
            with self.subTest(refuse=refuse), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'plan.json'
                path.write_text(json.dumps(self.plan))
                arm = FakeArm(self.model, self.center,
                              refuse=[self.plan['waypoints'][0]['q']] if refuse else [])
                context = MagicMock()
                context.__enter__.return_value = arm
                with patch('embodied.contracts.runtime.control.ControlClient', return_value=client), \
                        patch('embodied.applications.calibration.arm.CalibrationArm', return_value=context), \
                        patch('builtins.print'):
                    if refuse:
                        with self.assertRaisesRegex(ValueError, 'Only 25/26'):
                            main(['preview', str(path), '--run', '/fake-sim'])
                    else:
                        main(['preview', str(path), '--run', '/fake-sim'])
                report = json.loads(path.with_suffix('.preview.json').read_text())
                self.assertEqual(report['status'], 'failed' if refuse else 'complete')
                self.assertEqual(len(report['samples']), 25 if refuse else 26)
                if not refuse:
                    self.assertGreaterEqual(report['measured_diversity']['validation']['rotation_span_deg'], 20)
                    np.testing.assert_allclose(report['final_q'], self.center)
                context.__exit__.assert_called_once()

    def test_auto_selects_realman_preset_and_rechecks_a_saved_view_before_motion(self):
        from types import SimpleNamespace
        from embodied.applications.calibration.cli import run_auto
        arm = MagicMock()
        arm.session.parameters = {'model': {'xml_path': str(self.model.urdf.with_suffix('.xml'))},
                                  'effective_robot_config': {'model_plugin': 'realman.hardware'}}
        current = self.center + [.2, .1, -.07, .16, -.11, .08]
        arm.snapshot.return_value = {'q': current.tolist()}
        observed = inverse(self.model.fk(self.center) @ self.x) @ self.board
        observed[0, 3] += .005
        data = {'camera_id': 'wrist_rgb', 'camera': self.camera, 'target': target_spec(.1), 'mount': 'wrist',
                'center_observation': {'T_camera_target': observed.tolist()}}
        session = MagicMock()
        session.data = dict(data)
        with patch('embodied.applications.calibration.plan.kernel_model', return_value=self.model), \
                patch('embodied.applications.calibration.plan.make_realman_plan', return_value=self.plan) as preset, \
                patch('embodied.applications.calibration.plan.make_plan') as arx, \
                patch('embodied.applications.calibration.plan.run_plan', return_value=26), patch('builtins.print'):
            run_auto(SimpleNamespace(plan=None, step_rad=.25, speed=.17), arm, session, Mock())
        preset.assert_called_once()
        self.assertEqual(preset.call_args.args[1], current.tolist())
        arx.assert_not_called()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'plan.json'
            path.write_text(json.dumps(self.plan))
            args = SimpleNamespace(plan=path, step_rad=.25, speed=.17)
            with patch('embodied.applications.calibration.plan.kernel_model', return_value=self.model), \
                    patch('embodied.applications.calibration.plan.run_plan', return_value=26) as execute, \
                    patch('builtins.print') as output:
                self.assertTrue(run_auto(args, arm, session, Mock()))
                expected = self.model.fk(current) @ self.x @ observed
                np.testing.assert_allclose(session.data['plan']['view']['T_base_target'], expected)
                np.testing.assert_allclose(session.data['plan']['center'], current)
                validate_plan(session.data['plan'], self.model)
                execute.assert_called_once()
                execute.reset_mock()
                session.data = {**data, 'camera': {**self.camera, 'serial': 'other-camera'}}
                self.assertFalse(run_auto(args, arm, session, Mock()))
                self.assertIn('another camera', output.call_args.args[0])
                execute.assert_not_called()
                session.data = {**data, 'center_observation': {'T_camera_target':
                                from_rt(np.eye(3), [10., 10., 10.]).tolist()}}
                self.assertFalse(run_auto(args, arm, session, Mock()))
                execute.assert_not_called()
                # ARX saved plans retain their existing absolute-centre requirement.
                arm.session.parameters['effective_robot_config']['model_plugin'] = 'arx5'
                self.assertFalse(run_auto(args, arm, session, Mock()))
                self.assertIn('Teach the plan centre', output.call_args.args[0])
                execute.assert_not_called()


class Camera:
    sequence = 0

    def read(self):
        self.sequence += 1
        return {"image": np.zeros((32, 32, 3), dtype=np.uint8), "frame_number": self.sequence,
                "host_received_monotonic_ns": self.sequence * 1_000_000}


class Detector:
    def detect(self, image):
        return {"T_camera_target": np.eye(4), "reprojection_error_undistorted_px": .1, "corners_px": [[0, 0]] * 4}


class SessionTests(unittest.TestCase):
    def test_stationary_burst_saves_images_and_kernel_pose_once_per_pose(self):
        model = kernel_model()
        arm = FakeArm(model, [0., 1.0, 1.0, -0.9, 0., 0.])
        with tempfile.TemporaryDirectory() as directory:
            session = Session(Path(directory) / "session", "wrist_rgb", "wrist", {"serial": "1"}, target_spec(.1), {"model_id": "m"})
            sample = session.capture(Camera(), arm, Detector(), "fit")
            self.assertEqual(len(sample["frames"]), 20)
            self.assertTrue((session.directory / sample["frames"][0]["image"]).is_file())
            np.testing.assert_allclose(sample["T_base_eef"], model.fk(arm.q), atol=1e-9)
            with self.assertRaisesRegex(ValueError, "already captured"):
                session.capture(Camera(), arm, Detector(), "validation")
            session.finish("complete")
            data = json.loads((session.directory / "session.json").read_text())
            self.assertEqual((data["status"], data["camera_id"], data["mount"]), ("complete", "wrist_rgb", "wrist"))

    def test_motion_during_a_burst_rejects_it(self):
        model = kernel_model()
        arm = FakeArm(model, [0., 1.0, 1.0, -0.9, 0., 0.])
        original = arm.snapshot
        def drifting():
            arm.q = arm.q + .002
            return original()
        arm.snapshot = drifting
        with tempfile.TemporaryDirectory() as directory:
            session = Session(Path(directory) / "session", "wrist_rgb", "wrist", {}, target_spec(.1), {})
            with self.assertRaisesRegex(ValueError, "moved"):
                session.capture(Camera(), arm, Detector(), "fit")

    def test_pose_matrix_matches_quaternion_convention(self):
        rotation = cv2.Rodrigues(np.array([.3, -.2, .5]))[0]
        tcp = from_rt(rotation, [.1, .2, .3])
        arm = FakeArm(kernel_model(), np.zeros(6))
        arm.model = type("Fixed", (), {"fk": staticmethod(lambda q: tcp)})()
        np.testing.assert_allclose(pose_matrix(arm.snapshot()["pose"]), tcp, atol=1e-12)


class ResultTests(unittest.TestCase):
    def test_save_keeps_previous_and_load_checks_status_and_serial(self):
        with tempfile.TemporaryDirectory() as directory:
            result = {"status": "validated", "mount": "fixed", "T_base_camera": np.eye(4).tolist(), "camera": {"serial": "42"}}
            calibration.save("exterior_rgb", result, root=directory)
            calibration.save("exterior_rgb", {**result, "method": "second"}, root=directory)
            self.assertTrue((Path(directory) / "exterior_rgb.previous.json").is_file())
            self.assertEqual(calibration.load("exterior_rgb", "42", root=directory)["method"], "second")
            self.assertIsNone(calibration.load("exterior_rgb", "other camera", root=directory))
            self.assertIsNone(calibration.load("wrist_rgb", root=directory))
            calibration.save("wrist_rgb", {"status": "candidate"}, root=directory)
            self.assertIsNone(calibration.load("wrist_rgb", root=directory))
            calibration.save("wrist_rgb", {"status": "candidate", "operator_override": True}, root=directory)
            self.assertIsNotNone(calibration.load("wrist_rgb", root=directory))

    def test_episodes_record_compact_extrinsics(self):
        from embodied.data.capture.capture import hand_eye
        from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
        with tempfile.TemporaryDirectory() as directory:
            calibration.save("wrist_rgb", {"status": "validated", "mount": "wrist", "T_eef_camera": np.eye(4).tolist(),
                                           "camera": {"serial": "7"}, "fit": {"translation_rms_m": .001}, "samples": [1] * 99},
                             root=directory)
            with patch("embodied.applications.calibration.RESULTS", Path(directory)):
                recorded = {"wrist_rgb": hand_eye("wrist_rgb", {"serial": "7"}),
                            "exterior_rgb": hand_eye("exterior_rgb", {"serial": "8"}),
                            "synthetic": hand_eye("synthetic", {})}
            self.assertEqual(recorded["wrist_rgb"]["T_eef_camera"], np.eye(4).tolist())
            self.assertNotIn("samples", recorded["wrist_rgb"])
            self.assertIsNone(recorded["exterior_rgb"])
            self.assertIsNone(recorded["synthetic"])
            writer = RuntimeEpisodeWriter(Path(directory) / "raw", {"runtime_id": "test"}, "t",
                                          {"stream_id": "w", "width": 4, "height": 4, "fps": 30}, hand_eye=recorded)
            manifest = json.loads((writer.path / "episode.json").read_text())
            writer.close("duration")
            self.assertEqual(manifest["hand_eye"]["wrist_rgb"]["mount"], "wrist")

    def test_mount_defaults_and_explicit_setting(self):
        config = {"cameras": {"wrist_rgb": {}, "exterior_rgb": {}, "top": {"mount": "wrist"}, "bad": {"mount": "x"}}}
        self.assertEqual([calibration.mount(name, config) for name in ("wrist_rgb", "exterior_rgb", "top")],
                         ["wrist", "fixed", "wrist"])
        with self.assertRaisesRegex(ValueError, "mount"):
            calibration.mount("bad", config)
        with self.assertRaisesRegex(ValueError, "Unknown camera"):
            calibration.mount("missing", config)


class CliTests(unittest.TestCase):
    def test_auto_preview_updates_and_stop_holds_at_each_capture_stage(self):
        from embodied.applications.calibration.cli import main
        from embodied.contracts.runtime.control import ControlSession
        settings = {'cameras': {'wrist_rgb': {'serial': '1', 'width': 32, 'height': 32, 'fps': 30}}}
        for stop_at in (None, 'retry', 'hold', 'preview', 'move', 'burst', 'write', 'close'):
            with self.subTest(stop_at=stop_at), tempfile.TemporaryDirectory() as folder:
                phase, displayed, texts = ['prepare'], [], []
                camera = MagicMock()
                camera.__enter__.return_value = camera
                camera.metadata = {'serial': '1'}
                camera.read.side_effect = Camera().read
                client = MagicMock()
                feedback = {'q': [0.] * 6, 'dq': [0.] * 6, 'pose': [0., 0., 0., 0., 0., 0., 1.],
                            'gripper_width': .02, 'feedback_monotonic_ns': 1_000_000, 'model_id': 'test'}
                client.identity = {'runtime_id': 'test'}
                operation_reads, attempts = [], []

                def status():
                    operation_reads.append(phase[0])
                    running = operation_reads.count('move') < 3
                    return {**feedback, 'operations': [{'operation_id': 'op',
                             'status': 'running' if running else 'completed'}]}

                def preview(*args, **kwargs):
                    phase[0] = 'preview'
                    return {'times': [0., .1]}

                def execute(plan):
                    phase[0] = 'move'
                    return {'operation_id': 'op'}

                client.status.side_effect = status
                client.preview.side_effect = preview
                client.execute.side_effect = execute
                client.wait.side_effect = lambda *args, **kwargs: ControlSession.wait(client, *args, **kwargs)

                def auto(args, arm, session, sample):
                    attempts.append(1)
                    if stop_at == 'retry' and len(attempts) == 1:
                        return False
                    phase[0] = 'hold'
                    arm.hold(settle_s=0)
                    arm.move([0.] * 6, .17)
                    phase[0] = 'burst'
                    sample('fit', 0)
                    phase[0] = 'done'
                    return True

                def key(delay):
                    displayed.append(phase[0])
                    if phase[0] == 'prepare':
                        return ord('c')
                    return ord('q') if phase[0] == stop_at else -1

                write_image = cv2.imwrite
                def write(path, image):
                    phase[0] = 'write'
                    return write_image(path, image)

                output = Path(folder) / 'session'
                with patch('embodied.applications.calibration.cli.config', return_value=settings), \
                        patch('embodied.devices.sensors.client.SharedCamera', return_value=camera), \
                        patch('embodied.applications.calibration.arm.ControlSession', return_value=client), \
                        patch('embodied.applications.calibration.cli.MarkerDetector', return_value=Detector()), \
                        patch('embodied.applications.calibration.cli.run_auto', side_effect=auto), \
                        patch('embodied.applications.calibration.cli.solve_session') as solve_session, \
                        patch('embodied.applications.calibration.cli.finish'), \
                        patch('cv2.imshow'), patch('cv2.destroyAllWindows'), \
                        patch('cv2.waitKey', side_effect=key), \
                        patch('cv2.getWindowProperty', side_effect=lambda *args: 0 if stop_at == 'close' and phase[0] == 'move' else 1), \
                        patch('cv2.putText', wraps=cv2.putText) as put_text, \
                        patch('cv2.imwrite', side_effect=write):
                    argv = ['capture', 'wrist_rgb', '--auto', '--size-m', '.1', '--run', folder, '--output', str(output)]
                    if stop_at in (None, 'retry'):
                        main(argv)
                    else:
                        with self.assertRaises(KeyboardInterrupt):
                            main(argv)
                    texts = [call.args[1] for call in put_text.call_args_list]
                data = json.loads((output / 'session.json').read_text())
                client.close.assert_called_once()
                self.assertEqual(client.command.call_args.args, ('hold',))
                self.assertEqual(data['status'], 'complete' if stop_at in (None, 'retry') else 'interrupted')
                if stop_at in (None, 'retry'):
                    self.assertTrue({'hold', 'preview', 'move', 'burst', 'write'} <= set(displayed))
                    self.assertTrue(any(text.startswith('1 samples') for text in texts))
                    self.assertEqual(len(data['samples'][0]['frames']), 20)
                    raw = cv2.imread(str(output / data['samples'][0]['frames'][0]['image']))
                    self.assertFalse(np.any(raw))  # Overlay must not enter calibration images.
                    solve_session.assert_called_once()
                    self.assertEqual(len(attempts), 2 if stop_at == 'retry' else 1)
                else:
                    solve_session.assert_not_called()
                    if stop_at in ('hold', 'preview'):
                        client.execute.assert_not_called()

    def test_summary_reports_both_insufficient_validation_and_maximum_error(self):
        from embodied.applications.calibration.cli import finish
        result = {'status': 'candidate', 'method': 'relative_svd',
                  'fit': {'count': 13, 'translation_rms_m': .00346, 'rotation_rms_deg': .55},
                  'leave_one_out': {'passed': True},
                  'validation': {'count': 3, 'translation_rms_m': .0059, 'rotation_rms_deg': 1.19,
                                 'translation_max_m': .008216, 'rotation_max_deg': 1.47,
                                 'failure': 'Need at least 4 distinct poses'},
                  'quality_limits': {'translation_m': .008, 'rotation_deg': 2.}}
        with patch('builtins.print') as output, patch('embodied.applications.calibration.save') as save:
            finish('/session', result)
        message = '\n'.join(call.args[0] for call in output.call_args_list)
        self.assertIn('Need at least 4 distinct poses', message)
        self.assertIn('8.22 mm', message)
        self.assertIn('per-pose limits 8 mm / 2°', message)
        self.assertIn('Raw samples and candidate result are saved', message)
        save.assert_not_called()

    def test_marker_and_status_need_no_arm_or_camera_drivers(self):
        from embodied.applications.calibration.cli import main
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.startswith(("arx5_interface", "pyrealsense2", "xrobotoolkit", "embodied.devices.sensors.client")):
                raise AssertionError(f"Offline command imported {name}")
            return original(name, *args, **kwargs)
        config = {"cameras": {"wrist_rgb": {"serial": "1"}, "exterior_rgb": {"serial": "2"}}, "hand_eye": {"marker_size_m": .08}}
        with tempfile.TemporaryDirectory() as folder, patch("builtins.__import__", side_effect=guarded), \
                patch("embodied.applications.calibration.cli.config", return_value=config), \
                patch("embodied.applications.calibration.RESULTS", Path(folder)):
            main(["marker", "--output", str(Path(folder) / "marker.svg")])
            self.assertEqual(json.loads((Path(folder) / "marker.json").read_text())["black_square_size_m"], .08)
            main(["status"])

    def test_solve_saves_a_validated_session(self):
        from embodied.applications.calibration.cli import main
        rows, x, _ = synthetic("fixed")
        with tempfile.TemporaryDirectory() as folder:
            session = Path(folder) / "session"
            session.mkdir()
            (session / "session.json").write_text(json.dumps({
                "kind": "hand_eye_samples", "camera_id": "exterior_rgb", "mount": "fixed", "samples": rows,
                "robot": {}, "camera": {"serial": "2"}, "target": target_spec(.1)}))
            with patch("embodied.applications.calibration.RESULTS", Path(folder) / "results"):
                main(["solve", str(session), "--save"])
                saved = calibration.load("exterior_rgb", "2", root=Path(folder) / "results")
            np.testing.assert_allclose(saved["T_base_camera"], x, atol=1e-8)
            self.assertTrue((session / "result.json").is_file())


if __name__ == "__main__":
    unittest.main()
