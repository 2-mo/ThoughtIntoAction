"""Fake-controller checks; no robot, Quest SDK or live service is opened."""
from types import SimpleNamespace
from contextlib import closing
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
import importlib.util
from unittest.mock import patch

import numpy as np

from embodied.contracts.control import ControlCommand, Mode, RobotState
from embodied.devices.robots.realman.hardware import Backend, JointModel
from embodied.devices.robots.realman.hardware_profile import Profile
from embodied.devices.robots.realman.kinematics import CandidateRejected


class Vendor:
    lower, upper = np.full(6, -360.), np.full(6, 360.)
    reject = False
    singular = False
    def fk(self, degrees):
        # Test geometry in which all six joint coordinates affect the TCP.
        from scipy.spatial.transform import Rotation
        q = np.deg2rad(degrees)
        return np.r_[q[:3]*.1, Rotation.from_rotvec(q[3:]).as_quat()]
    def solve(self, target, seed):
        from scipy.spatial.transform import Rotation
        q = np.r_[np.asarray(target)[:3]*10, Rotation.from_quat(target[3:]).as_rotvec()]
        return int(self.reject), np.rad2deg(q)
    def singularity(self, q):
        return int(self.singular), .1


class Client:
    def __init__(self):
        self.sent = []
        self.gripper = dict(enable=1, status=1, error=0, mode=1, current_force=0,
                            temperature=25, actpos=1002)
    def query(self, key):
        return dict(self.gripper) if key == 'get_gripper_state' else {}
    def send(self, message): self.sent.append(message)
    def exchange(self, message, key):
        self.sent.append(message)
        return {'state': True}


class Connection:
    def __init__(self, vendor):
        self.client, self.vendor = Client(), vendor
        self.config = {'joint_speed_deg_s': [90.]*2+[112.5]*4,
                       'linear_speed_m_s': .25, 'angular_speed_rad_s': .3}
        self.q = np.zeros(6)
        self.info, self.fault = {}, ''
        self.active, self.closed = False, False
        self.lock = threading.RLock()
    def read_state(self, now):
        return RobotState(self.q.copy(), np.full(6, np.nan), self.vendor.fk(np.rad2deg(self.q)),
                          float('nan'), time.monotonic(), fault=self.fault)
    def stop(self):
        if self.active: self.client.send({'command': 'set_arm_stop'})
        self.active = False
    def close(self): self.closed = True


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'run with outputs/envs/control/bin/python')
class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.vendor = Vendor()
        self.connection = Connection(self.vendor)
        self.config = Profile('fake', '', '')
        self.backend = Backend(self.config, connection=self.connection, vendor=self.vendor)
    def tearDown(self): self.backend.close()
    def step(self, **fields):
        self.backend.read_state(time.monotonic())
        self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING, **fields), time.monotonic())
    def motions(self): return [x for x in self.connection.client.sent if x['command'] == 'movej_canfd']

    def test_startup_is_read_only_and_1002_is_saturated_nominal_width(self):
        self.assertEqual(self.connection.client.sent, [])
        self.assertAlmostEqual(self.backend.state.gripper_width, .07)
        self.assertTrue(np.isfinite(self.backend.state.dq).all())
        self.connection.client.gripper['actpos'] = 1006
        self.backend.gripper = self.connection.client.gripper
        with self.assertRaisesRegex(ValueError, 'tolerance'): self.backend.width()

    def test_all_six_axes_use_joint_wire_and_speed_budgets(self):
        self.step(joint_target=np.full(6, .2))
        wire = self.motions()[-1]
        self.assertEqual(len(wire['joint']), 6)
        self.assertTrue(all(v > 0 for v in wire['joint']))
        q = np.deg2rad(np.asarray(wire['joint'])/1000)
        self.assertTrue(np.all(q <= self.backend.velocity_limits/25+2e-5))
        self.assertLessEqual(np.linalg.norm(self.vendor.fk(np.rad2deg(q))[:3]), .25/25+2e-6)
        self.assertFalse(any(x['command'] == 'movep_canfd' for x in self.connection.client.sent))

    def test_hold_discards_cached_goal_and_disabled_does_not_home_or_open(self):
        self.step(joint_target=np.full(6, .1))
        self.step(hold=True)
        self.step()
        self.assertEqual(self.motions()[-1]['joint'], [0]*6)
        before = len(self.connection.client.sent)
        self.backend.step(ControlCommand(Mode.DISABLED), time.monotonic())
        messages = self.connection.client.sent[before:]
        self.assertFalse(any(x['command'].startswith(('move', 'set_gripper')) for x in messages))
        np.testing.assert_array_equal(self.backend.controller.q_target, self.connection.q)

    def test_unreachable_and_singular_targets_never_send_motion(self):
        self.vendor.reject = True
        self.step(pose_target=np.array([.01, 0, 0, 0, 0, 0, 1.]))
        self.assertFalse(self.motions())
        self.vendor.reject = False
        self.vendor.singular = True
        self.step(joint_target=np.full(6, .1))
        self.assertFalse(self.motions())
        self.vendor.singular = False
        self.step(joint_target=np.full(6, .01))
        self.assertTrue(self.motions())

    def test_bad_ik_residual_or_limits_rejected_before_controller_clip(self):
        model = JointModel(self.vendor)
        self.vendor.solve = lambda pose, seed: (0, np.full(6, 400.))
        with self.assertRaises(CandidateRejected): model.solve_ik(np.array([0,0,0,0,0,0,1.]), np.zeros(6))
        self.vendor.solve = lambda pose, seed: (0, np.zeros(6))
        with self.assertRaises(CandidateRejected): model.solve_ik(np.array([.1,0,0,0,0,0,1.]), np.zeros(6))

    def test_mode_acknowledgements_hold_without_ARX_home_or_damping(self):
        for mode in (Mode.FREE_DRIVE, Mode.DISABLED, Mode.COMMAND_FOLLOWING):
            self.backend.step(ControlCommand(mode, hold=True), time.monotonic())
            state = self.backend.read_state(time.monotonic())
            self.assertEqual(state.mode, mode)
            self.assertEqual(state.gripper_mode, mode)
        self.assertEqual(self.connection.client.sent, [])

    def test_cancellation_during_ik_prevents_submission(self):
        cancelled = False
        original = self.vendor.solve
        def solve(*args):
            nonlocal cancelled
            cancelled = True
            return original(*args)
        self.vendor.solve = solve
        with self.assertRaises(InterruptedError):
            self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING,
                pose_target=np.array([.001,0,0,0,0,0,1.])), time.monotonic(),
                cancel_requested=lambda: cancelled)
        self.assertFalse(self.motions())

    def test_cancel_fault_and_out_of_range_refuse_motion(self):
        with self.assertRaises(ValueError): self.step(joint_target=np.full(6, 100.))
        self.connection.fault = '4099'
        with self.assertRaisesRegex(RuntimeError, '4099'): self.step(joint_target=np.full(6, .01))
        self.connection.fault = ''
        with self.assertRaises(InterruptedError):
            self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING), time.monotonic(), cancel_requested=lambda: True)
        self.assertFalse(self.motions())

    def test_gripper_segments_contact_and_timeout(self):
        b = self.backend
        b._gripper_target(0., 1.)
        self.assertEqual(self.connection.client.sent[-1]['position'], 902)
        count = len(self.connection.client.sent)
        b._gripper_target(0., 2.)
        self.assertEqual(len(self.connection.client.sent), count)
        b.gripper.update(mode=6, actpos=950)
        b._gripper_target(0., 3.)
        self.assertEqual(len(self.connection.client.sent), count)
        b.gripper['mode'] = 1
        b._gripper_target(0., 4.)
        self.assertEqual(len(self.connection.client.sent), count)
        b._gripper_target(.07, 5.)
        self.assertEqual(self.connection.client.sent[-1]['position'], 1000)
        b.gripper['mode'] = 4
        with self.assertRaisesRegex(RuntimeError, '5 s'): b._gripper_target(.07, 11.)

    def test_partial_initialization_closes_connection(self):
        c = Connection(self.vendor)
        del c.client.gripper['actpos']
        with self.assertRaisesRegex(ValueError, 'Incomplete'): Backend(self.config, connection=c, vendor=self.vendor)
        self.assertTrue(c.closed)

    def offline(self):
        self.connection.client.gripper = dict.fromkeys(self.connection.client.gripper, 0)
        self.backend.last_gripper_poll = float('-inf')
        return self.backend.read_state(time.monotonic())

    def test_startup_offline_keeps_six_axis_arm_available_without_auto_open(self):
        c = Connection(self.vendor)
        c.client.gripper = dict.fromkeys(c.client.gripper, 0)
        with closing(Backend(self.config, connection=c, vendor=self.vendor)) as b:
            self.assertIsNone(b.state.gripper_width)
            self.assertEqual(c.client.sent, [])
            b.step(ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=np.full(6, .01),
                gripper_target=0.), time.monotonic())
            self.assertEqual(len(c.client.sent), 1)
            self.assertEqual(len(c.client.sent[0]['joint']), 6)
            self.assertIsNone(b.telemetry()['gripper_target'])

    def test_gripper_accepts_native_float32_endpoint_but_rejects_real_overflow(self):
        self.connection.client.gripper['actpos'] = 950
        self.backend.gripper['actpos'] = 950
        maximum = np.float32(self.config.gripper_max_width)
        self.step(hold=True, gripper_target=float(maximum))
        commands = [m for m in self.connection.client.sent if m['command'] == 'set_gripper_position']
        self.assertEqual(commands, [{'command': 'set_gripper_position', 'position': 1000, 'block': False}])
        for invalid in (float(np.nextafter(maximum, np.float32(np.inf))), -.001, float('nan')):
            with self.subTest(target=invalid):
                with self.assertRaisesRegex(ValueError, 'Invalid EG2 nominal width'):
                    self.backend._gripper_target(invalid, time.monotonic())

    def test_loss_and_readonly_recovery_drop_old_target_until_release(self):
        self.backend._gripper_target(0., time.monotonic())
        self.assertIsNotNone(self.backend.pending_gripper)
        self.offline()
        self.assertIsNone(self.backend.pending_gripper)
        self.step(joint_target=np.full(6, .01), gripper_target=0.)
        before = len(self.connection.client.sent)
        self.connection.client.gripper.update(enable=1, status=1, mode=1, actpos=500)
        for i in range(2):
            self.backend.last_gripper_poll = float('-inf')
            state = self.backend.read_state(time.monotonic())
            self.assertEqual(state.gripper_width is not None, i == 1)
        self.backend._gripper_target(0., time.monotonic())
        self.assertEqual(len(self.connection.client.sent), before)
        self.backend._gripper_target(None, time.monotonic())
        self.backend._gripper_target(0., time.monotonic())
        self.assertEqual(self.connection.client.sent[-1]['position'], 400)

    def test_initialization_requires_explicit_idle_request_and_sends_only_once(self):
        self.offline()
        for fields in ({'hold': False}, {'hold': True, 'engaged': True}):
            with self.assertRaisesRegex(ValueError, 'explicit arm hold'):
                self.step(gripper_initialize_open=True, **fields)
        self.assertEqual(self.connection.client.sent, [])
        self.step(hold=True, gripper_initialize_open=True)
        self.step(hold=True, gripper_initialize_open=True)
        self.assertEqual(self.connection.client.sent, [dict(command='set_gripper_release', speed=500, block=False)])
        self.assertIsNone(self.backend.width())
        self.connection.client.gripper.update(enable=1, status=1, mode=1, actpos=1000)
        for _ in range(2):
            self.backend.last_gripper_poll = float('-inf')
            self.step(hold=True, gripper_initialize_open=True)
        self.assertTrue(self.backend.gripper_ready)
        self.assertEqual(len(self.connection.client.sent), 1)

    def test_nonzero_gripper_fault_is_isolated_but_not_cleared_or_initialized(self):
        self.connection.client.gripper['error'] = 1
        self.backend.last_gripper_poll = float('-inf')
        self.step(hold=True, gripper_initialize_open=True)
        self.assertEqual(self.connection.client.sent, [])
        self.assertIsNone(self.backend.state.gripper_width)
        self.step(joint_target=np.full(6, .01))
        self.assertTrue(self.motions())

    def test_transport_failure_is_not_masked_as_offline(self):
        self.backend.last_gripper_poll = float('-inf')
        with patch.object(self.connection.client, 'query', side_effect=TimeoutError('no reply')):
            with self.assertRaises(TimeoutError): self.backend.read_state(time.monotonic())

    def test_common_worker_records_standard_feedback_and_stops_without_damping(self):
        from embodied.control import worker
        from embodied.contracts.paths import PROJECT_ROOT
        config = PROJECT_ROOT/'configs/robots/realman/left.json'
        connection = Connection(self.vendor)
        clock = SimpleNamespace(wait=lambda: (_ for _ in ()).throw(InterruptedError('test-finished')))
        connection.client.gripper = dict.fromkeys(connection.client.gripper, 0)
        with tempfile.TemporaryDirectory() as folder, \
                patch('embodied.devices.robots.realman.hardware.Connection', return_value=connection), \
                patch('embodied.devices.robots.realman.hardware.OfflineModel', return_value=self.vendor), \
                patch.object(worker, 'PeriodicClock', return_value=clock):
            with self.assertRaisesRegex(RuntimeError, 'test-finished'):
                worker.run('real', 'robot-fake-realman-worker', str(config), Path(folder), telemetry=True)
            report = json.loads((Path(folder)/'report.json').read_text())
            self.assertEqual(report['shutdown_policy'], 'controller-stop')
            self.assertFalse(report['damping_requested'])
            self.assertTrue(report['tuning_recording']['complete'])
            self.assertEqual(report['tick_telemetry']['dropped'], 0)
            self.assertIsNone(report['gripper_min_m'])
        self.assertTrue(connection.closed)
        self.assertEqual(connection.client.sent, [])


if __name__ == '__main__': unittest.main()
