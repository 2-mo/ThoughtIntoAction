"""Offline contract checks for managed authority; never imports the hardware SDK."""
from pathlib import Path
from dataclasses import replace
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]
from embodied.control.commands import CommandEngine, MotionPlanner, SettlingWindow
from embodied.contracts.control import RobotState, ControlCommand, Mode
from embodied.control.settings import load_settings
from embodied.control.motion_limits import MotionLimits
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS


def reported_capabilities(velocity):
    return {'joint_velocity_limits_rad_s': velocity,
            'source': {'kind': 'sdk_runtime_configuration'}}


class MotionLimitsTests(unittest.TestCase):
    def test_backend_capabilities_resolve_each_axis_and_configured_period(self):
        from embodied.control.motion_limits import MotionLimits
        caps = np.array([.12, .24, .36, .48, .6, .72])
        for hz in (100, 500):
            config = replace(load_settings().robot, control_hz=hz, joint_velocity_max=(9.,) * 6)
            limits = MotionLimits.load(config, backend='real',
                capabilities=reported_capabilities(caps.tolist()))
            self.assertEqual(limits.describe()['source']['kind'], 'sdk_runtime_configuration')
            self.assertEqual(limits.describe()['control_hz'], hz)
            for speed in (0., .001, .3, 20.):
                with self.subTest(hz=hz, speed=speed):
                    resolved = limits.resolve({'speed': speed, 'acceleration': 20., 'jerk': 100.})
                    expected = caps if speed == 0 else np.minimum(caps, speed)
                    np.testing.assert_allclose(resolved['joint_velocity_rad_s'], expected)
                    np.testing.assert_allclose(limits.step_limits(resolved), expected / hz)
            for backend in ('sim', 'offline'):
                nominal = MotionLimits.load(config, backend=backend)
                np.testing.assert_array_equal(nominal.resolve({'speed': 0})['joint_velocity_rad_s'], [9.] * 6)
                self.assertNotEqual(nominal.describe()['source']['kind'], 'sdk_runtime_configuration')

    def test_real_capability_never_falls_back_to_nominal_or_invalid_values(self):
        from embodied.control.motion_limits import MotionLimits
        for values in (None, [1.] * 5, [1.] * 7, [0.] * 6, [-1.] * 6,
                       [float('nan')] * 6, [float('inf')] * 6):
            capabilities = None if values is None else reported_capabilities(values)
            with self.subTest(values=values), self.assertRaises(ValueError):
                MotionLimits.load(load_settings().robot, backend='real', capabilities=capabilities)

    def test_gateway_engine_and_planner_share_the_backend_capability(self):
        import tempfile
        from embodied.control.gateway import Gateway
        from embodied.control.motion_limits import MotionLimits
        config = replace(load_settings().robot, control_hz=100)
        limits = MotionLimits.load(config, backend='real',
            capabilities=reported_capabilities([.12, .24, .36, .48, .6, .72]))
        with tempfile.TemporaryDirectory() as folder, patch('embodied.control.gateway.socket.socket'), \
                patch('embodied.control.gateway.threading.Thread'), patch.object(Path, 'chmod'):
            gateway = Gateway(folder, 'offline-test', config, motion_limits=limits)
            self.assertIs(gateway.engine.motion_limits, limits)
            self.assertIs(gateway.planner.motion_limits, limits)
            self.assertEqual(gateway.describe()['motion_limits'], limits.describe())
            gateway.close()

    def test_gateway_accepts_legacy_factory_without_masking_constructor_errors(self):
        import tempfile
        from embodied.control.gateway import Gateway
        config = load_settings().robot
        calls = []

        def legacy_factory(robot, model_id):
            calls.append((robot, model_id))
            return SimpleNamespace()

        def broken_factory(robot, model_id, *, motion_limits):
            calls.append(motion_limits)
            raise TypeError('failure inside planner constructor')

        with tempfile.TemporaryDirectory() as folder, patch('embodied.control.gateway.socket.socket'), \
                patch('embodied.control.gateway.threading.Thread'), patch.object(Path, 'chmod'):
            gateway = Gateway(folder, 'offline-test', config, planner_factory=legacy_factory)
            self.assertEqual(len(calls), 1)
            self.assertIs(calls[0][0], config)
            gateway.close()
            with self.assertRaisesRegex(TypeError, 'failure inside planner constructor'):
                Gateway(folder, 'offline-test', config, planner_factory=broken_factory)
            self.assertEqual(len(calls), 2)


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.config = load_settings().robot
        self.planner = MotionPlanner(self.config, 'model')
        self.q = np.array([0., .6, 1., 0., .2, 0.])
        self.snapshot = {'q': self.q.tolist(), 'gripper_width': .04, 'epoch': 0, 'owner': None}

    def test_default_limits_match_explicit_ruckig_profile(self):
        request = {'kind': 'movej', 'q': (self.q + [np.deg2rad(10), 0, 0, 0, 0, 0]).tolist()}
        implicit = self.planner.plan(request, self.snapshot, 10.)
        defaults = MotionLimits.load(self.config).describe()['defaults']
        explicit = self.planner.plan({**request, **defaults}, self.snapshot, 10.)
        self.assertEqual(implicit['trajectory']['time_law'], 'ruckig')
        self.assertEqual({k: implicit['limits'][k] for k in ('speed', 'acceleration', 'jerk')},
                         defaults)
        self.assertEqual(implicit['plan_id'], explicit['plan_id'])
        np.testing.assert_array_equal(implicit['q'], explicit['q'])
        np.testing.assert_array_equal(implicit['times'], explicit['times'])

    def test_default_presets_reach_sdk_velocity_without_position_overshoot(self):
        """Catch the slow a=2/j=10 profile returning to the P/H controls."""
        home = np.array(self.config.park_q)
        ready = np.array(DEFAULT_GRASP_READY_JOINTS)
        limits = np.array(self.config.joint_velocity_max)
        for start, target in ((home, ready), (ready, home)):
            snapshot = {**self.snapshot, 'q': start.tolist()}
            plan = self.planner.plan({'kind': 'movej', 'q': target.tolist()}, snapshot, 10.)
            duration = plan['times'][-1]
            self.assertLess(duration, .7)
            samples = [plan['_trajectory'].sample(t) for t in np.linspace(0, duration, 1001)]
            q, dq = np.array([s[0] for s in samples]), np.array([s[1] for s in samples])
            self.assertTrue(np.all(q >= np.minimum(start, target) - 1e-9))
            self.assertTrue(np.all(q <= np.maximum(start, target) + 1e-9))
            self.assertTrue(np.all(np.abs(dq) <= limits + 1e-8))
            self.assertGreater(np.max(np.abs(dq) / limits), .99)
            np.testing.assert_allclose(q[-1], target, atol=1e-9)
            np.testing.assert_allclose(dq[[0, -1]], 0., atol=1e-9)

    def test_short_move_acceleration_and_jerk_tuning_does_not_promise_requested_speed(self):
        request = {'kind': 'movej', 'q': (self.q + [np.deg2rad(10), 0, 0, 0, 0, 0]).tolist(), 'speed': .5}
        baseline = self.planner.plan({**request, 'acceleration': .5, 'jerk': 2.}, self.snapshot, 10.)
        tuned = self.planner.plan({**request, 'acceleration': 2., 'jerk': 10.}, self.snapshot, 10.)
        self.assertLess(tuned['times'][-1], baseline['times'][-1])
        self.assertEqual({k: tuned['limits'][k] for k in ('speed', 'acceleration', 'jerk')},
                         {'speed': .5, 'acceleration': 2., 'jerk': 10.})
        metrics = tuned['metrics']
        self.assertEqual(metrics['duration_s'], tuned['times'][-1])
        self.assertGreater(metrics['peak_velocity_rad_s'], baseline['metrics']['peak_velocity_rad_s'])
        self.assertLess(metrics['peak_velocity_rad_s'], .5)
        self.assertLessEqual(metrics['peak_acceleration_rad_s2'], 2. * 1.001)
        self.assertLessEqual(metrics['peak_jerk_rad_s3'], 10. * 1.001)
        speeds = np.array([tuned['_trajectory'].sample(t)[1] for t in np.linspace(0, tuned['times'][-1], 1001)])
        self.assertAlmostEqual(np.max(np.abs(speeds)), metrics['peak_velocity_rad_s'], delta=2e-5)

    def test_motion_limits_reject_invalid_values_before_planning(self):
        request = {'kind': 'movej', 'q': (self.q + [.01, 0, 0, 0, 0, 0]).tolist()}
        for field in ('speed', 'acceleration', 'jerk'):
            invalid = [-1, float('nan'), float('inf'), -float('inf'), True, 'invalid', None]
            if field != 'speed':
                invalid.append(0)
            for value in invalid:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    self.planner.plan({**request, field: value}, self.snapshot, 10.)

    def test_movel_trajectory_respects_requested_derivatives(self):
        request = {'kind': 'movel', 'pose': self.planner.model.fk(self.q + [.05, 0, 0, 0, 0, 0]).tolist(),
                   'speed': .3, 'acceleration': .08, 'jerk': .3}
        plan = self.planner.plan(request, self.snapshot, 10.)
        times = np.linspace(0, plan['times'][-1], 2001)
        velocity = np.array([plan['_trajectory'].sample(t)[1] for t in times])
        acceleration = np.gradient(velocity, times, axis=0)
        jerk = np.gradient(acceleration, times, axis=0)
        for name, measured, metric in (
                ('speed', velocity, 'peak_velocity_rad_s'),
                ('acceleration', acceleration, 'peak_acceleration_rad_s2'),
                ('jerk', jerk, 'peak_jerk_rad_s3')):
            with self.subTest(limit=name):
                self.assertLessEqual(np.max(np.abs(measured)), request[name] * 1.01)
                self.assertLessEqual(plan['metrics'][metric], request[name] * 1.01)
        self.assertEqual({k: plan['limits'][k] for k in ('speed', 'acceleration', 'jerk')},
                         {key: request[key] for key in ('speed', 'acceleration', 'jerk')})

    def test_movej_uses_the_shared_ruckig_evaluator_between_sample_points(self):
        from embodied.motion.offline import plan_ruckig_joint_trajectory
        made = []
        def plan_shared(*args, **kwargs):
            result = plan_ruckig_joint_trajectory(*args, **kwargs)
            made.append(result)
            return result
        with patch('embodied.motion.offline.plan_ruckig_joint_trajectory', side_effect=plan_shared):
            plan = self.planner.plan({'q': (self.q + [.1, .02, -.02, .04, 0, 0]).tolist()}, self.snapshot, 10.)
        self.assertEqual(len(made), 1)
        self.assertEqual(plan['times'][-1], made[0].duration_s)
        for fraction in (.137329, .367119, .709371, .923137):
            elapsed = plan['times'][-1] * fraction
            expected_q, expected_dq, _ = made[0].at_time(elapsed)
            q, dq = plan['_trajectory'].sample(elapsed)
            np.testing.assert_allclose(q, expected_q, atol=1e-12)
            np.testing.assert_allclose(dq, expected_dq, atol=1e-12)

    def test_per_axis_sdk_caps_bound_plans_at_configured_control_rate(self):
        from embodied.control.motion_limits import MotionLimits
        caps = np.array([.12, .18, .24, .3, .36, .42])
        config = replace(self.config, control_hz=100)
        limits = MotionLimits.load(config, backend='real',
            capabilities=reported_capabilities(caps.tolist()))
        planner = MotionPlanner(config, 'model', motion_limits=limits)
        for speed in (0., .2, 20.):
            with self.subTest(speed=speed):
                plan = planner.plan({'q': (self.q + .08).tolist(), 'speed': speed}, self.snapshot, 10.)
                expected = caps if speed == 0 else np.minimum(caps, speed)
                np.testing.assert_allclose(plan['limits']['joint_velocity_rad_s'], expected)
                sample = [plan['_trajectory'].sample(t)[1] for t in np.linspace(0, plan['times'][-1], 1001)]
                self.assertTrue(np.all(np.max(np.abs(sample), axis=0) <= expected + 1e-7))
                self.assertLessEqual(len(plan['times']), int(np.ceil(plan['times'][-1] * config.control_hz)) + 2)

    def test_joint_and_linear_position_velocity_share_one_curve(self):
        requests = [
            {'kind': kind, 'q': (self.q + [delta,0,0,0,0,0]).tolist(), 'speed': .03}
            for kind, delta in (('movej', np.deg2rad(3)), ('jog', -np.deg2rad(3)))]
        requests += [{'kind': 'home', 'speed': .15}, {'kind': 'movel', 'speed': .03,
                      'pose': self.planner.model.fk(self.q + [.002,0,0,0,0,0]).tolist()}]
        for request in requests:
            with self.subTest(kind=request['kind']):
                plan = self.planner.plan(request, self.snapshot, 10.)
                curve, duration = plan['_trajectory'], plan['times'][-1]
                for elapsed, endpoint in ((-.1, plan['q'][0]), (0, plan['q'][0]),
                                          (duration, plan['q'][-1]), (duration+.1, plan['q'][-1])):
                    q, dq = curve.sample(elapsed)
                    np.testing.assert_allclose(q, endpoint, atol=1e-12)
                    np.testing.assert_array_equal(dq, np.zeros(6))
                for fraction in (.1131, .3973, .7031, .9255):
                    elapsed, epsilon = duration * fraction, duration * 1e-5
                    q, dq = curve.sample(elapsed)
                    difference = (curve.sample(elapsed+epsilon)[0] - curve.sample(elapsed-epsilon)[0]) / (2*epsilon)
                    np.testing.assert_allclose(dq, difference, atol=1e-9, rtol=1e-5)
                for index in np.linspace(0, len(plan['times'])-1, 23, dtype=int):
                    np.testing.assert_allclose(curve.sample(plan['times'][index])[0], plan['q'][index], atol=1e-12)
                speeds = np.array([curve.sample(t)[1] for t in np.linspace(0, duration, 1001)])
                self.assertLessEqual(float(np.max(np.abs(speeds))), request['speed']*1.01)

    def test_gateway_serializes_compact_curve_and_keeps_worker_cache_private(self):
        import hashlib
        import json
        import threading
        from collections import OrderedDict
        from unittest.mock import MagicMock
        from embodied.control.gateway import Gateway
        from embodied.contracts.rpc import MAX_MESSAGE
        gateway = Gateway.__new__(Gateway)
        gateway.engine = SimpleNamespace(runtime_id='test')
        gateway.planner, gateway.snapshot = self.planner, self.snapshot
        gateway.plan_lock, gateway.plans = threading.Lock(), OrderedDict()
        gateway.slots = SimpleNamespace(release=lambda: None)
        # A long, capped preview must not double the
        # RPC payload by duplicating a full dense velocity array.
        goal = self.q + [.24, .24, .24, .24, .24, .24]
        request = {'version': 1, 'runtime_id': 'test', 'action': 'preview',
                   'kind': 'movej', 'q': goal.tolist(), 'speed': .01}
        with patch('embodied.control.gateway.receive', return_value=request), patch('embodied.control.gateway.send') as send:
            gateway._peer(MagicMock())
        reply = send.call_args.args[1]
        self.assertTrue(reply['ok'], reply)
        public = reply['result']
        self.assertNotIn('_trajectory', public)
        self.assertIn('_trajectory', gateway.plans[public['plan_id']])
        self.assertLessEqual(len(public['q']), 4001)
        self.assertLess(len(json.dumps(reply, allow_nan=False, separators=(',', ':')).encode()), MAX_MESSAGE)
        identity = {key: public[key] for key in ('kind', 'epoch', 'owner_present', 'model_id',
                                                'q', 'times', 'gripper_width', 'trajectory', 'limits', 'metrics')}
        digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
        self.assertEqual(digest, public['plan_id'])
        for field in ('limits', 'metrics'):
            changed = json.loads(json.dumps(identity))
            changed[field] = {}
            self.assertNotEqual(hashlib.sha256(json.dumps(changed, sort_keys=True).encode()).hexdigest(), digest)
        identity['trajectory']['goal'][0] += .001
        self.assertNotEqual(hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest(), digest)


class SettlingTests(unittest.TestCase):
    def test_stationary_encoder_noise_requires_a_complete_window(self):
        window = SettlingWindow(MotionLimits.load().settling)
        for index in range(64):
            q = np.full(6, .00038147 * (index % 2))
            dq = np.full(6, (.03297 if index % 4 == 0 else -.01099))
            settled = window.update(index * .004, q, dq, np.zeros(6))
            self.assertEqual(settled, index >= 63)

    def test_persistent_speed_and_reversals_are_not_averaged_away(self):
        for alternating in (False, True):
            with self.subTest(alternating=alternating):
                window = SettlingWindow(MotionLimits.load().settling)
                for index in range(100):
                    speed = .03 * (-1 if alternating and index % 2 else 1)
                    self.assertFalse(window.update(index * .004, np.zeros(6), np.full(6, speed), np.zeros(6)))

    def test_position_motion_is_checked_independently_of_reported_velocity(self):
        window = SettlingWindow(MotionLimits.load().settling)
        for index in range(100):
            q = np.full(6, .004 if index % 2 else -.004)
            self.assertFalse(window.update(index * .004, q, np.zeros(6), np.zeros(6)))

    def test_samples_before_window_cannot_dilute_recent_velocity(self):
        window = SettlingWindow(MotionLimits.load().settling)
        for stamp, speed in ((0, 0), (.08, .0225), (.16, .0225), (.24, .0225), (.32, .0225)):
            self.assertFalse(window.update(stamp, np.zeros(6), np.full(6, speed), np.zeros(6)))

    def test_outside_tolerance_invalid_data_and_sampling_gaps_reset_window(self):
        cases = [('position', .26, .015, 0), ('invalid', .26, 0, float('nan')),
                 ('gap', .5, 0, 0), ('clock', .1, 0, 0)]
        for name, stamp, position, speed in cases:
            with self.subTest(case=name):
                window = SettlingWindow(MotionLimits.load().settling)
                for index in range(64):
                    window.update(index * .004, np.zeros(6), np.zeros(6), np.zeros(6))
                self.assertFalse(window.update(stamp, np.full(6, position), np.full(6, speed), np.zeros(6)))
                for index in range(1, 60):
                    self.assertFalse(window.update(stamp + index * .004, np.zeros(6), np.zeros(6), np.zeros(6)))


class CommandsTests(unittest.TestCase):
    def setUp(self):
        self.config = load_settings().robot
        self.engine = CommandEngine('run', self.config, 'model')
        self.now, self.sequence = 10., 0
        self.state = RobotState(np.asarray(self.config.park_q), np.zeros(6), np.array([0,0,0,0,0,0,1.]), .04, self.now)
        self.lease = self.request('acquire', source='cli')
        self.engine.next(self.state, self.now)

    def request(self, action, **fields):
        self.sequence += 1
        lease = getattr(self, 'lease', {})
        return self.engine.request({'version': 1, 'runtime_id': 'run', 'action': action,
            'sent_monotonic_ns': round(self.now*1e9), 'sequence': self.sequence,
            'token': lease.get('token'), 'epoch': lease.get('epoch'), **fields}, self.state, self.now)

    def test_single_owner_sequence_and_epoch(self):
        with self.assertRaisesRegex(ValueError, 'already owned'):
            self.request('acquire', source='phone')
        self.request('heartbeat')
        with self.assertRaisesRegex(ValueError, 'sequence'):
            self.request('heartbeat', sequence=1)
        self.request('release')
        self.lease = self.request('acquire', source='phone')
        with self.assertRaisesRegex(ValueError, 'epoch'):
            self.request('joint', q=self.state.q.tolist(), epoch=self.lease['epoch']-2)
        command = self.engine.next(self.state, self.now)
        self.assertTrue(command.hold and command.reset_reference)
        self.assertEqual(command.mode, Mode.COMMAND_FOLLOWING)

    def test_target_age_not_refreshed_by_heartbeat_or_queue(self):
        self.request('joint', q=(self.state.q+.001).tolist(), ttl_ms=100)
        self.now += .08
        self.request('heartbeat')
        self.assertIsNotNone(self.engine.next(self.state, self.now).joint_target)
        self.now += .03
        self.request('heartbeat')
        self.assertTrue(self.engine.next(self.state, self.now).hold)
        with self.assertRaisesRegex(ValueError, 'expired'):
            self.request('joint', q=self.state.q.tolist(), sent_monotonic_ns=round((self.now-.11)*1e9))
        with self.assertRaisesRegex(ValueError, 'source sequence'):
            self.request('joint', q=self.state.q.tolist(), source_sequence=2**64)

    def test_free_drive_optionally_holds_the_gripper(self):
        self.request('free-drive')
        command = self.engine.next(self.state, self.now)
        self.assertEqual((command.mode, command.gripper_mode), (Mode.FREE_DRIVE, Mode.FREE_DRIVE))
        self.request('free-drive', gripper_width=.03)
        command = self.engine.next(self.state, self.now)
        self.assertEqual((command.mode, command.gripper_mode, command.gripper_target),
                         (Mode.FREE_DRIVE, Mode.COMMAND_FOLLOWING, .03))
        with self.assertRaisesRegex(ValueError, 'gripper width'):
            self.request('free-drive', gripper_width=self.config.gripper_max_width + .01)

    def test_lease_expiry_cancel_and_stop_hold_without_home(self):
        self.request('joint', q=self.state.q.tolist())
        self.now += .51
        command = self.engine.next(self.state, self.now)
        self.assertIsNone(self.engine.owner)
        self.assertTrue(command.hold and command.reset_reference)
        self.assertEqual(command.mode, Mode.COMMAND_FOLLOWING)
        self.request('stop')
        self.assertTrue(self.engine.stopped)
        with self.assertRaisesRegex(ValueError, 'latched'):
            self.request('acquire', source='cli')

    def test_execute_identity_start_state_idempotence_and_cancel(self):
        planner = MotionPlanner(self.config, 'model')
        snapshot = self.engine.status(self.state, self.now)
        plan = planner.plan({'q': (self.state.q + [.04,0,0,0,0,0]).tolist()}, snapshot, self.now)
        self.request('execute', _plan=plan)
        self.state.q = self.state.q + [.04,0,0,0,0,0]
        self.assertEqual(self.request('execute', _plan=plan)['status'], 'running')
        self.request('cancel')
        self.assertEqual(self.engine.operations[plan['plan_id']]['status'], 'cancelled')
        self.assertTrue(self.engine.next(self.state, self.now).reset_reference)
        bad = {**plan, 'model_id': 'other'}
        with self.assertRaisesRegex(ValueError, 'owner/model'):
            self.request('execute', _plan=bad)
        bad = {**plan, 'plan_id': 'new'}
        with self.assertRaisesRegex(ValueError, 'moved'):
            self.request('execute', _plan=bad)

    def test_operation_settles_with_stationary_noise_but_times_out_when_moving(self):
        for moving in (False, True):
            with self.subTest(moving=moving):
                self.setUp()
                planner = MotionPlanner(self.config, 'model')
                plan = planner.plan({'q': self.state.q.tolist()}, self.engine.status(self.state, self.now), self.now)
                self.request('execute', _plan=plan)
                end = self.now + plan['times'][-1]
                for index in range(1600):
                    self.now = end + index * .004
                    self.state.received_at = self.now
                    self.state.dq[:] = .03 if moving else (.03297 if index % 4 == 0 else -.01099)
                    self.request('heartbeat')
                    command = self.engine.next(self.state, self.now)
                    if self.engine.operation is None:
                        break
                operation = self.engine.operations[plan['plan_id']]
                self.assertEqual(operation['status'], 'failed' if moving else 'completed')
                if moving:
                    self.assertEqual(operation['reason'], 'Robot did not settle')
                    self.assertTrue(command.hold and command.reset_reference)
                else:
                    np.testing.assert_allclose(command.joint_target, plan['q'][-1])
                    np.testing.assert_array_equal(command.joint_velocity_target, np.zeros(6))
                    self.assertFalse(command.reset_reference)
                self.assertEqual(len(self.engine.settling.samples), 0)

    def complete_operation(self):
        planner = MotionPlanner(self.config, 'model')
        goal = self.state.q + [.02, 0, 0, 0, 0, 0]
        plan = planner.plan({'q': goal.tolist(), 'gripper_width': .03},
                            self.engine.status(self.state, self.now), self.now)
        self.request('execute', _plan=plan)
        end = self.now + plan['times'][-1]
        # Encoder feedback is within tolerance but differs from the command.
        self.state.q = goal - [.001, 0, 0, 0, 0, 0]
        self.state.gripper_width = .029
        for index in range(30):
            self.now = end + index * .01
            self.request('heartbeat')
            self.engine.next(self.state, self.now)
        self.assertEqual(self.engine.operations[plan['plan_id']]['status'], 'completed')
        return goal

    def test_completed_and_released_operation_keeps_final_position_with_zero_velocity(self):
        from embodied.devices.robots.arx5.kinematics import JointTargetController, X5Model
        goal = self.complete_operation()
        controller = JointTargetController(self.config, X5Model(self.config))
        controller.update(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True), self.state.q, self.now)
        for released in (False, True):
            if released:
                self.request('release')
                self.assertIsNone(self.engine.owner)
            self.now += .01
            command = self.engine.next(self.state, self.now)
            self.assertTrue(command.planned)
            self.assertFalse(command.hold or command.reset_reference)
            self.assertEqual(command.gripper_target, .03)
            np.testing.assert_array_equal(controller.update(command, self.state.q, self.now), goal)
            np.testing.assert_array_equal(controller.dq_target, np.zeros(6))
        self.lease = self.request('acquire', source='next')
        command = self.engine.next(self.state, self.now)
        self.assertTrue(command.hold and command.reset_reference)
        np.testing.assert_array_equal(controller.update(command, self.state.q, self.now), self.state.q)

    def test_abnormal_stop_clears_completed_reference_and_holds_current_feedback(self):
        from embodied.devices.robots.arx5.kinematics import JointTargetController, X5Model
        for action in ('hold', 'cancel', 'stop', 'expiry', 'fault'):
            with self.subTest(action=action):
                self.setUp()
                self.complete_operation()
                self.state.q += [.0015, 0, 0, 0, 0, 0]
                if action == 'expiry':
                    self.now += .51
                elif action == 'fault':
                    self.engine.clear('feedback-fault')
                else:
                    self.request(action)
                command = self.engine.next(self.state, self.now)
                self.assertTrue(command.hold and command.reset_reference)
                controller = JointTargetController(self.config, X5Model(self.config))
                np.testing.assert_array_equal(controller.update(command, self.state.q, self.now), self.state.q)
                np.testing.assert_array_equal(controller.dq_target, np.zeros(6))

    def test_operation_velocity_is_cleared_on_every_owner_stop_path(self):
        from embodied.devices.robots.arx5.kinematics import JointTargetController
        for action in ('hold', 'cancel', 'release', 'stop', 'expiry'):
            with self.subTest(action=action):
                self.setUp()
                planner = MotionPlanner(self.config, 'model')
                controller = JointTargetController(self.config, planner.model)
                controller.update(ControlCommand(Mode.COMMAND_FOLLOWING, hold=True), self.state.q, self.now)
                plan = planner.plan({'q': (self.state.q + [.02,0,0,0,0,0]).tolist()},
                                    self.engine.status(self.state, self.now), self.now)
                self.request('execute', _plan=plan)
                start = self.now
                for elapsed in np.arange(.004, plan['times'][-1]/2, .004):
                    self.now = start + elapsed
                    self.request('heartbeat')
                    controller.update(self.engine.next(self.state, self.now), self.state.q, self.now)
                self.assertGreater(controller.dq_target[0], 0)
                if action == 'expiry':
                    self.now += .51
                else:
                    self.request(action)
                command = self.engine.next(self.state, self.now)
                controller.update(command, self.state.q, self.now)
                self.assertTrue(command.hold)
                np.testing.assert_array_equal(controller.dq_target, np.zeros(6))

    def test_q_only_planner_plugin_keeps_position_only_output(self):
        planner = MotionPlanner(self.config, 'model')
        plan = planner.plan({'q': (self.state.q + [.02,0,0,0,0,0]).tolist()},
                            self.engine.status(self.state, self.now), self.now)
        del plan['_trajectory']
        self.request('execute', _plan=plan)
        self.now += .1
        command = self.engine.next(self.state, self.now)
        self.assertIsNotNone(command.joint_target)
        self.assertIsNone(command.joint_velocity_target)

    def test_explicit_ingress_epoch_deadline_and_fault_are_source_independent(self):
        from embodied.devices.inputs.quest.ingress import Candidate
        self.request('release')
        self.engine.ingress_channels = {'xr-test'}
        self.lease = self.request('acquire', source='arbitrary_label', ingress='xr-test', purpose='teleop')
        self.engine.next(self.state, self.now)
        command = ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=self.state.pose)
        def candidate(**fields):
            return {'xr-test': Candidate(**dict(epoch=self.engine.epoch, command=command,
                                                expires_at=self.now+.1, phase='teleop') | fields)}
        self.assertIs(self.engine.next(self.state, self.now, candidate()), command)
        self.assertEqual(self.engine.phase, 'teleop')
        self.assertTrue(self.engine.next(self.state, self.now, candidate(epoch=self.engine.epoch-1)).hold)
        self.assertTrue(self.engine.next(self.state, self.now, candidate(expires_at=self.now)).hold)
        with self.assertRaisesRegex(ValueError, 'installed ingress'):
            self.request('joint', q=self.state.q.tolist())
        self.request('cancel')
        self.assertTrue(self.engine.next(self.state, self.now, candidate()).hold)
        self.engine.next(self.state, self.now, candidate(fault='router exited'))
        self.assertIsNone(self.engine.owner)
        self.assertEqual(self.engine.last_revocation['reason'], 'router exited')
        self.lease = self.request('acquire', source='phone', purpose='teleop')
        self.request('joint', q=self.state.q.tolist())
        self.engine.next(self.state, self.now, candidate(fault='router exited'))
        self.assertEqual(self.engine.owner['source'], 'phone')
        self.assertIsNotNone(self.engine.next(self.state, self.now, candidate(fault='router exited')).joint_target)

    def test_joint_and_linear_plans_obey_sampled_derivative_bounds(self):
        planner = MotionPlanner(self.config, 'model')
        q = np.array([0., .6, 1., 0., .2, 0.])
        snapshot = {**self.engine.status(self.state, self.now), 'q': q.tolist()}
        target = planner.model.fk(q + [.002,0,0,0,0,0])
        for request in ({'kind':'movej','q':(q+.02).tolist()}, {'kind':'movel','pose':target.tolist()}):
            plan = planner.plan({**request, 'speed': .15, 'acceleration': .5, 'jerk': 2.}, snapshot, self.now)
            v = np.gradient(plan['q'], plan['times'], axis=0)
            a = np.gradient(v, plan['times'], axis=0)
            j = np.gradient(a, plan['times'], axis=0)
            self.assertLessEqual(np.max(np.abs(v)), .1515)
            self.assertLessEqual(np.max(np.abs(a)), .505)
            self.assertLessEqual(np.max(np.abs(j)), 2.02)


    def test_gateway_plans_respect_the_table_plane(self):
        from embodied.control.workspace import Workspace
        planner = MotionPlanner(self.config, 'model')
        self.assertEqual(planner.workspace.describe(), {'minimum_tcp_z_m': 0.0})  # config/workspace.json
        q = np.array([0., .6, 1., 0., .2, 0.])
        snapshot = {**self.engine.status(self.state, self.now), 'q': q.tolist()}
        start = planner.model.fk(q)
        planner.workspace = Workspace(table_enabled=True, table_z_m=float(start[2]) - .01)
        planner.plan({'kind': 'movej', 'q': (q + [0, -.02, 0, 0, 0, 0]).tolist()}, snapshot, self.now)
        below = start.copy(); below[2] -= .03
        with self.assertRaisesRegex(ValueError, 'table plane'):
            planner.plan({'kind': 'movel', 'pose': below.tolist()}, snapshot, self.now)
        with self.assertRaisesRegex(ValueError, 'table plane'):
            planner.workspace.check_pose(below)
        Workspace(table_enabled=False).check_pose(below)
        for bad in ({'table_enabled': 1}, {'table_enabled': True, 'table_z_m': float('nan')},
                    {'table_enabled': True, 'table_clearance_m': -.01}):
            with self.assertRaises(ValueError):
                Workspace(**bad)


    def test_gateway_checks_streamed_targets_before_queueing(self):
        import threading
        from embodied.control.gateway import Gateway
        from embodied.control.workspace import Workspace
        gateway = Gateway.__new__(Gateway)  # no socket: only the request-thread check
        gateway.config = self.config
        gateway.planner, gateway.plan_lock = MotionPlanner(self.config, 'model'), threading.Lock()
        q = np.array([0., .6, 1., 0., .2, 0.])
        pose = gateway.planner.model.fk(q)
        gateway.planner.workspace = Workspace(table_enabled=True, table_z_m=float(pose[2]) + .01)
        with self.assertRaisesRegex(ValueError, 'table plane'):
            gateway._check_workspace('cartesian', {'pose': pose.tolist()})
        with self.assertRaisesRegex(ValueError, 'table plane'):
            gateway._check_workspace('joint', {'q': q.tolist()})
        gateway._check_workspace('cartesian', {'pose': [0, 0]})  # malformed: left to CommandEngine
        gateway.planner.workspace = Workspace(table_enabled=True, table_z_m=float(pose[2]) - .01)
        gateway._check_workspace('cartesian', {'pose': pose.tolist()})
        gateway._check_workspace('joint', {'q': q.tolist()})


    def test_describe_reports_kernel_limits_without_a_lease(self):
        import threading
        from embodied.control.gateway import Gateway, ACTIONS
        gateway = Gateway.__new__(Gateway)
        gateway.config, gateway.model_id, gateway.engine = self.config, 'model', self.engine
        gateway.planner, gateway.plan_lock = MotionPlanner(self.config, 'model'), threading.Lock()
        gateway.capabilities = {}
        gateway.simulation = None
        described = gateway.describe()
        self.assertEqual(described['robot']['joint_min'], list(self.config.joint_min))
        self.assertEqual(described['robot']['control_hz'], 250)
        self.assertEqual(described['workspace'], {'minimum_tcp_z_m': 0.0})
        self.assertEqual(described['motion_limits']['defaults'],
                         MotionLimits.load(self.config).describe()['defaults'])
        self.assertEqual(described['motion_limits']['joint_velocity_limits_rad_s'], list(self.config.joint_velocity_max))
        self.assertEqual(described['motion_limits']['source']['kind'], 'offline_configuration')
        self.assertEqual(described['motion_limits']['settling']['position_tolerance_rad'], .0035)
        self.assertEqual(set(described['actions']), set(ACTIONS) - {
            'simulation-reset', 'simulation-restore', 'simulation-snapshot', 'grasp'})
        # The platform's offline reader returns the same robot facts.
        from embodied.contracts.robot import load_robot_profile
        profile = load_robot_profile(ROOT / 'configs/robots/arx5')
        self.assertEqual(described['robot']['joint_count'], len(self.config.park_q))
        for name in ('joint_min', 'joint_max', 'park_q', 'control_hz', 'gripper_max_width', 'gripper_velocity_max_m_s'):
            self.assertEqual(profile['robot'][name], described['robot'][name], name)
        self.assertEqual(profile['workspace'], described['workspace'])


class NativeGateTests(unittest.TestCase):
    def setUp(self):
        from embodied.devices.inputs.quest.sources import NativeIngress
        with patch('embodied.devices.inputs.quest.sources.RobotBridge'), patch('embodied.devices.inputs.quest.sources.create_node'), patch('embodied.devices.inputs.quest.sources.open_service'), patch('embodied.devices.inputs.quest.sources.make_subscriber'):
            self.source = NativeIngress('test', ['input','coordinator','arm','gripper'])
        self.state = SimpleNamespace(gripper_width=.04)
        self.command = ControlCommand(Mode.COMMAND_FOLLOWING)
        self.source.bridge.poll_commands.return_value = self.command
        self.now, self.sequence = 10., 0

    def frame(self, grip, valid=True, epoch=1, selected=True, mask=0, buttons=0, **overrides):
        from embodied.devices.inputs.quest.native_health import DependencyHealth, TeleopGate, TeleopRouteHealth
        self.sequence += 1
        self.now = round(self.now + .01, 6)
        samples = {
            'input': SimpleNamespace(sequence=self.sequence, timestamp_us=round(self.now*1e6), connection_state=1,
                                     tracking_valid=valid, buttons_bitset=buttons, grip=grip),
            'permit': TeleopGate(round(self.now*1e6), self.sequence, 1, 1, 1),
            'arm': TeleopRouteHealth(round(self.now*1e6), self.sequence, 1),
            'gripper': TeleopRouteHealth(round(self.now*1e6), self.sequence, 1),
            'dependencies': DependencyHealth(round(self.now*1e9), self.sequence, mask),
        }
        samples.update(overrides)
        with patch('embodied.devices.inputs.quest.sources.time.monotonic', return_value=self.now), patch('embodied.devices.inputs.quest.sources.time.time', return_value=self.now), patch('embodied.devices.inputs.quest.sources.drain_latest', side_effect=list(samples.values())):
            return self.source.poll(self.state, self.now, epoch, selected)

    def test_handover_requires_new_release_and_tracking_failure_withdraws(self):
        self.assertTrue(self.frame(.9).command.hold)
        self.assertTrue(self.frame(0).command.hold)
        self.assertIs(self.frame(.9).command, self.command)
        self.assertIn('tracking', self.frame(.9, False).fault)
        self.assertTrue(self.frame(.9, epoch=2).command.hold)
        self.frame(0, epoch=2)
        self.assertIs(self.frame(.9, epoch=2).command, self.command)
        self.assertIn('coordinator', self.frame(.9, epoch=2, mask=2).fault)

    def test_native_permit_and_unsolicited_disabled_cannot_bypass_hold(self):
        from embodied.devices.inputs.quest.native_health import TeleopGate
        self.frame(0)
        self.frame(.9)
        gate = TeleopGate(round((self.now+.01)*1e6), self.sequence+1, 0, 0, 0)
        self.assertTrue(self.frame(.9, permit=gate).command.hold)
        self.source.bridge.poll_commands.return_value = ControlCommand(Mode.DISABLED)
        self.assertTrue(self.frame(.9).command.hold)

    def test_explicit_home_can_resynchronize_native_mode_after_reacquisition(self):
        self.source.bridge.poll_commands.return_value = ControlCommand(Mode.DISABLED)
        self.assertTrue(self.frame(0).command.hold)
        self.assertTrue(self.frame(.9).command.hold)
        self.assertEqual(self.frame(0, buttons=12).command.mode, Mode.DISABLED)
        # A new lease cannot inherit the previous operator's Home authorization.
        self.assertTrue(self.frame(0, epoch=2).command.hold)
        self.source.bridge.poll_commands.return_value = self.command
        self.frame(0, epoch=2, buttons=12)
        # The coordinator's mode response can arrive after the short chord ends.
        self.source.bridge.poll_commands.return_value = ControlCommand(Mode.DISABLED)
        self.assertEqual(self.frame(0, epoch=2).command.mode, Mode.DISABLED)

    def test_duplicate_regressed_and_old_producer_stamps_never_renew_health(self):
        from embodied.devices.inputs.quest.sources import FreshSample
        tracker = FreshSample(.1)
        self.assertTrue(tracker.update(SimpleNamespace(sequence=2,timestamp_us=10_000_000),10.,10.))
        for seq, stamp in [(2,10_050_000),(1,10_050_000),(3,9_000_000),(3,11_000_000)]:
            self.assertFalse(tracker.update(SimpleNamespace(sequence=seq,timestamp_us=stamp),10.05,10.05))
        self.assertFalse(tracker.fresh(10.11))

    def test_native_health_abi(self):
        import ctypes
        from embodied.devices.inputs.quest.native_health import TeleopGate, TeleopRouteHealth
        self.assertEqual(ctypes.sizeof(TeleopGate),32)
        self.assertEqual(TeleopGate.action.offset,20)
        self.assertEqual(ctypes.sizeof(TeleopRouteHealth),24)

    def test_managed_rpc_only_worker_still_publishes_readonly_feedback(self):
        import json, os, subprocess, tempfile, time, uuid
        from embodied.control.ipc.services import create_node, open_service, make_subscriber, drain_latest
        from embodied.control.ipc.types import JointVector15
        prefix = 'arx5-feedback-' + uuid.uuid4().hex[:10]
        node = create_node()
        subscriber = make_subscriber(open_service(node, prefix+'/robot/arm/states/joint_position', JointVector15))
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory/'runtime.json').write_text(json.dumps({'input':'idle','native':None}))
            with (directory/'worker.log').open('w') as log:
                worker = subprocess.Popen([sys.executable,'-B','-m','embodied.control','_worker','--managed',
                    '--backend','sim','--prefix',prefix,'--directory',str(directory)], cwd=ROOT,
                    stdout=log,stderr=subprocess.STDOUT,
                    env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
                try:
                    deadline = time.monotonic()+5
                    stamps=[]
                    while time.monotonic()<deadline and len(stamps)<3:
                        self.assertIsNone(worker.poll(),(directory/'worker.log').read_text())
                        sample=drain_latest(subscriber)
                        if sample is not None:
                            self.assertEqual(sample.len,6)
                            stamps.append(sample.timestamp_us)
                        time.sleep(.015)
                    self.assertEqual(len(stamps),3)
                    self.assertTrue(all(b>a for a,b in zip(stamps,stamps[1:])))
                finally:
                    worker.terminate()
                    worker.wait(timeout=10)
                self.assertEqual(worker.returncode,0,(directory/'worker.log').read_text())
                self.assertIsNone(json.loads((directory/'report.json').read_text())['error'])

    def test_simulation_uses_existing_controller_injection(self):
        from embodied.devices.robots.arx5.simulation import MujocoBackend
        from embodied.devices.robots.arx5.kinematics import JointTargetController
        with patch('embodied.devices.robots.arx5.simulation.JointTargetController', wraps=JointTargetController) as factory:
            backend = MujocoBackend(load_settings().robot, controller_factory=factory)
            self.addCleanup(backend.close)
            factory.assert_called_once_with(backend.config, backend.kinematics)
            state = backend.read_state(10.)
            backend.step(ControlCommand(Mode.COMMAND_FOLLOWING,hold=True),10.)
            np.testing.assert_allclose(backend.controller.q_target, state.q)


if __name__ == '__main__':
    unittest.main(verbosity=2)
