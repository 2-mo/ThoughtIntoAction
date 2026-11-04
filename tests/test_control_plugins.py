"""Offline checks of the common runtime against a non-ARX seven-axis model."""
import unittest
import json
import tempfile
from pathlib import Path
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from embodied.control.commands import MotionPlanner
from embodied.control.settings import load_settings


class SevenAxisModel:
    lower = np.full(7, -2.)
    upper = np.full(7, 2.)

    def fk(self, q):
        return np.r_[np.asarray(q)[:3] * .1 + [0., 0., .3], 0., 0., 0., 1.]

    def solve_ik(self, pose, seed):
        q = np.asarray(seed).copy()
        q[:3] = (np.asarray(pose)[:3] - [0., 0., .3]) / .1
        return q, True, 0.


@dataclass(frozen=True)
class TestProfile:
    name: str = 'Seven-axis test device'
    control_hz: int = 100
    park_q: tuple = (0.,) * 7
    simulation_plugin: str = 'test.sim'
    hardware_plugin: str = 'test.real'


class ControlPluginTests(unittest.TestCase):
    def test_managed_worker_uses_post_read_time_for_standstill_and_acquisition(self):
        from embodied.control import worker
        from embodied.control.commands import CommandEngine
        from embodied.contracts.control import RobotState
        settings = load_settings('configs/robots/realman/sim.json')
        config = settings.robot
        clock = [100.]
        acquired = []
        observed = []

        def read_state(now):
            clock[0] += .006  # Model a synchronous hardware query finishing after tick start.
            return RobotState(np.asarray(config.park_q), np.zeros(6),
                              np.array([0., 0., .3, 0., 0., 0., 1.]), None, clock[0])

        class Gateway:
            def __init__(self, directory, runtime_id, config, planner_factory, ingresses, **options):
                self.engine = CommandEngine(runtime_id, config, config.model_id,
                    motion_limits=options['motion_limits'], capabilities=options['capabilities'])

            def tick(self, state, now):
                observed.append(now-state.received_at)
                command = self.engine.next(state, now)
                if self.engine.stationary and not acquired:
                    acquired.append(self.engine.request({'version': 1, 'runtime_id': 'robot-clock-test',
                        'action': 'acquire', 'source': 'quest', 'purpose': 'teleop'}, state, now))
                self.feedback = state
                if len(observed) >= 15:
                    self.engine.stopped = True
                return command

            def close(self):
                pass

        backend = SimpleNamespace(read_state=read_state, step=lambda *a: None, close=lambda: None)
        driver = SimpleNamespace(preflight=lambda config: {'ready': True}, resources=lambda config: (),
            open=lambda *a, **kw: backend,
            capabilities=lambda *a: {'joint_velocity_limits_rad_s': [1.]*6, 'source': {'kind': 'test'}},
            shutdown=lambda *a, **kw: {'policy': 'test-hold'})
        registry = SimpleNamespace(spec=lambda *a, **kw: SimpleNamespace(hardware=False),
            create=lambda *a, **kw: driver, load=lambda *a, **kw: None)
        bridge = SimpleNamespace(publish_state=lambda *a: None, close=lambda: None)
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(worker, 'load_settings', return_value=settings), \
             patch.object(worker, 'write_parameter_snapshot'), \
             patch.object(worker, 'RobotBridge', return_value=bridge), \
             patch.object(worker.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(worker, 'PeriodicClock', return_value=SimpleNamespace(
                 wait=lambda: clock.__setitem__(0, clock[0]+.04))), \
             patch('embodied.contracts.registry.Registry', return_value=registry), \
             patch('embodied.control.gateway.Gateway', Gateway):
            (Path(folder)/'runtime.json').write_text(json.dumps({'native': None}))
            worker.run('sim', 'robot-clock-test', None, Path(folder), managed=True)
        self.assertTrue(all(age >= 0 for age in observed))
        self.assertEqual(len(acquired), 1)

    def test_profile_does_not_read_unselected_input(self):
        settings = load_settings(vr_path='/nonexistent/unselected-input.json')
        self.assertEqual(settings.robot.joint_count, 6)
        self.assertNotIn('vr', vars(settings))

    def test_bad_feedback_releases_driver_and_resources(self):
        from embodied.control import worker
        from embodied.contracts.control import RobotState
        events = []
        profile = TestProfile()
        # Invalid feedback reaches the common boundary without an ARX adapter.
        backend = SimpleNamespace(
            read_state=lambda now: RobotState(np.zeros(6), np.zeros(7), np.array([0., 0., .3, 0., 0., 0., 1.]), .02, now),
            close=lambda: events.append('backend'))
        def shutdown(*args, **kwargs):
            events.append('shutdown')
            return {'policy': 'test-hold'}
        driver = SimpleNamespace(preflight=lambda config: {'ready': True},
            resources=lambda config: ('offline-test',), open=lambda *a, **kw: backend,
            capabilities=lambda config, parameters: {'joint_velocity_limits_rad_s': [1.] * len(config.park_q),
                                                     'source': {'kind': 'test'}},
            shutdown=shutdown)
        registry = SimpleNamespace(spec=lambda *a, **kw: SimpleNamespace(hardware=False),
                                   create=lambda *a, **kw: driver)
        bridge = SimpleNamespace(close=lambda: events.append('bridge'))
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(worker, 'load_settings', return_value=SimpleNamespace(robot=profile)), \
             patch.object(worker, 'write_parameter_snapshot'), \
             patch.object(worker, 'RobotBridge', return_value=bridge), \
             patch('embodied.contracts.registry.Registry', return_value=registry), \
             patch('embodied.contracts.device_lease.DeviceLease', return_value=SimpleNamespace(close=lambda: events.append('lease'))):
            with self.assertRaisesRegex(RuntimeError, '7 joints'):
                worker.run('sim', 'robot-offline-test', None, Path(folder))
        self.assertEqual(events, ['shutdown', 'backend', 'bridge', 'lease'])

    def test_movej_and_movel_keep_seven_axes_and_driver_speed_caps(self):
        profile = SimpleNamespace(park_q=(0.,) * 7, model_plugin='test.seven',
                                  control_hz=100, gripper_max_width=.05,
                                  joint_velocity_max=(.3, .4, .5, .6, .7, .8, .9))
        with patch('embodied.contracts.registry.Registry.create', return_value=SevenAxisModel()) as factory:
            planner = MotionPlanner(profile, 'seven-model')
        factory.assert_called_once_with('kinematics', 'test.seven', runtime='control', config=profile)
        snapshot = {'q': [0.] * 7, 'gripper_width': .02, 'epoch': 0, 'owner': None}
        for request in ({'kind': 'movej', 'q': [.1] * 7},
                        {'kind': 'movel', 'pose': [.01, .01, .31, 0., 0., 0., 1.]}):
            with self.subTest(kind=request['kind']):
                plan = planner.plan({**request, 'speed': 0, 'speed_scale': .5}, snapshot, 1.)
                self.assertEqual(plan['q'].shape[1], 7)
                caps = np.asarray(profile.joint_velocity_max) * .5
                np.testing.assert_allclose(plan['limits']['joint_velocity_rad_s'], caps)
                for t in np.linspace(0, plan['times'][-1], 101):
                    q, dq = plan['_trajectory'].sample(t)
                    self.assertEqual(q.shape, (7,))
                    self.assertTrue(np.all(np.abs(dq) <= caps * 1.01))
                q, dq = plan['_trajectory'].sample(plan['times'][-1])
                np.testing.assert_allclose(q, plan['q'][-1])
                np.testing.assert_array_equal(dq, np.zeros(7))


if __name__ == '__main__':
    unittest.main()
