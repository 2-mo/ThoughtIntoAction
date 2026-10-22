"""Scene restoration, ownership boundaries and task predicates; no services/devices."""
from copy import deepcopy
import queue
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from embodied.applications.operations.simulation import Simulation
from embodied.contracts.runtime.control import ControlSession
from embodied.control.commands import CommandEngine
from embodied.control.gateway import Gateway
from embodied.control.settings import load_settings
from embodied.devices.robots.arx5.simulation import MujocoBackend


class SimulationStateTests(unittest.TestCase):
    def setUp(self):
        self.config = load_settings().robot
        self.backend = MujocoBackend(self.config)
        self.addCleanup(self.backend.close)

    def test_restore_preserves_full_integration_state_and_reset_is_repeatable(self):
        initial = self.backend.simulation_snapshot()
        self.backend.data.time = 1.25
        self.backend.data.qpos[0] = .12
        self.backend.data.qvel[0] = .03
        self.backend.data.ctrl[0] = .5
        self.backend.data.qfrc_applied[1] = .1
        import mujoco
        mujoco.mj_forward(self.backend.model, self.backend.data)
        saved = self.backend.simulation_snapshot()
        self.backend.simulation_reset()
        np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], initial['state'])
        self.backend.simulation_restore(saved)
        np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], saved['state'])
        self.assertAlmostEqual(self.backend.read_state(time.monotonic()).q[0], .12)
        self.assertEqual(self.backend.controller.mode.value, 'command-following')
        self.assertIsNone(self.backend.telemetry()['sent_q'])
        self.backend.simulation_reset()
        np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], initial['state'])

    def test_bad_state_or_identity_cannot_partially_mutate_physics(self):
        saved = self.backend.simulation_snapshot()
        candidates = []
        for key, value in [('model_id', 'another-model'), ('engine_version', 'other'),
                           ('configuration_id', 'other'), ('state_spec', 0), ('state', [0.])]:
            candidates.append({**saved, key: value})
        for value in (float('nan'), True, '0'):
            bad = deepcopy(saved)
            bad['state'][0] = value
            candidates.append(bad)
        for candidate in candidates:
            with self.subTest(candidate=candidate.get('model_id')):
                with self.assertRaises(ValueError):
                    self.backend.simulation_restore(candidate)
                np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], saved['state'])

    def test_offline_render_uses_separate_data_and_releases_renderer(self):
        saved = self.backend.simulation_snapshot()
        saved['state'][0] = 4.0  # time is the first integration-state element
        before = self.backend.simulation_snapshot()
        with patch('embodied.devices.robots.arx5.simulation.mujoco.Renderer') as factory:
            renderer = factory.return_value.__enter__.return_value
            renderer.render.return_value = np.ones((12, 16, 3), dtype=np.uint8)
            image = self.backend.render_snapshot(saved, width=16, height=12)
        data = renderer.update_scene.call_args.args[0]
        self.assertIsNot(data, self.backend.data)
        self.assertEqual(data.time, 4.)
        self.assertEqual(image.shape, (12, 16, 3))
        factory.return_value.__exit__.assert_called_once()
        np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], before['state'])

    def gateway(self):
        gateway = object.__new__(Gateway)
        gateway.simulation = self.backend
        gateway.engine = CommandEngine('run', self.config, self.config.model_id)
        state = self.backend.read_state(time.monotonic())
        self.lease = gateway.engine.request({'version': 1, 'runtime_id': 'run',
            'action': 'acquire', 'source': 'test'}, state, time.monotonic())
        return gateway, state

    def request(self, action, sequence=1, **fields):
        return {'version': 1, 'runtime_id': 'run', 'action': action,
                'token': self.lease['token'], 'epoch': self.lease['epoch'],
                'sequence': sequence, **fields}

    def test_restore_invalidates_previews_and_updates_tick_feedback(self):
        gateway, state = self.gateway()
        saved = self.backend.simulation_snapshot()
        self.backend.data.qpos[0] = .1
        state = self.backend.read_state(time.monotonic())
        previous_epoch = gateway.engine.epoch
        future = {'request': self.request('simulation-restore', snapshot=saved),
                  'deadline': time.monotonic_ns() + 1_000_000_000, 'done': threading.Event()}
        gateway.pending = queue.Queue()
        gateway.pending.put(future)
        gateway.ingresses = {}
        acknowledged = future['done'].set
        def acknowledge():
            self.assertEqual(gateway.snapshot['q'], saved['observation']['q'])
            acknowledged()
        with patch.object(future['done'], 'set', side_effect=acknowledge):
            gateway.tick(state, time.monotonic())
        self.assertNotIn('error', future)
        self.assertTrue(future['done'].is_set())
        self.assertEqual(future['result']['epoch'], previous_epoch + 1)
        self.assertAlmostEqual(gateway.feedback.q[0], saved['observation']['q'][0])
        self.assertEqual(gateway.snapshot['q'], gateway.feedback.q.tolist())
        old_plan = {'epoch': previous_epoch, 'owner_present': True,
                    'model_id': self.config.model_id, 'expires': time.monotonic() + 10}
        self.lease['epoch'] = gateway.engine.epoch
        with self.assertRaisesRegex(ValueError, 'Preview expired'):
            gateway.engine.request(self.request('execute', sequence=2, _plan=old_plan),
                                   gateway.feedback, time.monotonic())

    def test_active_operation_and_hardware_have_no_reset_side_effect(self):
        gateway, state = self.gateway()
        original = self.backend.simulation_snapshot()
        gateway.engine.operation = {'status': 'running'}
        with self.assertRaisesRegex(ValueError, 'active operation'):
            gateway._simulation_request(self.request('simulation-reset'), state, time.monotonic())
        self.assertEqual(gateway.engine.epoch, self.lease['epoch'])
        np.testing.assert_array_equal(self.backend.simulation_snapshot()['state'], original['state'])
        gateway.simulation = None
        with self.assertRaisesRegex(ValueError, 'does not support'):
            gateway._simulation_request(self.request('simulation-reset', sequence=2), state, time.monotonic())

    def test_session_renews_on_returned_epoch(self):
        session = object.__new__(ControlSession)
        session.lock = threading.RLock()
        session.error = None
        session.lease = {'token': 't', 'epoch': 3}
        session.sequence = 0
        with patch('embodied.contracts.runtime.control.ControlClient.request', return_value={'status': 'applied', 'epoch': 4}) as call:
            session.command('simulation-reset')
            session.command('heartbeat')
        self.assertEqual(call.call_args.kwargs['epoch'], 4)

    def test_task_reuses_scene_and_checks_observation_not_motion_completion(self):
        client, session = MagicMock(), MagicMock()
        client.describe.return_value = {'actions': ['simulation-snapshot']}
        snapshot = self.backend.simulation_snapshot()
        client.request.return_value = snapshot
        robot = SimpleNamespace(backend='sim', _connected=lambda: client, _acquire=lambda: session)
        simulation = Simulation(robot)
        task = simulation.task('reach', instruction='Reach joint 1 target', goal='q[0] > 0.1',
                               success=lambda obs: obs['q'][0] > .1)
        self.assertFalse(task.check_success())
        snapshot['observation']['q'][0] = .2
        self.assertTrue(task.check_success())
        task.reset()
        session.command.assert_called_once()
        self.assertEqual(session.command.call_args.kwargs['snapshot']['observation']['q'][0],
                         self.config.park_q[0])
        self.assertEqual(task.describe()['name'], 'reach')
        with self.assertRaisesRegex(ValueError, 'absent'):
            simulation.task('bad', instruction='missing object', success=lambda obs: True, objects=['missing'])
        robot.backend = 'real'
        with self.assertRaisesRegex(ValueError, 'simulation capabilities'):
            simulation.reset()


if __name__ == '__main__':
    unittest.main()
