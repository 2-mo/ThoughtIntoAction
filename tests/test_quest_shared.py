"""Shared Quest readiness and explicit controls, without SDKs or live IPC."""
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest
import numpy as np
from unittest.mock import patch

from embodied.contracts.control import ControlCommand, Mode
from embodied.devices.inputs.quest.ingress import Candidate
from embodied.devices.inputs.quest.shared import Ingress, Routing, tomllib
from embodied.devices.inputs.quest.sources import FreshSample, NativeIngress


class SharedQuestTests(unittest.TestCase):
    def test_one_mode_coordinator_per_hand_without_gripper_health_dependency(self):
        settings = SimpleNamespace(sdk_library='/unused/sdk',
            vr=SimpleNamespace(input_prefix='robot-test-input', controller='left', base_yaw_deg=0,
                grip_threshold=.7, position_scale=(1., 1., 1.)),
            robot=SimpleNamespace(gripper_max_width=.07, control_hz=25,
                teleop_linear_speed_m_s=.25, teleop_angular_speed_rad_s=.3))
        with tempfile.TemporaryDirectory() as folder:
            paths = Routing.configure(Path(folder), 'robot-test-left', settings)
            configs = {name: tomllib.loads(path.read_text()) for name, path in paths.items()}
            coordinator = configs['coordinator']
            self.assertEqual([m['member_id'] for m in coordinator['members']], ['arm'])
            self.assertEqual([r['route_id'] for r in coordinator['routes']], ['arm'])
            self.assertEqual(coordinator['mode_controller_input_topic'],
                'robot-test-input/xr/left_controller/states/input')
            self.assertEqual(set(Routing.commands(paths, 'robot-test-left')), {'arm', 'coordinator'})
            self.assertEqual(configs['arm']['relative_hold_timeout_ms'], 80)
            self.assertEqual(configs['arm']['relative_fault_timeout_ms'], 120)

    def setUp(self):
        self.ingress = Ingress.__new__(Ingress)
        self.ingress.status = {'ready': True, 'permit': True}
        self.input = SimpleNamespace(trigger=0., grip=0., buttons_bitset=0)
        sample = FreshSample(.1)
        sample.sample, sample.expires_at = self.input, 100.
        self.ingress.samples = {'input': sample}
        self.ingress.gripper_released = self.ingress.gripper_armed = False
        self.ingress.gripper_epoch = None
        self.ingress.gripper_max_width = .07
        self.ingress.home_q = np.array([0., 0., np.pi/2, 0., np.pi/2, 0.])
        self.ingress.home_released = self.ingress.returning_home = self.ingress.home_blocked = False
        self.ingress.home_since = None
        self.state = SimpleNamespace(gripper_width=.07)
        self.native = Candidate(1, ControlCommand(Mode.COMMAND_FOLLOWING,
            gripper_target=.01, pose_target=[0., 0., 0., 0., 0., 0., 1.], engaged=True), 100., 1)

    def poll(self, now, **kwargs):
        with patch.object(NativeIngress, 'poll', return_value=self.native), \
             patch('embodied.devices.inputs.quest.shared.time.monotonic', return_value=now):
            return self.ingress.poll(self.state, now, kwargs.get('epoch', 1), kwargs.get('selected', True)).command

    def test_recovery_requires_trigger_release_then_new_press(self):
        self.input.trigger = 1.
        self.assertIsNone(self.poll(1.).gripper_target)
        self.input.trigger = 0.
        self.assertIsNone(self.poll(2.).gripper_target)
        self.input.trigger = .5
        self.assertEqual(self.poll(3.).gripper_target, .035)
        self.state.gripper_width = None
        self.assertIsNone(self.poll(4.).gripper_target)
        self.state.gripper_width = .05
        self.assertIsNone(self.poll(5.).gripper_target)
        self.input.trigger = 0.
        self.poll(6.)
        self.input.trigger = .5
        self.assertEqual(self.poll(7.).gripper_target, .035)
        self.assertIsNone(self.poll(8., epoch=2).gripper_target)

    def start_home(self):
        self.poll(1.)
        self.input.buttons_bitset = 8
        self.assertIsNone(self.poll(2.).joint_target)
        self.assertIsNone(self.poll(2.99).joint_target)
        return self.poll(3.01)

    def test_partial_startup_trigger_close_release_open_and_mode_sync(self):
        self.state.gripper_width = .01274
        self.native = self.native.__class__(1, ControlCommand(Mode.COMMAND_FOLLOWING,
            gripper_mode=Mode.FREE_DRIVE, gripper_target=.03274), 100., 1)
        self.assertIsNone(self.poll(1.).gripper_target)
        self.input.trigger = 1.
        command = self.poll(2.)
        self.assertEqual(command.gripper_target, 0.)
        self.assertEqual(command.gripper_mode, Mode.COMMAND_FOLLOWING)
        self.input.trigger = 0.
        self.assertEqual(self.poll(3.).gripper_target, .07)
        self.native = self.native.__class__(1, ControlCommand(Mode.FREE_DRIVE), 100., 1)
        self.assertIsNone(self.poll(4.).gripper_target)
        self.assertEqual(self.poll(4.1).gripper_mode, Mode.FREE_DRIVE)

    def test_home_requires_neutral_then_one_second_and_stops_on_release(self):
        self.input.buttons_bitset = 8
        self.assertIsNone(self.poll(.1).joint_target)
        self.assertIsNone(self.poll(.9).joint_target)
        self.input.buttons_bitset = 0
        command = self.start_home()
        np.testing.assert_allclose(command.joint_target, [0, 0, np.pi/2, 0, np.pi/2, 0])
        self.assertTrue(command.reset_reference)
        command = self.poll(3.02)
        self.assertFalse(command.hold or command.reset_reference or command.gripper_initialize_open)
        self.assertIsNone(command.gripper_target)
        self.assertIsNone(command.pose_target)
        np.testing.assert_array_equal(command.joint_target, self.ingress.home_q)
        self.input.buttons_bitset = 0
        command = self.poll(3.03)
        self.assertTrue(command.hold and command.reset_reference)
        self.assertIsNone(command.joint_target)
        self.assertFalse(self.ingress.status['returning_home'])

    def test_home_cancels_on_conflict_and_requires_all_controls_released(self):
        for field, value in [('grip', 1.), ('trigger', 1.), ('buttons_bitset', 12)]:
            with self.subTest(field=field):
                self.setUp()
                self.start_home()
                setattr(self.input, field, value)
                command = self.poll(3.02)
                self.assertTrue(command.hold and command.reset_reference)
                self.assertIsNone(command.joint_target)
                self.input.grip = self.input.trigger = 0.
                self.input.buttons_bitset = 8
                self.assertIsNone(self.poll(5.).joint_target)
                self.assertTrue(self.poll(5.).hold)
                self.input.buttons_bitset = 0
                self.poll(6.)
                self.input.buttons_bitset = 8
                self.poll(7.)
                self.assertIsNotNone(self.poll(8.01).joint_target)

    def test_home_cancels_on_tracking_or_authority_loss_without_resuming(self):
        for kwargs in ({'selected': False}, {'epoch': 2}, {'stale': True}):
            with self.subTest(condition=kwargs):
                self.setUp()
                self.start_home()
                if kwargs.get('stale'):
                    self.ingress.samples['input'].expires_at = 0.
                command = self.poll(3.02, **kwargs)
                self.assertTrue(command.hold and command.reset_reference)
                self.assertIsNone(command.joint_target)
                self.ingress.samples['input'].expires_at = 100.
                self.assertIsNone(self.poll(5., epoch=kwargs.get('epoch', 1)).joint_target)

    def test_home_reaches_configured_joints_in_simulation_without_opening_gripper(self):
        from embodied.control.settings import load_settings
        from embodied.devices.robots.realman.simulation import Backend
        config = load_settings('configs/robots/realman/sim.json').robot
        backend = Backend(config)
        self.addCleanup(backend.close)
        import mujoco
        start = np.asarray(config.park_q) + [.2, -.15, .1, -.15, .15, -.2]
        backend.data.qpos[:6] = start
        backend.data.qpos[6:] = .01
        backend.target = start.copy()
        backend.width_target = .02
        mujoco.mj_forward(backend.model, backend.data)
        self.start_home()
        for i in range(250):
            now = 3.02 + i*.01
            self.state = backend.read_state(now)
            backend.step(self.poll(now), now)
        np.testing.assert_allclose(backend.read_state(6.).q, config.park_q, atol=.001)
        self.assertAlmostEqual(backend.read_state(6.).gripper_width, .02, places=4)
        self.assertFalse(self.poll(6.).gripper_initialize_open)


if __name__ == '__main__': unittest.main()
