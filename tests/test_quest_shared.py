"""Shared Quest readiness and explicit controls, without SDKs or live IPC."""
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from embodied.contracts.control import ControlCommand, Mode
from embodied.devices.inputs.quest.ingress import Candidate
from embodied.devices.inputs.quest.shared import Ingress, Routing, tomllib
from embodied.devices.inputs.quest.sources import FreshSample, NativeIngress


class SharedQuestTests(unittest.TestCase):
    def test_gripper_has_independent_coordinator_and_permit(self):
        settings = SimpleNamespace(sdk_library='/unused/sdk',
            vr=SimpleNamespace(input_prefix='robot-test-input', controller='left', base_yaw_deg=0,
                grip_threshold=.7, position_scale=(1., 1., 1.)),
            robot=SimpleNamespace(gripper_max_width=.07, control_hz=50,
                teleop_linear_speed_m_s=.25, teleop_angular_speed_rad_s=.3))
        with tempfile.TemporaryDirectory() as folder:
            paths = Routing.configure(Path(folder), 'robot-test-left', settings)
            configs = {name: tomllib.loads(path.read_text()) for name, path in paths.items()}
            arm, grip = configs['coordinator'], configs['gripper-coordinator']
            self.assertEqual([m['member_id'] for m in arm['members']], ['arm'])
            self.assertEqual([m['member_id'] for m in grip['members']], ['gripper'])
            self.assertEqual([r['route_id'] for r in arm['routes']], ['arm'])
            self.assertEqual([r['route_id'] for r in grip['routes']], ['gripper'])
            self.assertNotEqual(arm['permit_topic'], grip['permit_topic'])
            self.assertEqual(configs['gripper']['group_permit_topic'], grip['permit_topic'])
            self.assertEqual(configs['gripper']['control_group_id'], grip['group_id'])
            self.assertEqual(arm['mode_controller_input_topic'], grip['mode_controller_input_topic'])
            self.assertEqual(configs['arm']['relative_hold_timeout_ms'], 40)
            self.assertIs(type(configs['arm']['relative_hold_timeout_ms']), int)
            self.assertIs(type(configs['gripper']['relative_fault_timeout_ms']), int)
            self.assertEqual(set(Routing.commands(paths, 'robot-test-left')),
                             {'arm', 'gripper', 'coordinator', 'gripper-coordinator'})
            settings.robot.control_hz = 25
            paths = Routing.configure(Path(folder), 'robot-test-left', settings)
            arm = tomllib.loads(paths['arm'].read_text())
            self.assertEqual(arm['relative_hold_timeout_ms'], 80)
            self.assertEqual(arm['relative_fault_timeout_ms'], 120)
            self.assertIs(type(arm['relative_fault_timeout_ms']), int)

    def setUp(self):
        self.ingress = Ingress.__new__(Ingress)
        self.ingress.status = {'ready': True}
        self.input = SimpleNamespace(trigger=0., grip=0., buttons_bitset=0)
        sample = FreshSample(.1)
        sample.sample, sample.expires_at = self.input, 100.
        self.ingress.samples = {'input': sample}
        self.ingress.gripper_released = self.ingress.gripper_armed = False
        self.ingress.gripper_epoch = None
        self.ingress.initialize_released = self.ingress.initialize_sent = False
        self.ingress.initialize_since = None
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
        self.assertEqual(self.poll(3.).gripper_target, .01)
        self.state.gripper_width = None
        self.assertIsNone(self.poll(4.).gripper_target)
        self.state.gripper_width = .05
        self.assertIsNone(self.poll(5.).gripper_target)
        self.input.trigger = 0.
        self.poll(6.)
        self.input.trigger = .5
        self.assertEqual(self.poll(7.).gripper_target, .01)
        self.assertIsNone(self.poll(8., epoch=2).gripper_target)

    def test_initialize_requires_neutral_then_one_second_secondary_only(self):
        self.input.buttons_bitset = 8
        self.assertFalse(self.poll(1.).gripper_initialize_open)
        self.assertFalse(self.poll(3.).gripper_initialize_open)
        self.input.buttons_bitset = 0
        self.poll(4.)
        self.input.buttons_bitset = 8
        self.assertFalse(self.poll(5.).gripper_initialize_open)
        command = self.poll(6.01)
        self.assertTrue(command.gripper_initialize_open)
        self.assertTrue(command.hold and command.reset_reference)
        self.assertFalse(command.engaged)
        self.assertIsNone(command.pose_target)
        self.assertIsNone(command.gripper_target)
        self.assertFalse(self.poll(7.).gripper_initialize_open)

    def test_initialize_does_not_fire_with_grip_trigger_primary_or_stale_input(self):
        for field, value in [('grip', 1.), ('trigger', 1.), ('buttons_bitset', 12)]:
            with self.subTest(field=field):
                self.input.grip = self.input.trigger = 0.
                self.input.buttons_bitset = 0
                self.poll(1.)
                self.input.buttons_bitset = 8
                setattr(self.input, field, value)
                self.poll(2.)
                self.assertFalse(self.poll(4.).gripper_initialize_open)
        self.input.grip = self.input.trigger = 0.
        self.input.buttons_bitset = 0
        self.poll(5.)
        self.input.buttons_bitset = 8
        self.poll(6.)
        self.assertFalse(self.poll(7.1, selected=False).gripper_initialize_open)
        self.ingress.samples['input'].expires_at = 0.
        self.assertFalse(self.poll(8.).gripper_initialize_open)


if __name__ == '__main__': unittest.main()
