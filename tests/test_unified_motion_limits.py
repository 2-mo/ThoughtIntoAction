"""One immutable per-run policy for plans, streams and API defaults; no hardware."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from embodied.contracts.control import ControlCommand, Mode, RobotState
from embodied.control.commands import CommandEngine
from embodied.control.motion_limits import DEFAULT_PATH, MotionLimits


class UnifiedMotionLimitsTests(unittest.TestCase):
    def setUp(self):
        self.config = SimpleNamespace(park_q=(0., 0.), joint_velocity_max=(4., 5.),
                                      control_hz=250, gripper_max_width=.08)
        self.settings = json.loads(DEFAULT_PATH.read_text())

    def test_runtime_snapshot_defaults_overrides_and_driver_caps(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'motion.json'
            path.write_text(json.dumps(self.settings))
            policy = MotionLimits.load(self.config, path=path)
            self.settings['defaults'].update(speed=.4, acceleration=3., jerk=15.)
            self.settings['cartesian']['tcp_speed'] = .12
            path.write_text(json.dumps(self.settings))
            updated = MotionLimits.load(self.config, path=path)
        self.assertEqual(policy.resolve({})['joint_velocity_rad_s'], [4., 5.])
        self.assertEqual(updated.resolve({})['joint_velocity_rad_s'], [.4, .4])
        self.assertEqual(updated.resolve({})['acceleration'], 3.)
        self.assertEqual(updated.resolve({'speed': 0})['joint_velocity_rad_s'], [4., 5.])
        self.assertEqual(updated.resolve({'speed': 10, 'speed_scale': .5})['joint_velocity_rad_s'], [2., 2.5])
        self.assertEqual(policy.resolve_cartesian({}), {})
        self.assertEqual(updated.resolve_cartesian({}), {'tcp_speed': .12})
        self.assertEqual(updated.resolve_cartesian({'tcp_speed': .3}), {'tcp_speed': .3})
        np.testing.assert_allclose(updated.step_limits(), np.array([.4, .4]) / 250)

    def test_real_requires_driver_capabilities_and_invalid_config_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'connected driver'):
            MotionLimits.load(self.config, backend='real')
        for field, value in [('speed', -1), ('acceleration', 0), ('jerk', float('inf'))]:
            settings = {**self.settings, 'defaults': {**self.settings['defaults'], field: value}}
            with self.subTest(field=field), self.assertRaises(ValueError):
                MotionLimits(**settings, joint_velocity_limits=[4., 5.], source={}, control_hz=250)
        settings = {**self.settings, 'cartesian': {**self.settings['cartesian'], 'tcp_speed': 0}}
        with self.assertRaises(ValueError):
            MotionLimits(**settings, joint_velocity_limits=[4., 5.], source={}, control_hz=250)

    def test_native_ingress_uses_same_step_bounds_as_rpc(self):
        engine = CommandEngine('fixture', self.config, 'test', ingress_channels=['vr'])
        state = RobotState(np.zeros(2), np.zeros(2), np.array([0., 0., .3, 0., 0., 0., 1.]), .04, 1.)
        lease = engine.request({'version': 1, 'runtime_id': 'fixture', 'action': 'acquire',
                                'source': 'test', 'ingress': 'vr'}, state, 1.)
        engine.next(state, 1.)  # Establish holding reference before target delivery.
        candidate = SimpleNamespace(epoch=lease['epoch'], fault=None, expires_at=1.2,
            command=ControlCommand(Mode.COMMAND_FOLLOWING, joint_target=np.ones(2), joint_step_limit=100.),
            sequence=1, phase='teleop')
        command = engine.next(state, 1.01, {'vr': candidate})
        np.testing.assert_allclose(command.joint_step_limit, np.array([4., 5.]) / 250)

    def test_native_router_toml_uses_central_configuration(self):
        import tomllib
        from embodied.devices.inputs.quest import native_config
        self.settings['native_router']['smoothing_enabled'] = True
        self.settings['native_router']['cartesian']['max_linear_velocity'] = .3
        self.settings['native_router']['gripper']['max_velocity'] = .04
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'motion.json'
            path.write_text(json.dumps(self.settings))
            with patch.object(native_config, 'DEFAULT_PATH', path):
                files = native_config.write_native_configs(Path(folder), 'arx5-motion-test', .08, '/unused/sdk.so')
            arm = tomllib.loads(files['arm'].read_text())
            gripper = tomllib.loads(files['gripper'].read_text())
        self.assertTrue(arm['ruckig_smoothing_enabled'])
        self.assertEqual(arm['follower_motion_limits']['cartesian']['max_linear_velocity'], .3)
        self.assertEqual(gripper['follower_motion_limits']['max_velocity'], [.04])


if __name__ == '__main__':
    unittest.main()
