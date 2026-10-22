"""The browser forwards model capabilities instead of imposing hardware constants."""
from pathlib import Path
import unittest
from unittest.mock import AsyncMock, Mock, patch

import numpy as np

from embodied.applications.console.kernel.arx import ArxConsole, joint_index, speed


class ConsoleCapabilities(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.console = object.__new__(ArxConsole)
        self.console.session = Mock()
        self.console.mode = 'active'
        self.console.stream = self.console.task = self.console.free_drive = None
        self.console.robot = {'joint_min': [-2.] * 7, 'joint_max': [2.] * 7,
                              'control_hz': 250, 'gripper_max_width': .12,
                              'gripper_velocity_max_m_s': .2}
        self.console.motion_limits = {'joint_velocity_limits_rad_s': [1., 2., 3., 4., 5., 6., 7.]}
        self.console.reference = ([0.] * 7, .02)
        self.console.status = AsyncMock(return_value={'q': [0.] * 7,
                                                      'pose': [.1, .2, .3, 0, 0, 0, 1],
                                                      'gripper_width': .02})
        self.console.execute = AsyncMock(return_value={'q': [0.] * 7})
        self.console.settle = AsyncMock()
        self.console.session.preview.return_value = {'q': [[0.] * 7, [.1] * 7],
                                                     'tcp': [[.1, .2, .3, 0, 0, 0, 1]], 'times': [0, 1]}

    async def test_movej_and_park_delegate_exact_speed_scale(self):
        for percent in (1, 25, 100):
            with self.subTest(percent=percent):
                await self.console.joint_motion([0.] * 7, {'speed_percent': percent})
                self.console.session.preview.assert_called_with('movej', q=[0.] * 7,
                                                                 speed_scale=percent / 100)
        await self.console.preview_park({'speed_percent': 75})
        self.console.session.preview.assert_called_with('movej', q=[0.] * 7, speed_scale=.75)

    async def test_movel_uses_the_same_kernel_velocity_scale(self):
        await self.console.preview_xyz({'x_mm': 0, 'y_mm': 0, 'z_mm': 10, 'speed_percent': 50})
        args, kwargs = self.console.session.preview.call_args
        self.assertEqual(args, ('movel',))
        self.assertNotIn('speed', kwargs)
        self.assertEqual(kwargs['speed_scale'], .5)

    async def test_joint_selection_uses_model_dimensions(self):
        self.assertEqual(joint_index({'joint': 7}, 7), 6)
        with self.assertRaisesRegex(ValueError, '1..4'):
            joint_index({'joint': 5}, 4)
        self.console.joint_motion = AsyncMock()
        await self.console.move_joint({'joint': 7, 'target_deg': 90}, action='move_joint_target')
        self.assertAlmostEqual(self.console.joint_motion.call_args.args[0][6], np.pi / 2)

    async def test_gripper_drag_and_opening_follow_capabilities(self):
        await self.console.start_stream('gripper', 1, {'speed_percent': 25, 'minimum_mm': 100})
        self.assertEqual(self.console.stream['rate'], .05)
        self.assertEqual(self.console.stream['minimum'], .1)
        self.console.stream = None
        await self.console.start_stream('teleoperation', 2, {})
        self.assertEqual(self.console.stream['open'], .12)

    async def test_invalid_description_releases_new_lease(self):
        self.console.session = None
        self.console.run = Path('/unused')
        session = Mock()
        session.__enter__ = Mock()
        session.describe.return_value = {'robot': self.console.robot, 'motion_limits': {}}
        with patch('embodied.applications.console.kernel.arx.ControlSession', return_value=session):
            with self.assertRaises(KeyError):
                await self.console.connect({})
        session.close.assert_called_once()
        self.assertIsNone(self.console.session)

    def test_percent_is_a_scale_without_a_second_speed_floor(self):
        self.assertEqual(speed({'speed_percent': 1}), (1, .01))
        self.assertEqual(speed({'speed_percent': 100}), (100, 1.))
        for value in (True, 0, 101):
            with self.subTest(value=value), self.assertRaises(ValueError):
                speed({'speed_percent': value})


if __name__ == '__main__':
    unittest.main()
