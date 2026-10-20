"""MuJoCo workbench collection reads only the left controller (todo K-04)."""
import unittest
from unittest.mock import patch

from embodied.monitor.session import BUTTON_LAYOUT, SimulationSession


class SimulationButtonTests(unittest.TestCase):
    def setUp(self):
        self.session = SimulationSession(simulation=None)
        self.calls = []
        self.session.move = lambda home=False, record_after=False: self.calls.append(('move', home, record_after))
        self.session.save = lambda: self.calls.append(('save',))
        self.session.discard_current = lambda: self.calls.append(('discard',))
        self.clock = 0.
        self.stamp = 0

    def frame(self, seconds, *, left=None, right=None, enabled=True):
        """Hold the given buttons for `seconds`, then release, in 20 ms frames."""
        for pressed in (True, False):
            end = self.clock + (seconds if pressed else .1)
            while self.clock < end:
                self.clock += .02
                self.stamp += 1
                hands = {'left': {'buttons': (left or {}) if pressed else {}, 'grip': 0, 'axes': [0, 0]},
                         'right': {'buttons': (right or {}) if pressed else {}, 'grip': 0, 'axes': [0, 0]}}
                with patch('embodied.monitor.session.time.monotonic', return_value=self.clock):
                    self.session.update({'device_timestamp_ns': self.stamp, 'tracking': True, 'hands': hands}, enabled)

    def stick(self, x, y):
        for axes in ((x, y), (0, 0)):
            self.clock += .02
            self.stamp += 1
            hands = {'left': {'buttons': {}, 'grip': 0, 'axes': list(axes)}, 'right': {'buttons': {}, 'grip': 0, 'axes': [0, 0]}}
            with patch('embodied.monitor.session.time.monotonic', return_value=self.clock):
                self.session.update({'device_timestamp_ns': self.stamp, 'tracking': True, 'hands': hands}, True)

    def test_right_controller_buttons_never_drive_collection(self):
        self.stick(-1, 0)
        for buttons in ({'A': True}, {'B': True}, {'A': True, 'B': True}, {'R3': True}):
            self.frame(1.2, right=buttons)
        self.assertEqual(self.calls, [])

    def test_left_controller_layout(self):
        self.frame(.1, left={'X': True})
        self.assertEqual(self.calls, [], 'X needs an active task')
        self.stick(-1, 0)
        self.assertTrue(self.session.task_active)
        self.frame(.1, left={'X': True})
        self.frame(.1, left={'Y': True})
        self.frame(1.2, left={'Y': True})
        self.assertEqual(self.calls, [('move', False, True), ('move', False, False), ('discard',)])
        self.stick(1, 0)
        self.assertFalse(self.session.task_active)
        self.stick(-1, 0)
        self.frame(1.2, left={'L3': True})
        self.assertFalse(self.session.task_active)
        self.assertEqual(self.session.count, 0)

    def test_disabled_teleop_ignores_collection_buttons(self):
        self.session.task_active = True
        self.frame(.1, left={'X': True}, enabled=False)
        self.assertEqual(self.calls, [])

    def test_layout_lists_only_mapped_keys(self):
        self.assertEqual(set(BUTTON_LAYOUT['left']['keys']), {'X', 'Y'})
        self.assertEqual(BUTTON_LAYOUT['left']['stick'], 'L3')
        self.assertEqual(BUTTON_LAYOUT['right']['holds'], [])


if __name__ == '__main__':
    unittest.main()
