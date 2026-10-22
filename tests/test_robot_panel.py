"""Preset controls: no real drivers, live kernel, or hardware motion."""
from pathlib import Path
from types import SimpleNamespace
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from embodied.applications.workstation import bench, robot_panel
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS


class RobotPanelTests(unittest.TestCase):
    def setUp(self):
        self.found = (Path('/test/runtime'), {'managed': True, 'plugins': {'robot': 'arx.real'}})
        self.state = {'owner': None, 'stopped': False, 'q': [0.] * 6,
                      'dq': [0.] * 6, 'operations': []}
        self.robot = Mock()
        self.robot.__enter__ = Mock(return_value=self.robot)
        self.robot.state = self.state
        self.robot.describe.return_value = {'robot': {'park_q': [.1] * 6}}
        self.robot.movej.return_value = SimpleNamespace(operation_id='test-op', metrics={
            'duration_s': 1.9, 'peak_velocity_rad_s': 1.5})
        self.connect = self.enterContext(patch.object(robot_panel.Robot, 'connect', return_value=self.robot))
        client = self.enterContext(patch.object(robot_panel, 'ControlClient'))
        client.return_value.status.return_value = self.state
        self.controls = robot_panel.RobotControls()
        self.addCleanup(self.controls.close)

    def test_open_and_poll_never_acquire_control(self):
        self.controls.rows(self.found)
        self.controls.press('r', self.found)
        self.connect.assert_not_called()
        self.robot.movej.assert_not_called()

    def test_p_and_h_use_known_targets_and_backend_speed_without_extra_caps(self):
        for key, target in [('p', list(DEFAULT_GRASP_READY_JOINTS)), ('h', [.1] * 6)]:
            with self.subTest(key=key):
                self.controls.press(key, self.found)
                self.robot.movej.assert_called_with(target, speed=0, wait=False)
                self.controls.close()

    def test_repeat_keys_never_queue_and_space_cancels_only_owned_motion(self):
        self.controls.press(' ', self.found)
        self.connect.assert_not_called()
        self.controls.press('p', self.found)
        self.controls.press('h', self.found)
        self.robot.movej.assert_called_once()
        self.controls.press(' ', self.found)
        self.robot.cancel.assert_called_once()
        self.robot.close.assert_called_once()
        self.assertIsNone(self.controls.motion)

    def test_kernel_settling_failure_is_preserved_and_releases_control(self):
        self.controls.press('p', self.found)
        self.state['operations'] = [{'operation_id': 'test-op', 'status': 'failed',
                                     'reason': 'Robot did not settle'}]
        rows = dict(self.controls.rows(self.found))
        self.assertEqual(rows['操作结果'].level, 'error')
        self.assertIn('未满足到位条件', str(rows['操作结果']))
        self.robot.close.assert_called_once()
        self.robot.cancel.assert_not_called()
        self.assertIsNone(self.controls.motion)

    def test_occupied_or_unknown_robot_never_receives_motion(self):
        self.state['owner'] = {'source': 'another-controller'}
        self.controls.press('h', self.found)
        self.robot.movej.assert_not_called()
        self.assertEqual(self.controls.message.level, 'error')
        self.state['owner'] = None
        self.controls.press('p', (self.found[0], {'managed': True, 'plugins': {'robot': 'other.real'}}))
        self.robot.movej.assert_not_called()

    def test_close_releases_even_when_cancel_fails(self):
        self.controls.press('p', self.found)
        self.robot.cancel.side_effect = RuntimeError('connection lost')
        with self.assertRaisesRegex(RuntimeError, 'connection lost'):
            self.controls.close()
        self.robot.close.assert_called_once()

    def test_attaching_ui_never_signals_or_respawns_kernel_pane(self):
        calls = []
        def tmux(*args, **kwargs):
            calls.append(args)
            return subprocess.CompletedProcess(args, 0, stdout='%ui\n' if args[0] == 'split-window' else 'old log\n')
        with tempfile.TemporaryDirectory() as root, \
             patch.object(bench, 'ROOT', Path(root)), \
             patch.object(bench, 'window', side_effect=lambda role: ('%engine', 'bash') if role == 'kernel' else None), \
             patch.object(bench, 'tmux', side_effect=tmux), \
             patch.object(bench, 'panel_command', return_value='test-only-panel'):
            self.assertEqual(bench.ensure_robot_controls(), '%ui')
        self.assertFalse(any(args[0] in ('send-keys', 'respawn-pane', 'kill-pane') for args in calls))
        self.assertIn(('set-option', '-p', '-t', '%ui', '@bench_role', 'robot-controls'), calls)


if __name__ == '__main__':
    unittest.main()
