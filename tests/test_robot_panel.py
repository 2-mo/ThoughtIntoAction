"""Preset controls: no real drivers, live kernel, or hardware motion."""
from pathlib import Path
from types import SimpleNamespace
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from embodied.applications.workstation import bench, robot_panel
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
from embodied.control.motion_limits import MotionLimits


class RobotPanelTests(unittest.TestCase):
    def setUp(self):
        self.found = (Path('/test/runtime'), {'backend': 'real', 'managed': True, 'plugins': {'robot': 'arx.real'}})
        self.state = {'owner': None, 'stopped': False, 'q': [0.] * 6,
                      'dq': [0.] * 6, 'operations': [], 'gripper_width': .02}
        self.robot = Mock()
        self.robot.__enter__ = Mock(return_value=self.robot)
        self.robot.state = self.state
        self.robot.describe.return_value = {'robot': {'park_q': [.1] * 6}}
        self.robot.movej.return_value = SimpleNamespace(operation_id='test-op', metrics={
            'duration_s': 1.9, 'peak_velocity_rad_s': 1.5})
        self.session = Mock()
        self.session.parameters = {'effective_robot_config': {'gripper_max_width': .082, 'gripper_velocity_max_m_s': .04}}
        self.session.preview.return_value = {'plan_id': 'home', 'metrics': {'duration_s': 1., 'peak_velocity_rad_s': 1.5}}
        self.session.execute.return_value = {'operation_id': 'home'}
        self.robot.home.side_effect = lambda **_: robot_panel.ReleaseThenHome(self.session)
        self.connect = self.enterContext(patch.object(robot_panel.Robot, 'connect', return_value=self.robot))
        client = self.enterContext(patch.object(robot_panel, 'ControlClient'))
        client.return_value.status.return_value = self.state
        client.return_value.describe.return_value = {'motion_limits': MotionLimits.load().describe()}
        self.client = client.return_value
        self.controls = robot_panel.RobotControls()
        self.addCleanup(self.controls.close)

    def test_open_and_poll_never_acquire_control(self):
        self.controls.rows(self.found)
        self.controls.press('r', self.found)
        self.connect.assert_not_called()
        self.robot.movej.assert_not_called()

    def test_restart_socket_error_clears_on_feedback_recovery_without_erasing_action_result(self):
        self.controls.message = robot_panel.Status('动作未通过：未满足到位条件', 'error')
        self.client.status.side_effect = FileNotFoundError(2, 'No such file or directory')
        rows = dict(self.controls.rows(self.found))
        self.assertIn('No such file', str(rows['反馈查询']))
        self.assertIn('动作未通过', str(rows['操作结果']))
        self.client.status.side_effect = None
        rows = dict(self.controls.rows(self.found))
        self.assertNotIn('反馈查询', rows)
        self.assertIn('速度 rad/s', rows)
        self.assertIn('动作未通过', str(rows['操作结果']))
        self.connect.assert_not_called()

    def test_interrupted_motion_remains_unconfirmed_after_feedback_recovers(self):
        self.controls.press('p', self.found)
        self.client.status.side_effect = FileNotFoundError(2, 'No such file or directory')
        self.robot.cancel.side_effect = RuntimeError('old socket gone')
        rows = dict(self.controls.rows(self.found))
        self.assertIn('结束控制未确认', str(rows['反馈查询']))
        self.assertIn('未确认到位', str(rows['操作结果']))
        self.robot.close.assert_called_once()
        self.client.status.side_effect = None
        rows = dict(self.controls.rows(self.found))
        self.assertNotIn('反馈查询', rows)
        self.assertIn('未确认到位', str(rows['操作结果']))

    def test_new_runtime_does_not_show_old_parameters_when_description_is_unavailable(self):
        self.controls.rows(self.found)
        self.client.describe.side_effect = FileNotFoundError(2, 'No such file or directory')
        rows = dict(self.controls.rows((Path('/test/new-runtime'), self.found[1])))
        self.assertIn('反馈查询', rows)
        self.assertNotIn('生效参数', rows)

    def test_p_and_h_use_known_targets_and_kernel_defaults_without_extra_caps(self):
        for key, target in [('p', list(DEFAULT_GRASP_READY_JOINTS)), ('h', [.1] * 6)]:
            with self.subTest(key=key):
                self.controls.press(key, self.found)
                if key == 'p':
                    self.robot.movej.assert_called_with(target, wait=False)
                else:
                    self.robot.home.assert_called_once_with(release_gripper=True, wait=False)
                self.controls.close()

    def test_h_waits_for_gripper_feedback_and_space_can_cancel_opening(self):
        self.controls.press('h', self.found)
        self.controls.rows(self.found)
        self.session.command.assert_called_with('gripper', gripper_width=.082, ttl_ms=200)
        self.session.execute.assert_not_called()
        self.controls.press(' ', self.found)
        self.controls.rows(self.found)
        self.session.execute.assert_not_called()
        self.robot.cancel.assert_called_once()

    def test_h_only_moves_after_opening_and_keeps_jaws_open_in_home_plan(self):
        self.controls.press('h', self.found)
        self.controls.rows(self.found)
        self.state['gripper_width'] = .081
        rows = dict(self.controls.rows(self.found))
        self.session.preview.assert_called_once_with('home', gripper_width=.082)
        self.assertIn('夹爪已张开', str(rows['操作结果']))
        self.state['operations'] = [{'operation_id': 'home', 'status': 'completed'}]
        self.controls.rows(self.found)
        self.robot.cancel.assert_not_called()
        self.robot.close.assert_called_once()

    def test_h_blocked_gripper_preserves_the_failure_and_never_moves_arm(self):
        self.controls.press('h', self.found)
        self.controls.motion.open_deadline = 0
        rows = dict(self.controls.rows(self.found))
        self.assertIn('夹爪未能张开', str(rows['操作结果']))
        self.session.execute.assert_not_called()
        self.robot.cancel.assert_called_once()

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
        self.assertIn('J4', str(rows['操作结果']))
        self.assertIn('1.00°', str(rows['操作结果']))
        self.robot.close.assert_called_once()
        self.robot.cancel.assert_not_called()
        self.assertIsNone(self.controls.motion)

    def test_completed_result_keeps_recorded_error_and_runtime_tolerance(self):
        self.controls.press('p', self.found)
        self.state['operations'] = [{'operation_id': 'test-op', 'status': 'completed',
            'settling': {'worst_joint': 4, 'max_joint_error_rad': .005711059570312482,
                         'position_tolerance_rad': .017453292519943295}}]
        rows = dict(self.controls.rows(self.found))
        self.assertEqual(rows['操作结果'].level, 'ready')
        self.assertIn('J4 最大偏差 0.33° / 容差 1.00°', str(rows['操作结果']))
        self.assertIn('每轴误差 < 1.00°', rows['到位判定'])
        self.robot.cancel.assert_not_called()

    def test_damping_requires_two_explicit_keys_and_owned_real_runtime(self):
        with patch.object(bench, 'owned_run', return_value=self.found), patch.object(bench, 'stop_kernel') as stop:
            self.controls.press('d', self.found)
            stop.assert_not_called()
            self.controls.press('r', self.found)  # Any other key disarms.
            self.controls.press('d', self.found)
            stop.assert_not_called()
            self.controls.press('d', self.found)
            stop.assert_called_once()
            self.connect.assert_not_called()
        with patch.object(bench, 'owned_run', return_value=None), patch.object(bench, 'stop_kernel') as stop:
            self.controls.press('d', self.found)
            self.controls.press('d', self.found)
            stop.assert_not_called()

    def test_damping_confirmation_expires_and_sim_cannot_shutdown(self):
        with patch.object(bench, 'owned_run', return_value=self.found), patch.object(bench, 'stop_kernel') as stop:
            self.controls.press('d', self.found)
            self.controls.damping_armed_until = 1.
            self.controls.rows(self.found)
            self.controls.press('d', self.found)
            stop.assert_not_called()
            sim = (self.found[0], {**self.found[1], 'backend': 'sim'})
            self.controls.press('d', sim)
            stop.assert_not_called()

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
