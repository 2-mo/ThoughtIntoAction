"""Unified menu actions use existing lifecycle owners; never open hardware."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/workspace'))
import dashboard
w = dashboard.workspace


class DashboardTests(unittest.TestCase):
    def setUp(self):
        selection = patch.object(w, 'selected_mode', return_value='real')
        selection.start()
        self.addCleanup(selection.stop)

    def test_complete_shutdown_orders_all_services(self):
        events = []
        with patch.object(w, 'exists', return_value=True), \
             patch.object(w, 'stop_pane', side_effect=lambda role: events.append(role)), \
             patch.object(w, 'teleop_running', return_value=False), \
             patch.object(w, 'arm_info', return_value=None), \
             patch('embodied.monitor.control.command', side_effect=lambda action: events.append('viewer-service')), \
             patch('embodied.sensors.lifecycle.stop_service', side_effect=lambda: events.append('camera-service')):
            w.shutdown_services()
        self.assertEqual(events, ['teleop', 'arm', 'web', 'viewer-service', 'camera-service'])

    def test_complete_restart_does_not_start_when_shutdown_fails(self):
        with patch.object(w, 'shutdown_services', side_effect=RuntimeError('camera still stopping')), \
             patch.object(w, 'tmux') as tmux:
            with self.assertRaisesRegex(RuntimeError, 'still stopping'):
                w.restart_all()
        tmux.assert_not_called()

    def test_full_close_keeps_workbench_when_a_service_fails_to_stop(self):
        with patch.object(w, 'DeviceLease'), \
             patch.object(w, 'shutdown_services', side_effect=RuntimeError('still saving')), \
             patch.object(w, 'tmux') as tmux:
            with self.assertRaisesRegex(RuntimeError, 'saving'):
                dashboard.operate('z')
        tmux.assert_not_called()

    def test_stop_controls_waits_for_teleop_before_arm(self):
        with patch.object(w, 'DeviceLease'), patch.object(w, 'stop_pane') as stop:
            dashboard.operate('x')
        self.assertEqual([c.args for c in stop.call_args_list], [('teleop',), ('arm',)])

    def test_failed_teleop_stop_preserves_arm(self):
        with patch.object(w, 'DeviceLease'), patch.object(w, 'stop_pane', side_effect=RuntimeError('still saving')) as stop:
            with self.assertRaisesRegex(RuntimeError, 'saving'):
                dashboard.operate('x')
        stop.assert_called_once_with('teleop')

    def test_reconnect_uses_console_restart_key_without_killing_it(self):
        with patch.object(w, 'panes', return_value={'arm': ('%1', False, 'arm-console')}), \
             patch.object(w, 'arm_info', return_value=None), \
             patch.object(w, 'tmux', return_value=subprocess.CompletedProcess([], 0, stdout='1\n')) as tmux:
            w.start_role('arm')
        tmux.assert_any_call('send-keys', '-t', '%1', 'r')
        self.assertFalse(any(c.args[0] in ('respawn-pane', 'kill-pane') for c in tmux.call_args_list))

    def test_online_arm_is_reused(self):
        with patch.object(w, 'panes', return_value={'arm': ('%1', False, 'arm-console')}), \
             patch.object(w, 'arm_info', return_value={'state': {}}), patch.object(w, 'tmux') as tmux:
            w.start_role('arm')
        tmux.assert_not_called()

    def test_old_crashed_console_is_closed_before_replacement(self):
        events = []
        with patch.object(w, 'panes', return_value={'arm': ('%1', False, 'console')}), \
             patch.object(w, 'arm_info', return_value=None), \
             patch.object(w, 'old_arm_console_exited', return_value=True), \
             patch.object(w, 'pane_command', return_value='new console'), \
             patch.object(w, 'stop_pane', side_effect=lambda role: events.append('stop')), \
             patch.object(w, 'tmux', side_effect=lambda *args: events.append(args[0]) or subprocess.CompletedProcess([], 0, stdout='')):
            w.start_role('arm')
        self.assertLess(events.index('stop'), events.index('respawn-pane'))

    def test_waiting_pane_gets_no_command_keys(self):
        with patch.object(w, 'panes', return_value={'arm': ('%1', False, 'waiting')}), \
             patch.object(w, 'arm_info', return_value=None), patch.object(w, 'tmux') as tmux:
            w.start_role('arm')
            with self.assertRaisesRegex(RuntimeError, '未就绪'):
                dashboard.arm_key('h')
        tmux.assert_not_called()

    def test_arm_action_is_forwarded_to_existing_console(self):
        with patch.object(w, 'panes', return_value={'arm': ('%1', False, 'arm-console')}), \
             patch.object(w, 'arm_info', return_value={'state': {}}), patch.object(w, 'tmux') as tmux:
            dashboard.arm_key('p')
        tmux.assert_called_once_with('send-keys', '-t', '%1', 'p')


if __name__ == '__main__':
    unittest.main()
