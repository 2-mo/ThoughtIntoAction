"""Console commands must preserve service isolation and motion cancellation."""
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import signal

from robot_control.console import Console, fit, recovery_command


class ArmConsoleTests(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(socket='/tmp/custom-arm.sock', urdf='/tmp/custom.urdf')
        self.console = Console(self.args, False, io.StringIO())
        self.console.service = Mock()
        self.console.service.poll.return_value = None
        self.console.ready.set()

    def test_custom_service_and_virtual_flag_forwarded(self):
        cmd = recovery_command('pregrasp', self.args, False)
        self.assertEqual(cmd[-6:], ['--socket', self.args.socket, '--urdf', self.args.urdf, '--virtual', '--pregrasp'])
        self.assertNotIn('--virtual', recovery_command('home', self.args, True))
        self.assertNotIn('--stop', recovery_command('stop', self.args, False))

    def test_busy_motion_rejects_new_motion_and_only_cancels_once(self):
        action = self.console.action = Mock()
        action.poll.return_value = None
        with patch.object(self.console, 'launch') as launch:
            self.console.command('pregrasp')
            self.console.command('damping')
            launch.assert_not_called()
            self.console.command('stop')
            self.console.command('stop')
        action.send_signal.assert_called_once_with(signal.SIGINT)
        self.console.service.send_signal.assert_not_called()

    def test_damping_shortcut_uses_selected_service(self):
        with patch.object(self.console, 'launch') as launch:
            self.console.command('damping')
        launch.assert_called_once_with(recovery_command('damping', self.args, False))

    def test_discovery_shortcuts_do_not_use_arm_recovery(self):
        for command, action in [('discovery-on', 'on'), ('discovery-off', 'off')]:
            cmd = recovery_command(command, self.args, True)
            self.assertEqual(cmd[-2:], ['quest-discovery', action])
            self.assertNotIn('--socket', cmd)
            self.assertNotIn('--home', cmd)

    def test_no_commands_before_ready_or_after_service_exit(self):
        with patch.object(self.console, 'launch') as launch:
            self.console.ready.clear()
            self.console.command('home')
            self.console.ready.set()
            self.console.service.poll.return_value = 1
            self.console.command('home')
            launch.assert_not_called()

    def test_shutdown_waits_for_recovery_before_stopping_service(self):
        action = self.console.action = Mock()
        action.poll.return_value = None
        calls = Mock()
        calls.attach_mock(action, 'action')
        calls.attach_mock(self.console.service, 'service')
        self.console.close()
        names = [call[0] for call in calls.mock_calls]
        self.assertLess(names.index('action.wait'), names.index('service.terminate'))

    def test_chinese_clipping(self):
        self.assertEqual(fit('H 回零 P', 6), 'H 回零')
        self.assertEqual(fit('abc', 0), '')


if __name__ == '__main__':
    unittest.main()
