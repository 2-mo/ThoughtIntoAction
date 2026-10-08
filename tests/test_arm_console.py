"""Console commands must preserve service isolation and motion cancellation."""
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import signal

from robot_control.console import Console, fit, recovery_command
from robot_control.startup import WAITING_FOR_ARM, NO_MOTOR_FEEDBACK, waiting_for_arm


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

    def test_viewer_commands_do_not_touch_arm_recovery(self):
        for command, action in [('viewer-start','start'), ('viewer-stop','stop'), ('camera-off','camera-off')]:
            cmd = recovery_command(command, self.args, True)
            self.assertEqual(cmd[-2:], ['viewer-control', action])
            self.assertNotIn('--socket', cmd)

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

    def test_rich_screen_shows_crash_recovery_without_terminal_or_hardware(self):
        from rich.console import Console as RichConsole
        output = io.StringIO()
        terminal = RichConsole(file=output, width=120, height=26)
        live = Mock()
        self.console.service.poll.return_value = -signal.SIGABRT
        with patch('robot_control.console.read_key', return_value='q'):
            self.console.render_loop(terminal, live)
        terminal.print(live.update.call_args.args[0])
        rendered = output.getvalue()
        self.assertIn('X5 机械臂控制台', rendered)
        self.assertIn('SDK 异常退出', rendered)
        self.assertIn('R 重连', rendered)

    def test_missing_power_retries_only_before_first_ready(self):
        self.console.hardware = True
        self.console.ready.clear()
        self.console.service_argv = ['service']
        self.console.service.poll.return_value = WAITING_FOR_ARM
        with patch('robot_control.console.time.monotonic', side_effect=[10, 14, 15]), \
             patch.object(self.console, 'launch') as launch:
            self.assertTrue(self.console.retry_startup())
            self.assertTrue(self.console.retry_startup())
            launch.assert_not_called()
            self.assertTrue(self.console.retry_startup())
            launch.assert_called_once_with(['service'], service=True)
        self.console.ready.set()
        self.console.service.poll.return_value = WAITING_FOR_ARM
        with patch.object(self.console, 'launch') as launch:
            self.assertFalse(self.console.retry_startup())
            launch.assert_not_called()

    def test_config_error_is_not_a_power_wait(self):
        self.assertTrue(waiting_for_arm(RuntimeError(NO_MOTOR_FEEDBACK), service_started=False))
        self.assertFalse(waiting_for_arm(RuntimeError(NO_MOTOR_FEEDBACK), service_started=True))
        self.assertFalse(waiting_for_arm(RuntimeError('SDK missing'), service_started=False))
        self.console.hardware = True
        self.console.ready.clear()
        self.console.service_argv = ['service']
        self.console.service.poll.return_value = 1
        with patch.object(self.console, 'launch') as launch:
            self.assertFalse(self.console.retry_startup())
            launch.assert_not_called()

    def test_restart_crashed_service_reuses_settings_and_clears_old_state(self):
        self.console.service.poll.return_value = -signal.SIGABRT
        self.console.service_argv = ['service', '--socket', self.args.socket, '--no-interactive']
        self.console.retry_at = 123
        self.console.service_reader = Mock()
        self.console.service_reader.is_alive.return_value = False
        with patch.object(self.console, 'launch') as launch:
            self.assertTrue(self.console.restart_service())
            launch.assert_called_once_with(self.console.service_argv, service=True)
        self.console.service_reader.join.assert_called_once_with(timeout=1)
        self.assertFalse(self.console.ready.is_set())
        self.assertIsNone(self.console.retry_at)
        self.assertEqual(self.console.activity, '正在启动服务…')

    def test_restart_never_kills_live_service_or_overlaps_unfinished_action(self):
        self.console.service_argv = ['service']
        with patch.object(self.console, 'launch') as launch:
            self.assertFalse(self.console.restart_service())
            self.console.service.poll.return_value = -signal.SIGABRT
            self.console.action = Mock()
            self.console.action.poll.return_value = None
            self.assertFalse(self.console.restart_service())
            launch.assert_not_called()
        self.console.service.terminate.assert_not_called()


if __name__ == '__main__':
    unittest.main()
