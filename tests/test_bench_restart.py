"""Restart ownership, shutdown ordering and failure boundaries without hardware."""
from contextlib import ExitStack
from pathlib import Path
import subprocess
import unittest
from unittest.mock import Mock, patch

from embodied.applications.workstation import bench, restart


class BenchRestartTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(bench, 'exists', return_value=True))
        self.stack.enter_context(patch.object(bench, 'check_session_owner'))

    def robot_context(self, *, command=None, owned=True):
        active = (Path('/test/run'), {'backend': 'real', 'managed': True})
        self.owned = self.stack.enter_context(patch.object(bench, 'owned_run', return_value=active if owned else None))
        self.stack.enter_context(patch.object(bench, 'window', return_value=('%robot', 'python')))
        self.stop = self.stack.enter_context(patch.object(bench, 'stop_kernel'))
        self.start = self.stack.enter_context(patch.object(bench, 'start_kernel'))
        self.busy = self.stack.enter_context(patch.object(bench, 'kernel_busy', return_value=False))
        self.stack.enter_context(patch.object(bench, 'tmux', return_value=subprocess.CompletedProcess(
            [], 0, stdout=(bench.KERNELS[2][2] if command is None else command))))
        self.guard = Mock()
        self.stack.enter_context(patch('embodied.devices.sensors.control.recording_guard', return_value=self.guard))
        self.stack.enter_context(patch('embodied.devices.sensors.control.check_legacy_session'))
        self.activity = self.stack.enter_context(patch.object(restart, '_check_robot_activity'))
        return active

    def test_robot_preserves_external_kernel(self):
        self.robot_context(owned=False)
        with self.assertRaisesRegex(RuntimeError, '不是本工作台'):
            restart.restart('robot', robot_confirmed=True)
        self.stop.assert_not_called()
        self.start.assert_not_called()

    def test_robot_requires_confirmation_and_a_recorded_known_command(self):
        self.robot_context(command='')
        with self.assertRaisesRegex(RuntimeError, '需要确认'):
            restart.restart('robot')
        for command in ('', 'echo fake; bash scripts/arm/kernel_real.sh'):
            with patch.object(bench, 'tmux', return_value=subprocess.CompletedProcess([], 0, stdout=command)):
                with self.assertRaisesRegex(RuntimeError, '未记录'):
                    restart.restart('robot', robot_confirmed=True)
        self.stop.assert_not_called()
        self.start.assert_not_called()

    def test_robot_waits_for_exit_and_avoids_repeated_confirmation(self):
        active = self.robot_context(command=bench.KERNELS[2][2] + ' --yes')
        self.owned.side_effect = [active, active, active, None]
        with patch.object(restart.time, 'sleep') as sleep:
            restart.restart('robot', robot_confirmed=True)
        self.stop.assert_called_once()
        sleep.assert_called_once_with(.1)
        self.start.assert_called_once_with(bench.KERNELS[2][2] + ' --yes')
        self.guard.close.assert_called_once()

    def test_robot_exit_timeout_never_starts_or_force_kills(self):
        self.robot_context()
        with patch.object(restart.time, 'monotonic', side_effect=[0., 31.]):
            with self.assertRaisesRegex(RuntimeError, '保留现场'):
                restart.restart('robot', robot_confirmed=True)
        self.stop.assert_called_once()
        self.start.assert_not_called()
        self.guard.close.assert_called_once()

    def test_robot_activity_failure_preserves_running_robot(self):
        self.robot_context()
        self.activity.side_effect = RuntimeError('正在采集')
        with self.assertRaisesRegex(RuntimeError, '正在采集'):
            restart.restart('robot', robot_confirmed=True)
        self.stop.assert_not_called()
        self.start.assert_not_called()
        self.guard.close.assert_called_once()

    def test_robot_instance_change_is_not_stopped(self):
        active = self.robot_context()
        self.owned.side_effect = [active, (Path('/different/run'), active[1])]
        with self.assertRaisesRegex(RuntimeError, '实例已变化'):
            restart.restart('robot', robot_confirmed=True)
        self.stop.assert_not_called()

    def test_camera_starts_only_after_successful_stop(self):
        calls = Mock()
        with patch('embodied.devices.sensors.lifecycle.stop_service', calls.stop), \
             patch('embodied.devices.sensors.client.ensure_service', calls.start):
            restart.restart('camera')
            self.assertEqual([call[0] for call in calls.mock_calls], ['stop', 'start'])
            calls.reset_mock()
            calls.stop.side_effect = RuntimeError('正在录制')
            with self.assertRaisesRegex(RuntimeError, '正在录制'):
                restart.restart('camera')
            calls.start.assert_not_called()

    def test_interfaces_preserve_robot_command_camera_service_and_unrelated_tasks(self):
        panes = {role: ('%' + role, 'python') for role in bench.WINDOWS}
        panes['site'] = ('%site', '')
        commands = {role: 'python -m embodied.applications.workstation.bench_panels ' + role
                    for role in bench.WINDOWS}
        commands.update(home='python -m embodied.applications.workstation.bench _dashboard',
                        vr='python unrelated_task.py')

        def tmux(*args, **kwargs):
            value = commands[args[3][1:]] if args[0] == 'display-message' else ''
            return subprocess.CompletedProcess([], 0, stdout=value)

        with patch.object(bench, 'window', side_effect=panes.get) as window, \
             patch.object(bench, 'tmux', side_effect=tmux) as run, \
             patch.object(bench, 'refresh_panel') as refresh, \
             patch('embodied.devices.sensors.lifecycle.stop_service') as camera_stop, \
             patch('embodied.devices.sensors.client.ensure_service') as camera_start:
            message = restart.restart('interfaces')
        self.assertEqual([call.args[0] for call in refresh.call_args_list], ['%web', '%camera', '%phone', '%home'])
        self.assertFalse(any(call.args[0] in ('kernel', 'shell') for call in window.call_args_list))
        self.assertTrue(all('-k' not in call.args for call in run.call_args_list))
        run.assert_any_call('respawn-pane', '-t', '%site', '-c', str(bench.ROOT), bench.panel_command('site'))
        camera_stop.assert_not_called()
        camera_start.assert_not_called()
        self.assertIn('保留其它任务：VR', message)

    def test_activity_check_rejects_live_capture_and_external_rpc_owner(self):
        with patch('embodied.applications.monitor.kernel_view.collection_snapshot',
                   return_value={'recording': {'live': True}}), \
             patch('embodied.contracts.runtime.control.ControlClient') as client:
            with self.assertRaisesRegex(RuntimeError, '采集'):
                restart._check_robot_activity(Path('/test/run'), {'managed': True})
            client.assert_not_called()
        with patch('embodied.applications.monitor.kernel_view.collection_snapshot', return_value={}), \
             patch('embodied.contracts.runtime.control.ControlClient') as client:
            client.return_value.status.return_value = {'owner': {'source': 'policy', 'ingress': 'rpc'}}
            with self.assertRaisesRegex(RuntimeError, '控制任务'):
                restart._check_robot_activity(Path('/test/run'), {'managed': True})
            client.return_value.status.return_value = {'owner': {'source': 'quest', 'ingress': 'native'}}
            restart._check_robot_activity(Path('/test/run'), {'managed': True})


if __name__ == '__main__':
    unittest.main()
