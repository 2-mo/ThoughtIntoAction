"""Waiting panes and web-service ownership, without devices or real child processes."""
import builtins
import io
import os
from pathlib import Path
import signal
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from embodied.applications.workstation import bench, bench_panels as panels


class BenchPanelTests(unittest.TestCase):
    def test_monitor_opens_once_after_listening_and_ssh_keeps_url(self):
        with patch.object(panels, 'service_address', return_value=(18081, 'http://127.0.0.1:18081')), \
             patch.object(panels, 'service_host', return_value='127.0.0.1'), \
             patch.object(panels, 'listening', return_value=False) as listening, \
             patch.object(panels.shutil, 'which', return_value='/usr/bin/xdg-open'), \
             patch.object(panels.subprocess, 'Popen') as spawn, \
             patch.dict(os.environ, {'DISPLAY': ':0', 'SSH_CONNECTION': ''}):
            service = panels.Service('web')
            service.open_when_ready()
            spawn.assert_not_called()
            listening.return_value = True
            service.open_when_ready()
            service.open_when_ready()
            spawn.assert_called_once()
            self.assertEqual(spawn.call_args.args[0], ['/usr/bin/xdg-open', 'http://127.0.0.1:18081'])
            self.assertTrue(spawn.call_args.kwargs['start_new_session'])
            spawn.reset_mock()
            with patch.dict(os.environ, {'SSH_CONNECTION': 'client 1 server 22'}):
                message = panels.open_browser()
                spawn.assert_not_called()
                self.assertIn('http://127.0.0.1:18081', message)

    def test_default_web_is_the_kernel_collection_monitor(self):
        command = panels.SERVICE_COMMANDS['web']
        self.assertEqual(command[:3], ['./run.sh', 'web', 'monitor'])
        self.assertIn('--kernel', command)
        self.assertIn('--no-camera', command)
        self.assertIn('--no-connections', command)
        self.assertEqual(command[command.index('--mode') + 1], 'real')
        with patch.object(panels, 'load_config', return_value={'viewer_service': {'port': 18089}}), \
             patch.dict(os.environ, {'SSH_CONNECTION': ''}):
            self.assertEqual(panels.service_address('web'), (18089, 'http://127.0.0.1:18089'))
        with patch.object(panels, 'load_config', return_value={'viewer_service': {'port': 18081}}), \
             patch.dict(os.environ, {'SSH_CONNECTION': '100.117.88.16 12345 100.112.78.10 22'}):
            self.assertEqual(panels.service_address('web'), (18081, 'http://100.112.78.10:18081'))
            self.assertEqual(panels.service_command('web')[-2:], ['--host', '100.112.78.10'])

    def test_missing_devices_keep_kernel_vr_and_camera_waiting_without_sdk(self):
        config = {
            'robot': {'can_port': 'legacy-can', 'can_device': '/missing/adapter'},
            'camera_service': {'socket': '/missing/camera.sock'},
            'cameras': {'wrist_rgb': {'enabled': True, 'serial': 'missing-camera'}},
        }
        original_import = builtins.__import__

        def without_sdk(name, *args, **kwargs):
            if name.split('.')[0] in {'arx5_interface', 'pyrealsense2', 'xrobotoolkit_sdk', 'mujoco'}:
                self.fail(f'waiting panel attempted to import device SDK: {name}')
            return original_import(name, *args, **kwargs)

        with patch.object(bench, 'current_run', return_value=None), \
             patch.object(panels, 'load_config', return_value=config), \
             patch.object(panels, 'load_robot_profile', return_value={'robot': {'can_interface': 'missing-can'}}), \
             patch.object(panels, 'resolve_path', side_effect=Path), \
             patch.object(Path, 'exists', return_value=False), \
             patch.object(Path, 'is_char_device', return_value=False), \
             patch.object(Path, 'glob', return_value=[]), \
             patch.object(panels.subprocess, 'Popen') as spawn, \
             patch.object(panels.subprocess, 'run') as command, \
             patch('builtins.__import__', side_effect=without_sdk):
            states = {role: panels.snapshot(role) for role in ('kernel', 'vr', 'camera')}
            spawn.assert_not_called()
            command.assert_not_called()
        for role, (title, rows, hints) in states.items():
            with self.subTest(role=role):
                self.assertTrue(title)
                self.assertIn('等待', str(rows))
                self.assertTrue(hints)
        self.assertIn('未通电', str(states['kernel'][2]))
        self.assertIn('missing-can', dict(states['kernel'][1])['配置'])
        self.assertNotIn('legacy-can', dict(states['kernel'][1])['配置'])
        self.assertIn('USB', str(states['vr'][2]))
        self.assertIn('等待 USB 相机连接', str(states['camera'][1]))

    def test_missing_robot_interface_reports_configuration_without_guessing(self):
        with patch.object(bench, 'current_run', return_value=None), \
             patch.object(panels, 'load_config', return_value={'robot': {'can_port': 'legacy-can'}}), \
             patch.object(panels, 'load_robot_profile', return_value={'robot': {}}):
            _, rows, hints = panels.snapshot('kernel')
        self.assertIn('未配置 can_interface', str(rows))
        self.assertTrue(hints)

    def test_failed_service_keeps_logs_and_can_be_retried(self):
        failed = Mock(pid=10101, returncode=None)
        running = Mock(pid=20202, returncode=None)
        children = iter([failed, running])
        statuses = {failed.pid: 2, running.pid: None}

        def observe_child(kind, pid, options):
            self.assertEqual(kind, os.P_PID)
            self.assertTrue(options & os.WNOWAIT)
            code = statuses[pid]
            return None if code is None else SimpleNamespace(si_code=os.CLD_EXITED, si_status=code)

        def signal_child(pid, sig):
            if statuses[pid] is None:
                statuses[pid] = 0

        def launch(*args, **kwargs):
            kwargs['stdout'].write(b'web startup diagnostic\n')
            self.assertTrue(kwargs['start_new_session'])
            return next(children)

        with tempfile.TemporaryDirectory() as root, \
             patch.object(panels, 'ROOT', Path(root)), \
             patch.object(panels, 'listening', return_value=False), \
             patch.object(panels, 'service_address', return_value=(8000, 'http://127.0.0.1:8000')), \
             patch.object(panels.subprocess, 'Popen', side_effect=launch) as spawn, \
             patch.object(panels.os, 'waitid', side_effect=observe_child), \
             patch.object(panels.os, 'killpg', side_effect=signal_child):
            service = panels.Service('web')
            service.start()
            rows = dict(service.snapshot()[1])
            self.assertIn('已退出（2）', rows['服务'])
            self.assertIn('R 重试', rows['服务'])
            self.assertTrue(Path(rows['日志']).is_file())
            self.assertEqual(service.logs(), ['web startup diagnostic'])
            service.start()
            self.assertEqual(spawn.call_count, 2)
            self.assertIs(service.process, running)
            self.assertIn('运行中', dict(service.snapshot()[1])['服务'])
            service.stop()
            failed.poll.assert_not_called()
            running.poll.assert_not_called()

    def test_occupied_port_is_observed_without_spawning_or_stopping_its_owner(self):
        with patch.object(panels, 'listening', return_value=True), \
             patch.object(panels, 'service_address', return_value=(8090, 'http://127.0.0.1:8090/apps/site/')), \
             patch.object(panels.subprocess, 'Popen') as spawn, \
             patch.object(panels.os, 'killpg') as signal_group:
            service = panels.Service('site')
            service.start()
            self.assertTrue(service.external)
            self.assertIn('已有端口监听', dict(service.snapshot()[1])['服务'])
            service.stop()
            spawn.assert_not_called()
            signal_group.assert_not_called()

    def test_noninteractive_serve_is_only_a_snapshot(self):
        with patch.object(panels.sys.stdin, 'isatty', return_value=False), \
             patch.object(panels, 'listening', return_value=False), \
             patch.object(panels, 'service_address', return_value=(8000, 'http://127.0.0.1:8000')), \
             patch.object(panels.subprocess, 'Popen') as spawn, \
             patch.object(panels.Service, 'start') as start, \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(panels.main(['web', '--serve']), 0)
            spawn.assert_not_called()
            start.assert_not_called()
        self.assertIn('http://127.0.0.1:8000', output.getvalue())
        self.assertIn('尚未监听', output.getvalue())
        self.assertNotIn('\x1b[', output.getvalue())

    def test_cleanup_signals_only_the_group_created_by_this_panel(self):
        child = Mock(pid=31415, returncode=None)
        events = []
        child.wait.side_effect = lambda **kwargs: events.append('reaped') or 0
        exited = SimpleNamespace(si_code=os.CLD_EXITED, si_status=0)
        with tempfile.TemporaryDirectory() as root, \
             patch.object(panels, 'ROOT', Path(root)), \
             patch.object(panels, 'listening', return_value=False), \
             patch.object(panels, 'service_address', return_value=(8000, 'http://127.0.0.1:8000')), \
             patch.object(panels.subprocess, 'Popen', return_value=child) as spawn, \
             patch.object(panels.os, 'waitid', return_value=exited), \
             patch.object(panels.os, 'killpg', side_effect=lambda *args: events.append(args)) as signal_group:
            service = panels.Service('web')
            service.stop()
            signal_group.assert_not_called()
            service.start()
            self.assertTrue(spawn.call_args.kwargs['start_new_session'])
            service.stop()
            self.assertEqual([call.args for call in signal_group.call_args_list],
                             [(child.pid, signal.SIGTERM), (child.pid, signal.SIGKILL)])
            self.assertEqual(events, [(child.pid, signal.SIGTERM), (child.pid, signal.SIGKILL), 'reaped'])
            child.poll.assert_not_called()
            self.assertIsNone(service.process)
            service.stop()
            self.assertEqual(signal_group.call_count, 2)

            # If somebody already reaped the leader, its PID could now name an
            # unrelated process: cleanup must discard the handle without signals.
            signal_group.reset_mock()
            service.start()
            with patch.object(panels.os, 'waitid', side_effect=ChildProcessError):
                service.stop()
            signal_group.assert_not_called()
            self.assertEqual(child.wait.call_count, 1)
            self.assertIsNone(service.process)
            self.assertIn('未向旧 PID 发送信号', service.message)


if __name__ == '__main__':
    unittest.main()
