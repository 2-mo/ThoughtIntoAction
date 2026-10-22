"""Waiting panes and web-service ownership, without devices or real child processes."""
import builtins
from concurrent.futures import Future
from contextlib import nullcontext
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from embodied.applications.workstation import bench, bench_panels as panels


class BenchPanelTests(unittest.TestCase):
    def test_transport_timeout_and_close_cancel_the_private_process_group(self):
        # These disposable Python children model a stalled CLI and its ADB child.
        # They never call adb, open devices, or touch a running workbench.
        script = ('import subprocess, sys, time; '
                  'subprocess.Popen([sys.executable, "-c", "import time; time.sleep(5)"]); '
                  'time.sleep(5)')
        for cancel in (False, True):
            with self.subTest(cancel=cancel):
                stop = threading.Event()
                timer = threading.Timer(.15, stop.set) if cancel else None
                if timer:
                    timer.start()
                started = time.monotonic()
                try:
                    with patch.object(panels.os, 'killpg', wraps=os.killpg) as kill, \
                         self.assertRaises(InterruptedError if cancel else subprocess.TimeoutExpired):
                        panels.transport_command([sys.executable, '-c', script], timeout=.3 if not cancel else 5, stop=stop)
                    self.assertEqual(kill.call_count, 1)
                    self.assertEqual(kill.call_args.args[1], signal.SIGKILL)
                    self.assertLess(time.monotonic() - started, 2)
                finally:
                    if timer:
                        timer.join()

    def test_vr_transport_and_robot_input_are_independent(self):
        transport = {'receiver_listening': True, 'usb_status': 'missing',
                     'reverse_configured': False, 'usb_connected': False, 'boot_completed': True}
        found = (Path('/test-run'), {'input': 'none'})
        rows = dict(panels.vr_rows(found, transport)[0])
        self.assertIn('未检测到', str(rows['VR']))
        self.assertEqual(rows['主机接收'].level, 'ready')
        self.assertEqual(rows['遥操接入'].level, 'disabled')
        for status, expected in [('unauthorized', '允许 USB 调试'), ('offline', '离线'), ('ready', '建立转发')]:
            transport['usb_status'] = status
            self.assertIn(expected, str(panels.vr_rows(found, transport)[0][0][1]))
        transport.update(reverse_configured=True, client_running=True)
        self.assertIn('等待头显 Connect', str(panels.vr_rows(found, transport)[0][0][1]))
        transport.update(usb_connected=True)
        self.assertEqual(panels.vr_rows(found, transport)[0][0][1].level, 'waiting')
        with patch.object(panels, 'vr_control_rows', return_value=([('遥操接入', panels.Status('跟踪就绪', 'ready'))], [])):
            self.assertEqual(panels.vr_rows(found, transport)[0][0][1].level, 'ready')
            transport['error'] = 'ADB query failed'
            self.assertEqual(panels.vr_rows(found, transport)[0][0][1].level, 'error')

    def test_vr_status_preserves_partial_evidence_and_bounds_probe_time(self):
        with patch.object(panels, 'transport_command', return_value=subprocess.CompletedProcess(
                [], 1, stdout='{"receiver_listening": true, "error": "ADB failed"}')) as command:
            state = panels.vr_transport_status()
            self.assertTrue(state['receiver_listening'])
            self.assertEqual(state['error'], 'ADB failed')
            self.assertLessEqual(command.call_args.kwargs['timeout'], 3)
            self.assertEqual(command.call_args.args[0][-2:], ['status', '--json'])
            command.side_effect = subprocess.TimeoutExpired('adb', 1)
            self.assertIn('error', panels.vr_transport_status())

    def test_vr_background_connect_waits_for_authorization_and_does_not_repeat_failures(self):
        jobs = []

        def submit(fn, stop):
            future = Future()
            jobs.append((fn, future))
            return future

        with patch.object(panels, 'ThreadPoolExecutor') as executor, \
             patch.object(panels.time, 'monotonic', return_value=10) as now:
            executor.return_value.submit.side_effect = submit
            vr = panels.VRConnection(auto_prepare=True)
            self.assertTrue(vr.snapshot()['checking'])
            vr.snapshot()  # Pending query returns immediately without duplicating it.
            self.assertEqual(len(jobs), 1)
            state = {'receiver_listening': True, 'usb_status': 'unauthorized', 'usb_serial': None}
            jobs[-1][1].set_result(state)
            self.assertEqual(vr.snapshot()['usb_status'], 'unauthorized')
            self.assertEqual(len(jobs), 1)
            now.return_value = 12
            vr.snapshot()
            state = dict(state, usb_status='ready', usb_serial='test-device', boot_completed=True, client_running=False)
            jobs[-1][1].set_result(state)
            vr.snapshot()
            self.assertIs(jobs[-1][0], panels.prepare_vr)
            vr.snapshot()
            self.assertEqual(len(jobs), 3)
            jobs[-1][1].set_result(panels.Status('prepare failed', 'error'))
            vr.snapshot()
            self.assertIs(jobs[-1][0], panels.vr_transport_status)
            jobs[-1][1].set_result(state)
            vr.snapshot()
            self.assertEqual(len(jobs), 4)  # Same failure is not retried on every repaint.
            self.assertIn('prepare failed', str(vr.message))
            now.return_value = 14
            vr.snapshot()  # Bounded backoff automatically retries a transient startup failure.
            self.assertIs(jobs[-1][0], panels.prepare_vr)
            jobs[-1][1].set_result(panels.Status('still starting', 'waiting'))
            vr.snapshot()
            jobs[-1][1].set_result(state)
            vr.snapshot()
            vr.retry = True  # R explicitly retries the same state.
            vr.snapshot()
            self.assertIs(jobs[-1][0], panels.prepare_vr)
            jobs[-1][1].set_result(panels.Status('prepared', 'ready'))
            vr.snapshot()
            jobs[-1][1].set_result(dict(state, reverse_configured=True, usb_connected=True, client_running=True))
            vr.snapshot()
            self.assertIsNone(vr.message)
            vr.close()
            executor.return_value.shutdown.assert_called_once_with(wait=False, cancel_futures=True)

    def test_vr_dashboard_monitor_never_prepares_and_noninteractive_serve_is_readonly(self):
        future = Future()
        with patch.object(panels, 'ThreadPoolExecutor') as executor:
            executor.return_value.submit.return_value = future
            vr = panels.VRConnection()
            vr.snapshot()
            future.set_result({'receiver_listening': False, 'usb_status': 'missing'})
            vr.snapshot()
            executor.return_value.submit.assert_called_once_with(panels.vr_transport_status, vr.stop)
            vr.close()
        with patch.object(panels.sys.stdin, 'isatty', return_value=False), \
             patch.object(panels, 'snapshot', return_value=('VR', [], [])), \
             patch.object(panels, 'prepare_vr') as prepare, \
             patch.object(panels, 'VRConnection') as monitor, \
             patch('sys.stdout', new_callable=io.StringIO):
            self.assertEqual(panels.main(['vr', '--serve']), 0)
            prepare.assert_not_called()
            monitor.assert_not_called()

    def test_late_headset_boot_and_usb_replug_resume_preparation(self):
        with patch.object(panels, 'ThreadPoolExecutor') as executor, \
             patch.object(panels.time, 'monotonic', return_value=10) as now:
            executor.return_value.submit.side_effect = lambda fn, stop: Future()
            vr = panels.VRConnection(auto_prepare=True)
            base = {'receiver_listening': True, 'usb_status': 'missing', 'boot_completed': False,
                    'reverse_configured': False, 'client_running': False}
            # Start workbench with headset off, then USB, authorization and Android boot.
            for state in [base, dict(base, usb_status='unauthorized'),
                          dict(base, usb_status='ready', usb_serial='QUEST')]:
                now.return_value += 2
                vr.snapshot()
                vr.future.set_result(state)
                vr.snapshot()
                self.assertIsNone(vr.future)
                self.assertFalse(vr.preparing)
            now.return_value += 2
            vr.snapshot()
            ready = dict(base, usb_status='ready', usb_serial='QUEST', boot_completed=True)
            vr.future.set_result(ready)
            vr.snapshot()
            self.assertTrue(vr.preparing)
            vr.future.set_result(panels.Status('prepared', 'ready'))
            vr.snapshot()
            vr.future.set_result(dict(ready, reverse_configured=True, client_running=True))
            vr.snapshot()
            self.assertIsNone(vr.attempt)
            # A reconnect with the same serial must recreate the lost reverse mapping.
            now.return_value += 2
            vr.snapshot()
            vr.future.set_result(base)
            vr.snapshot()
            now.return_value += 2
            vr.snapshot()
            vr.future.set_result(ready)
            vr.snapshot()
            self.assertTrue(vr.preparing)
            vr.close()

    def test_robot_waiting_page_preserves_failure_until_an_explicit_retry(self):
        result = subprocess.CompletedProcess([], 0, stdout='failed\n')
        with patch.dict(os.environ, {'TMUX_PANE': '%test'}), \
             patch.object(bench, 'tmux', return_value=result) as tmux:
            panels.mark_waiting('kernel')
        tmux.assert_called_once_with('display-message', '-p', '-t', '%test', '#{@bench_activity}', check=False)

    def test_robot_distinguishes_confirmation_from_launch_failure(self):
        with patch.dict(os.environ, {'EMBODIED_BENCH_SESSION': 'test-only'}), \
             patch.object(bench, 'current_run', return_value=None), \
             patch.object(panels, 'load_config', return_value={'robot': {}}), \
             patch.object(panels, 'load_robot_profile', return_value={'robot': {'can_interface': 'test-can'}}), \
             patch.object(Path, 'exists', return_value=False), \
             patch.object(Path, 'is_char_device', return_value=False), \
             patch.object(bench, 'activity', return_value='kernel') as state:
            rows, _ = panels.kernel_rows()
            self.assertIn('等待启动确认', str(dict(rows)['robot']))
            self.assertEqual(dict(rows)['robot'].level, 'waiting')
            state.return_value = 'failed'
            rows, _ = panels.kernel_rows()
            self.assertIn('启动失败', str(dict(rows)['robot']))
            self.assertEqual(dict(rows)['robot'].level, 'error')

    def test_shared_camera_service_retries_without_owning_or_stopping_it(self):
        with patch('embodied.devices.sensors.client.ensure_service', side_effect=[RuntimeError('startup failed'), None]) as ensure, \
             patch.object(panels, 'load_config', return_value={'paths': {'log_dir': '/tmp/camera-logs'}}), \
             patch.object(panels.subprocess, 'Popen') as spawn, \
             patch.object(panels.os, 'killpg') as signal_group:
            service = panels.Service('camera')
            service.start()
            self.assertEqual(service.message.level, 'error')
            self.assertIn('startup failed', str(service.message))
            self.assertIn('R 重试', str(service.message))
            service.start()
            self.assertEqual(ensure.call_count, 2)
            self.assertIn('共享服务已连接', service.message)
            self.assertEqual(service.log_path, Path('/tmp/camera-logs/camera-service.log'))
            self.assertIsNone(service.process)
            service.stop()
            spawn.assert_not_called()
            signal_group.assert_not_called()

    def test_interactive_camera_starts_and_r_retries_without_stopping_shared_service(self):
        with patch.object(panels.sys.stdin, 'isatty', return_value=True), \
             patch.object(panels.sys.stdout, 'isatty', return_value=True), \
             patch('embodied.devices.sensors.client.ensure_service', side_effect=[RuntimeError('offline'), None]) as ensure, \
             patch('embodied.core.terminal_keys.terminal_keys', return_value=nullcontext()), \
             patch('embodied.core.terminal_keys.read_key', side_effect=['r', 'eof']), \
             patch('rich.live.Live'), \
             patch.object(panels, 'snapshot', return_value=('共享相机', [], [])), \
             patch.object(panels.Service, 'logs', return_value=[]), \
             patch.object(panels.os, 'killpg') as signal_group:
            self.assertEqual(panels.main(['camera', '--serve']), 0)
            self.assertEqual(ensure.call_count, 2)
            signal_group.assert_not_called()

    def test_camera_ready_requires_enabled_fresh_frames_instead_of_metadata(self):
        config = {'camera_service': {'socket': '/camera.sock'},
                  'cameras': {'wrist_rgb': {'enabled': True, 'serial': 'expected'}}}
        camera = {'enabled': True, 'metadata': {'model': 'D435i'}, 'error': None}
        with patch.object(panels, 'load_config', return_value=config), \
             patch.object(panels, 'resolve_path', side_effect=Path), \
             patch.object(Path, 'exists', return_value=True), \
             patch.object(Path, 'glob', return_value=[]), \
             patch('embodied.core.session_protocol.request', return_value={'cameras': {'wrist_rgb': {'camera': camera}}}):
            for age, level in [(None, 'waiting'), (.1, 'ready'), (.5, 'error'), (4., 'error'), (float('nan'), 'waiting')]:
                with self.subTest(age=age):
                    camera['age_s'] = age
                    rows = dict(panels.camera_rows()[0])
                    self.assertEqual(rows['wrist_rgb'].level, level)
            camera.update(enabled=False, age_s=.1)
            self.assertEqual(dict(panels.camera_rows()[0])['wrist_rgb'].level, 'disabled')
            camera.update(enabled=True, error='device disconnected')
            self.assertEqual(dict(panels.camera_rows()[0])['wrist_rgb'].level, 'error')

    def test_camera_usb_serial_is_not_compared_with_sdk_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            usb = Path(directory)
            (usb / 'serial').write_text('usb-descriptor-serial')
            (usb / 'product').write_text('Intel RealSense D435i')
            options = {'enabled': True, 'serial': 'different-sdk-serial'}
            config = {'camera_service': {'socket': str(usb / 'missing.sock')}, 'cameras': {'wrist_rgb': options}}
            with patch.object(panels, 'load_config', return_value=config), \
                 patch.object(panels, 'resolve_path', side_effect=Path), \
                 patch.object(Path, 'glob', return_value=[usb / 'product']):
                rows = dict(panels.camera_rows()[0])
                self.assertEqual(rows['共享服务'].level, 'waiting')
                self.assertEqual(rows['wrist_rgb'].level, 'waiting')
                self.assertIn('RealSense USB 可见', str(rows['wrist_rgb']))
                self.assertIn('等待相机服务确认', str(rows['wrist_rgb']))
                self.assertNotIn('未匹配', str(rows))
                self.assertNotIn('usb-descriptor-serial', str(rows))
                self.assertIn('Intel RealSense D435i', rows['USB 识别'])

    def test_noninteractive_camera_serve_never_ensures_service(self):
        with patch.object(panels.sys.stdin, 'isatty', return_value=False), \
             patch.object(panels, 'snapshot', return_value=('共享相机', [], ['等待启动'])), \
             patch('embodied.devices.sensors.client.ensure_service') as ensure, \
             patch.object(panels.Service, 'start') as start, \
             patch('sys.stdout', new_callable=io.StringIO):
            self.assertEqual(panels.main(['camera', '--serve']), 0)
            ensure.assert_not_called()
            start.assert_not_called()

    def test_vr_color_requires_valid_tracking_and_distinguishes_errors(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch('embodied.contracts.runtime.control.ControlClient') as client:
            run = Path(directory)
            self.assertEqual(panels.vr_control_rows(None)[0][0][1].level, 'disabled')
            self.assertEqual(panels.vr_control_rows((run, {'input': 'none'}))[0][0][1].level, 'disabled')
            found = (run, {'input': 'quest'})
            self.assertEqual(panels.vr_control_rows(found)[0][0][1].level, 'waiting')
            client.assert_not_called()
            (run / 'commands.sock').touch()
            for ingress, expected in [(None, 'waiting'), ({'ready': False}, 'waiting'),
                                      ({'ready': True, 'armed': True, 'permit': True}, 'ready')]:
                client.return_value.status.return_value = {'ingresses': {'native': ingress}}
                self.assertEqual(panels.vr_control_rows(found)[0][0][1].level, expected)
            client.return_value.status.side_effect = RuntimeError('Runtime feedback is stale')
            state = panels.vr_control_rows(found)[0][0][1]
            self.assertEqual(state.level, 'error')
            self.assertIn('Runtime feedback is stale', str(state))

    def test_arm_fault_is_red_in_both_the_dashboard_and_device_panel(self):
        from rich.console import Console
        from rich.color import Color

        with tempfile.TemporaryDirectory() as directory, \
             patch('embodied.contracts.runtime.control.ControlClient') as client, \
             patch('embodied.applications.workstation.ports.summary', return_value=('测试端口', False)), \
             patch.object(panels, 'vr_transport_status', return_value={'usb_status': 'missing'}):
            run = Path(directory)
            found = (run, {'backend': 'sim', 'input': 'none'})
            self.assertEqual(bench.kernel_summary(*found)[1].level, 'waiting')
            (run / 'commands.sock').touch()
            client.return_value.status.return_value = {'owner': None, 'q': [0] * 6,
                'phase': 'hold', 'stopped': False, 'gripper_width': .04}
            self.assertEqual(bench.kernel_summary(*found)[1].level, 'ready')
            self.assertIn('仿真反馈', str(bench.kernel_summary(*found)[1]))
            client.return_value.status.return_value['stopped'] = True
            self.assertEqual(bench.kernel_summary(*found)[1].level, 'waiting')
            client.return_value.status.side_effect = RuntimeError('feedback stale [green]')
            with patch.object(bench, 'current_run', return_value=found):
                views = [bench.dashboard_view(found, None, ''), panels.render(panels.snapshot('kernel'))]
            console = Console(width=140, force_terminal=True, color_system='standard')
            for view in views:
                fragments = [s for s in console.render(view) if '状态读取失败' in s.text]
                self.assertTrue(fragments)
                self.assertTrue(all(s.style.color == Color.parse('red') for s in fragments))
                self.assertTrue(any('[green]' in s.text for s in fragments))

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
             patch.object(panels, 'vr_transport_status', return_value={'usb_status': 'missing'}), \
             patch('builtins.__import__', side_effect=without_sdk):
            states = {role: panels.snapshot(role) for role in ('kernel', 'vr', 'camera')}
            spawn.assert_not_called()
            command.assert_not_called()
        for role, (title, rows, hints) in states.items():
            with self.subTest(role=role):
                self.assertTrue(title)
                self.assertTrue(rows)
                self.assertTrue(hints)
        self.assertIn('未通电', str(states['kernel'][2]))
        self.assertIn('missing-can', dict(states['kernel'][1])['配置'])
        self.assertNotIn('legacy-can', dict(states['kernel'][1])['配置'])
        self.assertIn('USB', str(states['vr'][2]))
        self.assertIn('等待 USB 相机连接', str(states['camera'][1]))
        for role in ('kernel', 'vr'):
            self.assertEqual(states[role][1][0][1].level, 'waiting')
        self.assertEqual(dict(states['kernel'][1])['USB-CAN'].level, 'waiting')

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
