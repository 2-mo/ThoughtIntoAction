"""Workbench state and isolated tmux lifecycle; no device or service is started."""
from datetime import datetime
import io
import json
import os
from pathlib import Path
import shutil
import shlex
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from embodied.applications.workstation import bench

ROOT = Path(__file__).resolve().parents[1]
BOOT = Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def run(root, name, *, pid=999_999_999, ready=True, managed=False):
    directory = Path(root) / name
    directory.mkdir()
    (directory / 'runtime.json').write_text(json.dumps({'supervisor_pid': pid, 'boot_id': BOOT, 'backend': 'sim', 'managed': managed}))
    if ready:
        (directory / 'runtime.ready').touch()
    if managed:
        (directory / 'commands.sock').touch()
    return directory


class BenchStateTests(unittest.TestCase):
    def test_real_restart_requires_one_explicit_confirmation(self):
        found = (Path('/run/test-only'), {'backend': 'real', 'managed': True})
        with patch.object(bench, 'owned_run', return_value=found), \
             patch.object(sys.stdin, 'isatty', return_value=True), \
             patch('sys.stdout', new_callable=io.StringIO), \
             patch('builtins.input', return_value='n') as answer, \
             patch('embodied.applications.workstation.restart.restart', return_value='已重启') as restart:
            bench.main(['restart', '--target', 'robot'])
            restart.assert_not_called()
            answer.assert_called_once()
            answer.reset_mock()
            answer.return_value = 'y'
            bench.main(['restart', '--target', 'robot'])
            answer.assert_called_once()
            restart.assert_called_once_with('robot', robot_confirmed=True)
            restart.reset_mock()
            with patch.object(sys.stdin, 'isatty', return_value=False):
                bench.main(['restart', '--target', 'robot'])
                restart.assert_not_called()

    def test_restart_menu_routes_digits_without_starting_a_kernel(self):
        with patch.object(sys.stdin, 'isatty', return_value=True), \
             patch.object(sys.stdin, 'fileno', return_value=0), \
             patch.object(sys.stdout, 'isatty', return_value=True), \
             patch('termios.tcgetattr', return_value=[]), \
             patch('termios.tcsetattr'), patch('tty.setcbreak'), \
             patch('select.select', return_value=([sys.stdin], [], [])), \
             patch.object(bench.os, 'read', side_effect=[b'r', b'2', b'']), \
             patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench, 'latest_episode', return_value=None), \
             patch.object(bench, 'dashboard_view') as view, \
             patch('rich.live.Live'), \
             patch('embodied.applications.workstation.bench_panels.VRConnection'), \
             patch.object(bench, 'type_command', return_value='重启命令已提交') as command, \
             patch.object(bench, 'start_kernel') as start:
            bench.dashboard()
        command.assert_called_once_with('./run.sh bench restart --target robot')
        start.assert_not_called()
        self.assertTrue(any(call.args[-1] is True for call in view.call_args_list))

    def test_dashboard_shows_each_device_without_starting_services(self):
        from rich.console import Console
        from rich.color import Color
        from embodied.applications.workstation import bench_panels as panels
        from embodied.applications.workstation.status import Status

        states = {
            'kernel': ('robot', [('内核', Status('等待机械臂反馈', 'waiting'))], []),
            'camera': ('相机', [('腕部相机', Status('图像正常', 'ready')),
                               ('外部相机', Status('图像已过期', 'error'))], []),
        }
        with patch.object(panels, 'snapshot', side_effect=states.__getitem__), \
             patch.object(panels, 'vr_rows', return_value=([('VR', Status('未启用', 'disabled'))], [])), \
             patch('embodied.applications.workstation.ports.summary', return_value=('测试端口', False)), \
             patch('embodied.devices.sensors.client.ensure_service') as start:
            view = bench.dashboard_view(None, None, '')
            start.assert_not_called()
        console = Console(width=140, force_terminal=True, color_system='standard')
        fragments = list(console.render(view))
        rendered = ''.join(fragment.text for fragment in fragments)
        for name in ('robot', 'VR', '腕部相机', '外部相机'):
            self.assertIn(name, rendered)
        for icon in ('◇', '◉', '▣'):
            self.assertIn(icon, rendered)
            self.assertTrue(any(icon in fragment.text and fragment.style.color == Color.parse('bright_cyan')
                                for fragment in fragments))
        self.assertNotIn('🦾', rendered)
        self.assertNotIn('🥽', rendered)
        self.assertNotIn('📷', rendered)
        for message, color in [('等待机械臂反馈', 'yellow'), ('图像正常', 'green'), ('图像已过期', 'red')]:
            self.assertTrue(any(message in fragment.text and fragment.style.color == Color.parse(color)
                                for fragment in fragments))

    def test_panel_reload_waits_for_cleanup_and_never_force_kills(self):
        done = subprocess.CompletedProcess([], 0, stdout='1\n')
        with patch.object(bench, 'tmux', return_value=done) as tmux:
            bench.refresh_panel('%web', 'new-web-panel')
        self.assertEqual(tmux.call_args_list[0].args, ('send-keys', '-t', '%web', 'C-c'))
        tmux.assert_any_call('respawn-pane', '-t', '%web', '-c', str(bench.ROOT), 'new-web-panel')
        self.assertTrue(all('-k' not in call.args for call in tmux.call_args_list))

        still_running = subprocess.CompletedProcess([], 0, stdout='0\n')
        with patch.object(bench, 'tmux', return_value=still_running) as tmux, \
             patch.object(bench.time, 'monotonic', side_effect=[0., 9.]):
            with self.assertRaisesRegex(RuntimeError, '保留原进程'):
                bench.refresh_panel('%web', 'new-web-panel')
        self.assertFalse(any(call.args[0] == 'respawn-pane' for call in tmux.call_args_list))

    def test_motion_tests_bind_current_backend_run_and_wait_for_enter(self):
        with tempfile.TemporaryDirectory(prefix='bench actions ') as root:
            directory = run(root, 'managed', managed=True)
            for backend in ('real', 'sim'):
                rows = {key: (command, enter, needs) for key, _, command, enter, needs
                        in bench.actions(directory, {'backend': backend})}
                for key in ('b', 'x', 't'):
                    command, enter, needs = rows[key]
                    arguments = shlex.split(command)
                    self.assertEqual(arguments[arguments.index('--run') + 1], str(directory))
                    self.assertEqual(arguments[arguments.index('--backend') + 1], backend)
                    self.assertNotIn('--speed', arguments)
                    self.assertNotIn('--tcp-speed', arguments)
                    self.assertFalse(enter)
                    self.assertTrue(needs)
                self.assertIn('--degrees 10 --repeats 1', rows['b'][0])
            (directory / 'commands.sock').unlink()
            rows = {key: command for key, _, command, _, _ in bench.actions(directory, {'backend': 'real'})}
            self.assertTrue(all(rows[key] is None for key in ('b', 'x', 't')))

    def test_staging_a_motion_does_not_submit_it(self):
        with patch.object(bench, 'window', return_value=('%command', 'bash')), \
             patch.object(bench, 'tmux') as tmux:
            bench.type_command('test-motion --run selected', enter=False)
        self.assertNotIn(('send-keys', '-t', '%command', 'Enter'), [call.args for call in tmux.call_args_list])
        tmux.assert_any_call('select-window', '-t', '%command')

    def test_missing_tmux_shows_next_steps_without_starting_any_process(self):
        with patch.object(bench.shutil, 'which', return_value=None), \
             patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench.subprocess, 'run') as process, \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(bench.main(['start']), 0)
            process.assert_not_called()
        self.assertIn('sudo apt install tmux', output.getvalue())
        self.assertIn('./run.sh kernel doctor', output.getvalue())

    def test_piped_dashboard_is_a_read_only_snapshot(self):
        with patch.object(sys.stdin, 'isatty', return_value=False), \
             patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench.subprocess, 'run') as process, \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            bench.dashboard()
            process.assert_not_called()
        self.assertIn('未启动新的服务', output.getvalue())
        self.assertNotIn('\x1b[', output.getvalue())

    def test_current_run_is_the_newest_ready_run_with_a_live_supervisor(self):
        with tempfile.TemporaryDirectory() as root:
            older = run(root, '20260101-000000-arx5-a', pid=os.getpid())
            run(root, '20260102-000000-arx5-b', pid=os.getpid(), ready=False)
            run(root, '20260103-000000-arx5-c')  # supervisor gone
            self.assertEqual(bench.current_run(root)[0], older)
            newer = run(root, '20260104-000000-arx5-d', pid=os.getpid())
            self.assertEqual(bench.current_run(root)[0], newer)
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(bench.current_run(root))

    def test_menu_offers_gateway_actions_only_on_managed_kernels(self):
        with tempfile.TemporaryDirectory() as root:
            native = run(root, 'native')
            managed = run(root, 'managed', managed=True)
            now = datetime(2026, 10, 22, 7, 0, 0)
            plain = {key: command for key, _, command, _, _ in actions(native, {'backend': 'real'}, None, now)}
            full = {key: command for key, _, command, _, _ in actions(managed, {'backend': 'sim'}, Path(root) / 'ep', now)}
        self.assertEqual(plain['w'], 'web')
        self.assertIsNone(plain['h'])
        self.assertIsNone(plain['k'])  # nothing recorded yet
        self.assertIn('--cameras wrist_rgb,exterior_rgb', plain['c'])
        self.assertIn('--camera synthetic', full['c'])
        self.assertEqual(full['w'], 'web')
        self.assertIn('calib handeye capture wrist_rgb --auto --run', full['h'])
        self.assertIn('--source outputs/datasets/runtime --output outputs/datasets/lerobot/20261022-070000', full['e'])
        self.assertEqual(sorted(key for key, *_ in bench.KERNELS), ['1', '2', '3', '4', '5'])

    def test_owned_run_requires_the_kernel_to_descend_from_this_pane(self):
        active = (ROOT / 'outputs/runtime/control/test-only', {'supervisor_pid': 1234})
        pane_info = subprocess.CompletedProcess([], 0, stdout='5678\n')
        with patch.object(bench, 'current_run', return_value=active), \
             patch.object(bench, 'window', return_value=('%test', 'python')), \
             patch.object(bench, 'tmux', return_value=pane_info), \
             patch.object(bench, 'descendant_pid', return_value=False) as descendant:
            self.assertIsNone(bench.owned_run())
            descendant.assert_called_once_with(1234, 5678)
            descendant.return_value = True
            self.assertEqual(bench.owned_run(), active)

    def test_stop_preserves_a_kernel_started_manually_in_another_pane(self):
        active = (ROOT / 'outputs/runtime/control/test-only', {'supervisor_pid': 1234, 'backend': 'real'})
        pane_info = subprocess.CompletedProcess([], 0, stdout='%kernel|222\n%command|333\n')
        with patch.object(bench, 'exists', return_value=True), \
             patch.object(bench, 'check_session_owner'), \
             patch.object(bench, 'owned_run', return_value=None), \
             patch.object(bench, 'current_run', return_value=active), \
             patch.object(bench, 'descendant_pid', side_effect=lambda pid, ancestor: ancestor == 333), \
             patch.object(bench, 'tmux', return_value=pane_info) as tmux:
            with self.assertRaisesRegex(RuntimeError, '其他窗口（%command）.*工作台保留'):
                bench.stop()
        # Listing ancestry is allowed; sending Ctrl+C or killing the session is not.
        tmux.assert_called_once_with('list-panes', '-s', '-t', bench.SESSION, '-F', '#{pane_id}|#{pane_pid}')

    def test_kernel_selection_cannot_replace_a_busy_or_starting_kernel(self):
        for command, activity in [('python', 'kernel'), ('bash', 'kernel'), ('python', '')]:
            with self.subTest(command=command, activity=activity), \
                 patch.object(bench, 'current_run', return_value=None), \
                 patch.object(bench, 'window', return_value=('%test', command)), \
                 patch.object(bench, 'activity', return_value=activity), \
                 patch.object(bench, 'tmux') as tmux:
                with self.assertRaises(RuntimeError):
                    bench.start_kernel('test-kernel-command')
                tmux.assert_not_called()

    def test_only_an_interactive_open_prepares_the_robot_confirmation(self):
        active = (Path('/run/test-only'), {'backend': 'real'})
        for attach, tty, state, found, expected in [
                (True, True, 'waiting', None, True),
                (False, True, 'waiting', None, False),
                (True, False, 'waiting', None, False),
                (True, True, 'failed', None, False),
                (True, True, 'waiting', active, False),
                (True, True, 'kernel', None, False)]:
            with self.subTest(attach=attach, tty=tty, state=state, active=bool(found)), \
                 patch.object(sys.stdin, 'isatty', return_value=tty), \
                 patch.object(sys.stdout, 'isatty', return_value=tty), \
                 patch.object(bench, 'current_run', return_value=found), \
                 patch.object(bench, 'activity', return_value=state), \
                 patch.object(bench, 'window', return_value=('%test', 'python')), \
                 patch.object(bench, 'tmux') as tmux, \
                 patch.object(bench, 'start_kernel') as start:
                bench.prepare_robot_start(attach)
                if expected:
                    start.assert_called_once_with(bench.REAL + ' --profile real-manual')
                    self.assertNotIn('--yes', start.call_args.args[0])
                else:
                    start.assert_not_called()
                if state == 'kernel':
                    tmux.assert_called_once_with('select-window', '-t', '%test')
                else:
                    tmux.assert_not_called()

    def test_failed_robot_is_not_an_active_controller(self):
        with patch.object(bench, 'window', return_value=('%test', 'python')), \
             patch.object(bench, 'activity', return_value='failed'):
            self.assertFalse(bench.kernel_busy())

    def test_robot_failure_keeps_original_logs_until_acknowledged(self):
        with patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench, 'ensure_robot_controls'), \
             patch.object(bench, 'window', return_value=('%test', 'python')), \
             patch.object(bench, 'activity', return_value='waiting'), \
             patch.object(bench, 'panel_command', return_value="printf 'panel-entered\\n'"), \
             patch.object(bench, 'tmux') as tmux:
            bench.start_kernel("printf 'original-test-error\\n' >&2; false")
        command = next(call.args[-1] for call in tmux.call_args_list if call.args[0] == 'respawn-pane')
        # Execute only the shell error-handling path, with tmux replaced by a
        # harmless stub. The fake failing command cannot start any device.
        with tempfile.TemporaryDirectory() as directory:
            stub = Path(directory) / 'tmux'
            stub.write_text('#!/bin/sh\nexit 0\n')
            stub.chmod(0o755)
            env = dict(os.environ, PATH=directory + os.pathsep + os.environ['PATH'])
            process = subprocess.Popen(shlex.split(command), stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
            try:
                self.assertEqual(process.stdout.readline().strip(), 'original-test-error')
                self.assertEqual(process.stdout.readline().strip(), '')
                self.assertIn('启动失败（退出码 1）', process.stdout.readline())
                self.assertIsNone(process.poll())  # waiting for acknowledgment, not the panel
                output, _ = process.communicate('\n', timeout=3)
                self.assertIn('panel-entered', output)
                self.assertEqual(process.returncode, 0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()

    def test_kernel_selection_replaces_only_the_waiting_panel(self):
        with patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench, 'ensure_robot_controls'), \
             patch.object(bench, 'window', return_value=('%test', 'python')), \
             patch.object(bench, 'activity', return_value='waiting'), \
             patch.object(bench, 'panel_command', return_value='test-waiting-panel'), \
             patch.object(bench, 'tmux') as tmux:
            bench.start_kernel('test-kernel-command')
        commands = [call.args for call in tmux.call_args_list]
        self.assertIn(('set-option', '-p', '-t', '%test', '@bench_kernel_command', 'test-kernel-command'), commands)
        replaced = [command for command in commands if command[0] == 'respawn-pane']
        self.assertEqual(len(replaced), 1)
        self.assertIn('%test', replaced[0])
        self.assertIn('test-kernel-command', replaced[0][-1])
        self.assertIn('test-waiting-panel', replaced[0][-1])


def actions(directory, identity, episode, now):
    return bench.actions(directory, identity, episode, now)


@unittest.skipUnless(shutil.which('tmux'), 'tmux not installed')
class BenchTmuxTests(unittest.TestCase):
    def test_session_name_collision_preserves_the_other_session(self):
        session = f'bench-collision-{os.getpid()}-{time.monotonic_ns()}'
        bench.tmux('new-session', '-d', '-s', session, 'sleep 60')
        self.addCleanup(subprocess.run, ['tmux', 'kill-session', '-t', session], capture_output=True)
        original = bench.tmux('list-panes', '-t', session, '-F', '#{pane_id}|#{pane_pid}').stdout
        with patch.object(bench, 'SESSION', session):
            with self.assertRaisesRegex(RuntimeError, '已被其他工作区使用'):
                bench.prepare_session()
            with self.assertRaisesRegex(RuntimeError, '已被其他工作区使用'):
                bench.stop()
        self.assertEqual(bench.tmux('list-panes', '-t', session, '-F', '#{pane_id}|#{pane_pid}').stdout, original)

    def test_all_windows_reopen_repair_and_close_without_touching_external_kernel(self):
        session = f'bench-test-{os.getpid()}-{time.monotonic_ns()}'
        self.addCleanup(subprocess.run, ['tmux', 'kill-session', '-t', session], capture_output=True)
        expected = ['1:总控', '2:命令', '3:robot', '4:网页', '5:VR', '6:相机', '7:宣传', '8:手机']

        def names():
            return bench.tmux('list-windows', '-t', session, '-F', '#{window_index}:#{window_name}').stdout.split()

        def panes():
            rows = bench.tmux('list-panes', '-s', '-t', session, '-F', '#{@bench_role}|#{pane_id}|#{pane_pid}').stdout.splitlines()
            return {role: (pane, pid) for role, pane, pid in (row.split('|') for row in rows)}

        def until(predicate):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if predicate():
                    return
                time.sleep(.05)
            self.fail('isolated tmux pane did not reach its expected state')

        # Replacing every service command keeps this test safe even while a real
        # robot is being controlled by another terminal on the same machine.
        with patch.object(bench, 'SESSION', session), \
             patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench, 'panel_command', side_effect=lambda role: bench.shell(
                 f'embodied.applications.workstation.bench_panels {role}: waiting for device')), \
             patch.object(bench, 'dashboard_command', return_value=bench.shell('embodied.applications.workstation.bench _dashboard')):
            bench.start(attach=False)
            self.assertEqual(names(), expected)
            until(lambda: all(bench.idle(role) for role in bench.WINDOWS))
            original = panes()
            bench.tmux('rename-window', '-t', original['kernel'][0], '内核')
            bench.start(attach=False)
            self.assertEqual(names(), expected)
            self.assertEqual(panes(), original)  # reopening preserves live work

            # Migrate an attached-era layout (kernel first) and its old hashed
            # name in place. Only the dashboard must reload its session env.
            legacy = session + '-legacy'
            self.addCleanup(subprocess.run, ['tmux', 'kill-session', '-t', legacy], capture_output=True)
            bench.tmux('swap-window', '-s', original['home'][0], '-t', original['kernel'][0])
            bench.tmux('set-option', '-p', '-t', original['kernel'][0], '@bench_activity', 'kernel')
            bench.tmux('set-option', '-p', '-t', original['kernel'][0], '@bench_ui_revision', 'busy-old-kernel')
            bench.tmux('rename-session', '-t', session, legacy)
            with patch.object(bench, 'DEFAULT_SESSION', session), patch.object(bench, 'LEGACY_SESSION', legacy):
                bench.start(attach=False)
            until(lambda: bench.idle('home'))
            self.assertEqual(names(), expected)
            self.assertNotEqual(panes()['home'][1], original['home'][1])
            for role in set(bench.WINDOWS) - {'home'}:
                self.assertEqual(panes()[role], original[role])
            self.assertEqual(bench.activity('kernel'), 'kernel')
            self.assertEqual(bench.tmux('display-message', '-p', '-t', original['kernel'][0], '#{@bench_ui_revision}').stdout.strip(),
                             'busy-old-kernel')
            bench.tmux('set-option', '-p', '-t', original['kernel'][0], '@bench_activity', 'waiting')
            original = panes()

            # An existing user's workbench picks up the new test menu without
            # replacing the kernel, command shell or running web/device panels.
            bench.tmux('set-option', '-p', '-t', original['home'][0], '@bench_ui_revision', 'old-menu')
            bench.start(attach=False)
            until(lambda: bench.idle('home'))
            upgraded = panes()
            self.assertNotEqual(upgraded['home'][1], original['home'][1])
            for role in set(bench.WINDOWS) - {'home'}:
                self.assertEqual(upgraded[role], original[role])
            original = upgraded

            bench.tmux('kill-window', '-t', original['camera'][0])
            bench.start(attach=False)
            self.assertEqual(names(), expected)
            repaired = panes()
            self.assertNotEqual(repaired['camera'], original['camera'])
            for role in set(bench.WINDOWS) - {'camera'}:
                self.assertEqual(repaired[role], original[role])

            web = repaired['web'][0]
            bench.tmux('set-option', '-w', '-t', web, 'remain-on-exit', 'on')
            bench.tmux('respawn-pane', '-k', '-t', web, 'true')
            until(lambda: bench.window('web')[1] == '')
            bench.start(attach=False)
            until(lambda: bench.idle('web'))
            self.assertEqual(names(), expected)
            self.assertNotEqual(panes()['web'][1], repaired['web'][1])

            until(lambda: bench.idle('shell'))
            bench.type_command('echo bench-marker-$((40+2))')
            until(lambda: 'bench-marker-42' in bench.tmux('capture-pane', '-p', '-t', f'{session}:2').stdout)

            external = (ROOT / 'outputs/runtime/control/external-test-only', {'supervisor_pid': os.getpid(), 'backend': 'real'})
            with patch.object(bench, 'current_run', return_value=external), \
                 patch.object(bench, 'owned_run', return_value=None), \
                 patch.object(bench.os, 'kill') as signal, \
                 patch('sys.stdout', new_callable=io.StringIO) as output:
                bench.stop(wait_s=.2)
                signal.assert_not_called()
            self.assertFalse(bench.exists())
            self.assertNotIn('SDK 阻尼', output.getvalue())


if __name__ == '__main__':
    unittest.main()
