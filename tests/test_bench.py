"""Workbench state and isolated tmux lifecycle; no device or service is started."""
from datetime import datetime
import io
import json
import os
from pathlib import Path
import shutil
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

    def test_kernel_selection_replaces_only_the_waiting_panel(self):
        with patch.object(bench, 'current_run', return_value=None), \
             patch.object(bench, 'window', return_value=('%test', 'python')), \
             patch.object(bench, 'activity', return_value='waiting'), \
             patch.object(bench, 'panel_command', return_value='test-waiting-panel'), \
             patch.object(bench, 'tmux') as tmux:
            bench.start_kernel('test-kernel-command')
        commands = [call.args for call in tmux.call_args_list]
        replaced = [command for command in commands if command[0] == 'respawn-pane']
        self.assertEqual(len(replaced), 1)
        self.assertIn('%test', replaced[0])
        self.assertIn('test-kernel-command', replaced[0][-1])
        self.assertIn('test-waiting-panel', replaced[0][-1])


def actions(directory, identity, episode, now):
    return bench.actions(directory, identity, episode, now)


@unittest.skipUnless(shutil.which('tmux'), 'tmux not installed')
class BenchTmuxTests(unittest.TestCase):
    def test_all_windows_reopen_repair_and_close_without_touching_external_kernel(self):
        session = f'bench-test-{os.getpid()}-{time.monotonic_ns()}'
        self.addCleanup(subprocess.run, ['tmux', 'kill-session', '-t', session], capture_output=True)
        expected = ['1:内核', '2:命令', '3:总控', '4:网页', '5:VR', '6:相机', '7:宣传']

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
             patch.object(bench, 'panel_command', side_effect=lambda role: bench.shell(f'{role}: waiting for device')), \
             patch.object(bench, 'dashboard_command', return_value=bench.shell('test dashboard')):
            bench.start(attach=False)
            self.assertEqual(names(), expected)
            until(lambda: all(bench.idle(role) for role in bench.WINDOWS))
            original = panes()
            bench.start(attach=False)
            self.assertEqual(names(), expected)
            self.assertEqual(panes(), original)  # reopening preserves live work

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
