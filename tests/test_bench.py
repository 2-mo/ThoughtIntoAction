"""The unified tmux workbench: run discovery, the key menu, and a real tmux round trip."""
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

from embodied.workstation import bench

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
        self.assertIsNone(plain['w'])
        self.assertIsNone(plain['h'])
        self.assertIsNone(plain['k'])  # nothing recorded yet
        self.assertIn('--cameras wrist_rgb,exterior_rgb', plain['c'])
        self.assertIn('--camera synthetic', full['c'])
        self.assertEqual(full['w'], 'web')
        self.assertIn('calib handeye capture wrist_rgb --auto --run', full['h'])
        self.assertIn('datasets/lerobot/20261022-070000', full['e'])
        self.assertEqual(sorted(key for key, *_ in bench.KERNELS), ['1', '2', '3', '4', '5'])


def actions(directory, identity, episode, now):
    return bench.actions(directory, identity, episode, now)


@unittest.skipUnless(shutil.which('tmux'), 'tmux not installed')
class BenchTmuxTests(unittest.TestCase):
    def test_start_types_commands_into_the_shell_window_and_stops(self):
        session = f'bench-test-{os.getpid()}'
        env = {**os.environ, 'EMBODIED_BENCH_SESSION': session, 'PYTHONPATH': str(ROOT / 'src')}
        command = [sys.executable, '-B', '-m', 'embodied.workstation.bench']
        self.addCleanup(subprocess.run, ['tmux', 'kill-session', '-t', session], capture_output=True)
        subprocess.run([*command, 'start', '--detach'], env=env, cwd=ROOT, check=True, capture_output=True)
        names = subprocess.run(['tmux', 'list-windows', '-t', session, '-F', '#{window_index}:#{window_name}'],
                               capture_output=True, text=True, check=True).stdout.split()
        self.assertEqual(names, ['1:内核', '2:命令', '3:总控'])
        probe = (f'import os; os.environ["EMBODIED_BENCH_SESSION"] = {session!r}; '
                 'from embodied.workstation import bench; print(bench.type_command("echo bench-marker-$((40+2))"))')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:  # the shell window becomes idle once bash has started
            result = subprocess.run([sys.executable, '-c', probe], env=env, cwd=ROOT, capture_output=True, text=True)
            if result.returncode == 0:
                break
            time.sleep(.2)
        self.assertEqual(result.returncode, 0, result.stderr)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            text = subprocess.run(['tmux', 'capture-pane', '-p', '-t', f'{session}:2'], capture_output=True, text=True).stdout
            if 'bench-marker-42' in text:
                break
            time.sleep(.2)
        self.assertIn('bench-marker-42', text)
        stopped = subprocess.run([*command, 'stop'], env=env, cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertNotEqual(subprocess.run(['tmux', 'has-session', '-t', session], capture_output=True).returncode, 0)


if __name__ == '__main__':
    unittest.main()
