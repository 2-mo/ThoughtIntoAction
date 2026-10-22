"""Isolated tmux server and dummy consoles; never launch device services."""
import importlib.util
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/workspace/tmux_workspace.py"
spec = importlib.util.spec_from_file_location("tmux_workspace", SCRIPT)
workspace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workspace)


@unittest.skipUnless(shutil.which("tmux"), "tmux not installed")
class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="questarm-workspace-test-")
        self.root = Path(self.temp.name)
        self.socket = str(self.root / "tmux.sock")
        (self.root / ".venv/bin").mkdir(parents=True)
        (self.root / ".venv/bin/python").touch()
        self.events = self.root / "events"
        dummy = self.root / "console.py"
        dummy.write_text('''import os, subprocess, sys, tty
role, socket, events = sys.argv[1:]
pane = os.environ['TMUX_PANE']
def option(*args):
    subprocess.run(['tmux', '-S', socket, 'set-option', *args], check=True)
option('-w', '-t', pane, 'remain-on-exit', 'on')
option('-p', '-t', pane, '@questarm_role', role)
tty.setcbreak(sys.stdin.fileno())
option('-p', '-t', pane, '@questarm_phase', 'console')
with open(events, 'a') as f: f.write('start ' + role + '\\n')
while sys.stdin.read(1) != 'q': pass
with open(events, 'a') as f: f.write('stop ' + role + '\\n')
''')
        def isolated(*args, check=True):
            result = subprocess.run(["tmux", "-S", self.socket, "-f", "/dev/null", *args],
                                    text=True, capture_output=True)
            if check and result.returncode:
                raise RuntimeError(result.stderr)
            return result
        self.patches = [patch.object(workspace, "ROOT", self.root),
                        patch.object(workspace, "selected_mode", return_value="real"),
                        patch.object(workspace, "SESSION", "test-workspace"),
                        patch.object(workspace, "tmux", isolated),
                        patch.object(workspace, "pane_command", lambda role: "exec " + shlex.join(
                            [sys.executable, str(dummy), role, self.socket, str(self.events)]))]
        for p in self.patches:
            p.start()

    def tearDown(self):
        subprocess.run(["tmux", "-S", self.socket, "kill-server"], capture_output=True)
        for p in reversed(self.patches):
            p.stop()
        self.temp.cleanup()

    def ready(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            panes = workspace.panes()
            if len(panes) == 4 and all(p[2] == "console" for p in panes.values()):
                return panes
            time.sleep(.05)
        self.fail("dummy consoles failed to start")

    def test_real_tmux_reuse_ordered_stop_and_restart_guard(self):
        workspace.start()
        before = self.ready()
        workspace.start()
        self.assertEqual(workspace.panes(), before)
        windows = workspace.tmux("list-windows", "-t", "test-workspace", "-F", "#{window_index}").stdout.split()
        self.assertEqual(windows, ["1", "2", "3", "4"])
        with patch.object(sys, "argv", [str(SCRIPT), "restart", "arm"]):
            with self.assertRaisesRegex(RuntimeError, "仍在运行"):
                workspace.main()
        with patch.object(workspace, "arm_info", return_value={"hardware": True}), \
             patch.object(workspace, "teleop_running", return_value=False):
            workspace.stop()
        self.assertFalse(workspace.exists())
        stops = [line for line in self.events.read_text().splitlines() if line.startswith("stop")]
        self.assertEqual(stops, ["stop teleop", "stop arm", "stop web"])

    def test_external_teleop_keeps_arm_running(self):
        workspace.start()
        self.ready()
        with patch.object(workspace, "teleop_running", return_value=True):
            with self.assertRaisesRegex(RuntimeError, "仍有遥操"):
                workspace.stop()
        self.assertTrue(workspace.exists())
        self.assertFalse(workspace.panes()["arm"][1])
        self.assertNotIn("stop arm", self.events.read_text())

    def test_simulation_details_stay_live_and_close_normally(self):
        runner = self.root / 'details.py'
        runner.write_text(f'''import sys, subprocess, os
sys.path.insert(0, {str(SCRIPT.parent)!r})
import tmux_workspace as w
import details
def tmux(*args, check=True):
    return subprocess.run(['tmux', '-S', {self.socket!r}, *args],
                          text=True, capture_output=True, check=check)
w.tmux = tmux
w.selected_mode = lambda: 'simulation'
w.tmux('set-option', '-w', '-t', os.environ['TMUX_PANE'], 'remain-on-exit', 'on')
w.tmux('set-option', '-p', '-t', os.environ['TMUX_PANE'], '@questarm_role', sys.argv[1])
details.viewer_status = lambda: {{'simulation': {{'message': '等待 Quest 连接', 'tracking': False}}}}
details.main(sys.argv[1])
''')
        dummy_command = workspace.pane_command
        def command(role):
            return ('exec ' + shlex.join([sys.executable, str(runner), role])
                    if role in ('arm', 'teleop') else dummy_command(role))
        with patch.object(workspace, 'pane_command', side_effect=command), \
             patch.object(workspace, 'selected_mode', return_value='simulation'):
            workspace.start()
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                windows = workspace.panes()
                if all(windows.get(role, ('', True, ''))[2] == 'details' for role in ('arm', 'teleop')):
                    break
                time.sleep(.05)
            else:
                self.fail('persistent details did not start')
            # Wait beyond a refresh, catching the original immediate-exit bug.
            time.sleep(1.2)
            for role in ('arm', 'teleop'):
                pane, dead, state = workspace.panes()[role]
                self.assertFalse(dead)
                self.assertEqual(state, 'details')
                self.assertIn('MuJoCo', workspace.tmux('capture-pane', '-p', '-t', pane).stdout)
                workspace.stop_pane(role)
                self.assertTrue(workspace.panes()[role][1])
            workspace.start()  # Reopening also repairs old dead simulation panes.
            self.assertTrue(all(not workspace.panes()[r][1] for r in ('arm', 'teleop')))

    def test_reuse_existing_services_without_executing_scripts(self):
        result = subprocess.CompletedProcess([], 0, stdout="1\n")
        with patch.dict("os.environ", {"TMUX_PANE": "%1"}), \
             patch.object(workspace, "tmux", return_value=result), \
             patch.object(workspace, "check_cancelled"), \
             patch.object(workspace, "arm_info", return_value={"hardware": True}), \
             patch.object(workspace, "teleop_running", return_value=True), \
             patch.object(workspace, "execute") as execute:
            workspace.run_pane("arm")
            workspace.run_pane("teleop")
            execute.assert_not_called()

    def test_virtual_arm_never_starts_hardware_teleop(self):
        result = subprocess.CompletedProcess([], 0, stdout="1\n")
        with patch.dict("os.environ", {"TMUX_PANE": "%1"}), \
             patch.object(workspace, "tmux", return_value=result), \
             patch.object(workspace, "check_cancelled"), \
             patch.object(workspace, "teleop_running", return_value=False), \
             patch.object(workspace, "arm_info", return_value={"hardware": False}), \
             patch.object(workspace, "execute") as execute:
            with self.assertRaisesRegex(RuntimeError, "虚拟机械臂"):
                workspace.run_pane("teleop")
            execute.assert_not_called()

    def test_offline_devices_wait_then_start_without_timeout(self):
        result = subprocess.CompletedProcess([], 0, stdout="1\n")
        info = {"hardware": True, "state": {"host_monotonic_ns": time.monotonic_ns()}}
        with patch.dict("os.environ", {"TMUX_PANE": "%1"}), \
             patch.object(workspace, "tmux", return_value=result), \
             patch.object(workspace, "check_cancelled"), \
             patch.object(workspace.time, "sleep"), \
             patch.object(workspace, "panes", return_value={"arm": ("%2", True, "console")}), \
             patch.object(workspace, "teleop_running", return_value=False), \
             patch.object(workspace, "arm_info", side_effect=[None, info, info]), \
             patch.object(workspace, "quest_wait_reason", side_effect=["等待 Quest", None]), \
             patch.object(workspace, "execute") as execute:
            workspace.run_pane("teleop")
            execute.assert_called_once_with("scripts/shortcuts/启动遥操.sh")

    def test_quest_boot_and_authorization_states(self):
        def output(text):
            return subprocess.CompletedProcess([], 0, stdout=text, stderr="")
        cases = [
            ([output("List of devices attached\n")], "开机"),
            ([output("QUEST unauthorized usb:1-1")], "授权"),
            ([output("QUEST offline usb:1-1")], "连接就绪"),
            ([output("QUEST device usb:1-1"), output("0")], "完成开机"),
            ([output("QUEST device usb:1-1"), output("1")], None),
        ]
        for outputs, expected in cases:
            with self.subTest(expected=expected), patch.object(workspace.subprocess, "run", side_effect=outputs):
                actual = workspace.quest_wait_reason()
                if expected is None:
                    self.assertIsNone(actual)
                else:
                    self.assertIn(expected, actual)
        with patch.object(workspace.subprocess, "run", return_value=output("A device usb:1\nB device usb:2")):
            with self.assertRaisesRegex(RuntimeError, "多台"):
                workspace.quest_wait_reason()

    def test_power_wait_console_can_stop_without_arm_rpc(self):
        workspace.start()
        panes = self.ready()
        workspace.tmux("set-option", "-p", "-t", panes["arm"][0], "@questarm_phase", "arm-console")
        with patch.object(workspace, "arm_info", return_value=None), \
             patch.object(workspace, "teleop_running", return_value=False):
            workspace.stop()
        self.assertFalse(workspace.exists())


if __name__ == "__main__":
    unittest.main()
