"""Exercise public commands and resources without opening devices."""

from pathlib import Path
import json
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EntryPointTests(unittest.TestCase):
    def test_teleop_shortcut_wakes_first_but_help_and_wake_failure_do_not_start_control(self):
        for arguments, fail in ((["--enable-session", "--no-actuate"], False),
                                (["--help"], False), ([], True)):
            with self.subTest(arguments=arguments, wake_failure=fail), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                shortcut = root / "启动遥操.sh"
                shutil.copy2(ROOT / shortcut.name, shortcut)
                runner = root / "run.sh"
                runner.write_text('''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
with (Path(__file__).parent / "calls.jsonl").open("a") as stream:
    stream.write(json.dumps(sys.argv[1:]) + "\\n")
if sys.argv[1:3] == ["quest-wake", "start"] and os.environ["TEST_WAKE_FAIL"] == "1":
    sys.exit(1)
''')
                runner.chmod(0o755)
                result = subprocess.run([str(shortcut), *arguments], cwd="/tmp", capture_output=True,
                                        text=True, env={**os.environ, "TEST_WAKE_FAIL": str(int(fail))})
                calls = [json.loads(line) for line in (root / "calls.jsonl").read_text().splitlines()]
                control = ["--profile", "presets", "teleop-presets-x5", "--actuate", "--enable-session",
                           "--collection-task", "X5 遥操采集", "--scale-factor", "1.0", "--max-joint-speed-rad-s", "2.0", *arguments]
                self.assertEqual(calls, [["quest-transport", "prepare"], ["quest-wake", "start"]] if fail else
                                 [control] if "--help" in arguments else [["quest-transport", "prepare"], ["quest-wake", "start"], ["quest-transport", "wait"], control])
                self.assertEqual(result.returncode, int(fail), result.stderr)

    def test_public_commands_show_help_from_other_directories(self):
        for command in (
            "teleop-official-x5", "teleop-presets-x5", "teleop",
            "control-arm", "view-arm", "arm-service", "plugins", "check-xr", "check-arm",
            "test-joint", "visualize-xr", "prepare-connections",
            "teleop-x5-2025", "test-joint-2025", "rebuild-xr-sdk",
        ):
            with self.subTest(command=command):
                result = subprocess.run(
                    [str(ROOT / "run.sh"), command, "--help"],
                    cwd="/tmp", capture_output=True, text=True, timeout=30,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("usage:", result.stdout.lower())

    def test_daily_shortcuts_and_xr_page_resolve_from_other_directories(self):
        from quest_teleop.xr_visualizer.cli import HTML_PATH
        self.assertTrue(HTML_PATH.is_file())
        self.assertIn(str(ROOT / "src"), str(HTML_PATH))
        for script in ("启动遥操.sh", "启动仿真.sh", "启动机械臂服务.sh"):
            with self.subTest(script=script):
                result = subprocess.run([str(ROOT / script), "--help"], cwd="/tmp",
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("--help", result.stdout)
