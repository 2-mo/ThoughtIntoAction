"""Launcher contracts: previews and simulation must never open hardware."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
from scripts.launch import build_command, resolve_python, MODULE_COMMANDS, PYTHON_COMMANDS
from robot_core.project import load_config


class LauncherTests(unittest.TestCase):
    def preview(self, *arguments):
        result = subprocess.run(
            [str(ROOT / "run.sh"), "--print-command", *arguments],
            cwd="/tmp", capture_output=True, text=True, check=True,
        )
        return json.loads(result.stdout)

    def test_default_is_read_only_presets(self):
        result = self.preview()
        self.assertEqual(result["command"], "teleop-presets-x5")
        self.assertNotIn("--actuate", result["argv"])

    def test_monitor_shortcut_is_independent_from_control(self):
        script = (ROOT / "scripts/viewer/启动仿真.sh").read_text()
        self.assertIn('view-arm', script)
        self.assertNotIn('--input', script)
        result = self.preview('view-arm')
        self.assertEqual(result['command'], 'view-arm')
        for flag in ('--can-port', '--input', '--backend', '--actuate'):
            self.assertNotIn(flag, result['argv'])
        control = self.preview('--profile', 'virtual-arm', 'control-arm', '--input', 'quest')
        self.assertEqual(control['command'], 'control-arm')
        self.assertNotIn('--viewer', control['argv'])
        self.assertNotIn("--telemetry", control["argv"])

    def test_main_profile_preserves_user_parameters(self):
        result = self.preview("--profile", "presets")
        tokens = result["argv"]
        for key, value in [("--control-mode", "pose"), ("--scale-factor", "1.0"),
                           ("--max-joint-speed-rad-s", "2.0")]:
            self.assertEqual(tokens[tokens.index(key) + 1], value)

    def test_retired_simulation_commands_have_no_aliases(self):
        for command in ("simulate", "sim-preview", "simulate-quest", "simulate-x5"):
            result = subprocess.run([str(ROOT / "run.sh"), command, "--help"],
                                    cwd="/tmp", capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertFalse((ROOT / "configs/profiles/quest-simulation.toml").exists())
        self.assertFalse((ROOT / "src/quest_teleop/legacy/simulation.py").exists())

    def test_profile_mismatch_fails_before_execution(self):
        result = subprocess.run([str(ROOT / "run.sh"), "--print-command", "--profile", "presets", "control-arm"], capture_output=True)
        self.assertNotEqual(result.returncode, 0)

    def test_local_paths_resolve_from_repository(self):
        with tempfile.TemporaryDirectory() as folder:
            config_path = Path(folder) / "local.toml"
            config_path.write_text('[robot]\ncan_port="can7"\n[paths]\nlog_dir="outputs/test logs"\n')
            result = self.preview("--config", str(config_path), "arm-service")
            self.assertIn("can7", result["argv"])
            self.assertEqual(result["log_dir"], str(ROOT / "outputs/test logs"))

    def test_xr_only_environment_excludes_optional_x5_driver(self):
        _, env = build_command("check-xr", [], load_config())
        self.assertNotIn("third_party/X5/", env["PYTHONPATH"])
        self.assertNotIn("Desktop", env["PYTHONPATH"])
        self.assertIn("third_party/XRoboToolkit-Teleop-Sample-Python", env["PYTHONPATH"])

    def test_optional_backend_selection_supplies_its_native_library_paths(self):
        _, env = build_command("test-joint", ["--backend", "arx2025", "--help"], load_config())
        self.assertIn("third_party/X5/", env["PYTHONPATH"])
        self.assertIn("bimanual/api/arx_x5_src", env["LD_LIBRARY_PATH"])

    def test_commands_resolve_without_compatibility_source_paths(self):
        for name in MODULE_COMMANDS | PYTHON_COMMANDS:
            argv, env = build_command(name, ["--help"], load_config())
            with self.subTest(command=name):
                self.assertNotIn(str(ROOT / "myTeleop"), env["PYTHONPATH"])
                self.assertNotIn(str(ROOT / "scripts"), env["PYTHONPATH"])
                self.assertFalse(any("myTeleop" in token for token in argv))
                if name in PYTHON_COMMANDS:
                    self.assertTrue(Path(argv[2]).is_file())
                else:
                    self.assertEqual(argv[2:4], ["-m", MODULE_COMMANDS[name]])

    def test_dry_run_cannot_be_disabled(self):
        result = subprocess.run([str(ROOT / "run.sh"), "--print-command", "teleop-dry-run", "--no-dry-run"], capture_output=True)
        self.assertNotEqual(result.returncode, 0)

    def test_default_interpreter_uses_project_venv_from_another_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / "local.toml"
            config.write_text("")
            with patch.dict(os.environ):
                os.environ.pop("QUEST_TELEOP_PYTHON", None)
                argv = self.preview("--config", str(config), "check-xr")["argv"]
            # Preserve the venv symlink path; resolving it would select the base interpreter.
            self.assertEqual(argv[0], str(ROOT / ".venv/bin/python"))

    def test_missing_environment_reports_sync_instead_of_using_another_python(self):
        with patch.dict(os.environ, {"QUEST_TELEOP_PYTHON": "/missing-quest-env/bin/python"}):
            with self.assertRaisesRegex(ValueError, "uv sync --locked"):
                resolve_python(load_config())

    def test_source_checks_can_bootstrap_before_environment_exists(self):
        with patch.dict(os.environ, {"QUEST_TELEOP_PYTHON": "/missing-quest-env/bin/python"}):
            for command in ("check-dependencies", "fetch-dependencies"):
                result = subprocess.run([str(ROOT / "run.sh"), command, "--help"],
                                        cwd="/tmp", capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
