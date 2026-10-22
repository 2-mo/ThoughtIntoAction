"""Launcher contracts: previews and simulation must never open hardware."""

import json
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
from scripts.launch import build_command, resolve_python, MODULE_COMMANDS, PYTHON_COMMANDS
from embodied.core.project import load_config


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
        script = (ROOT / "scripts/viewer/monitor.sh").read_text()
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
        self.assertFalse((ROOT / "src/legacy/quest_teleop/legacy/simulation.py").exists())

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
        self.assertNotIn("x5-official-", env["PYTHONPATH"])
        self.assertNotIn("Desktop", env["PYTHONPATH"])
        self.assertNotIn("third_party", env["PYTHONPATH"])
        self.assertNotIn("third_party", env["LD_LIBRARY_PATH"])

    def test_optional_backend_selection_supplies_its_native_library_paths(self):
        _, env = build_command("test-joint", ["--backend", "arx2025", "--help"], load_config())
        self.assertIn("outputs/dependencies/x5-official-", env["PYTHONPATH"])
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
            for command in ("check-dependencies", "fetch-dependencies", "check-all"):
                result = subprocess.run([str(ROOT / "run.sh"), command, "--help"],
                                        cwd="/tmp", capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_bench_is_available_without_project_environment_or_legacy_sdk(self):
        config = load_config()
        config['environment']['arm_sdk_python'] = '/missing-device-sdk'
        with patch.dict(os.environ, {'QUEST_TELEOP_PYTHON': '/missing-project-env/bin/python'}):
            argv, env = build_command('bench', ['status'], config)
        self.assertEqual(argv[0], sys.executable)
        self.assertEqual(env['PYTHONPATH'], str(ROOT / 'src'))
        self.assertNotIn('LD_LIBRARY_PATH', env)

    def test_examples_use_project_environment_without_device_sdk_paths(self):
        config = load_config()
        config['environment']['arm_sdk_python'] = '/missing-device-sdk'
        arguments = ['movej', '--joint', '2', '--degrees', '5', '--preview-only']
        with patch.dict(os.environ, {'PYTHONHOME': '/old-python', 'LD_LIBRARY_PATH': '/old-sdk'}):
            argv, env = build_command('examples', arguments, config)
        self.assertEqual(argv[2:], ['-m', 'examples', *arguments])
        self.assertEqual(env['PYTHONPATH'], str(ROOT / 'src'))
        self.assertNotIn('PYTHONHOME', env)
        self.assertNotIn('LD_LIBRARY_PATH', env)


class CatalogTests(unittest.TestCase):
    """`./run.sh <group> <command>` names existing commands, scripts and components."""

    def test_every_entry_has_a_target(self):
        from embodied.applications.workstation.catalog import GROUPS
        import scripts.launch as launch
        flat = (launch.PYTHON_COMMANDS | launch.SHELL_COMMANDS | launch.MAINTENANCE_COMMANDS
                | launch.MODULE_COMMANDS | launch.BOOTSTRAP_COMMANDS)
        components = set(json.loads((ROOT / "configs/components/commands.json").read_text())["commands"])
        for group, (_, commands) in GROUPS.items():
            for name, (_, target) in commands.items():
                with self.subTest(command=f"{group} {name}"):
                    if target[0] == "run":
                        self.assertIn(target[1][0], flat)
                    elif target[0] == "component":
                        self.assertIn(target[1], components)
                    elif target[0] == "bash":
                        self.assertTrue((ROOT / target[1]).is_file())

    def test_groups_resolve_to_launcher_commands(self):
        from scripts.launch import resolve_group
        self.assertEqual(resolve_group(["arm", "check", "--virtual"]), ["check-arm", "--virtual"])
        self.assertEqual(resolve_group(["calib", "payload", "--help"]), ["components", "run", "calibration.payload", "--help"])
        self.assertEqual(resolve_group(["phone", "--run", "R"]), ["phone", "--run", "R"])  # single-word command
        with patch("builtins.print"):
            self.assertEqual(resolve_group(["check", "bogus"]), 2)
            self.assertEqual(resolve_group(["data"]), 0)

    def test_bare_run_prints_a_compact_home(self):
        result = subprocess.run([str(ROOT / "run.sh")], capture_output=True, text=True, cwd="/tmp")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("kernel", result.stdout)
        self.assertIn("./run.sh examples", result.stdout)
        self.assertIn("./run.sh <功能> <命令>", result.stdout)
        self.assertLess(len(result.stdout.splitlines()), 30)
        self.assertNotIn('\x1b[', result.stdout)

    def test_targeted_help_and_search_need_no_environment(self):
        with patch.dict(os.environ, {'QUEST_TELEOP_PYTHON': '/missing-project-env/bin/python'}):
            group = subprocess.run([str(ROOT / 'run.sh'), 'help', 'data'], capture_output=True, text=True)
            search = subprocess.run([str(ROOT / 'run.sh'), 'search', '相机'], capture_output=True, text=True)
        self.assertEqual(group.returncode, 0, group.stderr)
        self.assertIn('export', group.stdout)
        self.assertNotIn('gripper-position', group.stdout)
        self.assertEqual(search.returncode, 0, search.stderr)
        self.assertIn('./run.sh camera check', search.stdout)
        self.assertNotIn('./run.sh arm home', search.stdout)

    def test_help_aliases_are_compact_and_full_catalog_is_explicit(self):
        from scripts.launch import main
        for arguments in (['help'], ['-h'], ['--help'], ['help', '--all']):
            with self.subTest(arguments=arguments), \
                 patch('sys.stdout', new_callable=io.StringIO) as output, \
                 patch('scripts.launch.load_config', side_effect=AssertionError('help must not load device config')), \
                 patch('scripts.launch.os.execvpe') as execute:
                self.assertEqual(main(arguments), 0)
                text = output.getvalue()
                execute.assert_not_called()
                self.assertNotIn('\x1b[', text)
                if '--all' in arguments:
                    self.assertIn('./run.sh arm gripper-position', text)
                    self.assertIn('./run.sh web tailnet', text)
                else:
                    self.assertLessEqual(len(text.splitlines()), 25)
                    self.assertNotIn('gripper-position', text)
                    self.assertIn('./run.sh help --all', text)
                    self.assertIn('./run.sh search 相机', text)
                    self.assertIn('./run.sh examples', text)

    def test_examples_directory_help_reuses_the_existing_catalog_without_devices(self):
        from examples.__main__ import EXAMPLES
        from scripts.launch import main
        with patch('sys.stdout', new_callable=io.StringIO) as output, \
             patch('scripts.launch.load_config', side_effect=AssertionError('help must not load device config')), \
             patch('examples.__main__.import_module', side_effect=AssertionError('listing must not import examples')):
            self.assertEqual(main(['help', 'examples']), 0)
        for name in EXAMPLES:
            self.assertIn(name, output.getvalue())

    def test_terminal_help_works_without_rich(self):
        from scripts.launch import main
        with patch('sys.stdout', new_callable=io.StringIO) as output, \
             patch.object(output, 'isatty', return_value=True), \
             patch.dict(os.environ, {'TERM': 'xterm'}), \
             patch.dict(sys.modules, {'rich.console': None}):
            self.assertEqual(main(['help']), 0)
            self.assertIn('QuestArm', output.getvalue())
            self.assertIn('./run.sh help --all', output.getvalue())

    def test_rich_help_keeps_literal_parameters_on_narrow_terminals(self):
        from rich.console import Console
        from embodied.applications.workstation import help as command_help
        for width in (48, 80):
            with self.subTest(width=width):
                output = io.StringIO()
                console = Console(file=output, width=width, color_system=None, force_terminal=True)
                with patch.object(command_help, '_console', return_value=console):
                    command_help.print_help('collect')
                text = output.getvalue()
                self.assertIn('./run.sh collect run', text)
                self.assertIn('[--cameras', text)
                self.assertIn('wrist_rgb,exterior_rgb]', text)
                self.assertIn('<RUN>', text)
                self.assertNotIn('…', text)

    def test_unknown_command_offers_a_recovery_command(self):
        result = subprocess.run([str(ROOT / 'run.sh'), 'kernle'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('./run.sh kernel', result.stderr)
        self.assertNotIn('Traceback', result.stderr)
        self.assertLess(len(result.stderr.splitlines()), 6)

    def test_grouped_script_previews_never_execute(self):
        from scripts.launch import main
        from embodied.applications.workstation.catalog import GROUPS
        for group, (_, commands) in GROUPS.items():
            for name, (_, target) in commands.items():
                if target[0] not in ('bash', 'workspace'):
                    continue
                with self.subTest(command=f'{group} {name}'), \
                     patch('scripts.launch.os.execvp') as execute, \
                     patch('scripts.launch.os.execv') as execute_path, \
                     patch('sys.stdout', new_callable=io.StringIO) as output:
                    self.assertEqual(main(['--print-command', group, name, '--help']), 0)
                    execute.assert_not_called()
                    execute_path.assert_not_called()
                    preview = json.loads(output.getvalue())
                    self.assertEqual(preview['command'], f'{group} {name}')
                    self.assertEqual(preview['argv'][-1], '--help')


if __name__ == "__main__":
    unittest.main()
