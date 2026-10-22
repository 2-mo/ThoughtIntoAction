"""Isolated control launch and import provenance; never start a real backend."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from embodied.applications.workstation.control import ROOT, control_environment, verify_sources


class ControlEntryTests(unittest.TestCase):
    def test_remote_start_uses_web_view_without_a_desktop_dependency(self):
        import io
        from embodied.applications.workstation import control
        for environment, explicit, expected in [
                ({'SSH_CONNECTION': 'remote 123 local 22', 'DISPLAY': 'localhost:10.0'}, False, True),
                ({}, False, True),
                ({'DISPLAY': ':0'}, False, False),
                ({'DISPLAY': ':0'}, True, True)]:
            with self.subTest(environment=environment, explicit=explicit), \
                 patch.dict(os.environ, environment, clear=True), \
                 patch.object(control.sys, 'platform', 'linux'), \
                 patch.object(control, 'verify_sources', return_value={}), \
                 patch('sys.stdout', new_callable=io.StringIO) as output:
                self.assertEqual(control.main(['--print-command', 'start', '--backend', 'sim',
                                               '--input', 'idle', *(['--headless'] if explicit else [])]), 0)
                command = json.loads(output.getvalue())['argv']
                self.assertEqual('--headless' in command, expected)

    def test_real_bootstrap_hands_over_before_loading_numeric_plugins(self):
        script = '''
import json, sys
from pathlib import Path
from unittest.mock import patch
from embodied.applications.workstation import control
arguments = ['start', '--backend', 'real', '--input', 'idle', '--managed']
def handover(python, command, env):
    assert 'numpy' not in sys.modules
    assert command == [str(control.CONTROL_PYTHON), '-B', '-m',
                       'embodied.applications.workstation.control', *arguments]
    assert env['PYTHONPATH'] == str(control.ROOT / 'src')
    raise SystemExit(0)
with patch.object(control, 'CONTROL_PYTHON', Path('/control-env/bin/python')), \\
     patch.object(control, 'command_for', return_value=['/control-env/bin/python']), \\
     patch.object(control, 'verify_sources', return_value={}), \\
     patch.object(control, 'hardware_environment', side_effect=AssertionError('plugin imported in bootstrap')), \\
     patch.object(control.os, 'execvpe', side_effect=handover):
    control.main(arguments)
'''
        result = subprocess.run(['/usr/bin/python3', '-S', '-c', script],
                                env={**os.environ, 'PYTHONPATH': str(ROOT / 'src')},
                                text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_control_interpreter_prepares_plugin_environment_before_final_exec(self):
        from embodied.applications.workstation import control
        class Executed(Exception):
            pass
        config = str((Path.cwd() / 'custom-robot.json').resolve())
        final_command = ['/control-env/bin/python', '-B', '-m', 'embodied.control', 'run']
        prepared = {'LD_LIBRARY_PATH': '/driver/libs', 'PYTHONPATH': '/driver/python'}
        with patch.object(control, 'CONTROL_PYTHON', Path('/control-env/bin/python')), \
             patch.object(control.sys, 'prefix', '/control-env'), \
             patch.object(control, 'verify_sources', return_value={}), \
             patch.object(control, 'command_for', return_value=final_command), \
             patch.object(control, 'hardware_environment', return_value=prepared) as prepare, \
             patch.object(control.os, 'chdir'), \
             patch.object(control.os, 'execvpe', side_effect=Executed) as execute:
            with self.assertRaises(Executed):
                control.main(['start', '--backend', 'real', '--managed', '--input', 'idle',
                              '--robot-plugin', 'selected.hardware', '--config', 'custom-robot.json'])
        self.assertEqual(prepare.call_args.kwargs, {'robot_plugin': 'selected.hardware', 'config': config})
        execute.assert_called_once_with(final_command[0], final_command, prepared)

    def test_legacy_and_arx_lease_exclude_each_other_across_processes(self):
        from embodied.contracts.device_lease import DeviceLease
        import uuid
        device = 'offline-lease-' + uuid.uuid4().hex
        lease = DeviceLease(device)
        self.addCleanup(lease.close)
        source = ('from embodied.contracts.device_lease import DeviceLease; '
                  'DeviceLease(' + repr(device) + ')')
        env = {**os.environ, 'PYTHONPATH': str(ROOT / 'src')}  # the control interpreter loads the same L0 lease
        python = str(ROOT / 'outputs/envs/control/bin/python')
        if not Path(python).is_file():
            self.skipTest('Control environment not installed')
        blocked = subprocess.run([python, '-B', '-c', source], env=env, capture_output=True, text=True)
        self.assertNotEqual(blocked.returncode, 0)
        self.assertIn('owned by another', blocked.stderr)
        lease.close()
        released = subprocess.run([python, '-B', '-c', source], env=env, capture_output=True, text=True)
        self.assertEqual(released.returncode, 0, released.stderr)

    def test_source_import_is_accounted_for(self):
        self.assertEqual(verify_sources()["status"], "verified")

    def test_provenance_checks_distribution_assets_without_freezing_platform_code(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            vendor = root / "src/embodied/devices/robots/arx5/vendor/arx5-sdk"
            native = root / "src/embodied/devices/inputs/quest/native"
            vendor.mkdir(parents=True)
            (native / 'bin').mkdir(parents=True)
            (vendor / 'source.tar').write_bytes(b'sdk archive')
            (vendor / 'gripper_turns.h').write_bytes(b'patch')
            (native / 'bin/router').write_bytes(b'native binary')
            digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
            (vendor / 'PROVENANCE.json').write_text(json.dumps({
                'archive_sha256': digest(vendor / 'source.tar'),
                'gripper_turns_sha256': digest(vendor / 'gripper_turns.h'),
            }))
            (native / 'SHA256SUMS').write_text(f"{digest(native / 'bin/router')}  bin/router\n")
            with patch('embodied.applications.workstation.control.ROOT', root), \
                    patch('embodied.applications.workstation.control.SDK_VENDOR', vendor):
                self.assertEqual(verify_sources()['files'], 3)
                (root / 'src/platform.py').write_text('maintained local source')
                self.assertEqual(verify_sources()['status'], 'verified')
                (vendor / 'source.tar').write_bytes(b'changed archive')
                with self.assertRaisesRegex(ValueError, 'SDK asset checksum mismatch'):
                    verify_sources()

    def test_control_removes_foreign_environment_paths(self):
        names = ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "LD_LIBRARY_PATH", "UV_PROJECT_ENVIRONMENT")
        with patch.dict(os.environ, dict.fromkeys(names, "/foreign/python312")):
            env = control_environment()
        self.assertFalse((set(names) - {'PYTHONPATH', 'UV_PROJECT_ENVIRONMENT'}) & env.keys())
        self.assertEqual(env['PYTHONPATH'], str(ROOT / 'src'))
        self.assertEqual(env['UV_PROJECT_ENVIRONMENT'], str(ROOT / 'outputs/envs/control'))
        self.assertEqual(env["OPENBLAS_NUM_THREADS"], "1")

    def test_verify_works_without_legacy_environment_or_sdk(self):
        env = {**os.environ, "QUEST_TELEOP_PYTHON": "/missing/python", "PYTHONDONTWRITEBYTECODE": "1"}
        result = subprocess.run([str(ROOT / "run.sh"), "control", "verify"], cwd="/tmp",
                                env=env, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["status"], "verified")

    def test_demo_never_accepts_a_real_backend(self):
        from embodied.applications.workstation.control import command_for
        from argparse import Namespace
        with patch("embodied.applications.workstation.control.Path.is_file", return_value=True):
            with self.assertRaisesRegex(ValueError, "simulation only"):
                command_for(Namespace(command="start", backend="real", input="demo"))


if __name__ == "__main__":
    unittest.main()
