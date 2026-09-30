#!/usr/bin/env python3
"""Offline relocation, dependency boundary and bundled-asset checks."""
from contextlib import redirect_stdout
from dataclasses import replace
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from arx5_vr.settings import load_settings
from arx5_vr.model import X5Model


class PortableChecks(unittest.TestCase):
    def test_external_profile_paths_resolve_from_each_config_file(self):
        with tempfile.TemporaryDirectory(prefix='vr relocation 中文 ') as directory:
            config = Path(directory) / 'robot.json'
            vr_dir = Path(directory) / 'vr'
            vr_dir.mkdir()
            config.write_text(json.dumps({'model_path': 'model/arm.xml', 'tool_path': 'tool.json'}))
            vr = vr_dir / 'input.json'
            vr.write_text(json.dumps({'sdk_library': 'lib/sdk.so'}))
            with patch.dict(os.environ, {'XROBOT_SDK_LIBRARY': ''}):
                settings = load_settings(config, vr)
            self.assertEqual(settings.robot.model_path, str(config.parent / 'model/arm.xml'))
            self.assertEqual(settings.robot.tool_path, str(config.parent / 'tool.json'))
            self.assertEqual(settings.sdk_library, str(vr_dir / 'lib/sdk.so'))

    def test_sdk_environment_override_does_not_change_robot(self):
        base = load_settings()
        with patch.dict(os.environ, {'XROBOT_SDK_LIBRARY': '~/vr sdk/lib.so'}):
            settings = load_settings()
        self.assertEqual(settings.robot, base.robot)
        self.assertEqual(settings.sdk_library, str(Path('~/vr sdk/lib.so').expanduser().resolve()))

    def test_model_loads_from_separate_directory_and_rejects_stale_tool(self):
        config = load_settings().robot
        model = X5Model(config)
        self.assertEqual(model.model.nq, 8)
        self.assertEqual(model.tcp_link, 'tcp_link')
        self.assertTrue(model.urdf_path.resolve().is_relative_to(ROOT / 'models'))
        self.assertEqual(model.tool_path, ROOT / 'config/tool.json')
        self.assertEqual(model.fk(config.park_q).shape, (7,))
        with tempfile.TemporaryDirectory() as directory:
            tool = Path(directory) / 'tool.json'
            tool.write_bytes(model.tool_path.read_bytes() + b' ')
            with self.assertRaisesRegex(ValueError, 'Model/tool mismatch'):
                X5Model(replace(config, tool_path=str(tool)))

    def test_model_rebuild_preserves_urdf_and_mujoco_bytes(self):
        from scripts.build_model import build
        model_dir = ROOT / 'models/x5'
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            build(model_dir / 'x5_2025.source.urdf', model_dir / 'meshes',
                  tool=ROOT / 'config/tool.json', output_dir=output)
            for name in ('x5_2025.urdf', 'x5_2025.xml'):
                self.assertEqual((output / name).read_bytes(), (model_dir / name).read_bytes())
            X5Model(replace(load_settings().robot, model_path=str(output / 'x5_2025.xml')))

    def test_control_modules_import_without_vr_or_robot_sdk(self):
        code = '''
import importlib.abc, sys
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'iceoryx2', 'arx5_interface'}:
            raise ImportError('Optional SDK imported: ' + fullname)
sys.meta_path.insert(0, Block())
from arx5_vr import real, sim, model, types
'''
        result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_doctor_allows_simulation_without_hardware_packages(self):
        from arx5_vr import app
        original = app.importlib.metadata.version
        def optional_missing(name):
            if name in {'arx5-interface'}:
                raise app.importlib.metadata.PackageNotFoundError(name)
            return original(name)
        for name in ('config/robot.json',):
            with self.subTest(profile=name), patch.object(app.importlib.metadata, 'version', optional_missing):
                output = io.StringIO()
                with redirect_stdout(output):
                    result = app.doctor(ROOT / name)
                self.assertEqual(result, 0, output.getvalue())

    def test_native_config_dry_run_never_loads_external_sdk(self):
        from arx5_vr.native_config import write_native_configs
        with tempfile.TemporaryDirectory(prefix='vr-native-') as directory:
            config = load_settings().robot
            paths = write_native_configs(Path(directory), 'arx5-migration-check',
                                         config.gripper_max_width, '/missing/xr-sdk.so',
                                         0.)
            self.assertIn('gripper', paths)
            result = subprocess.run([str(ROOT / 'native/bin/rollio-device-xrobot'), 'run',
                                     '--config', str(paths['xrobot']), '--dry-run'],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_bundled_native_files_match_provenance(self):
        for line in (ROOT / 'native/SHA256SUMS').read_text().splitlines():
            digest, relative = line.split(maxsplit=1)
            self.assertEqual(hashlib.sha256((ROOT / relative).read_bytes()).hexdigest(), digest, relative)

    def test_release_excludes_environment_recordings_and_external_links(self):
        from scripts.package_release import release_files
        paths = release_files()
        for path in paths:
            self.assertFalse({'.venv', '__pycache__', 'run', 'dist', '.env', 'backups'}
                             .intersection(path.relative_to(ROOT).parts))
        self.assertIn(ROOT / 'uv.lock', paths)
        self.assertIn(ROOT / 'native/bin/rollio-teleop-router', paths)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'arx5_vr').mkdir()
            (root / 'arx5_vr/escape.py').symlink_to('/etc/hosts')
            with patch('scripts.package_release.FILES', ()), self.assertRaisesRegex(ValueError, 'External symlink'):
                release_files(root)


if __name__ == '__main__':
    unittest.main(verbosity=2)
