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
import tarfile
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]

from embodied.control.settings import load_settings
from embodied.devices.robots.arx5.kinematics import X5Model


class PortableChecks(unittest.TestCase):
    def test_motion_runs_from_release_in_the_same_source_layout(self):
        from scripts.control import package_release
        with tempfile.TemporaryDirectory(prefix='motion release ') as directory:
            directory = Path(directory)
            output = directory / 'control.tar.gz'
            # Avoid recompressing SDK binaries and meshes in this source-layout check.
            with patch.object(package_release, 'DIRECTORIES', ('src/embodied/contracts', 'src/embodied/motion')), \
                    patch.object(package_release, 'FILES', ('src/embodied/__init__.py', 'configs/plugins.json')), \
                    patch.object(sys, 'argv', ['package_release', '--output', str(output)]), \
                    redirect_stdout(io.StringIO()):
                package_release.main()
            with tarfile.open(output) as archive:
                names = archive.getnames()
                self.assertEqual(len(names), len(set(names)))
                self.assertFalse(any('/shared/' in name for name in names))
                archive.extractall(directory)
            root = directory / package_release.ARCHIVE_ROOT
            for line in (root / 'MANIFEST.sha256').read_text().splitlines():
                digest, relative = line.split(maxsplit=1)
                self.assertEqual(hashlib.sha256((root / relative).read_bytes()).hexdigest(), digest)
            script = f'''import sys
from pathlib import Path
sys.path.insert(0, {str(root / 'src')!r})
import numpy as np
import embodied.motion.offline as motion
from embodied.motion import plan_ruckig_joint_trajectory
assert Path(motion.__file__).is_relative_to({str(root)!r})
assert plan_ruckig_joint_trajectory is motion.plan_ruckig_joint_trajectory
assert 'embodied.motion.pinocchio_backend' not in sys.modules
assert not any(name.startswith('legacy.') for name in sys.modules)
q = np.zeros(6); goal = q.copy(); goal[0] = np.deg2rad(10)
plan = plan_ruckig_joint_trajectory(q, goal, q, min_position=np.full(6,-2.),
    max_position=np.full(6,2.), max_velocity=.5, max_acceleration=2., max_jerk=10., sample_rate_hz=250)
assert .82 < plan.duration_s < .83
np.testing.assert_allclose(plan.at_time(plan.duration_s)[0], goal, atol=1e-12)
np.testing.assert_array_equal(plan.at_time(plan.duration_s)[1], q)
'''
            result = subprocess.run([sys.executable, '-I', '-c', script], cwd=directory,
                                    text=True, capture_output=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_external_profile_paths_resolve_from_each_config_file(self):
        with tempfile.TemporaryDirectory(prefix='vr relocation 中文 ') as directory:
            config = Path(directory) / 'robot.json'
            vr_dir = Path(directory) / 'vr'
            vr_dir.mkdir()
            config.write_text(json.dumps({'profile': 'arx5', 'model_path': 'model/arm.xml', 'tool_path': 'tool.json'}))
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
        self.assertTrue(model.urdf_path.resolve().is_relative_to(ROOT / 'assets/robots/x5/runtime'))
        self.assertEqual(model.tool_path, ROOT / 'configs/robots/arx5/tool.json')
        self.assertEqual(model.fk(config.park_q).shape, (7,))
        with tempfile.TemporaryDirectory() as directory:
            tool = Path(directory) / 'tool.json'
            tool.write_bytes(model.tool_path.read_bytes() + b' ')
            with self.assertRaisesRegex(ValueError, 'Model/tool mismatch'):
                X5Model(replace(config, tool_path=str(tool)))

    def test_model_rebuild_preserves_urdf_and_mujoco_bytes(self):
        from scripts.control.build_model import build
        model_dir = ROOT / 'assets/robots/x5/runtime'
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            build(model_dir / 'x5_2025.source.urdf', model_dir / 'meshes',
                  tool=ROOT / 'configs/robots/arx5/tool.json', output_dir=output)
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
from embodied.control import app, settings
from embodied.devices.robots.arx5 import real, simulation, kinematics
from embodied.contracts import control
'''
        result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_doctor_allows_simulation_without_hardware_packages(self):
        from embodied.control import app
        settings = load_settings()
        registry = Mock()
        driver = registry.create.return_value
        driver.preflight.return_value = {'ready': True, 'errors': []}
        output = io.StringIO()
        with patch.object(app, 'load_settings', return_value=settings), \
                patch.object(app, 'Registry', return_value=registry), redirect_stdout(output):
            result = app.doctor(ROOT / 'configs/robots/arx5/robot.json')
        self.assertEqual(result, 0, output.getvalue())
        registry.create.assert_called_once_with('robot', settings.robot.simulation_plugin, runtime='control')
        driver.preflight.assert_called_once_with(settings.robot)
        registry.load.assert_not_called()
        self.assertNotIn('vr', settings.__dict__)

    def test_native_config_dry_run_never_loads_external_sdk(self):
        from embodied.devices.inputs.quest.native_config import write_native_configs
        with tempfile.TemporaryDirectory(prefix='vr-native-') as directory:
            config = load_settings().robot
            paths = write_native_configs(Path(directory), 'arx5-migration-check',
                                         config.gripper_max_width, '/missing/xr-sdk.so',
                                         0.)
            self.assertIn('gripper', paths)
            result = subprocess.run([str(ROOT / 'src/embodied/devices/inputs/quest/native/bin/rollio-device-xrobot'), 'run',
                                     '--config', str(paths['xrobot']), '--dry-run'],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_bundled_native_files_match_provenance(self):
        native = ROOT / 'src/embodied/devices/inputs/quest/native'
        for line in (native / 'SHA256SUMS').read_text().splitlines():
            digest, relative = line.split(maxsplit=1)
            self.assertEqual(hashlib.sha256((native / relative).read_bytes()).hexdigest(), digest, relative)

    def test_release_excludes_environment_recordings_and_external_links(self):
        from scripts.control.package_release import release_files
        paths = release_files()
        for path in paths:
            self.assertFalse({'.venv', '__pycache__', 'run', 'dist', '.env', 'backups'}
                             .intersection(path.relative_to(ROOT).parts))
        self.assertIn(ROOT / 'configs/environments/control/uv.lock', paths)
        self.assertIn(ROOT / 'src/embodied/devices/inputs/quest/native/bin/rollio-teleop-router', paths)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'src/embodied/control').mkdir(parents=True)
            (root / 'src/embodied/control/escape.py').symlink_to('/etc/hosts')
            with patch('scripts.control.package_release.FILES', ()), \
                    patch('scripts.control.package_release.DIRECTORIES', ('src/embodied/control',)), \
                    self.assertRaisesRegex(ValueError, 'External symlink'):
                release_files(root)


if __name__ == '__main__':
    unittest.main(verbosity=2)
