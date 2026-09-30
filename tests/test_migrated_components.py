"""Cross-component ownership and portability contracts; no hardware."""
import json
from pathlib import Path
import subprocess
import sys
import unittest
import uuid

from component_api.registry import Registry
from component_plugins.cli import command
from component_plugins.ownership import leased_connect, leased_disconnect, leased_operation
from robot_core.device_lease import DeviceLease

ROOT = Path(__file__).resolve().parents[1]


class MigrationContracts(unittest.TestCase):
    def test_discovery_does_not_import_device_sdks(self):
        script = '''
import sys
from component_api.registry import Registry
r=Registry()
assert r.spec('device', 'rebot_dm').hardware
assert r.spec('input', 'mock').simulation_only
assert not {'motorbridge','arx5_interface','pyrealsense2','torch'} & sys.modules.keys()
'''
        result = subprocess.run([sys.executable, '-c', script], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_migrated_device_and_native_worker_share_ownership(self):
        device = 'migration-test-' + uuid.uuid4().hex

        class Adapter:
            connected = False
            @leased_connect(lambda self: device)
            def connect(self):
                self.connected = True
            @leased_disconnect
            def disconnect(self):
                self.connected = False

        arm = Adapter()
        with self.assertRaisesRegex(RuntimeError, 'owned'):
            with_lease = DeviceLease(device)
            try:
                arm.connect()
            finally:
                with_lease.close()
        self.assertFalse(arm.connected)
        arm.connect()
        try:
            with self.assertRaisesRegex(RuntimeError, 'owned'):
                DeviceLease(device)
        finally:
            arm.disconnect()
        DeviceLease(device).close()

    def test_failed_connect_releases_lease(self):
        device = 'migration-test-' + uuid.uuid4().hex
        class Broken:
            @leased_connect(lambda self: device)
            def connect(self):
                raise ValueError('startup failed')
        with self.assertRaisesRegex(ValueError, 'startup failed'):
            Broken().connect()
        DeviceLease(device).close()

    def test_motor_maintenance_uses_the_same_exclusive_lease(self):
        device = 'migration-test-' + uuid.uuid4().hex
        @leased_operation(lambda: device)
        def maintenance():
            with self.assertRaisesRegex(RuntimeError, 'owned'):
                DeviceLease(device)
            return 'completed'
        self.assertEqual(maintenance(), 'completed')
        DeviceLease(device).close()

    def test_commands_resolve_only_local_implementation_paths(self):
        commands = json.loads((ROOT / 'configs/components/commands.json').read_text())['commands']
        for name, spec in commands.items():
            argv, cwd, env = command(name, ['--help'])
            self.assertTrue(cwd.is_relative_to(ROOT))
            self.assertEqual(env['PYTHONPATH'], str(ROOT / 'src'))
            for source in ('arx5_test', 'reBotArm_control_py', 'robot-teleop'):
                self.assertNotIn(str(ROOT.parent / source), ' '.join(argv))
            for field in ('script', 'shell'):
                if field in spec:
                    self.assertTrue((ROOT / spec[field]).is_file())


if __name__ == '__main__':
    unittest.main()
