"""Native discovery settings never start/stop a service or touch a device."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('quest_discovery', Path(__file__).resolve().parents[1] / 'scripts/quest/discovery.py')
discovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(discovery)


class DiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / 'nested/discovery.ini'

    def test_missing_config_defaults_off(self):
        self.assertFalse(discovery.read(self.path).getboolean('TCP', 'BroadcastEnabled', fallback=False))
        self.assertIsNone(discovery.observed(self.path))

    def test_atomic_updates_preserve_other_settings_and_native_key_case(self):
        self.path.parent.mkdir()
        self.path.write_text('[TCP]\nBroadcastEnabled=true\nTcpBindPort=63901\n')
        discovery.write_enabled(self.path, False)
        self.assertFalse(discovery.read(self.path).getboolean('TCP', 'BroadcastEnabled'))
        self.assertEqual(discovery.read(self.path).getint('TCP', 'TcpBindPort'), 63901)
        discovery.write_enabled(self.path, True)
        self.assertTrue(discovery.read(self.path).getboolean('TCP', 'BroadcastEnabled'))
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])

    def test_cli_requires_service_ack(self):
        with patch.object(discovery, 'config_path', return_value=self.path), \
                patch.object(discovery, 'observed', return_value=None), \
                patch.object(discovery.time, 'monotonic', side_effect=[0, 8]):
            self.assertEqual(discovery.main(['off']), 1)
        self.assertFalse(discovery.read(self.path).getboolean('TCP', 'BroadcastEnabled'))

    def test_cli_confirmed_without_restarting_service(self):
        with patch.object(discovery, 'config_path', return_value=self.path), \
                patch.object(discovery, 'observed', return_value=True):
            self.assertEqual(discovery.main(['on']), 0)

    def test_stale_status_is_not_reported_as_applied(self):
        self.path.parent.mkdir()
        Path(str(self.path) + '.status').write_text('[Service]\nProtocol=1\nPid=1\nCheckedAtMs=0\n[TCP]\nBroadcastEnabled=false\n')
        self.assertIsNone(discovery.observed(self.path))


if __name__ == '__main__':
    unittest.main()
