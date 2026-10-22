"""`collect run --cameras` asks the shared camera service for each configured camera by id."""
from contextlib import nullcontext
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from embodied.data.capture import capture


class Stop(Exception):
    pass


class CaptureCameraTests(unittest.TestCase):
    def test_direct_camera_plugin_does_not_read_wrist_config(self):
        created = []
        class Registry:
            def spec(self, kind, name):
                return SimpleNamespace(capabilities=('rgb',))
            def create(self, kind, name, **options):
                created.append((name, options))
                return nullcontext(SimpleNamespace(metadata={}))
        with patch('embodied.contracts.registry.Registry', Registry), \
                patch('embodied.core.project.load_config', side_effect=AssertionError('unrelated wrist configuration')), \
                patch.object(capture, 'RuntimeReader', side_effect=Stop):
            with self.assertRaises(Stop):
                capture.main(['--run', '/nonexistent', '--task', 't', '--camera', 'synthetic',
                              '--camera-id', 'front_rgb', '--camera-options', '{"width": 32}'])
        self.assertEqual(created, [('synthetic', {'width': 32})])

    def test_only_external_camera_needs_no_wrist_configuration(self):
        created = []
        class Registry:
            def spec(self, kind, name):
                return SimpleNamespace(capabilities=('shared-service', 'rgb'))
            def create(self, kind, name, **options):
                created.append(options)
                return nullcontext(SimpleNamespace(metadata={}))
        config = {'cameras': {'external_rgb': {'driver': 'synthetic', 'serial': '2', 'width': 32, 'height': 24, 'fps': 30}}}
        with patch('embodied.contracts.registry.Registry', Registry), \
                patch('embodied.core.project.load_config', return_value=config), \
                patch('embodied.devices.sensors.control.recording_guard', return_value=SimpleNamespace(close=lambda: None)), \
                patch.object(capture, 'RuntimeReader', side_effect=Stop):
            with self.assertRaises(Stop):
                capture.main(['--run', '/nonexistent', '--task', 't', '--cameras', 'external_rgb'])
        self.assertEqual(created[0]['camera_id'], 'external_rgb')

    def test_each_shared_camera_is_requested_by_its_id(self):
        created = []

        class Registry:
            def spec(self, kind, name):
                return SimpleNamespace(capabilities=('shared-service', 'rgb'))

            def create(self, kind, name, **options):
                created.append(options)
                return nullcontext(SimpleNamespace(metadata={}))

        config = {'cameras': {'wrist_rgb': {'driver': 'realsense', 'serial': '1', 'width': 64, 'height': 48, 'fps': 30},
                              'exterior_rgb': {'driver': 'realsense', 'serial': '2', 'width': 64, 'height': 48, 'fps': 30}}}
        with patch('embodied.contracts.registry.Registry', Registry), \
                patch('embodied.core.project.load_config', return_value=config), \
                patch('embodied.devices.sensors.control.recording_guard', return_value=SimpleNamespace(close=lambda: None)), \
                patch.object(capture, 'RuntimeReader', side_effect=Stop):
            with self.assertRaises(Stop):
                capture.main(['--run', '/nonexistent', '--task', 't', '--cameras', 'wrist_rgb,exterior_rgb'])
        self.assertEqual([(c['camera_id'], c['serial']) for c in created], [('wrist_rgb', '1'), ('exterior_rgb', '2')])

    def test_external_camera_cannot_inherit_an_unrelated_wrist_driver(self):
        config = {'cameras': {'wrist_rgb': {'driver': 'realsense'}, 'external_rgb': {'serial': '2'}}}
        with patch('embodied.contracts.registry.Registry') as registry, \
                patch('embodied.core.project.load_config', return_value=config), \
                patch('embodied.devices.sensors.control.recording_guard', return_value=SimpleNamespace(close=lambda: None)):
            with self.assertRaisesRegex(SystemExit, '1'):
                capture.main(['--run', '/nonexistent', '--task', 't', '--cameras', 'external_rgb'])
            registry.return_value.create.assert_not_called()


if __name__ == '__main__':
    unittest.main()
