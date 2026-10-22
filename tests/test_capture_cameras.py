"""`collect run --cameras` asks the shared camera service for each configured camera by id."""
from contextlib import nullcontext
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from embodied.data.capture import capture


class Stop(Exception):
    pass


class CaptureCameraTests(unittest.TestCase):
    def test_each_shared_camera_is_requested_by_its_id(self):
        created = []

        class Registry:
            def spec(self, kind, name):
                return SimpleNamespace(capabilities=('shared-service', 'rgb'))

            def create(self, kind, name, **options):
                created.append(options)
                return nullcontext(SimpleNamespace(metadata={}))

        config = {'cameras': {'wrist_rgb': {'driver': 'realsense', 'serial': '1', 'width': 64, 'height': 48, 'fps': 30},
                              'exterior_rgb': {'serial': '2', 'width': 64, 'height': 48, 'fps': 30}}}
        with patch('embodied.contracts.registry.Registry', Registry), \
                patch('embodied.core.project.load_config', return_value=config), \
                patch('embodied.devices.sensors.control.recording_guard', return_value=SimpleNamespace(close=lambda: None)), \
                patch.object(capture, 'RuntimeReader', side_effect=Stop):
            with self.assertRaises(Stop):
                capture.main(['--run', '/nonexistent', '--task', 't', '--cameras', 'wrist_rgb,exterior_rgb'])
        self.assertEqual([(c['camera_id'], c['serial']) for c in created], [('wrist_rgb', '1'), ('exterior_rgb', '2')])


if __name__ == '__main__':
    unittest.main()
