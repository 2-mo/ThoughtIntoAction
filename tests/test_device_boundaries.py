"""Hardware capabilities stay at the kernel/device boundary; no devices or servers."""
from pathlib import Path
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from embodied.contracts.protocol import validate_request
from embodied.devices.sensors.client import SharedCamera
from embodied.devices.sensors.service import CameraService


class ProtocolDimensions(unittest.TestCase):
    def test_model_joint_count_is_not_a_protocol_constant(self):
        request = {'version': 1, 'runtime_id': 'test', 'action': 'preview', 'kind': 'movej',
                   'q': [0.] * 7, 'speed': 0, 'speed_scale': .25}
        self.assertEqual(validate_request(request), [])
        self.assertEqual(validate_request(request, joint_count=7), [])
        self.assertEqual(validate_request(request, joint_count=6), ['preview: q must be 6 finite numbers'])
        for value in ([], [True], [float('nan')], [float('inf')]):
            with self.subTest(value=value):
                self.assertTrue(validate_request({**request, 'q': value}))


class CameraRouting(unittest.TestCase):
    def make_service(self, **kwargs):
        service = CameraService({'front': {'driver': 'synthetic'}, 'overhead': {'driver': 'synthetic'}}, **kwargs)
        self.addCleanup(service.close)
        return service

    def test_arbitrary_names_default_to_configuration_order(self):
        service = self.make_service()
        state = service.dispatch({'command': 'monitor'})
        self.assertEqual(state['default_camera_id'], 'front')
        self.assertEqual(state['camera']['id'], 'front')
        self.assertEqual(set(state['cameras']), {'front', 'overhead'})

    def test_explicit_default_routes_monitor_and_frame(self):
        service = self.make_service(default_camera_id='overhead')
        frame = {'frame_number': 7, 'host_received_monotonic_ns': time.monotonic_ns()}
        service.feeds['overhead'].frame = frame
        self.assertEqual(service.dispatch({'command': 'monitor'})['camera']['id'], 'overhead')
        self.assertEqual(service.dispatch({'command': 'frame'}), frame)
        self.assertEqual(service.dispatch({'command': 'monitor', 'camera_id': 'front'})['camera']['id'], 'front')
        with self.assertRaisesRegex(ValueError, 'Unknown default camera'):
            self.make_service(default_camera_id='missing')

    def test_named_default_reaches_the_same_zmq_channel(self):
        pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        metadata = {'serial': 's', 'width': 3, 'height': 2, 'fps': 30, 'zmq_frame_protocol': 1}
        response = {'camera': {'id': 'overhead', 'metadata': metadata}}
        frames = Mock()
        frames.read.return_value = ({'host_received_monotonic_ns': time.monotonic_ns()}, pixels.tobytes())
        with patch('embodied.devices.sensors.client.ensure_service'), \
             patch('embodied.devices.sensors.client.request', return_value=response), \
             patch('embodied.devices.sensors.client.socket_path', return_value=Path('/camera.sock')):
            with SharedCamera('s', 3, 2) as camera:
                camera.frames = frames
                np.testing.assert_array_equal(camera.read()['image'], pixels)
        frames.read.assert_called_once_with('overhead')
        frames.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
