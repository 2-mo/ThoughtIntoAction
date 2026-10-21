"""GR00T on the kernel takes both views from the shared camera service, by configured camera id."""
import time
import unittest
from unittest.mock import patch

import numpy as np

from embodied.policy.groot.cameras import SharedRgbCameras, default_camera_specs

CONFIG = {'cameras': {'wrist_rgb': {'serial': '405622074531', 'width': 64, 'height': 48, 'fps': 30},
                      'exterior_rgb': {'serial': '335522070264', 'width': 64, 'height': 48, 'fps': 30}}}


class FakeShared:
    opened = []

    def __init__(self, serial, width, height, fps, *, camera_id=None):
        self.camera_id, self.number = camera_id, 0
        FakeShared.opened.append((camera_id, serial))

    def __enter__(self):
        return self

    def read(self):
        time.sleep(.01)
        self.number += 1
        value = 200 if self.camera_id == 'exterior_rgb' else 20
        return {'image': np.full((48, 64, 3), value, np.uint8), 'host_received_monotonic_ns': time.monotonic_ns(),
                'frame_number': self.number}

    def __exit__(self, *exc):
        return False


class SharedCamerasTests(unittest.TestCase):
    def test_both_views_by_camera_id_and_serials_from_config(self):
        FakeShared.opened = []
        with patch('embodied.core.project.load_config', return_value=CONFIG), \
                patch('embodied.sensors.client.SharedCamera', FakeShared):
            with SharedRgbCameras() as cameras:
                pair = cameras.latest()
            specs = default_camera_specs()
        self.assertEqual(sorted(FakeShared.opened), [('exterior_rgb', '335522070264'), ('wrist_rgb', '405622074531')])
        self.assertEqual((pair.exterior.image[0, 0, 0], pair.wrist.image[0, 0, 0]), (200, 20))
        self.assertEqual({spec.name: spec.serial for spec in specs}, {'exterior': '335522070264', 'wrist': '405622074531'})


if __name__ == '__main__':
    unittest.main()
