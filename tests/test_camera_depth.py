"""Metric depth stays lossless and refuses stale or misregistered measurements."""
import base64
import copy
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from embodied.core.local_server import LocalServer
from embodied.devices.sensors.client import SharedCamera
from embodied.devices.sensors.realsense import RealSenseCamera
from embodied.devices.sensors.service import CameraFeed


def depth_frame():
    return {"depth_image": np.array([[0, 301, 65535], [1234, 500, 800]], dtype=np.uint16),
            "width": 3, "height": 2, "aligned_to_rgb": True, "depth_scale_m": .001,
            "host_received_monotonic_ns": time.monotonic_ns(), "stream_id": "stream-one",
            "frame_number": 5, "depth_frame_number": 8, "device_timestamp_ms": 123.,
            "depth_device_timestamp_ms": 124., "device_timestamp_domain": "hardware_clock",
            "depth_device_timestamp_domain": "hardware_clock",
            "camera": {"serial": "fake", "width": 3, "height": 2, "fps": 30}}


class CameraDepthTests(unittest.TestCase):
    def test_readonly_ipc_preserves_uint16_scale_and_frame_identity(self):
        feed = CameraFeed()
        feed.depth = depth_frame()
        feed.metadata = feed.depth["camera"]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.sock"
            server = LocalServer(path, feed.dispatch)
            self.addCleanup(server.close)
            with patch("embodied.devices.sensors.client.socket_path", return_value=path):
                with SharedCamera("fake", 3, 2) as camera:
                    result = camera.read_depth()
            np.testing.assert_array_equal(result["depth_image"], feed.depth["depth_image"])
            self.assertEqual(result["depth_image"].dtype, np.dtype("uint16"))
            self.assertAlmostEqual(result["depth_image"][1, 0] * result["depth_scale_m"], 1.234)
            self.assertEqual(result["depth_frame_number"], 8)
            self.assertEqual(result["frame_number"], 5)
            feed.depth = None
            with self.assertRaisesRegex(RuntimeError, "No metric depth"):
                feed.dispatch({"command": "depth"})

    def test_rejects_unusable_depth_before_returning_pixels(self):
        raw = depth_frame()
        raw["depth_base64"] = base64.b64encode(raw.pop("depth_image").astype("<u2").tobytes()).decode()
        cases = [({"host_received_monotonic_ns": time.monotonic_ns() - 1_000_000_000}, "stale"),
                 ({"aligned_to_rgb": False}, "aligned"), ({"depth_scale_m": 0.}, "metric"),
                 ({"width": 4}, "dimensions"), ({"depth_base64": "AAAA"}, "buffer size"),
                 ({"depth_device_timestamp_ms": 300.}, "synchronized"),
                 ({"depth_device_timestamp_domain": "different"}, "synchronized"),
                 ({"camera": {**raw["camera"], "serial": "other"}}, "settings differ")]
        for changes, error in cases:
            with self.subTest(changes=changes):
                with patch("embodied.devices.sensors.client.request", return_value={**copy.deepcopy(raw), **changes}):
                    with self.assertRaisesRegex((RuntimeError, ValueError), error):
                        SharedCamera("fake", 3, 2).read_depth()

    def test_driver_uses_sdk_aligned_depth_not_native_depth_pixels(self):
        camera = RealSenseCamera("fake")
        camera.depth_scale = .001
        color, native, aligned = Mock(), Mock(), Mock()
        color.get_frame_number.return_value = 5
        color.get_timestamp.return_value = 123.
        color.get_data.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        aligned.get_frame_number.return_value = 8
        aligned.get_timestamp.return_value = 124.
        aligned.get_data.return_value = np.full((2, 3), 321, dtype=np.uint16)
        frames = Mock()
        frames.get_color_frame.return_value = color
        frames.get_depth_frame.return_value = native
        camera.pipeline = Mock()
        camera.pipeline.try_wait_for_frames.return_value = (True, frames)
        camera.align = Mock()
        camera.align.process.return_value.get_depth_frame.return_value = aligned
        result = camera.read()
        camera.align.process.assert_called_once_with(frames)
        native.get_data.assert_not_called()
        np.testing.assert_array_equal(result["depth_image"], np.full((2, 3), 321, dtype=np.uint16))
        self.assertEqual(result["frame_number"], 5)
        self.assertEqual(result["depth_frame_number"], 8)


if __name__ == "__main__":
    unittest.main()
