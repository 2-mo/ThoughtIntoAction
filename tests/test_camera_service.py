"""Synthetic camera IPC verifies shared pixels and viewer independence without USB."""
import base64
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from urllib.request import build_opener, ProxyHandler

import numpy as np
from robot_core.camera_client import SharedCamera
from robot_core.camera_service import CameraFeed
from robot_core.local_server import LocalServer
from robot_core.model import RobotModel
from robot_simulation.scene import Scene


class CameraServiceTests(unittest.TestCase):
    def test_two_clients_and_scene_share_frames_without_teleop_or_device_sdk(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.sock"
            config = {"camera_service": {"socket": str(path)},
                      "runtime": {"socket": str(Path(folder) / "missing.sock")}}
            pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
            feed = CameraFeed()
            feed.metadata = {"serial": "fake", "width": 3, "height": 2, "fps": 30}
            feed.error = None
            stamp = time.monotonic_ns()
            feed.frame = {"frame_number": 5, "host_received_monotonic_ns": stamp,
                          "device_timestamp_ms": 12.5, "device_timestamp_domain": "hardware_clock",
                          "bgr_base64": base64.b64encode(pixels.tobytes()).decode()}
            feed.preview = (stamp, "preview")
            server = LocalServer(path, feed.dispatch)
            self.addCleanup(server.close)
            with patch("robot_core.camera_client.load_config", return_value=config), patch("robot_core.project.load_config", return_value=config):
                with SharedCamera("fake", 3, 2, 30) as one, SharedCamera("fake", 3, 2, 30) as two:
                    first, second = one.read(), two.read()
                    np.testing.assert_array_equal(first["image"], pixels)
                    np.testing.assert_array_equal(second["image"], pixels)
                    self.assertEqual(first["host_received_monotonic_ns"], stamp)
                scene = Scene(RobotModel())
                self.addCleanup(scene.close)
                http = build_opener(ProxyHandler({}))
                data = json.loads(http.open(scene.url + "api/sensors").read())
                self.assertEqual(data["preview"]["jpeg_base64"], "preview")
                self.assertNotIn("error", data)
                self.assertEqual(data["input_error"], "遥操会话未连接")
                self.assertIsNone(data.get("input"))
                state = json.loads(http.open(scene.url + "api/state").read())
                self.assertIsNone(state["actual"])
                self.assertIsNotNone(scene._manifest["reference"]["actual"])
                feed.frame = {**feed.frame, "host_received_monotonic_ns": stamp - 2_000_000_000}
                with SharedCamera("fake", 3, 2, 30) as client:
                    with self.assertRaisesRegex(RuntimeError, "stale"):
                        client.read()
                with self.assertRaisesRegex(RuntimeError, "settings differ"):
                    SharedCamera("another", 3, 2, 30).__enter__()

    def test_camera_protocol_is_read_only(self):
        with self.assertRaises(ValueError):
            CameraFeed().dispatch({"command": "record.start"})

    def test_depth_colors_use_fixed_metric_scale_and_black_invalid_pixels(self):
        from robot_core.camera_service import colorize_depth
        values = np.array([[0, 200, 1000, 2000, 3000]], dtype=np.uint16)
        colors = colorize_depth(values, .001)
        np.testing.assert_array_equal(colors[0, 0], [0, 0, 0])
        np.testing.assert_array_equal(colors[0, 3], colors[0, 4])
        self.assertFalse(np.array_equal(colors[0, 1], colors[0, 2]))
        other = colorize_depth(np.array([[1000]], dtype=np.uint16), .001)
        np.testing.assert_array_equal(colors[0, 2], other[0, 0])
