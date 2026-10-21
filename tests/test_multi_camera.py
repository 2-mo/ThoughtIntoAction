"""Named feeds preserve recording identity and isolate device/lifecycle failures."""
import base64
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from embodied.sensors.client import SharedCamera
from embodied.sensors.service import CameraService
from embodied.core.local_server import LocalServer
from embodied.core.session_protocol import request
from embodied.sensors.realsense import configure_rgb, RGB_OPTIONS


def options(serial, model="D435i"):
    return dict(serial=serial, model=model, enabled=True, width=3, height=2, fps=30)


class MultiCameraTests(unittest.TestCase):
    def test_shared_rgb_profile_override_validation_and_write_order(self):
        service = CameraService({"wrist_rgb": options("wrist"),
            "exterior_rgb": {**options("exterior"), "color": {"gain": 60}}},
            color_options={"gain": 64, "white_balance": 4600})
        self.assertEqual(service.feeds["wrist_rgb"].options["color"], {"gain": 64, "white_balance": 4600})
        self.assertEqual(service.feeds["exterior_rgb"].options["color"], {"gain": 60, "white_balance": 4600})
        values = dict.fromkeys(RGB_OPTIONS, 0.)
        values.update(enable_auto_white_balance=1., enable_auto_exposure=1.)
        writes = []
        def write(key, value):
            writes.append(key)
            values[key] = value
        sensor = SimpleNamespace(supports=lambda _: True, is_option_read_only=lambda _: False,
            get_option_range=lambda _: SimpleNamespace(min=0, max=6500, step=10),
            set_option=write, get_option=values.__getitem__)
        rs = SimpleNamespace(option=SimpleNamespace(**{key: key for key in RGB_OPTIONS}))
        for bad in ({"unknown": 1}, {"white_balance": 4600, "gain": 6501}, {"gain": 1}, {"gain": float('nan')}):
            with self.assertRaises(ValueError):
                configure_rgb(sensor, rs, bad)
            self.assertEqual(writes, [])
        profile = {"white_balance": 4600, "exposure": 160, "enable_auto_exposure": 0,
                   "enable_auto_white_balance": 0}
        actual = configure_rgb(sensor, rs, profile)
        self.assertLess(writes.index("enable_auto_white_balance"), writes.index("white_balance"))
        self.assertLess(writes.index("enable_auto_exposure"), writes.index("exposure"))
        self.assertEqual({key: actual[key] for key in profile}, profile)
        writes.clear()
        configure_rgb(sensor, rs, profile)
        self.assertEqual(writes, [])

    def test_named_pixels_default_recording_channel_and_unknown_ids(self):
        service = CameraService({"wrist_rgb": options("wrist"), "exterior_rgb": options("exterior", "D435")})
        stamp = time.monotonic_ns()
        for number, feed in enumerate(service.feeds.values()):
            feed.enabled = True
            feed.error = None
            feed.metadata = {**feed.options, "stream_id": str(number)}
            feed.frame = {"host_received_monotonic_ns": stamp, "frame_number": 1, "stream_id": str(number),
                          "bgr_base64": base64.b64encode(bytes([number]) * 18).decode()}
            feed.preview = (stamp, f"preview-{number}")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.sock"
            server = LocalServer(path, service.dispatch)
            self.addCleanup(server.close)
            with patch("embodied.sensors.client.socket_path", return_value=path):
                with SharedCamera("wrist", 3, 2) as wrist, SharedCamera("exterior", 3, 2, camera_id="exterior_rgb") as exterior:
                    np.testing.assert_array_equal(wrist.read()["image"], np.zeros((2, 3, 3), dtype=np.uint8))
                    np.testing.assert_array_equal(exterior.read()["image"], np.ones((2, 3, 3), dtype=np.uint8))
                data = request(path, "monitor")
                self.assertEqual(data["preview"]["jpeg_base64"], "preview-0")
                self.assertEqual(data["cameras"]["exterior_rgb"]["preview"]["jpeg_base64"], "preview-1")
                self.assertEqual(data["default_camera_id"], "wrist_rgb")
                for camera_id in ("missing", False, []):
                    with self.assertRaisesRegex(RuntimeError, "Unknown camera"):
                        request(path, "set_enabled", camera_id=camera_id, enabled=False)
                self.assertTrue(all(feed.enabled for feed in service.feeds.values()))

    def test_one_camera_can_stop_and_reopen_without_restarting_the_other(self):
        service = CameraService({"wrist_rgb": options("wrist"), "exterior_rgb": options("exterior")})
        started = {key: threading.Event() for key in service.feeds}
        stopped = {key: threading.Event() for key in service.feeds}
        def capture(key):
            def run(options, stop):
                started[key].set()
                stop.wait()
                stopped[key].set()
            return run
        for key, feed in service.feeds.items():
            feed.capture = capture(key)
        self.addCleanup(service.close)
        with patch("embodied.sensors.control.check_legacy_session"), patch("embodied.sensors.control.recording_guard"):
            service.start()
            for event in started.values():
                self.assertTrue(event.wait(1))
            wrist_thread = service.feeds["wrist_rgb"].thread
            service.dispatch({"command": "set_enabled", "camera_id": "exterior_rgb", "enabled": False})
            self.assertTrue(stopped["exterior_rgb"].is_set())
            self.assertFalse(stopped["wrist_rgb"].is_set())
            self.assertTrue(wrist_thread.is_alive())
            started["exterior_rgb"].clear()
            service.dispatch({"command": "set_enabled", "camera_id": "exterior_rgb", "enabled": True})
            self.assertTrue(started["exterior_rgb"].wait(1))
            self.assertIs(service.feeds["wrist_rgb"].thread, wrist_thread)

    def test_failed_device_clears_only_its_own_frames_and_keeps_identity(self):
        service = CameraService({"wrist_rgb": options("wrist"), "exterior_rgb": options("missing", "D435")})
        service.feeds["wrist_rgb"].preview = (time.monotonic_ns(), "wrist-preview")
        failed = service.feeds["exterior_rgb"]
        failed.frame = {"old": True}
        stop = threading.Event()
        def missing(*args, **kwargs):
            stop.set()
            raise RuntimeError("Device disconnected")
        with patch("embodied.sensors.service.RealSenseCamera", side_effect=missing), patch("embodied.sensors.service.DeviceLease"):
            failed.capture(failed.options, stop)
        data = service.dispatch({"command": "monitor"})
        self.assertIn("Device disconnected", data["cameras"]["exterior_rgb"]["camera"]["error"])
        self.assertEqual(data["cameras"]["exterior_rgb"]["camera"]["serial"], "missing")
        self.assertIsNone(failed.frame)
        self.assertIsNone(failed.metadata)
        self.assertEqual(data["preview"]["jpeg_base64"], "wrist-preview")

    def test_duplicate_devices_and_invalid_capture_options_fail_before_start(self):
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            CameraService({"wrist_rgb": options("same"), "exterior_rgb": options("same")})
        for change in ({"fps": 0}, {"width": True}, {"enabled": "yes"}, {"serial": 123}):
            with self.assertRaises(ValueError):
                CameraService({"wrist_rgb": {**options("wrist"), **change}})

    def test_control_forwards_explicit_camera_id(self):
        from embodied.sensors.control import set_enabled
        with patch("embodied.sensors.control.ensure_service"), patch("embodied.sensors.control.request") as call:
            set_enabled(False, "exterior_rgb")
            self.assertEqual(call.call_args.kwargs["camera_id"], "exterior_rgb")
            self.assertFalse(call.call_args.kwargs["enabled"])
