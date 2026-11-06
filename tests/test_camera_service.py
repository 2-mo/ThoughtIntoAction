"""Synthetic camera IPC verifies shared pixels and viewer independence without USB."""
import base64
import json
import sys
from pathlib import Path
import tempfile
import time
import threading
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from unittest.mock import Mock
from urllib.request import build_opener, ProxyHandler

import numpy as np
from embodied.devices.sensors.client import SharedCamera
from embodied.devices.sensors.service import CameraFeed
from embodied.core.local_server import LocalServer
from embodied.devices.sensors.transport import FrameServer, FrameClient
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.applications.monitor.scene import Scene


class CameraServiceTests(unittest.TestCase):
    def test_realsense_first_frame_wait_is_longer_but_cancellable(self):
        from embodied.devices.sensors.realsense import RealSenseCamera
        camera = RealSenseCamera('fake')
        camera.depth_scale = None
        frame = Mock()
        frame.get_frame_number.return_value = 1
        frame.get_timestamp.return_value = 123.
        frame.get_data.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        frames = Mock()
        frames.get_color_frame.return_value = frame
        frames.get_depth_frame.return_value = None
        camera.pipeline = Mock()
        camera.pipeline.try_wait_for_frames.side_effect = [(False, None), (True, frames)]
        with patch('embodied.devices.sensors.realsense.time.monotonic', side_effect=[0., 3.]):
            self.assertEqual(camera.read()['frame_number'], 1)
        camera.pipeline.try_wait_for_frames.return_value = (False, None)
        camera.pipeline.try_wait_for_frames.side_effect = None
        with patch('embodied.devices.sensors.realsense.time.monotonic', side_effect=[0., 3.]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                camera.read()
        camera.last_number = None
        with patch('embodied.devices.sensors.realsense.time.monotonic', side_effect=[0., 11.]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                camera.read()
        camera.pipeline.try_wait_for_frames.side_effect = lambda _: (camera.stop.set() or False, None)
        with self.assertRaisesRegex(RuntimeError, 'stopped'):
            camera.read()
        self.assertTrue(all(call.args == (100,) for call in camera.pipeline.try_wait_for_frames.call_args_list))

    def test_realsense_cancelled_start_releases_pipeline(self):
        from embodied.devices.sensors.realsense import RealSenseCamera
        rs = Mock()
        device = Mock()
        device.get_info.side_effect = lambda key: {rs.camera_info.serial_number: 'fake',
            rs.camera_info.name: 'RealSense D435', rs.camera_info.usb_type_descriptor: '2.1'}[key]
        rs.context.return_value.query_devices.return_value = [device]
        profile = rs.pipeline.return_value.start.return_value
        profile.get_device.return_value = device
        profile.get_stream.return_value.as_video_stream_profile.return_value.get_intrinsics.return_value = SimpleNamespace(
            width=424, height=240, fx=1., fy=1., ppx=0., ppy=0., model='none', coeffs=[])
        camera = RealSenseCamera('fake', expected_model='D435')
        camera.stop.set()
        rs.pipeline.return_value.stop.reset_mock()
        with patch.dict(sys.modules, pyrealsense2=rs), \
             patch('embodied.devices.sensors.realsense.configure_rgb', return_value={}), \
             patch.object(camera, 'start_imu'):
            with self.assertRaisesRegex(RuntimeError, 'stopped'):
                camera.__enter__()
        rs.pipeline.return_value.stop.assert_called_once()
        self.assertIsNone(camera.pipeline)

    def test_realsense_skips_reused_rgb_but_rejects_backwards_or_stuck_frames(self):
        from embodied.devices.sensors.realsense import RealSenseCamera
        camera = RealSenseCamera('fake')
        camera.depth_scale = .001
        camera.last_number = 7
        frame = Mock()
        frame.get_frame_number.side_effect = [7, 7, 8]
        frame.get_timestamp.return_value = 123.
        frame.get_data.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        frames = Mock()
        frames.get_color_frame.return_value = frame
        frames.get_depth_frame.return_value = None
        camera.pipeline = Mock()
        camera.pipeline.try_wait_for_frames.return_value = (True, frames)
        self.assertEqual(camera.read()['frame_number'], 8)
        frame.get_frame_number.side_effect = None
        frame.get_frame_number.return_value = 6
        with self.assertRaisesRegex(RuntimeError, 'backwards'):
            camera.read()
        frame.get_frame_number.return_value = 8
        with patch('embodied.devices.sensors.realsense.time.monotonic', side_effect=[0., 3.]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                camera.read()

    def test_full_hd_zmq_frames_preserve_pixels_and_imu_timestamps(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'camera.sock'
            pixels = np.arange(1920*1080*3, dtype=np.uint8).reshape(1080, 1920, 3)
            feed = CameraFeed()
            feed.metadata = {'serial': 'fake', 'width': 1920, 'height': 1080, 'fps': 30, 'zmq_frame_protocol': 1}
            stamp = time.monotonic_ns()
            imu = {'supported': True, 'accel': {'xyz': [0, 9.81, 0], 'host_received_monotonic_ns': stamp-1000}}
            feed.frame = {'image': pixels, 'host_received_monotonic_ns': stamp, 'frame_number': 1, 'imu': imu}
            server = LocalServer(path, feed.dispatch)
            self.addCleanup(server.close)
            frames = FrameServer(path, {'wrist_rgb': feed})
            self.addCleanup(frames.close)
            with patch('embodied.devices.sensors.client.socket_path', return_value=path):
                with SharedCamera('fake', 1920, 1080) as camera:
                    frame = camera.read()
            np.testing.assert_array_equal(frame['image'], pixels)
            self.assertEqual(frame['imu']['accel']['host_received_monotonic_ns'], stamp-1000)
            self.assertGreaterEqual(feed.snapshot()['imu']['accel']['age_s'], 0)
            self.assertNotIn('age_s', imu['accel'])

    def test_imu_native_samples_and_failure_are_independent_of_rgb(self):
        from embodied.devices.sensors.realsense import RealSenseCamera
        rs = SimpleNamespace(stream=SimpleNamespace(accel='a', gyro='g'), format=SimpleNamespace(motion_xyz32f='motion'))
        sensor = Mock()
        sensor.get_stream_profiles.return_value = [SimpleNamespace(stream_type=lambda k=k:k, format=lambda:'motion', fps=lambda:200) for k in ('a','g')]
        camera = RealSenseCamera('fake')
        device = SimpleNamespace(query_sensors=lambda:[sensor])
        camera.start_imu(device, rs)
        frame = SimpleNamespace(as_motion_frame=lambda:SimpleNamespace(get_motion_data=lambda:SimpleNamespace(x=1,y=2,z=3)),
            get_profile=lambda:SimpleNamespace(stream_type=lambda:'a'), get_frame_number=lambda:7,
            get_timestamp=lambda:123.4, get_frame_timestamp_domain=lambda:'hardware_clock')
        sensor.start.call_args.args[0](frame)
        self.assertEqual(camera.imu_snapshot()['accel']['xyz'], [1,2,3])
        self.assertEqual(camera.imu_snapshot()['accel']['unit'], 'm/s²')
        camera.stop_imu()
        sensor.stop.assert_called_once()
        failed = RealSenseCamera('fake')
        sensor.open.side_effect = RuntimeError('permission denied')
        failed.start_imu(device, rs)
        self.assertIn('permission denied', failed.imu_snapshot()['error'])
        self.assertIsNone(failed.imu_sensor)

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
            with patch("embodied.devices.sensors.client.load_config", return_value=config), patch("embodied.core.project.load_config", return_value=config):
                with SharedCamera("fake", 3, 2, 30) as one, SharedCamera("fake", 3, 2, 30) as two:
                    first, second = one.read(), two.read()
                    np.testing.assert_array_equal(first["image"], pixels)
                    np.testing.assert_array_equal(second["image"], pixels)
                    self.assertEqual(first["host_received_monotonic_ns"], stamp)
                scene = Scene(X5PinocchioModel())
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

    def test_switch_stops_pipeline_preserves_off_state_and_blocks_recording(self):
        from embodied.devices.sensors.control import recording_guard
        with tempfile.TemporaryDirectory() as folder, patch("embodied.devices.sensors.control.socket_path", return_value=Path(folder)/"camera.sock"), patch("embodied.devices.sensors.control.check_legacy_session"):
            feed = CameraFeed({"enabled": True})
            opened, closed = threading.Event(), threading.Event()
            def capture(options, stop):
                feed.frame = {"host_received_monotonic_ns": time.monotonic_ns()}
                opened.set()
                stop.wait()
                closed.set()
            feed.capture = capture
            self.addCleanup(feed.close)
            feed.set_enabled(True)
            self.assertTrue(opened.wait(1))
            lease = recording_guard()
            try:
                with self.assertRaisesRegex(RuntimeError, "录制"):
                    feed.set_enabled(False)
                self.assertTrue(feed.enabled)
            finally:
                lease.close()
            feed.set_enabled(False)
            self.assertTrue(closed.is_set())
            self.assertIsNone(feed.frame)
            self.assertFalse(feed.dispatch({"command": "monitor"})["camera"]["enabled"])
            feed.dispatch({"command": "monitor"})
            self.assertIsNone(feed.thread)
            feed.set_enabled(True)
            self.assertTrue(feed.enabled)

    def test_old_session_cannot_be_silently_disconnected(self):
        from embodied.devices.sensors.control import check_legacy_session
        with patch("embodied.devices.sensors.control.request", return_value={"recording": None}):
            with self.assertRaisesRegex(RuntimeError, "退出遥操"):
                check_legacy_session()
        with patch("embodied.devices.sensors.control.request", return_value={"camera_switch_protocol": 1, "recording": {"id": "recording"}}):
            with self.assertRaisesRegex(RuntimeError, "录制"):
                check_legacy_session()

    def test_camera_protocol_is_read_only(self):
        with self.assertRaises(ValueError):
            CameraFeed().dispatch({"command": "record.start"})

    def test_depth_colors_use_fixed_metric_scale_and_black_invalid_pixels(self):
        from embodied.devices.sensors.service import colorize_depth
        values = np.array([[0, 200, 1000, 2000, 3000]], dtype=np.uint16)
        colors = colorize_depth(values, .001)
        np.testing.assert_array_equal(colors[0, 0], [0, 0, 0])
        np.testing.assert_array_equal(colors[0, 3], colors[0, 4])
        self.assertFalse(np.array_equal(colors[0, 1], colors[0, 2]))
        other = colorize_depth(np.array([[1000]], dtype=np.uint16), .001)
        np.testing.assert_array_equal(colors[0, 2], other[0, 0])
