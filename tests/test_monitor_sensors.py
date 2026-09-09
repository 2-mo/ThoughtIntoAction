"""Monitor reads existing IPC only; no device initialization."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from urllib.request import build_opener, ProxyHandler, Request
from urllib.error import HTTPError

from robot_core.model import RobotModel
from robot_simulation.scene import Scene
from quest_teleop.inputs import controller_snapshot, IdleInput
from robot_core.local_server import LocalServer
from quest_teleop.runtime.session import TeleopSession


class MonitorSensorTests(unittest.TestCase):
    def test_scene_only_requests_monitor_and_handles_missing_session(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "session.sock"
            config = {"runtime": {"socket": str(path)}, "camera_service": {"socket": str(Path(folder) / "camera.sock")}}
            calls = []
            def dispatch(payload):
                calls.append(payload)
                return {"input": {"sample_monotonic_ns": time.monotonic_ns()},
                        "preview": {"jpeg_base64": None, "age_s": None}}
            scene = Scene(RobotModel())
            self.addCleanup(scene.close)
            http = build_opener(ProxyHandler({}))
            with patch("robot_core.project.load_config", return_value=config), patch("robot_core.camera_client.load_config", return_value=config):
                server = LocalServer(path, dispatch)
                try:
                    data = json.loads(http.open(scene.url + "api/sensors").read())
                    self.assertLess(data["input_age_s"], .5)
                    self.assertEqual(calls, [{"command": "monitor"}])
                    with self.assertRaises(HTTPError) as error:
                        http.open(Request(scene.url + "api/sensors", data=b'{}'))
                    self.assertEqual(error.exception.code, 501)
                    self.assertEqual(len(calls), 1)
                finally:
                    server.close()
                data = json.loads(http.open(scene.url + "api/sensors").read())
                self.assertIn("error", data)

    def test_preview_age_tracks_encoded_frame_not_newer_capture(self):
        session = TeleopSession.__new__(TeleopSession)
        session.preview_jpeg = "old"
        session.preview_frame = {"host_received_monotonic_ns": time.monotonic_ns() - 2_000_000_000}
        session.latest_frame = {"host_received_monotonic_ns": time.monotonic_ns()}
        self.assertGreater(session.dispatch({"command": "preview"})["age_s"], 1)

    def test_controller_data_and_invalid_values(self):
        client = IdleInput()
        client.get_timestamp_ns = lambda: 123
        snapshot = controller_snapshot(client, 456)
        self.assertEqual(snapshot["device_timestamp_ns"], 123)
        self.assertEqual(snapshot["hands"]["right"]["pose"], [0, 0, 0, 0, 0, 0, 1])
        client.get_pose_by_name = lambda name: [float("nan")] * 7
        snapshot = controller_snapshot(client, 789)
        self.assertTrue(snapshot["error"])
        self.assertFalse(snapshot["hands"])
