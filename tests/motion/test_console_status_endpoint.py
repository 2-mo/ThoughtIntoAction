"""Read-only console status endpoint: token gate, payload passthrough, no kernel."""
import asyncio
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from fastapi import HTTPException

from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.transport import system


class FakeCatalog:
    def describe(self):
        return []


def status_endpoint(settings):
    router = system.create_system_router(FakeCatalog(), settings)
    route = next(route for route in router.routes if route.path == "/api/v1/system/status")
    return route.endpoint


class ConsoleStatusEndpointTests(unittest.TestCase):
    def test_missing_or_wrong_token_is_rejected(self):
        endpoint = status_endpoint(BackendSettings(token="secret"))
        for token in (None, "wrong"):
            with self.assertRaises(HTTPException) as raised:
                asyncio.run(endpoint(token=token))
            self.assertEqual(raised.exception.status_code, 401)

    def test_status_payload_is_passed_through(self):
        endpoint = status_endpoint(BackendSettings(token="secret"))
        payload = {"connected": True, "phase": "hold", "q": [0.0] * 6}
        with patch.object(system, "read_kernel_status", return_value=payload):
            self.assertEqual(asyncio.run(endpoint(token="secret")), payload)

    def test_read_kernel_status_without_run_is_disconnected(self):
        with tempfile.TemporaryDirectory() as folder:
            result = system.read_kernel_status(Path(folder) / "missing")
        self.assertFalse(result["connected"])
        self.assertIn("reason", result)


class ArmViewTests(unittest.TestCase):
    def test_single_arm_is_one_entry(self):
        arms = system.arm_views([0.1] * 6, [0.0] * 6)
        self.assertEqual(len(arms), 1)
        self.assertEqual(arms[0]["id"], "arm1")
        self.assertEqual(arms[0]["label"], "机械臂")
        self.assertEqual(len(arms[0]["joints"]), 6)

    def test_twelve_joints_split_into_two_arms(self):
        arms = system.arm_views(list(range(12)), [0.0] * 12)
        self.assertEqual([arm["id"] for arm in arms], ["arm1", "arm2"])
        self.assertEqual(arms[0]["joints"], [float(value) for value in range(6)])
        self.assertEqual(arms[1]["joints"], [float(value) for value in range(6, 12)])
        self.assertEqual(arms[1]["label"], "机械臂 2")

    def test_missing_feedback_still_reports_one_arm(self):
        self.assertEqual(len(system.arm_views([], [])), 1)
        self.assertEqual(system.arm_views(None, None)[0]["joints"], [])


if __name__ == "__main__":
    unittest.main()


class HomeTests(unittest.TestCase):
    def test_home_refuses_while_another_source_owns_the_lease(self):
        from embodied.applications.console.transport import system

        with patch.object(system, "live_runs", return_value=[Path("/tmp/run")]), \
             patch("embodied.contracts.runtime.control.ControlClient") as client:
            client.return_value.status.return_value = {"owner": {"source": "policy"}}
            with self.assertRaises(PermissionError):
                system.move_home()

    def test_home_previews_and_waits_for_completion(self):
        from embodied.applications.console.transport import system

        session = MagicMock()
        session.__enter__.return_value = session
        session.preview.return_value = {"times": [0.0, 0.5], "kind": "home"}
        session.execute.return_value = {"operation_id": "op-1"}
        session.status.side_effect = [
            {"operations": [{"operation_id": "op-1", "status": "running"}]},
            {"operations": [{"operation_id": "op-1", "status": "completed"}]},
        ]
        with patch.object(system, "live_runs", return_value=[Path("/tmp/run")]), \
             patch("embodied.contracts.runtime.control.ControlClient") as client, \
             patch("embodied.contracts.runtime.control.ControlSession", return_value=session):
            client.return_value.status.return_value = {"owner": None}
            result = system.move_home(25)
        self.assertEqual(result["operation"], "op-1")
        session.preview.assert_called_once_with("home", speed_scale=0.25)
        session.execute.assert_called_once()

    def test_home_replans_while_the_arm_is_still_settling(self):
        from embodied.applications.console.transport import system

        session = MagicMock()
        session.__enter__.return_value = session
        session.preview.return_value = {"times": [0.0, 0.5], "kind": "home"}
        session.execute.side_effect = [
            RuntimeError("ValueError: Robot moved since preview: J10 changed 0.42 deg"),
            {"operation_id": "op-1"},
        ]
        session.status.return_value = {"operations": [{"operation_id": "op-1", "status": "completed"}]}
        with patch.object(system, "live_runs", return_value=[Path("/tmp/run")]), \
             patch("embodied.contracts.runtime.control.ControlClient") as client, \
             patch("embodied.contracts.runtime.control.ControlSession", return_value=session):
            client.return_value.status.return_value = {"owner": None}
            result = system.move_home(25)
        self.assertEqual(result["operation"], "op-1")
        self.assertEqual(session.preview.call_count, 2)

    def test_home_retries_when_the_arm_is_not_yet_settled(self):
        from embodied.applications.console.transport import system

        session = MagicMock()
        session.__enter__.return_value = session
        session.preview.return_value = {"times": [0.0, 0.5], "kind": "home"}
        session.execute.return_value = {"operation_id": "op-1"}
        session.status.return_value = {"operations": [{"operation_id": "op-1", "status": "completed"}]}
        with patch.object(system, "live_runs", return_value=[Path("/tmp/run")]), \
             patch("embodied.contracts.runtime.control.ControlClient") as client, \
             patch("embodied.contracts.runtime.control.ControlSession",
                   side_effect=[RuntimeError("ValueError: Robot has not settled"), session]) as control:
            client.return_value.status.return_value = {"owner": None}
            result = system.move_home(25)
        self.assertEqual(result["operation"], "op-1")
        self.assertEqual(control.call_count, 2)


class ReadyTests(unittest.TestCase):
    def ready_file(self, folder, payload):
        path = Path(folder) / "ready_pose.json"
        path.write_text(json.dumps(payload))
        return path

    def test_ready_pose_comes_from_the_config_file(self):
        from embodied.applications.console.transport import system

        with tempfile.TemporaryDirectory() as folder:
            path = self.ready_file(folder, {"speed_percent": 40, "q": [0.1] * 12})
            with patch.object(system, "READY_PATH", path):
                pose = system.ready_pose()
        self.assertEqual(pose["speed_percent"], 40)
        self.assertEqual(len(pose["q"]), 12)

    def test_ready_pose_rejects_an_incomplete_vector(self):
        from embodied.applications.console.transport import system

        with tempfile.TemporaryDirectory() as folder:
            path = self.ready_file(folder, {"q": [0.0] * 6})
            with patch.object(system, "READY_PATH", path):
                with self.assertRaises(ValueError):
                    system.ready_pose()

    def test_move_ready_previews_a_joint_move_to_the_pose(self):
        from embodied.applications.console.transport import system

        session = MagicMock()
        session.__enter__.return_value = session
        session.preview.return_value = {"times": [0.0, 0.5], "kind": "movej"}
        session.execute.return_value = {"operation_id": "op-1"}
        session.status.return_value = {"operations": [{"operation_id": "op-1", "status": "completed"}]}
        with tempfile.TemporaryDirectory() as folder:
            path = self.ready_file(folder, {"speed_percent": 30, "q": [0.2] * 12})
            with patch.object(system, "READY_PATH", path), \
                 patch.object(system, "live_runs", return_value=[Path("/tmp/run")]), \
                 patch("embodied.contracts.runtime.control.ControlClient") as client, \
                 patch("embodied.contracts.runtime.control.ControlSession", return_value=session):
                client.return_value.status.return_value = {"owner": None}
                result = system.move_ready()
        self.assertEqual(result["operation"], "op-1")
        session.preview.assert_called_once_with("movej", speed_scale=0.3, q=[0.2] * 12)
