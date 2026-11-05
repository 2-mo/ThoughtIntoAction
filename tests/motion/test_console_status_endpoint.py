"""Read-only console status endpoint: token gate, payload passthrough, no kernel."""
import asyncio
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

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
