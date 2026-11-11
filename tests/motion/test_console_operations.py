"""Whitelisted console operations: validation, refusals, and the token gate."""
import asyncio
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from fastapi import HTTPException

from embodied.applications.console import operations
from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.transport.operations import create_operations_router


class OperationValidationTests(unittest.TestCase):
    def setUp(self):
        self.manager = operations.OperationManager()
        port_patch = patch.object(operations, "_port_listening", return_value=False)
        port_patch.start()
        self.addCleanup(port_patch.stop)

    def test_unknown_kind_is_rejected(self):
        with self.assertRaises(operations.OperationError) as raised:
            asyncio.run(self.manager.start("shell", {}))
        self.assertEqual(raised.exception.status, 404)

    def test_real_kernel_requires_confirmation(self):
        with patch.object(operations, "live_runs", return_value=[]):
            with self.assertRaises(operations.OperationError) as raised:
                self.manager._kernel({"profile": "real-manual"})
        self.assertEqual(raised.exception.status, 409)

    def test_real_kernel_with_confirmation_uses_can_setup_script(self):
        with patch.object(operations, "live_runs", return_value=[]):
            argv, _ = self.manager._kernel({"profile": "real-manual", "confirm": True})
        self.assertIn("kernel_real.sh", argv[1])
        self.assertIn("--yes", argv)

    def test_kernel_refused_while_another_is_live(self):
        with patch.object(operations, "live_runs", return_value=[Path("/tmp/run")]):
            with self.assertRaises(operations.OperationError) as raised:
                self.manager._kernel({"profile": "sim-manual"})
        self.assertEqual(raised.exception.status, 409)

    def test_sim_kernel_uses_profile_launcher(self):
        with patch.object(operations, "live_runs", return_value=[]):
            argv, _ = self.manager._kernel({"profile": "sim-manual"})
        self.assertEqual(argv[1:3], [str(operations.RUN_SH), "kernel"])
        self.assertIn("sim-manual", argv)

    def test_collect_refused_while_webrtc_video_is_listening(self):
        with patch.object(operations, "_port_listening", return_value=True):
            with self.assertRaises(operations.OperationError) as raised:
                self.manager._collect({"task": "叠书"})
        self.assertEqual(raised.exception.status, 409)

    def test_camera_refused_while_collecting(self):
        self.manager._operations["collect"] = operations.Operation(
            kind="collect", argv=[], log_path=Path("/tmp/collect.log"), started_at=0.0, state="running",
        )
        with self.assertRaises(operations.OperationError) as raised:
            self.manager._camera()
        self.assertEqual(raised.exception.status, 409)

    def test_collect_requires_task_text(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")):
            with self.assertRaises(operations.OperationError):
                self.manager._collect({"task": "   "})

    def test_collect_manual_mode_records_until_stopped(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "_enabled_cameras", return_value=["wrist_rgb"]):
            argv, _ = self.manager._collect({"task": "叠书", "mode": "manual", "seconds": 30})
        self.assertEqual(argv[argv.index("--duration") + 1], "0")

    def test_collect_fixed_mode_passes_duration_and_storage(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "_enabled_cameras", return_value=["wrist_rgb"]):
            argv, _ = self.manager._collect(
                {"task": "叠书", "mode": "fixed", "seconds": 12, "rgb": "png",
                 "output": "outputs/datasets/session-a", "cameras": ["wrist_rgb"]}
            )
        self.assertEqual(argv[argv.index("--duration") + 1], "12")
        self.assertEqual(argv[argv.index("--rgb") + 1], "png")
        self.assertEqual(argv[argv.index("--output") + 1], "outputs/datasets/session-a")
        self.assertEqual(argv[argv.index("--cameras") + 1], "wrist_rgb")

    def test_collect_rejects_disabled_camera(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "_enabled_cameras", return_value=["wrist_rgb"]):
            with self.assertRaises(operations.OperationError):
                self.manager._collect({"task": "叠书", "cameras": ["exterior_rgb"]})

    def test_collect_rejects_unknown_image_format(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "_enabled_cameras", return_value=["wrist_rgb"]):
            with self.assertRaises(operations.OperationError):
                self.manager._collect({"task": "叠书", "rgb": "tiff"})

    def test_collect_output_must_stay_under_datasets(self):
        self.assertEqual(operations._collect_output(""), "outputs/datasets/runtime")
        self.assertEqual(operations._collect_output(None), "outputs/datasets/runtime")
        for value in ("/tmp/episodes", "src/embodied", "/"):
            with self.assertRaises(operations.OperationError):
                operations._collect_output(value)

    def test_collect_rejects_unknown_mode(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "_enabled_cameras", return_value=["wrist_rgb"]):
            with self.assertRaises(operations.OperationError):
                self.manager._collect({"task": "叠书", "mode": "loop"})

    def test_seconds_bounds(self):
        for value in (0, 301, "x"):
            with self.assertRaises(operations.OperationError):
                operations._seconds(value, 30)
        self.assertEqual(operations._seconds("5", 30), 5)

    def test_policy_requires_confirmation(self):
        with self.assertRaises(operations.OperationError) as raised:
            self.manager._policy({"prompt": "pick the book", "confirm": False})
        self.assertEqual(raised.exception.status, 409)

    def test_remote_policy_builds_the_client_command(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "ControlClient") as client:
            client.return_value.status.return_value = {"owner": None}
            argv, run = self.manager._policy({
                "mode": "remote", "confirm": True, "prompt": "sweep the table",
                "server": "ws://127.0.0.1:8000", "seconds": 30,
                "rtc": True, "chunk_steps": 50, "reinfer_remaining": 30, "blend": 16,
                "guidance": 12.5,
            })
        self.assertEqual(argv[:4], ["bash", str(operations.RUN_SH), "policy", "remote"], argv)
        self.assertIn("--server", argv)
        self.assertIn("ws://127.0.0.1:8000", argv)
        self.assertIn("--rtc", argv)
        self.assertEqual(argv[argv.index("--reinfer-remaining") + 1], "30")
        self.assertEqual(argv[argv.index("--blend") + 1], "16")
        self.assertIn("12.5", argv)

    def test_remote_policy_rejects_bad_server_and_bounds(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "ControlClient") as client:
            client.return_value.status.return_value = {"owner": None}
            with self.assertRaises(operations.OperationError):
                self.manager._policy({"mode": "remote", "confirm": True, "prompt": "x", "server": "http://x"})
            with self.assertRaises(operations.OperationError):
                self.manager._policy({"mode": "remote", "confirm": True, "prompt": "x",
                                      "server": "ws://x", "rtc": True, "chunk_steps": 50,
                                      "reinfer_remaining": 60})
            with self.assertRaises(operations.OperationError):
                self.manager._policy({"mode": "remote", "confirm": True, "prompt": "x",
                                      "server": "ws://x", "rtc": True, "chunk_steps": 0})

    def test_run_must_live_under_the_kernel_runs_root(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(operations.OperationError):
                operations._resolve_run(Path(folder))
            with patch.object(operations, "live_runs", return_value=[]):
                with self.assertRaises(operations.OperationError):
                    operations._resolve_run(None)


class OperationRouterTests(unittest.TestCase):
    def test_snapshot_requires_the_pairing_token(self):
        router = create_operations_router(
            operations.OperationManager(), BackendSettings(token="secret")
        )
        route = next(route for route in router.routes if route.path == "/api/v1/operations")
        for token in (None, "wrong"):
            with self.assertRaises(HTTPException) as raised:
                asyncio.run(route.endpoint(token=token))
            self.assertEqual(raised.exception.status_code, 401)
        self.assertEqual(asyncio.run(route.endpoint(token="secret")), {"operations": []})


if __name__ == "__main__":
    unittest.main()
