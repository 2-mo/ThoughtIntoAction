"""Whitelisted console operations: validation, refusals, and the token gate."""
import asyncio
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import json
import signal

from fastapi import HTTPException

from embodied.applications.console import operations
from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.transport.operations import create_operations_router


def setup_document(arms: int) -> dict:
    robots = ([{"id": "arm1", "vendor": "arx", "model": "x5", "side": "single", "can_interface": "can0"}]
              if arms == 1 else
              [{"id": "arm1", "vendor": "arx", "model": "x5", "side": "left", "can_interface": "can1"},
               {"id": "arm2", "vendor": "arx", "model": "x5", "side": "right", "can_interface": "can0"}])
    return {"version": 1, "robots": robots,
            "cameras": [{"id": "wrist_rgb", "serial": "1", "mount": "wrist"}]}


class KernelModeTests(unittest.TestCase):
    def setUp(self):
        self.manager = operations.OperationManager()

    def test_modes_follow_the_saved_setup(self):
        from embodied.applications.console.core import devices

        with patch.object(devices, "load_setup", return_value=setup_document(2)):
            self.assertEqual(self.manager._mode_target("collect")["profile"], "real-dual-quest")
            self.assertEqual(self.manager._mode_target("policy")["profile"], "real-dual-manual")
        with patch.object(devices, "load_setup", return_value=setup_document(1)):
            self.assertEqual(self.manager._mode_target("collect")["profile"], "real-quest")
            self.assertEqual(self.manager._mode_target("policy")["profile"], "real-manual")

    def test_sim_and_unknown_modes(self):
        self.assertEqual(self.manager._mode_target("sim"),
                         {"vendor": "arx", "profile": "sim-manual", "real": False})
        with self.assertRaises(operations.OperationError):
            self.manager._mode_target("teleop")

    def test_realman_policy_mode_uses_the_generated_workbench_without_can(self):
        from embodied.applications.console.core import devices

        setup = {"version": 1, "robots": [{"id": "arm1", "vendor": "realman", "model": "rm65",
                                           "side": "single", "host": "192.168.1.19"}],
                 "cameras": [{"id": "wrist_rgb", "serial": "1", "mount": "wrist"}]}
        compiled = {"workbench": {"collect": "/tmp/set/realman-left.json",
                                  "policy": "/tmp/set/realman-manual.json"}}
        with patch.object(devices, "load_setup", return_value=setup), \
                patch.object(devices, "compile_setup", return_value=compiled), \
                patch.object(operations, "live_runs", return_value=[]):
            argv, _ = self.manager._kernel({"mode": "policy", "confirm": True})
        self.assertIn("/tmp/set/realman-manual.json", argv)
        self.assertIn("realman", argv)
        self.assertIn("none", argv)
        self.assertFalse(any("can-up" in part for part in argv))

    def test_realman_dual_starts_the_pair_supervisor(self):
        from embodied.applications.console.core import devices

        setup = {"version": 1, "robots": [
            {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "left", "host": "192.168.1.19"},
            {"id": "arm2", "vendor": "realman", "model": "rm65", "side": "right", "host": "192.168.1.18"},
        ], "cameras": [{"id": "wrist_rgb", "serial": "1", "mount": "wrist"}]}
        compiled = {"pair": {"left": "/tmp/set/realman-left.json", "right": "/tmp/set/realman-right.json"},
                    "controllers": {"left": "right", "right": "left"},
                    "workbench": {"left": "/tmp/set/realman-left.json", "right": "/tmp/set/realman-right.json"}}
        with patch.object(devices, "load_setup", return_value=setup), \
                patch.object(devices, "compile_setup", return_value=compiled), \
                patch.object(operations, "live_runs", return_value=[]):
            argv, _ = self.manager._kernel({"mode": "collect", "confirm": True})
        self.assertIn("robot-pair", argv)
        self.assertIn("quest", argv)
        self.assertIn("/tmp/set/realman-left.json", argv)
        self.assertIn("/tmp/set/realman-right.json", argv)
        self.assertEqual(argv[argv.index("--left-controller") + 1], "right")
        self.assertEqual(argv[argv.index("--right-controller") + 1], "left")

    def test_realman_dual_policy_uses_the_cli_source(self):
        from embodied.applications.console.core import devices

        setup = {"version": 1, "robots": [
            {"id": "arm1", "vendor": "realman", "model": "rm65", "side": "left", "host": "192.168.1.19"},
            {"id": "arm2", "vendor": "realman", "model": "rm65", "side": "right", "host": "192.168.1.18"},
        ], "cameras": [{"id": "wrist_rgb", "serial": "1", "mount": "wrist"}]}
        with patch.object(devices, "load_setup", return_value=setup):
            target = self.manager._mode_target("policy")
        self.assertEqual(target["source"], "cli")
        self.assertTrue(target["real"])

    def test_realman_collect_starts_the_quest_owned_workbench(self):
        from embodied.applications.console.core import devices

        setup = {"version": 1, "robots": [{"id": "arm1", "vendor": "realman", "model": "rm65",
                                           "side": "single", "host": "192.168.1.19"}],
                 "cameras": [{"id": "wrist_rgb", "serial": "1", "mount": "wrist"}]}
        compiled = {"workbench": {"collect": "/tmp/set/realman-left.json",
                                  "policy": "/tmp/set/realman-manual.json"}}
        with patch.object(devices, "load_setup", return_value=setup), \
                patch.object(devices, "compile_setup", return_value=compiled), \
                patch.object(operations, "live_runs", return_value=[]):
            argv, _ = self.manager._kernel({"mode": "collect", "confirm": True})
        self.assertIn("/tmp/set/realman-left.json", argv)
        self.assertIn("quest", argv)

    def test_policy_mode_uses_the_generated_manual_workbench(self):
        from embodied.applications.console.core import devices

        compiled = {"workbench": {"collect": "/tmp/set/real-dual-quest.json",
                                  "policy": "/tmp/set/real-dual-manual.json"}}
        with patch.object(devices, "load_setup", return_value=setup_document(2)), \
                patch.object(devices, "compile_setup", return_value=compiled), \
                patch.object(operations, "live_runs", return_value=[]):
            argv, _ = self.manager._kernel({"mode": "policy", "confirm": True})
        self.assertIn("/tmp/set/real-dual-manual.json", argv)

    def test_mode_refuses_a_setup_with_the_other_arm_count(self):
        from embodied.applications.console.core import devices

        with patch.object(devices, "load_setup", return_value=setup_document(1)), \
                patch.object(devices, "compile_setup", return_value={
                    "workbench": {"collect": "/tmp/single.json", "policy": "/tmp/single-manual.json"}}):
            with self.assertRaises(operations.OperationError) as raised:
                self.manager._setup_profile("real-dual-quest")
        self.assertEqual(raised.exception.status, 409)


class ExternalKernelStopTests(unittest.TestCase):
    def test_stop_signals_the_supervisor_of_a_kernel_this_console_did_not_start(self):
        manager = operations.OperationManager()
        calls = {"count": 0}

        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            (run / "runtime.json").write_text(json.dumps({"supervisor_pid": 4242}))

            def live(_root):
                calls["count"] += 1
                return [run] if calls["count"] == 1 else []

            with patch.object(operations, "live_runs", side_effect=live), \
                    patch.object(operations.os, "kill") as kill:
                result = asyncio.run(manager.stop("kernel"))
        kill.assert_called_once_with(4242, signal.SIGINT)
        self.assertEqual(result["state"], "stopped")
        self.assertTrue(result["external"])

    def test_stop_without_any_kernel_is_not_found(self):
        manager = operations.OperationManager()
        with patch.object(operations, "live_runs", return_value=[]):
            with self.assertRaises(operations.OperationError) as raised:
                asyncio.run(manager.stop("kernel"))
        self.assertEqual(raised.exception.status, 404)


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
        from embodied.applications.console.core import devices

        with patch.object(operations, "live_runs", return_value=[]), \
                patch.object(devices, "load_setup", return_value=None):
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

    def test_rm65_model_rejects_other_kernels(self):
        with patch.object(operations, "_resolve_run", return_value=Path("/tmp/run")), \
             patch.object(operations, "ControlClient") as client:
            client.return_value.parameters = {"data_contract": {"robot_type": "arx"}}
            client.return_value.status.return_value = {"owner": None}
            with self.assertRaisesRegex(operations.OperationError, "当前内核不是 RM65 单臂"):
                self.manager._policy({
                    "mode": "remote", "model_id": "rm65_cube_pi05", "confirm": True,
                    "prompt": "把黑色方块放到白色盒子里", "dry_run": True,
                })

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
