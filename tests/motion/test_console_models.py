"""Model registry: ids only, fixed commands, health-derived state."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from embodied.applications.console.core import models


REGISTRY = """
[[models]]
id = "a"
name = "模型 A"
prompt = "do a"
url = "ws://127.0.0.1:8008"
vram = "1 GB"
mount = ["bash", "-lc", "start {log}"]
unmount = ["bash", "-lc", "stop"]

[[models]]
id = "b"
name = "模型 B"
prompt = "do b"
url = "ws://10.0.0.2:8000"
mount_ready = false
note = "命令待核对"
unmount = ["bash", "-lc", "stop b"]
"""


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / "models.toml"
        self.path.write_text(REGISTRY)

    def tearDown(self):
        self.folder.cleanup()
        models._mounted.clear()

    def load(self):
        return models.load_models(self.path)

    def test_entries_keep_their_fixed_commands(self):
        entries = {entry["id"]: entry for entry in self.load()}
        self.assertEqual(entries["a"]["mount"], ["bash", "-lc", "start {log}"])
        self.assertTrue(entries["a"]["mountable"])
        self.assertFalse(entries["b"]["mountable"])

    def test_health_url_is_http_on_the_same_port(self):
        self.assertEqual(models.health_url("ws://127.0.0.1:8008"), "http://127.0.0.1:8008/healthz")
        self.assertEqual(models.health_url("ws://192.168.96.202:8008"),
                         "http://192.168.96.202:8008/healthz")

    def test_state_follows_the_probe_and_the_mount_time(self):
        entries = self.load()
        with patch.object(models, "probe", return_value=True):
            self.assertEqual(models.describe(entries)[0]["state"], "online")
        with patch.object(models, "probe", return_value=False):
            self.assertEqual(models.describe(entries)[0]["state"], "offline")
            models._mounted["a"] = models.time.monotonic()
            self.assertEqual(models.describe(entries)[0]["state"], "starting")
            models._mounted["a"] -= models.STARTING_S + 1
            self.assertEqual(models.describe(entries)[0]["state"], "offline")

    def test_mount_runs_the_registered_argv_and_log_path(self):
        entries = self.load()
        with patch.object(models, "probe", return_value=False), \
             patch.object(models.subprocess, "Popen") as popen:
            popen.return_value.pid = 4321
            result = models.mount("a", entries)
        self.assertEqual(result["state"], "starting")
        self.assertEqual(result["pid"], 4321)
        argv = popen.call_args.args[0]
        self.assertEqual(argv[:2], ["bash", "-lc"])
        self.assertTrue(argv[2].startswith("start "))
        self.assertTrue(argv[2].endswith("outputs/logs/models/a.log"))

    def test_unmount_without_a_registered_command_or_id(self):
        entries = self.load()
        with self.assertRaises(models.ModelError):
            models.mount("b", entries)          # mount_ready = false
        with self.assertRaises(models.ModelError):
            models.mount("missing", entries)    # ids only, never commands

    def test_unmount_runs_and_clears_the_mount_time(self):
        entries = self.load()
        models._mounted["a"] = models.time.monotonic()
        done = MagicMock(returncode=0, stdout="", stderr="")
        with patch.object(models.subprocess, "run", return_value=done) as run:
            result = models.unmount("a", entries)
        self.assertEqual(result["state"], "offline")
        run.assert_called_once()
        self.assertNotIn("a", models._mounted)


if __name__ == "__main__":
    unittest.main()
