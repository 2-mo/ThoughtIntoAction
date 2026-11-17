"""Remote-call settings: callers may only send prompt and duration."""
import json
import tempfile
import unittest
from pathlib import Path

from embodied.applications.console.core import policy_remote


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / "policy_remote.json"

    def tearDown(self):
        self.folder.cleanup()

    def test_defaults_apply_until_the_operator_saves(self):
        config = policy_remote.load_config(self.path)
        self.assertIsNone(config["model_id"])
        self.assertTrue(config["rtc"])
        self.assertEqual(config["chunk_steps"], 50)

    def test_saved_config_round_trips(self):
        policy_remote.save_config({"model_id": "towel_v2_local", "rtc": False,
                                   "reinfer_remaining": 20, "guidance": 12.5}, self.path)
        config = policy_remote.load_config(self.path)
        self.assertEqual(config["model_id"], "towel_v2_local")
        self.assertFalse(config["rtc"])
        self.assertEqual(config["reinfer_remaining"], 20)
        self.assertAlmostEqual(config["guidance"], 12.5)

    def test_out_of_range_values_are_refused(self):
        for payload in ({"chunk_steps": 0}, {"blend": 99}, {"guidance": 101},
                        {"reinfer_remaining": 60}, {"rtc": "yes"}, {"model_id": ""}):
            with self.assertRaises(policy_remote.RemoteError):
                policy_remote.save_config(payload, self.path)


class RequestTests(unittest.TestCase):
    def test_only_prompt_and_duration_are_read(self):
        fields = policy_remote.request_fields({
            "prompt": "  Fold the towel on the table.  ", "seconds": 30,
            "server": "ws://evil:1", "model_id": "other", "rtc": True,
        })
        self.assertEqual(fields, {"prompt": "Fold the towel on the table.", "seconds": 30})

    def test_defaults_and_limits(self):
        self.assertEqual(policy_remote.request_fields({"prompt": "x"})["seconds"], 60)
        for payload in ({"prompt": ""}, {"prompt": "x" * 501}, {"prompt": "x", "seconds": 0},
                        {"prompt": "x", "seconds": 301}, {"prompt": 5}):
            with self.assertRaises(policy_remote.RemoteError):
                policy_remote.request_fields(payload)


class ProgressTests(unittest.TestCase):
    def test_counts_runner_events(self):
        lines = [
            json.dumps({"event": "connected", "server": "ws://x"}),
            json.dumps({"event": "chunk", "index": 1, "latency_ms": 480.5}),
            json.dumps({"event": "chunk", "index": 2, "latency_ms": 512.0}),
            json.dumps({"event": "rejected", "tick": 9}),
            "Traceback (most recent call last):",
        ]
        result = policy_remote.progress(lines)
        self.assertEqual(result["chunks"], 2)
        self.assertEqual(result["rejected"], 1)
        self.assertEqual(result["last_chunk_latency_ms"], 512.0)


if __name__ == "__main__":
    unittest.main()
