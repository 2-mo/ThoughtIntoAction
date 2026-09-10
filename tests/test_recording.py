"""Exercise real recording/IPC/validation with fake feedback and RGB; no CAN access."""

import copy
import json
from pathlib import Path
import subprocess
import shutil
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request

import numpy as np

from robot_core.project import load_config
from quest_teleop.datasets import image_path, label_episode, load_episode, nearest_indices, validate_episode
from robot_core.model import RobotModel
from quest_teleop.recording import EpisodeWriter
from robot_core.local_server import LocalServer
from robot_core.session_protocol import request
from quest_teleop.runtime.monitor import make_server
from quest_teleop.runtime.session import TeleopSession

Q = [0., 1.5, 1.45, -1.2, 0., 0.]


def snapshot(index=1, actuate=True):
    stamp = time.monotonic_ns()
    submission = {"sequence": index, "host_monotonic_ns": stamp,
                  "pos": (np.array(Q) + .02).tolist(), "vel": [0.] * 6, "torque": [0.] * 6,
                  "gripper_pos": .04, "gripper_vel": 0., "gripper_torque": 0.,
                  "timestamp_argument": 0., "gripper_mode": "position"}
    return {"actual_pos": Q.copy(), "actual_vel": [0.] * 6, "actual_torque": [0.] * 6,
            "sdk_cmd_pos": (np.array(Q) + .01).tolist(), "sdk_cmd_vel": [0.] * 6,
            "sdk_cmd_torque": [0.1] * 6, "kp": [1.] * 6, "kd": [1.] * 6,
            "gripper_actual_pos": .04, "gripper_actual_vel": 0., "gripper_actual_torque": 0.,
            "feedback_timestamp": stamp / 1e9, "command_timestamp": stamp / 1e9,
            "controller_timestamp": stamp / 1e9, "gripper_mode": "position",
            "requested_pos": (np.array(Q) + .03).tolist(), "requested_gripper_pos": .04,
            "requested_monotonic_ns": stamp, "submission": submission if actuate else None,
            "actuate": actuate, "source": "quest", "preset_name": None,
            "sequence": index, "host_monotonic_ns": stamp}



class ServiceSampleRecordingTests(unittest.TestCase):
    def test_drain_preserves_all_service_samples_across_batches(self):
        session = TeleopSession.__new__(TeleopSession)
        session._sample_cursor = 10
        session.latest_state = {"arm_service_id": "test-service"}
        rows = [{**snapshot(i), "sample_sequence": i} for i in range(11, 22)]
        session.record_samples = lambda after: {
            "states": [row for row in rows if row["sample_sequence"] > after][:4],
            "latest": 21,
        }
        saved = []
        class Writer:
            def add(self, kind, row):
                saved.append((kind, row))
        session._drain_samples(Writer())
        self.assertEqual(session._sample_cursor, 21)
        self.assertEqual([row["sample_sequence"] for _, row in saved], list(range(11, 22)))
        self.assertTrue(all(kind == "state" for kind, _ in saved))
        self.assertEqual([row["host_monotonic_ns"] for _, row in saved],
                         [row["host_monotonic_ns"] for row in rows])
        session._drain_samples(Writer())
        self.assertEqual(len(saved), 11)


class FakeCamera:
    def __init__(self, serial, width, height, fps):
        self.metadata = {"serial": serial, "model": "fake D435i", "width": width,
                         "height": height, "fps": fps, "format": "bgr8"}
        self.number = 0
        self.closed = False
        self.fail = False

    def __enter__(self):
        return self

    def read(self):
        time.sleep(1 / self.metadata["fps"])
        if self.fail:
            raise RuntimeError("test camera disconnected")
        self.number += 1
        stamp = time.monotonic_ns()
        return {"image": np.zeros((self.metadata["height"], self.metadata["width"], 3), dtype=np.uint8),
                "frame_number": self.number, "host_received_monotonic_ns": stamp,
                "device_timestamp_ms": stamp / 1e6, "device_timestamp_domain": "fake_clock"}

    def __exit__(self, *args):
        self.closed = True


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = copy.deepcopy(load_config())
        self.config["runtime"]["socket"] = str(self.root / "session.sock")
        self.config["paths"].update(dataset_dir=str(self.root / "data"), log_dir=str(self.root / "logs"))
        self.config["cameras"]["wrist_rgb"].update(serial="fake", width=32, height=24, fps=30)
        self.config["recording"].update(min_free_bytes=0, min_duration_s=.15)
        self.robot = RobotModel()
        self.camera = FakeCamera("fake", 32, 24, 30)
        self.session = TeleopSession(self.config, self.robot.metadata(),
                                    {"actuate": True, "gripper_width_m": .082, "control_rate_hz": 50},
                                    camera_factory=lambda *args: self.camera).start()
        self.addCleanup(self.session.close)
        self.stop = threading.Event()
        self.publisher = threading.Thread(target=self.publish, daemon=True)
        self.publisher.start()
        self.addCleanup(self.stop_publisher)
        self.wait_for(lambda: self.session.latest_frame is not None)

    def test_a_toggle_starts_task_saves_and_starts_next_without_pregrasp(self):
        self.assertFalse(self.session.task_active)
        first = self.session.dispatch({"command": "collection.toggle_episode", "task": "pick"})
        run_id = self.session.task_run_id
        self.assertTrue(self.session.task_active)
        self.wait_for(lambda: self.session.recording.status()["counts"]["state"] >= 2)
        saved = self.session.dispatch({"command": "collection.toggle_episode", "task": "pick"})
        self.assertEqual(saved["id"], first["id"])
        self.assertEqual(saved["lifecycle"], "complete")
        self.assertIsNone(self.session.recording)
        second = self.session.dispatch({"command": "collection.toggle_episode", "task": "pick"})
        self.assertNotEqual(second["id"], first["id"])
        self.assertEqual(self.session.task_run_id, run_id)
        self.session.dispatch({"command": "collection.toggle_episode", "task": "pick"})
        self.assertEqual(len(self.session.counted_episodes), 2)

    def test_task_groups_episodes_and_exports_once_at_end(self):
        sequence = 1
        def samples(after):
            nonlocal sequence
            if not after:
                return {"states": [], "latest": sequence}
            sequence += 1
            return {"states": [{**snapshot(sequence), "sample_sequence": sequence}], "latest": sequence}
        self.session.record_samples = samples
        self.session.control_metadata["recording_rate_hz"] = 240
        with patch("quest_teleop.auto_dataset.finalize_task") as export:
            saved = []
            for _ in range(3):
                self.session.dispatch({"command": "collection.toggle_episode", "task": "pick place"})
                self.wait_for(lambda: self.session.recording.counts["state"] >= 2)
                self.session.dispatch({"command": "collection.stop_episode"})
                saved.append(dict(self.session.last_episode))
            self.assertFalse(export.called)
            target = self.session.task_dataset["path"]
            self.assertTrue(Path(target).name.startswith("pick_place_"))
            self.assertTrue(all(Path(row["path"]).parent.name == ".raw" for row in saved))
            self.session.dispatch({"command": "collection.discard_previous"})
            self.session.dispatch({"command": "collection.pause_task"})
            self.session.dispatch({"command": "collection.start_task"})
            self.assertEqual(self.session.task_dataset["path"], target)
            self.session.dispatch({"command": "collection.end_task"})
            export.assert_called_once()
            self.assertEqual(export.call_args.args[1], sorted(row["id"] for row in saved[:2]))
            self.assertEqual(export.call_args.args[2], target)
            self.assertEqual(self.session.task_dataset["state"], "complete")
            self.assertEqual(self.session.task_dataset["episodes"], 2)
            self.assertEqual(len(self.session.saved_episodes()), 2)

    def test_failed_task_export_keeps_task_for_retry(self):
        self.session.task_dataset = {"state": "collecting", "path": str(self.root / "pick")}
        self.session.counted_episodes = frozenset(["example"])
        self.session.control_metadata["recording_rate_hz"] = 240
        self.session.task_active = True
        with patch("quest_teleop.auto_dataset.finalize_task", side_effect=RuntimeError("encoding failed")):
            with self.assertRaisesRegex(RuntimeError, "encoding failed"):
                self.session.dispatch({"command": "collection.end_task"})
        self.assertTrue(self.session.task_active)
        self.assertEqual(self.session.counted_episodes, frozenset(["example"]))
        self.assertEqual(self.session.task_dataset["state"], "failed")

    def publish(self):
        counter = 0
        while not self.stop.is_set():
            counter += 1
            self.session.publish(snapshot(counter))
            self.stop.wait(.02)

    def stop_publisher(self):
        self.stop.set()
        self.publisher.join(timeout=2)

    def wait_for(self, condition, timeout=3):
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > end:
                self.fail("Timed out waiting for fake runtime")
            time.sleep(.01)

    def test_collection_advance_discard_and_next_episode_over_real_session(self):
        self.session.dispatch({"command": "collection.start_task"})
        first = self.session.dispatch({"command": "collection.start_episode", "task": "Place object"})
        self.assertIsNotNone(self.session.recording)
        time.sleep(.2)
        monitor = self.session.dispatch({"command": "monitor"})
        self.assertEqual(monitor["recording"]["id"], first["id"])
        self.assertGreater(monitor["recording"]["counts"]["frame"], 0)
        self.assertIn("collection", monitor)
        self.assertIn("runtime_error", monitor)
        self.session.dispatch({"command": "collection.stop_episode"})
        monitor = self.session.dispatch({"command": "monitor"})
        self.assertIsNone(monitor["recording"])
        self.assertEqual(monitor["last_episode"]["id"], first["id"])
        self.assertIsNone(self.session.recording)
        self.assertEqual(self.session.last_episode["id"], first["id"])
        self.assertEqual(self.session.last_episode["outcome"], "unknown")
        second = self.session.dispatch({"command": "collection.start_episode", "task": "Place object"})
        self.assertNotEqual(first["id"], second["id"])
        time.sleep(.2)
        self.session.dispatch({"command": "collection.discard"})
        self.assertIsNone(self.session.recording)
        self.assertEqual(self.session.last_episode["outcome"], "discarded")
        self.session.dispatch({"command": "collection.start_episode", "task": "Place object"})
        self.session.dispatch({"command": "collection.home"})
        self.assertIsNone(self.session.recording)
        self.assertEqual(self.session.last_episode["outcome"], "discarded")

    def test_count_reset_and_discard_previous_retain_files(self):
        self.session.dispatch({"command": "collection.start_task"})
        self.record_episode(outcome="unknown")
        previous = dict(self.session.last_episode)
        path = Path(previous["path"])
        files = sorted(p.relative_to(path) for p in path.rglob("*") if p.is_file())
        self.assertEqual(self.session.dispatch({"command": "monitor"})["collection_count"], 1)
        self.session.dispatch({"command": "collection.start_episode", "task": "Next"})
        current = self.session.recording
        with self.assertRaisesRegex(ValueError, "Episode 已变化"):
            self.session.dispatch({"command": "collection.discard_previous", "episode_id": "another-episode"})
        self.assertEqual(self.session.status()["collection_count"], 1)
        self.session.dispatch({"command": "collection.discard_previous"})
        self.assertIs(self.session.recording, current)
        self.assertEqual(self.session.status()["collection_count"], 0)
        self.assertEqual(json.loads((path / "episode.json").read_text())["outcome"], "discarded")
        self.assertEqual(files, sorted(p.relative_to(path) for p in path.rglob("*") if p.is_file()))
        self.session.dispatch({"command": "collection.discard_previous"})
        self.assertEqual(self.session.status()["collection_count"], 0)
        time.sleep(.2)
        self.session.dispatch({"command": "collection.stop_episode"})
        self.assertEqual(self.session.status()["collection_count"], 1)
        self.session.dispatch({"command": "collection.end_task"})
        self.assertEqual(self.session.status()["collection_count"], 0)
        last_path = Path(self.session.last_episode["path"])
        self.assertEqual(json.loads((last_path / "episode.json").read_text())["outcome"], "unknown")
        self.session.dispatch({"command": "collection.discard_previous"})
        self.assertEqual(self.session.status()["collection_count"], 0)

    def test_task_pause_resume_and_end_preserve_files(self):
        self.assertFalse(self.session.task_active)
        with self.assertRaisesRegex(ValueError, "任务采集未开启"):
            self.session.dispatch({"command": "collection.start_episode", "task": "pick"})
        self.session.dispatch({"command": "collection.start_task"})
        task_id = self.session.task_run_id
        self.assertTrue(self.session.task_active)
        self.assertIsNone(self.session.recording)
        self.session.dispatch({"command": "collection.start_episode", "task": "pick"})
        time.sleep(.2)
        self.session.dispatch({"command": "collection.pause_task"})
        self.assertFalse(self.session.task_active)
        self.assertIsNone(self.session.recording)
        self.assertEqual(self.session.status()["collection_count"], 1)
        path = Path(self.session.last_episode["path"])
        self.assertIn(task_id, (path / "episode.json").read_text())
        self.assertEqual(self.session.last_episode["outcome"], "unknown")
        self.session.dispatch({"command": "collection.start_task"})
        self.assertTrue(self.session.task_active)
        self.assertEqual(self.session.task_run_id, task_id)
        self.assertEqual(self.session.status()["collection_count"], 1)
        self.session.dispatch({"command": "collection.end_task"})
        self.assertFalse(self.session.task_active)
        self.assertFalse(self.session.task_paused)
        self.session.dispatch({"command": "collection.start_task"})
        self.assertNotEqual(self.session.task_run_id, task_id)
        self.assertEqual(self.session.status()["collection_count"], 0)
        self.assertIsNone(self.session.last_episode)
        self.assertTrue(path.is_dir())

    def record_episode(self, outcome="success"):
        result = request(self.session.socket_path, "record.start", task="Put the object in the tray")
        time.sleep(.4)
        request(self.session.socket_path, "record.stop", outcome=outcome)
        return Path(result["path"])

    def test_real_ipc_record_validate_and_headless_replay(self):
        root = self.record_episode()
        report = validate_episode(root, settings=self.config["recording"], robot=self.robot)
        self.assertTrue(report["valid"], report)
        self.assertTrue(report["training_eligible"], report)
        _, meta, states, frames = load_episode(root)
        self.assertFalse(meta["hand_eye_calibrated"])
        self.assertNotEqual(states[0]["submission"]["pos"], states[0]["sdk_cmd_pos"])
        self.assertNotEqual(states[0]["submission"]["pos"], states[1]["actual_pos"])
        self.assertGreater(len(frames), 5)
        repo = Path(__file__).resolve().parents[1]
        result = subprocess.run([str(repo / "run.sh"), "dataset", "replay", str(root), "--headless"],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)["hardware_opened"])

    def test_duplicate_start_and_motor_requests_rejected(self):
        request(self.session.socket_path, "record.start", task="test")
        with self.assertRaisesRegex(RuntimeError, "already"):
            request(self.session.socket_path, "record.start", task="second")
        with self.assertRaisesRegex(RuntimeError, "Supported"):
            request(self.session.socket_path, "arm.move", joints=Q)
        with self.assertRaisesRegex(RuntimeError, "owned"):
            LocalServer(self.session.socket_path, lambda req: {})
        self.assertTrue(request(self.session.socket_path, "status")["recording"])

    def test_camera_failure_ends_episode_without_stopping_owner(self):
        request(self.session.socket_path, "record.start", task="test")
        self.camera.fail = True
        self.wait_for(lambda: self.session.last_episode is not None)
        self.assertEqual(self.session.last_episode["lifecycle"], "interrupted")
        self.assertFalse(self.stop.is_set())
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            request(self.session.socket_path, "record.start", task="test")

    def test_session_exit_marks_active_episode_interrupted_and_releases_camera(self):
        result = request(self.session.socket_path, "record.start", task="test")
        self.stop_publisher()
        self.session.close()
        meta = json.loads((Path(result["path"]) / "episode.json").read_text())
        self.assertEqual(meta["lifecycle"], "interrupted")
        self.assertEqual(meta["outcome"], "unknown")
        self.assertTrue(self.camera.closed)
        self.assertFalse(self.session.socket_path.exists())

    def test_validation_detects_corruption_and_preserves_original_data(self):
        root = self.record_episode()
        frame_path = root / "frames.jsonl"
        rows = frame_path.read_text().splitlines()
        rows[1] = rows[0]
        frame_path.write_text("\n".join(rows) + "\n")
        report = validate_episode(root, settings=self.config["recording"], robot=self.robot)
        self.assertFalse(report["valid"])
        self.assertTrue(any("Repeated" in error or "Non-increasing" in error for error in report["errors"]))
        self.assertEqual(frame_path.read_text(), "\n".join(rows) + "\n")

    def test_unknown_and_read_only_outcomes_cannot_be_training_eligible(self):
        root = self.record_episode(outcome="unknown")
        report = validate_episode(root, settings=self.config["recording"], robot=self.robot)
        self.assertTrue(report["valid"], report)
        self.assertFalse(report["training_eligible"])
        meta_path = root / "episode.json"
        meta = json.loads(meta_path.read_text())
        meta["outcome"] = "success"
        meta["control"]["actuate"] = False
        meta_path.write_text(json.dumps(meta))
        self.assertFalse(validate_episode(root, settings=self.config["recording"], robot=self.robot)["training_eligible"])

    def test_http_is_read_only_and_does_not_own_session_lifecycle(self):
        request(self.session.socket_path, "record.start", task="test")
        server = make_server(self.session.socket_path, 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(url + "/status", timeout=3) as response:
                self.assertEqual(json.load(response)["session_id"], self.session.id)
            req = urllib.request.Request(url + "/record.stop", data=b"{}", method="POST")
            with self.assertRaises(urllib.error.HTTPError) as error:
                opener.open(req, timeout=3)
            self.assertEqual(error.exception.code, 501)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
        self.assertIsNotNone(request(self.session.socket_path, "status")["recording"])

    def test_stale_robot_prevents_start(self):
        self.stop_publisher()
        self.session.latest_state["host_monotonic_ns"] -= 1_000_000_000
        with self.assertRaisesRegex(RuntimeError, "fresh robot"):
            request(self.session.socket_path, "record.start", task="test")

    def test_service_sample_time_is_preserved_instead_of_refreshing_stale_feedback(self):
        self.stop_publisher()
        packet = snapshot()
        packet["host_monotonic_ns"] -= 1_000_000_000
        began = time.monotonic_ns()
        self.session.publish(packet)
        self.assertEqual(self.session.latest_state["host_monotonic_ns"], packet["host_monotonic_ns"])
        self.assertGreaterEqual(self.session.latest_state["host_received_monotonic_ns"], began)
        with self.assertRaisesRegex(RuntimeError, "fresh robot"):
            request(self.session.socket_path, "record.start", task="test")

    def test_review_labels_keep_history_and_refuse_damaged_success(self):
        root = self.record_episode(outcome="unknown")
        report = validate_episode(root, settings=self.config["recording"], robot=self.robot)
        self.assertTrue(report["valid"], report)
        with patch("quest_teleop.datasets.validate_episode", return_value=report):
            label_episode(root, "success", "reviewed in replay")
        meta = json.loads((root / "episode.json").read_text())
        self.assertEqual(meta["outcome_history"][-1]["previous"], "unknown")
        self.assertEqual(meta["outcome"], "success")
        (root / "frames.jsonl").write_text("broken")
        with self.assertRaises(ValueError):
            label_episode(root, "success")

    def test_low_disk_prevents_creation_of_episode(self):
        usage = shutil.disk_usage(self.root)
        self.config["recording"]["min_free_bytes"] = usage.free + 10**12
        with self.assertRaisesRegex(RuntimeError, "disk"):
            request(self.session.socket_path, "record.start", task="test")
        self.assertIsNone(self.session.recording)


class WriterTests(unittest.TestCase):
    def test_queue_overflow_is_explicit_and_never_complete(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(EpisodeWriter, "_write", return_value=None):
            writer = EpisodeWriter(folder, {}, "test", queue_size=1)
            writer.add("state", snapshot())
            self.assertFalse(writer.add("state", snapshot(2)))
            result = writer.stop("success")
            self.assertEqual(result["lifecycle"], "failed")
            self.assertEqual(result["dropped"]["state"], 1)

    def test_image_writer_failure_is_retained(self):
        with tempfile.TemporaryDirectory() as folder, patch("cv2.imwrite", return_value=False):
            writer = EpisodeWriter(folder, {}, "test")
            writer.add("frame", {"image": np.zeros((2, 2, 3), np.uint8), "host_received_monotonic_ns": time.monotonic_ns()})
            result = writer.stop()
            self.assertEqual(result["lifecycle"], "failed")
            self.assertIn("RGB", result["error"])

    def test_stopping_time_excludes_background_drain_time(self):
        with tempfile.TemporaryDirectory() as folder:
            writer = EpisodeWriter(folder, {}, "test")
            writer.add("state", snapshot())
            writer.stop()
            stopped = writer.manifest["stopped_monotonic_ns"]
            time.sleep(.03)
            writer.stop()
            self.assertEqual(writer.manifest["stopped_monotonic_ns"], stopped)

    def test_nearest_timestamp_alignment_and_path_escape(self):
        np.testing.assert_array_equal(nearest_indices([10, 20, 30], [1, 15, 26, 35]), [0, 0, 2, 2])
        with self.assertRaises(ValueError):
            nearest_indices([20, 10], [15])
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, "leaves"):
                image_path(Path(folder), "../not-an-image")


if __name__ == "__main__":
    unittest.main()
