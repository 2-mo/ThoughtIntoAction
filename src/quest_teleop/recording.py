"""Bounded, asynchronous raw episodes. No hardware imports or motor commands."""

from datetime import datetime
import json
from pathlib import Path
import queue
import threading
import time
import uuid

from robot_core.storage import save_json


class EpisodeWriter:
    def __init__(self, root, metadata, task, *, queue_size=128):
        if not isinstance(task, str) or not task.strip() or len(task) > 2000:
            raise ValueError("Provide a task description (1..2000 characters)")
        if queue_size < 1:
            raise ValueError("Writer queue must be positive")
        self.id = f"{datetime.now():%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:8]}"
        self.path = Path(root) / self.id
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "wrist_rgb").mkdir()
        self.queue = queue.Queue(maxsize=queue_size)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.accepting = True
        self.error = None
        self.stopped_ns = None
        self.counts = {"state": 0, "frame": 0}
        self.dropped = {"state": 0, "frame": 0}
        self.manifest = {"schema_version": 1, "kind": "quest_teleop_episode",
                         "id": self.id, "task": task.strip(), "lifecycle": "recording",
                         "outcome": "unknown", "started_unix_ns": time.time_ns(),
                         "started_monotonic_ns": time.monotonic_ns(), **metadata}
        save_json(self.path / "episode.json", self.manifest)
        self.thread = threading.Thread(target=self._write, name="episode-writer", daemon=True)
        self.thread.start()

    def add(self, kind, payload):
        """Producer only queues immutable snapshots; no compression or file IO here."""
        if kind not in self.counts:
            raise ValueError("Unknown episode record kind")
        with self.lock:
            if not self.accepting:
                return False
            timestamp = payload["host_monotonic_ns"] if kind == "state" else payload["host_received_monotonic_ns"]
            if timestamp < self.manifest["started_monotonic_ns"]:
                return False  # In-flight observation predates this episode.
            try:
                self.queue.put_nowait((kind, payload))
                return True
            except queue.Full:
                self.dropped[kind] += 1
                self.error = "Episode writer queue overflow; data is incomplete"
                self.accepting = False
                self.stopped_ns = time.monotonic_ns()
                self.stop_event.set()
                return False

    def _write(self):
        import cv2
        try:
            with (self.path / "states.jsonl").open("w") as states, (self.path / "frames.jsonl").open("w") as frames:
                last_flush = time.monotonic()
                while not self.stop_event.is_set() or not self.queue.empty():
                    try:
                        kind, payload = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if kind == "frame":
                        row = {k: v for k, v in payload.items() if k != "image"}
                        row["image"] = f"wrist_rgb/{self.counts['frame']:06d}.png"
                        if not cv2.imwrite(str(self.path / row["image"]), payload["image"],
                                           [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                            raise OSError("Cannot write RGB image")
                        frames.write(json.dumps(row, allow_nan=False) + "\n")
                    else:
                        states.write(json.dumps(payload, allow_nan=False) + "\n")
                    self.counts[kind] += 1
                    if time.monotonic() - last_flush > 1:
                        states.flush()
                        frames.flush()
                        last_flush = time.monotonic()
        except Exception as exc:
            with self.lock:
                self.error = f"{type(exc).__name__}: {exc}"
                self.accepting = False
            self.stop_event.set()

    def status(self):
        return {"id": self.id, "path": str(self.path), "queue_size": self.queue.qsize(),
                "task": self.manifest["task"], "lifecycle": self.manifest["lifecycle"],
                "outcome": self.manifest["outcome"],
                "counts": dict(self.counts), "dropped": dict(self.dropped), "error": self.error,
                "duration_s": ((self.stopped_ns or time.monotonic_ns()) - self.manifest["started_monotonic_ns"]) / 1e9}

    def stop(self, outcome="unknown", *, reason=None, timeout=10):
        if outcome not in {"success", "failure", "unknown", "discarded"}:
            raise ValueError("Outcome must be success/failure/unknown/discarded")
        with self.lock:
            self.accepting = False
            if self.stopped_ns is None:
                self.stopped_ns = time.monotonic_ns()
            self.stop_event.set()
        self.thread.join(timeout=timeout)
        if self.thread.is_alive():
            # Never claim completion while a writer can still modify the episode.
            raise RuntimeError("Writer is still finalizing; episode remains incomplete")
        self.manifest.update({"lifecycle": "failed" if self.error else "interrupted" if reason else "complete",
                              "outcome": outcome, "stopped_monotonic_ns": self.stopped_ns,
                              "counts": dict(self.counts), "dropped": dict(self.dropped),
                              "error": self.error, "stop_reason": reason})
        save_json(self.path / "episode.json", self.manifest)
        return {**self.status(), "lifecycle": self.manifest["lifecycle"], "outcome": outcome}
