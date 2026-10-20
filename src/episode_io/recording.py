"""Bounded, asynchronous raw episodes. No hardware imports or motor commands."""

from datetime import datetime
from pathlib import Path
import queue
import re
import threading
import time
import uuid

from robot_core.storage import save_json
from episode_io.cameras import camera_metadata


class EpisodeWriter:
    def __init__(self, root, metadata, task, *, queue_size=128, rgb_video=False, video_crf=20):
        if not isinstance(task, str) or not task.strip() or len(task) > 2000:
            raise ValueError("Provide a task description (1..2000 characters)")
        if queue_size < 1:
            raise ValueError("Writer queue must be positive")
        from episode_io.mcap_io import RecordWriter  # Check dependency before creating an episode.
        task_name = re.sub(r"[^\w\-]+", "_", task.strip(), flags=re.UNICODE).strip("_.")[:48] or "task"
        self.id = f"{task_name}_{datetime.now():%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:8]}"
        self.path = Path(root) / self.id
        self.path.mkdir(parents=True, exist_ok=False)
        self.cameras = camera_metadata(metadata)
        for camera_id in self.cameras:
            (self.path / camera_id).mkdir()
        self.camera_counts = dict.fromkeys(self.cameras, 0)
        self.camera_dropped = dict.fromkeys(self.cameras, 0)
        self.queue = queue.Queue(maxsize=queue_size)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.accepting = True
        self.error = None
        self.stopped_ns = None
        if not 18 <= video_crf <= 23:
            raise ValueError("video_crf must be between 18 and 23")
        self.rgb_video, self.video_crf = rgb_video, video_crf
        self.diagnostic_counts = {"diagnostics": 0, "event": 0}
        self.counts = {"state": 0, "frame": 0}
        self.dropped = {"state": 0, "frame": 0}
        self.manifest = {"schema_version": 1, "kind": "quest_teleop_episode",
                         "id": self.id, "task": task.strip(), "lifecycle": "recording",
                         "outcome": "unknown", "started_unix_ns": time.time_ns(),
                         "started_monotonic_ns": time.monotonic_ns(), **metadata}
        self.manifest["record_storage"] = {"format": "mcap", "file": "episode.mcap"}
        self.manifest["depth_recorded"] = False
        self.manifest["rgb_storage"] = {"format": "mp4" if rgb_video else "png", "codec": "libx264" if rgb_video else None, "crf": video_crf if rgb_video else None}
        save_json(self.path / "episode.json", self.manifest)
        self.thread = threading.Thread(target=self._write, name="episode-writer", daemon=True)
        self.thread.start()

    def add(self, kind, payload):
        """Producer only queues immutable snapshots; no compression or file IO here."""
        if kind not in self.counts:
            raise ValueError("Unknown episode record kind")
        if kind == 'frame' and payload.get('camera_id', 'wrist_rgb') not in self.cameras:
            raise ValueError('Frame belongs to an unconfigured recording camera')
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
                if kind == 'frame':
                    self.camera_dropped[payload.get('camera_id', 'wrist_rgb')] += 1
                self.error = "Episode writer queue overflow; data is incomplete"
                self.accepting = False
                self.stopped_ns = time.monotonic_ns()
                self.stop_event.set()
                return False

    def _write(self):
        import cv2
        videos = {}
        try:
            from episode_io.mcap_io import RecordWriter
            with (self.path / "episode.mcap").open("wb") as stream:
                records = RecordWriter(stream, self.manifest)
                last_flush = time.monotonic()
                while not self.stop_event.is_set() or not self.queue.empty():
                    try:
                        kind, payload = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if kind == "frame":
                        row = {k: v for k, v in payload.items() if k != "image"}
                        camera_id = payload.get('camera_id', 'wrist_rgb')
                        row['camera_id'] = camera_id
                        index = self.camera_counts[camera_id]
                        if self.rgb_video:
                            if camera_id not in videos:
                                from episode_io.video import RGBVideoWriter
                                camera = self.cameras[camera_id]
                                videos[camera_id] = RGBVideoWriter(self.path / f'{camera_id}.mp4', camera['width'], camera['height'], camera['fps'], self.video_crf)
                            videos[camera_id].write(payload['image'])
                            row.update(video=f'{camera_id}.mp4', video_frame_index=index,
                                       video_pts_s=index / self.cameras[camera_id]['fps'])
                        else:
                            row["image"] = f"{camera_id}/{index:06d}.png"
                            if not cv2.imwrite(str(self.path / row["image"]), payload["image"], [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                                raise OSError("Cannot write RGB image")

                    else:
                        row = payload

                    if kind == "state":
                        records.write_state(row, self.counts[kind])
                        self.diagnostic_counts = dict(records.extra_counts)
                    else:
                        records.write(kind, row, index)
                        self.camera_counts[camera_id] += 1
                    # Retain the legacy wrist frame count; other cameras have independent counts.
                    if kind == 'state' or camera_id == 'wrist_rgb':
                        self.counts[kind] += 1
                    if time.monotonic() - last_flush > 1:
                        stream.flush()
                        last_flush = time.monotonic()
                records.finish()
            while videos:
                _, video = videos.popitem()
                video.close()
        except Exception as exc:
            for video in videos.values():
                try:
                    video.close()
                except Exception:
                    pass
            with self.lock:
                self.error = f"{type(exc).__name__}: {exc}"
                self.accepting = False
            self.stop_event.set()

    def status(self):
        return {"id": self.id, "path": str(self.path), "queue_size": self.queue.qsize(),
                "task": self.manifest["task"], "lifecycle": self.manifest["lifecycle"],
                "outcome": self.manifest["outcome"],
                "counts": dict(self.counts), "diagnostic_counts": dict(self.diagnostic_counts), "dropped": dict(self.dropped), "error": self.error,
                "camera_counts": dict(self.camera_counts), "camera_dropped": dict(self.camera_dropped),
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
                              "counts": dict(self.counts), "diagnostic_counts": dict(self.diagnostic_counts), "dropped": dict(self.dropped),
                              "camera_counts": dict(self.camera_counts), "camera_dropped": dict(self.camera_dropped),
                              "error": self.error, "stop_reason": reason})
        save_json(self.path / "episode.json", self.manifest)
        return {**self.status(), "lifecycle": self.manifest["lifecycle"], "outcome": outcome}
