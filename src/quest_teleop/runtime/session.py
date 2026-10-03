"""Teleop snapshots, named camera streams, background logging and episode quality."""

import base64
from datetime import datetime
import hashlib
import json
import math
import os
import re
from pathlib import Path
import shutil
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor

from robot_core.plugins import Registry
from ..recording import EpisodeWriter
from robot_core.storage import save_json
from robot_core.local_server import LocalServer


class TeleopSession:
    def __init__(self, config, robot_metadata, control_metadata, *, camera_factory=None):
        from robot_core.project import PROJECT_ROOT, resolve_path
        self.config = config
        settings = config["recording"]
        for key in ("max_duration_s", "min_duration_s", "max_state_gap_s", "max_frame_gap_s", "max_sync_skew_s"):
            value = settings[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"recording.{key} must be finite and positive")
        for key, minimum in (("queue_size", 1), ("min_free_bytes", 0)):
            if type(settings[key]) is not int or settings[key] < minimum:
                raise ValueError(f"recording.{key} must be an integer >= {minimum}")
        self.robot_metadata, self.control_metadata = robot_metadata, control_metadata
        self.id = uuid.uuid4().hex
        self.socket_path = resolve_path(config["runtime"]["socket"])
        self.dataset_root = resolve_path(config["paths"]["dataset_dir"])
        self.log_root = resolve_path(config["paths"]["log_dir"]) / f"session-{datetime.now():%Y%m%d-%H%M%S}-{self.id[:8]}"
        self.log_root.mkdir(parents=True)
        self._sample_cursor = 0
        self.latest_state = None
        self.latest_frame = None
        self._camera_record_lease = None
        self.camera_metadata = None
        self.camera_error = None
        self.runtime_error = None
        self.preview_jpeg = None
        self.preview_frame = None
        self.recording = None
        self.last_episode = None
        self._web_replay = None
        self.task_paused = False
        self.task_active = False
        self.task_run_id = None
        self.task_dataset = None
        self._dataset_run_id = None
        self.counted_episodes = frozenset()
        self.operations = threading.Lock()
        self.stop_event = threading.Event()
        self.closed = False
        self.camera_factory = camera_factory or Registry().load("camera", config["cameras"]["wrist_rgb"]["driver"])
        self.camera_ids = settings.get('camera_ids', ['wrist_rgb'])
        if (not isinstance(self.camera_ids, list) or 'wrist_rgb' not in self.camera_ids
                or any(not isinstance(key, str) or key not in config['cameras'] for key in self.camera_ids)
                or len(set(self.camera_ids)) != len(self.camera_ids)):
            raise ValueError('recording.camera_ids must include wrist_rgb and unique configured camera IDs')
        self.camera_states = {key: {'metadata': None, 'frame': None, 'error': 'Camera starting'} for key in self.camera_ids}
        self.quality_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='episode-quality')
        self.quality_jobs = []
        self.quality_results = {}
        self.server = None
        self.threads = []
        self.sequence = 0
        self.collection_status = None
        source = hashlib.sha256()
        files = list((PROJECT_ROOT / "src").rglob("*.py")) + list((PROJECT_ROOT / "scripts").rglob("*.py"))
        files.extend(PROJECT_ROOT / name for name in ("dependencies.lock.toml", "configs/plugins.toml", "pyproject.toml", "uv.lock"))
        for path in sorted(files):
            source.update(str(path.relative_to(PROJECT_ROOT)).encode() + b"\0" + path.read_bytes())
        self.source_fingerprint = source.hexdigest()
        save_json(self.log_root / "session.json", {"id": self.id, "pid": os.getpid(),
                  "robot": robot_metadata, "control": control_metadata, "config": config,
                  "source_sha256": self.source_fingerprint,
                  "started_unix_ns": time.time_ns()})

    def start(self):
        try:
            self.server = LocalServer(self.socket_path, self.dispatch)
            workers = [(key, lambda key=key: self._camera_worker(key)) for key in self.camera_ids]
            for name, target in [*workers, ("session-log", self._background), ("arm-samples", self._sample_worker)]:
                thread = threading.Thread(target=target, name=name, daemon=True)
                self.threads.append(thread)
                thread.start()
            return self
        except BaseException:
            self.close()
            raise

    def publish(self, snapshot):
        """Called after SDK submission, in the control thread. No file IO/encoding/waits."""
        self.sequence += 1
        received = time.monotonic_ns()
        packet = {**snapshot, "sequence": self.sequence,
                  "host_monotonic_ns": snapshot.get("host_monotonic_ns", received),
                  "host_received_monotonic_ns": received}
        self.latest_state = packet
        writer = self.recording
        if writer is not None and not getattr(self, "record_samples", None):
            writer.add("state", packet)

    def _drain_samples(self, writer):
        while True:
            batch = self.record_samples(self._sample_cursor)
            for sample in batch['states']:
                self._sample_cursor = sample['sample_sequence']
                writer.add('state', {**sample, 'arm_service_id': (self.latest_state or {}).get('arm_service_id')})
            if self._sample_cursor >= batch['latest'] or not batch['states']:
                break

    def _sample_worker(self):
        while not self.stop_event.wait(.01):
            if not getattr(self, 'record_samples', None):
                continue
            try:
                with self.operations:
                    if self.recording is not None:
                        self._drain_samples(self.recording)
            except Exception as exc:
                self.runtime_error = '逐周期录制失败：' + str(exc)
                with self.operations:
                    if self.recording is not None:
                        self._stop_recording(reason=self.runtime_error)

    def _camera_worker(self, camera_id='wrist_rgb'):
        options = self.config["cameras"][camera_id]
        if not options["enabled"]:
            self.camera_states[camera_id] = {'metadata': None, 'frame': None, 'error': 'Recording camera disabled'}
            if camera_id == 'wrist_rgb':
                self.camera_error = "Wrist camera disabled"
            return
        while not self.stop_event.is_set():
            try:
                import cv2
                arguments = {} if camera_id == 'wrist_rgb' else {'camera_id': camera_id}
                with self.camera_factory(options["serial"], options["width"], options["height"], options["fps"], **arguments) as camera:
                    if camera_id == 'wrist_rgb':
                        self.camera_metadata = camera.metadata
                    last_preview = 0.
                    while not self.stop_event.is_set():
                        frame = camera.read()
                        frame = {**frame, 'camera_id': camera_id}
                        self.camera_states[camera_id] = {'metadata': camera.metadata, 'frame': frame, 'error': None}
                        if camera_id == 'wrist_rgb':
                            self.camera_error = None
                            self.latest_frame = frame
                        writer = self.recording
                        if writer is not None:
                            writer.add("frame", frame)
                        if camera_id == 'wrist_rgb' and time.monotonic() - last_preview >= .2:
                            thumbnail = cv2.resize(frame["image"], (320, 240))
                            ok, encoded = cv2.imencode(".jpg", thumbnail, [cv2.IMWRITE_JPEG_QUALITY, 75])
                            if ok:
                                self.preview_jpeg = base64.b64encode(encoded).decode("ascii")
                                self.preview_frame = {"host_received_monotonic_ns": frame["host_received_monotonic_ns"]}
                            last_preview = time.monotonic()
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                self.camera_states[camera_id] = {'metadata': None, 'frame': None, 'error': error}
                if camera_id == 'wrist_rgb':
                    self.camera_error = error
                    self.latest_frame = self.preview_frame = self.preview_jpeg = None
                self.stop_event.wait(.5)

    @staticmethod
    def age(packet, key):
        return None if packet is None else max(0., (time.monotonic_ns() - packet[key]) / 1e9)

    def status(self):
        writer = self.recording
        return {"session_id": self.id, "pid": os.getpid(), "mode": self.control_metadata.get("mode", "teleop"),
                "rotation_frame": self.control_metadata.get("rotation_frame", "base"),
                "actuate": self.control_metadata["actuate"], "state": self.latest_state,
                "state_age_s": self.age(self.latest_state, "host_monotonic_ns"),
                "camera": {"metadata": self.camera_metadata, "error": self.camera_error,
                           "age_s": self.age(self.latest_frame, "host_received_monotonic_ns")},
                "recording": writer.status() if writer else None,
                "last_episode": ({**self.last_episode, 'quality': self.quality_results.get(self.last_episode['id'], self.last_episode.get('quality'))}
                                 if self.last_episode else None),
                "task_dataset": self.task_dataset, "runtime_error": self.runtime_error,
                "collection_count": len(self.counted_episodes),
                "task_active": self.task_active, "task_paused": self.task_paused, "task_run_id": self.task_run_id,
                "log_dir": str(self.log_root),
                "collection": self.collection_status() if self.collection_status else None}

    def saved_episodes(self):
        result = []
        for manifest in [*self.dataset_root.glob("*/episode.json"), *self.dataset_root.glob(".raw/*/episode.json")]:
            try:
                if not manifest.resolve().is_relative_to(self.dataset_root.resolve()): continue
                meta = json.loads(manifest.read_text())
                if meta.get("kind") == "quest_teleop_episode" and meta.get("lifecycle") == "complete" and meta.get("outcome") != "discarded":
                    result.append({"id": manifest.parent.name, "task": meta.get("task", ""),
                                   "started": meta.get("started_unix_ns", 0)})
            except (OSError, ValueError, TypeError):
                continue
        return sorted(result, key=lambda row: (row["started"], row["id"]), reverse=True)

    def load_saved_episode(self, episode_id):
        from ..replay import RecordedEpisode
        if not isinstance(episode_id, str) or Path(episode_id).name != episode_id or episode_id in (".", "..", ""):
            raise ValueError("无效数据集编号")
        root = (self.dataset_root / episode_id).resolve()
        if not (root / "episode.json").is_file():
            root = (self.dataset_root / ".raw" / episode_id).resolve()
        if not root.is_relative_to(self.dataset_root.resolve()): raise ValueError("数据集路径越界")
        with self.operations:
            if self.closed or self.recording is not None: raise ValueError("请先完成并保存当前采集")
        return RecordedEpisode(root)

    def load_previous_episode(self):
        from ..replay import RecordedEpisode
        with self.operations:
            if self.closed or self.recording is not None:
                raise ValueError("请先保存当前采集，再回放上一条")
            previous = self.last_episode
        if previous is None:
            saved = self.saved_episodes()
            if not saved: raise ValueError("还没有完整保存的数据可回放")
            return self.load_saved_episode(saved[0]["id"])
        return RecordedEpisode(previous["path"])

    def dispatch(self, payload):
        command = payload.get("command")
        if command == "replay.list":
            return {"episodes": self.saved_episodes()}
        if command in ("collection.replay_previous", "collection.replay_episode"):
            from ..replay import WebReplay
            episode = self.load_previous_episode() if command.endswith("previous") else self.load_saved_episode(payload.get("episode_id"))
            with self.operations:
                if self.closed or self.recording is not None:
                    raise ValueError("采集状态已变化，请先完成保存")
                self._web_replay = WebReplay(episode)
            return {"episode": episode.meta["id"]}
        if command in ("replay.frame", "replay.control"):
            with self.operations:
                if command == "replay.control":
                    if self._web_replay is None: raise ValueError("请先加载上一条")
                    self._web_replay.control(payload.get("action"), payload.get("position_s"), payload.get("speed"))
                frame = self._web_replay.frame() if self._web_replay else None
                return {"replay": frame, "robot_replay": (self.latest_state or {}).get("robot_replay")}

        if command == "input.calibrate":
            callback = getattr(self, "frame_calibration", None)
            if callback is None:
                raise RuntimeError("This session does not support frame calibration")
            result = callback(payload["rotation"], payload["session_id"])
            save_json(self.log_root / f"direction-alignment-{time.time_ns()}.json", result)
            return result
        if command in ("input.pause", "input.resume"):
            callback = getattr(self, "input_pause", None)
            if callback is None:
                raise RuntimeError("This session has no Quest input lifecycle")
            return callback(command == "input.pause")
        if command == "status":
            return self.status()
        if command == "monitor":
            snapshot = self.status()
            state = snapshot["state"] or {}
            return {"camera_switch_protocol": 1, "camera": snapshot["camera"],
                    "preview": self.dispatch({"command": "preview"}),
                    "input": state.get("input"),
                    "button_feedback": state.get("button_feedback"),
                    "orientation_preview": state.get("orientation_preview"),
                    "loop_rates": state.get("loop_rates"),
                    "table_safety": state.get("table_safety"),
                    "state_age_s": snapshot["state_age_s"],
                    "motion_diagnostics": state.get("motion_diagnostics"),
                    "joint_limit": state.get("joint_limit"),
                    "timing": state.get("timing"),
                    "session_id": self.id,
                    "collection": snapshot["collection"],
                    "recording": snapshot["recording"],
                    "collection_count": snapshot["collection_count"],
                    "task_active": snapshot["task_active"],
                    "task_paused": snapshot["task_paused"],
                    "task_run_id": snapshot["task_run_id"],
                    "last_episode": snapshot["last_episode"],
                    "task_dataset": snapshot.get("task_dataset"),
                    "runtime_error": snapshot["runtime_error"]}
        if command == "preview":
            return {"jpeg_base64": self.preview_jpeg,
                    "age_s": self.age(self.preview_frame, "host_received_monotonic_ns")}
        with self.operations:
            if self.closed:
                raise RuntimeError("Session is closing")
            if command.startswith("collection."):
                action = command.removeprefix("collection.")
                if action == "toggle_episode":
                    if self.recording is not None:
                        result = self._stop_recording("unknown")
                        if result["lifecycle"] != "complete":
                            raise RuntimeError("当前 Episode 未完整保存，请检查采集错误")
                        return result
                    action = "start_episode"

                if action == "start_task":
                    if not self.task_active:
                        if not self.task_paused:
                            self.task_run_id = uuid.uuid4().hex
                            self.counted_episodes = frozenset()
                            self.last_episode = None
                        self.task_active, self.task_paused = True, False
                    return {"task_active": True, "task_run_id": self.task_run_id}
                if action in ("stop_episode", "pause_task", "end_task"):
                    if self.recording is not None:
                        result = self._stop_recording("unknown")
                        if result["lifecycle"] != "complete":
                            raise RuntimeError("当前 Episode 未完整保存，请检查采集错误")
                    if action == "pause_task":
                        self.task_paused = self.task_active or self.task_paused
                        self.task_active = False
                    elif action == "end_task":
                        self._finalize_task_dataset()
                        self.task_active = self.task_paused = False
                        self.counted_episodes = frozenset()
                    return {"task_active": self.task_active, "task_paused": self.task_paused}
                if action == "discard_previous":
                    if self.task_dataset and self.task_dataset["state"] == "complete":
                        raise ValueError("任务已导出；请在结束任务前丢弃条目，避免训练目录与原始数据不一致")
                    previous = self.last_episode
                    if payload.get("episode_id") is not None and (previous is None or previous["id"] != payload["episode_id"]):
                        raise ValueError("上一条 Episode 已变化；未丢弃其他数据")
                    if previous is not None and previous["outcome"] != "discarded":
                        from ..datasets import label_episode
                        label_episode(previous["path"], "discarded", note="Discard previous episode from Quest stick")
                        self.last_episode = {**previous, "outcome": "discarded"}
                        self.counted_episodes = self.counted_episodes - {previous["id"]}
                    return {"last_episode": self.last_episode, "collection_count": len(self.counted_episodes)}
                if action in ("discard", "home"):
                    if self.recording is not None:
                        self._stop_recording("discarded")
                    return {"state": "READY", "home": action == "home"}
                if action != "start_episode":
                    raise ValueError("Unknown collection action")
                if self.recording is not None:
                    return self.recording.status()
                if not self.task_active:
                    raise ValueError("任务采集未开启；左推右摇杆开启")
                command = "record.start"
            if command == "record.start":
                if self.recording is not None:
                    raise ValueError("An episode is already recording/finalizing")
                status = self.status()
                if self.runtime_error or self.camera_error:
                    raise RuntimeError(self.runtime_error or self.camera_error)
                if status["state_age_s"] is None or status["state_age_s"] > .25:
                    raise ValueError("No fresh robot snapshot; recording blocked")
                if status["camera"]["age_s"] is None or status["camera"]["age_s"] > .25:
                    raise ValueError("No fresh wrist RGB frame; recording blocked")
                for key in self.camera_ids:
                    camera_state = self.camera_states[key]
                    age = self.age(camera_state['frame'], 'host_received_monotonic_ns')
                    if camera_state['error'] or age is None or age > .25:
                        raise ValueError(f"{key}: {camera_state['error'] or 'No fresh RGB frame'}; recording blocked")
                self.dataset_root.mkdir(parents=True, exist_ok=True)
                if shutil.disk_usage(self.dataset_root).free < self.config["recording"]["min_free_bytes"]:
                    raise ValueError("Insufficient disk space for recording")
                meta = {"robot": self.robot_metadata, "camera": self.camera_metadata,
                        "cameras": {key: self.camera_states[key]['metadata'] for key in self.camera_ids},
                        "quality_limits": {key: self.config['recording'][key] for key in (
                            'min_duration_s', 'max_state_gap_s', 'max_frame_gap_s', 'max_sync_skew_s')},
                        "control": self.control_metadata, "session_id": self.id, "task_run_id": self.task_run_id,
                        "source_sha256": self.source_fingerprint,
                        "hand_eye_calibrated": self.config["cameras"]["wrist_rgb"]["hand_eye_calibrated"],
                        "timing": "Robot host sample / RGB host receipt; not exposure-synchronized",
                        "action_definition": "submission is the last accepted Python SDK waypoint; sdk_cmd is interpolated output; neither is state[t+1]"}
                if getattr(self, "record_samples", None):
                    self._sample_cursor = self.record_samples(0)["latest"]
                raw_root = self.dataset_root
                if getattr(self, "record_samples", None):
                    if not self.task_run_id:
                        self.task_run_id = uuid.uuid4().hex
                    if self._dataset_run_id != self.task_run_id:
                        task = re.sub(r"[^\w\-]+", "_", (payload.get("task") or "task").strip())[:48]
                        self.task_dataset = {"path": str(self.dataset_root / f"{task}_{datetime.now():%Y%m%d-%H%M%S}-{self.task_run_id[:8]}"),
                                             "state": "collecting", "episodes": 0}
                        self._dataset_run_id = self.task_run_id
                    meta["task_run_id"] = self.task_run_id
                    raw_root = self.dataset_root / ".raw"
                from robot_core.camera_client import SharedCamera, socket_path
                from robot_core.camera_control import recording_guard
                from robot_core.session_protocol import request
                lease = recording_guard() if self.camera_factory is SharedCamera else None
                try:
                    if lease:
                        feeds = request(socket_path(), "monitor", timeout=.5)
                        for key in self.camera_ids:
                            camera_state = feeds.get('cameras', {'wrist_rgb': feeds})[key]['camera']
                            if camera_state.get("enabled") is False or camera_state.get("age_s") is None or camera_state["age_s"] > .5:
                                raise RuntimeError(key + "：相机未开启或反馈已过期")
                    self.recording = EpisodeWriter(raw_root, meta, payload.get("task"),
                                                   queue_size=self.config["recording"]["queue_size"],
                                                   rgb_video=self.config["recording"].get("rgb_video", False),
                                                   video_crf=self.config["recording"].get("video_crf", 20))
                    self._camera_record_lease = lease
                except BaseException:
                    if lease:
                        lease.close()
                    raise
                return self.recording.status()
            if command == "record.stop":
                return self._stop_recording(payload.get("outcome", "unknown"))
            raise ValueError("Supported commands: status, preview, record.start, record.stop")

    def _stop_recording(self, outcome="unknown", reason=None):
        if self.recording is None:
            raise ValueError("No active episode")
        if getattr(self, "record_samples", None) and reason is None:
            self._drain_samples(self.recording)
        result = self.recording.stop(outcome, reason=reason)
        self.last_episode = result
        if result["lifecycle"] == "complete" and result["outcome"] != "discarded":
            self.counted_episodes = self.counted_episodes | {result["id"]}
        self.recording = None
        if self._camera_record_lease:
            self._camera_record_lease.close()
            self._camera_record_lease = None
        if self.task_dataset is not None:
            self.task_dataset["episodes"] = len(self.counted_episodes)
        result['quality'] = {'status': 'checking', 'episode': result['id']}
        save_json(Path(result['path']) / 'quality.json', result['quality'])
        self.quality_jobs.append(self.quality_pool.submit(self._check_quality, dict(result)))
        return result

    def _check_quality(self, episode):
        try:
            from ..datasets import validate_episode
            from robot_core.model import RobotModel
            from robot_core.project import resolve_path
            robot = RobotModel(resolve_path(self.config['robot']['urdf']), eef=self.robot_metadata['eef_frame'])
            report = validate_episode(episode['path'], settings=self.config['recording'], robot=robot)
            report.update(status='passed' if report['valid'] else 'failed', episode=episode['id'])
        except Exception as exc:
            report = {'status': 'failed', 'valid': False, 'episode': episode['id'],
                      'errors': ['质量检查未完成：' + str(exc)], 'warnings': []}
        save_json(Path(episode['path']) / 'quality.json', report)
        self.quality_results[episode['id']] = report
        return report

    def _finalize_task_dataset(self):
        if not self.config["recording"].get("package_on_task_end", False) or not self.task_dataset:
            return
        if self.task_dataset["state"] == "complete":
            return
        for job in self.quality_jobs:
            job.result()  # Include completed reports in the portable archive.
        self.quality_jobs.clear()
        episode_ids = []
        for manifest in (self.dataset_root / ".raw").glob("*/episode.json"):
            meta = json.loads(manifest.read_text())
            if meta.get("task_run_id") == self.task_run_id:
                episode_ids.append(manifest.parent.name)
        if not episode_ids:
            self.task_dataset["state"] = "empty"
            return
        from ..task_package import finalize_task
        self.task_dataset.update(state="saving", episodes=len(episode_ids), retained_episodes=len(self.counted_episodes))
        try:
            package = finalize_task(self.dataset_root / ".raw", sorted(episode_ids),
                          self.task_dataset["path"], self.control_metadata["recording_rate_hz"])
        except Exception as exc:
            self.task_dataset.update(state="failed", error=str(exc))
            raise
        self.task_dataset.pop("error", None)
        self.task_dataset.update(state="complete", path=str(package))
        print("任务压缩包已保存，可上传：" + self.task_dataset["path"], flush=True)

    def _background(self):
        try:
            with (self.log_root / "states.jsonl").open("w", buffering=1) as stream:
                while not self.stop_event.wait(.2):
                    if self.latest_state is not None:
                        stream.write(json.dumps(self.latest_state, allow_nan=False) + "\n")
                    with self.operations:
                        writer = self.recording
                        if writer is None:
                            continue
                        state_age = self.age(self.latest_state, "host_monotonic_ns")
                        frame_age = self.age(self.latest_frame, "host_received_monotonic_ns")
                        reason = writer.error or self.camera_error
                        for key in self.camera_ids:
                            current = self.camera_states[key]
                            age = self.age(current['frame'], 'host_received_monotonic_ns')
                            if current['error'] or age is None or age > .5:
                                reason = reason or key + ': RGB stream stale or disconnected'
                        if state_age is None or state_age > .5 or frame_age is None or frame_age > .5:
                            reason = reason or "Robot/RGB stream stale"
                        if writer.status()["duration_s"] >= self.config["recording"]["max_duration_s"]:
                            reason = reason or "Episode duration limit reached"
                        if shutil.disk_usage(self.dataset_root).free < self.config["recording"]["min_free_bytes"]:
                            reason = reason or "Low disk space"
                        if reason:
                            self._stop_recording(reason=reason)
        except Exception as exc:
            self.runtime_error = f"Background recording/logging failed: {exc}"
            with self.operations:
                if self.recording is not None:
                    try:
                        self._stop_recording(reason=self.runtime_error)
                    except Exception:
                        pass  # Manifest stays incomplete if a writer cannot finish.

    def close(self):
        self.stop_event.set()
        with self.operations:
            self.closed = True
            if self.recording is not None:
                try:
                    self._stop_recording(reason="Teleoperation session ended")
                except Exception as exc:
                    self.runtime_error = str(exc)
        if self.server is not None:
            self.server.close()
            self.server = None
        for thread in self.threads:
            thread.join(timeout=3)
        self.quality_pool.shutdown(wait=True)
        save_json(self.log_root / "closed.json", {"closed_unix_ns": time.time_ns(),
                  "last_episode": self.last_episode, "error": self.runtime_error})
