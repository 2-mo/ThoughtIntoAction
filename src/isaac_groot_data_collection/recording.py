"""Hardware-facing raw state and camera recorders."""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time
from typing import Any, Protocol

import numpy as np

from .raw_episode import atomic_json, utc_now


class SnapshotController(Protocol):
    def get_state(self): ...


def _json_line(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"


@lru_cache(maxsize=4)
def _select_h264_encoder(ffmpeg: str) -> str:
    probe = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s:v",
        "256x256",
        "-r",
        "1",
        "-i",
        "pipe:0",
        "-frames:v",
        "1",
        "-c:v",
        "h264_nvenc",
        "-pix_fmt",
        "yuv444p",
        "-f",
        "null",
        "-",
    ]
    try:
        result = subprocess.run(
            probe,
            input=bytes(256 * 256 * 3),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "libx264"
    return "h264_nvenc" if result.returncode == 0 else "libx264"


def warm_camera_video_encoder() -> str:
    ffmpeg = shutil.which("ffmpeg")
    return "unavailable" if ffmpeg is None else _select_h264_encoder(ffmpeg)


class SharedFrameTimeline:
    """Select at most one source frame per shared fixed-rate time slot."""

    def __init__(self, fps: int, *, master_video_key: str) -> None:
        if fps <= 0:
            raise ValueError("shared frame timeline FPS must be positive")
        self.fps = int(fps)
        self.master_video_key = str(master_video_key)
        self._lock = threading.Lock()
        self._origin_ns: int | None = None
        self._last_slots: dict[str, int] = {}

    def select(
        self,
        video_key: str,
        host_monotonic_ns: int,
    ) -> tuple[int, int] | None:
        video_key = str(video_key)
        timestamp_ns = int(host_monotonic_ns)
        with self._lock:
            if self._origin_ns is None:
                if video_key != self.master_video_key:
                    return None
                self._origin_ns = timestamp_ns
            delta_ns = timestamp_ns - self._origin_ns
            slot = (delta_ns * self.fps + 500_000_000) // 1_000_000_000
            if slot < 0 or slot <= self._last_slots.get(video_key, -1):
                return None
            self._last_slots[video_key] = slot
            target_ns = self._origin_ns + (
                slot * 1_000_000_000 + self.fps // 2
            ) // self.fps
            return slot, target_ns

    def metadata(self) -> dict[str, Any]:
        return {
            "mode": "native_rate_shared_guard",
            "fps": self.fps,
            "master_video_key": self.master_video_key,
        }


class ArmStateRecorder:
    def __init__(
        self,
        episode_path: Path | str,
        controller: SnapshotController,
        *,
        rate_hz: float = 100.0,
    ) -> None:
        self.episode_path = Path(episode_path)
        self.controller = controller
        self.rate_hz = float(rate_hz)
        if not np.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("state recorder rate must be positive")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._count = 0
        self._first_ns: int | None = None
        self._last_ns: int | None = None
        self._maximum_gap_ns = 0
        self._latest_joint_position_rad: tuple[float, ...] | None = None
        self._error: str | None = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("state recorder already started")
        state_dir = self.episode_path / "state"
        state_dir.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(
            target=self._run,
            name="groot-arm-state-recorder",
            daemon=True,
        )
        self._thread.start()

    @staticmethod
    def snapshot_payload(snapshot: Any, host_ns: int) -> dict[str, Any]:
        return {
            "host_monotonic_ns": host_ns,
            "controller_timestamp": float(snapshot.timestamp),
            "joint_position": np.asarray(
                snapshot.joint_position, dtype=np.float64
            ).tolist(),
            "joint_velocity": np.asarray(
                snapshot.joint_velocity, dtype=np.float64
            ).tolist(),
            "joint_torque": np.asarray(
                snapshot.joint_torque, dtype=np.float64
            ).tolist(),
            "gripper_position": float(snapshot.gripper_position),
            "gripper_velocity": float(snapshot.gripper_velocity),
            "gripper_torque": float(snapshot.gripper_torque),
            "eef_pose_6d": np.asarray(
                snapshot.eef_pose_6d, dtype=np.float64
            ).tolist(),
        }

    def _run(self) -> None:
        period_ns = int(round(1_000_000_000 / self.rate_hz))
        deadline = time.monotonic_ns()
        path = self.episode_path / "state" / "states.jsonl"
        try:
            with path.open("w", encoding="utf-8", buffering=1) as stream:
                while not self._stop.is_set():
                    before_ns = time.monotonic_ns()
                    snapshot = self.controller.get_state()
                    after_ns = time.monotonic_ns()
                    host_ns = (before_ns + after_ns) // 2
                    payload = self.snapshot_payload(snapshot, host_ns)
                    stream.write(_json_line(payload))
                    with self._lock:
                        self._count += 1
                        self._first_ns = host_ns if self._first_ns is None else self._first_ns
                        self._last_ns = host_ns
                        self._latest_joint_position_rad = tuple(
                            float(value) for value in payload["joint_position"]
                        )
                    deadline += period_ns
                    wait_ns = deadline - time.monotonic_ns()
                    if wait_ns > 0:
                        self._stop.wait(wait_ns / 1_000_000_000)
                    elif wait_ns < -5 * period_ns:
                        deadline = time.monotonic_ns()
                stream.flush()
        except BaseException as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            self._stop.set()

    def stop(self, timeout_s: float = 5.0) -> dict[str, Any]:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout_s)
            if thread.is_alive():
                raise RuntimeError("state recorder did not stop")
        summary = self.summary()
        atomic_json(self.episode_path / "state" / "summary.json", summary)
        return summary

    def summary(self) -> dict[str, Any]:
        with self._lock:
            first_ns = self._first_ns
            last_ns = self._last_ns
            count = self._count
            latest_joint_position_rad = self._latest_joint_position_rad
            error = self._error
        duration_s = (
            0.0
            if first_ns is None or last_ns is None
            else max(0.0, (last_ns - first_ns) / 1_000_000_000)
        )
        return {
            "sample_count": count,
            "first_host_monotonic_ns": first_ns,
            "last_host_monotonic_ns": last_ns,
            "duration_s": duration_s,
            "requested_rate_hz": self.rate_hz,
            "latest_joint_position_rad": (
                None
                if latest_joint_position_rad is None
                else list(latest_joint_position_rad)
            ),
            "error": error,
        }


class CameraEpisodeRecorder:
    def __init__(
        self,
        episode_path: Path | str,
        *,
        video_key: str,
        width: int,
        height: int,
        fps: int,
        video_crf: int,
        camera_metadata: dict[str, Any],
        shared_timeline: SharedFrameTimeline | None = None,
        queue_size: int = 90,
    ) -> None:
        self.episode_path = Path(episode_path)
        self.video_key = str(video_key)
        self.camera_path = self.episode_path / "cameras" / self.video_key
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.video_crf = int(video_crf)
        self.camera_metadata = dict(camera_metadata)
        self.shared_timeline = shared_timeline
        self._ffmpeg = shutil.which("ffmpeg")
        self.video_encoder = warm_camera_video_encoder()
        self._queue: queue.Queue[tuple[np.ndarray, dict[str, Any]] | None] = (
            queue.Queue(maxsize=queue_size)
        )
        self._thread: threading.Thread | None = None
        self._process: subprocess.Popen[bytes] | None = None
        self._lock = threading.Lock()
        self._count = 0
        self._dropped = 0
        self._source_callbacks = 0
        self._duplicate_source_frames = 0
        self._rate_limited_source_frames = 0
        self._source_frame_number_gaps = 0
        self._source_nonmonotonic_frames = 0
        self._last_source_identity: tuple[int | None, float | None] | None = None
        self._last_source_frame_number: int | None = None
        self._first_source_ns: int | None = None
        self._last_source_ns: int | None = None
        self._first_ns: int | None = None
        self._last_ns: int | None = None
        self._maximum_gap_ns = 0
        self._error: str | None = None
        self._stopped = False

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("camera recorder already started")
        self.camera_path.mkdir(parents=True, exist_ok=True)
        atomic_json(
            self.camera_path / "camera.json",
            {
                "video_key": self.video_key,
                **self.camera_metadata,
                **(
                    {"capture_timeline": self.shared_timeline.metadata()}
                    if self.shared_timeline is not None
                    else {}
                ),
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "storage": {
                    "container": "matroska",
                    "codec": "h264",
                    "encoder": self.video_encoder,
                    "pixel_format": "yuv444p",
                    "crf": self.video_crf,
                    "video_file": "video.mkv",
                    "frame_metadata_file": "frames.jsonl",
                },
                "stage": "recording",
                "started_at": utc_now(),
            },
        )
        self._thread = threading.Thread(
            target=self._run,
            name=f"groot-{self.video_key}-writer",
            daemon=True,
        )
        self._thread.start()

    def capture(
        self,
        image: np.ndarray,
        *,
        host_monotonic_ns: int,
        device_timestamp_ms: float | None,
        frame_number: int | None,
    ) -> None:
        if self._stopped:
            return
        frame = np.asarray(image)
        if frame.shape != (self.height, self.width, 3) or frame.dtype != np.uint8:
            with self._lock:
                self._error = f"unexpected RGB frame shape or dtype: {frame.shape} {frame.dtype}"
            return
        payload = {
            "host_monotonic_ns": int(host_monotonic_ns),
            "device_timestamp_ms": (
                None if device_timestamp_ms is None else float(device_timestamp_ms)
            ),
            "frame_number": None if frame_number is None else int(frame_number),
        }
        identity = (payload["frame_number"], payload["device_timestamp_ms"])
        with self._lock:
            self._source_callbacks += 1
            source_host_ns = payload["host_monotonic_ns"]
            self._first_source_ns = (
                source_host_ns
                if self._first_source_ns is None
                else self._first_source_ns
            )
            self._last_source_ns = source_host_ns
            if identity != (None, None) and identity == self._last_source_identity:
                self._duplicate_source_frames += 1
                return
            current_frame_number = payload["frame_number"]
            previous_frame_number = self._last_source_frame_number
            if current_frame_number is not None and previous_frame_number is not None:
                if current_frame_number > previous_frame_number + 1:
                    self._source_frame_number_gaps += (
                        current_frame_number - previous_frame_number - 1
                    )
                elif current_frame_number <= previous_frame_number:
                    self._source_nonmonotonic_frames += 1
            self._last_source_identity = identity
            self._last_source_frame_number = current_frame_number
        if self.shared_timeline is not None:
            selected = self.shared_timeline.select(
                self.video_key,
                payload["host_monotonic_ns"],
            )
            if selected is None:
                with self._lock:
                    self._rate_limited_source_frames += 1
                return
            timeline_slot, timeline_target_ns = selected
            payload["timeline_slot"] = timeline_slot
            payload["timeline_target_monotonic_ns"] = timeline_target_ns
        try:
            self._queue.put_nowait((frame.copy(), payload))
        except queue.Full:
            with self._lock:
                self._dropped += 1

    def _run(self) -> None:
        process: subprocess.Popen[bytes] | None = None
        temporary_video = self.camera_path / ".video.recording.mkv"
        try:
            ffmpeg = self._ffmpeg
            if ffmpeg is None:
                raise RuntimeError("ffmpeg is required to record camera video")
            encoder_options = (
                [
                    "-c:v",
                    "h264_nvenc",
                    "-preset",
                    "p4",
                    "-tune",
                    "hq",
                    "-rc",
                    "constqp",
                    "-qp",
                    str(self.video_crf),
                ]
                if self.video_encoder == "h264_nvenc"
                else [
                    "-c:v",
                    "libx264",
                    "-preset",
                    "veryfast",
                    "-crf",
                    str(self.video_crf),
                ]
            )
            process = subprocess.Popen(
                [
                    ffmpeg,
                    "-hide_banner",
                    "-loglevel",
                    "error",
                    "-y",
                    "-f",
                    "rawvideo",
                    "-pix_fmt",
                    "rgb24",
                    "-s:v",
                    f"{self.width}x{self.height}",
                    "-r",
                    str(self.fps),
                    "-i",
                    "-",
                    "-an",
                    *encoder_options,
                    "-pix_fmt",
                    "yuv444p",
                    str(temporary_video),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            with self._lock:
                self._process = process
            assert process.stdin is not None
            metadata_path = self.camera_path / "frames.jsonl"
            with metadata_path.open("w", encoding="utf-8", buffering=1) as stream:
                while True:
                    item = self._queue.get()
                    if item is None:
                        break
                    image, payload = item
                    with self._lock:
                        index = self._count
                    process.stdin.write(image.tobytes())
                    payload["frame_index"] = index
                    stream.write(_json_line(payload))
                    host_ns = int(payload["host_monotonic_ns"])
                    with self._lock:
                        if self._last_ns is not None:
                            self._maximum_gap_ns = max(
                                self._maximum_gap_ns,
                                host_ns - self._last_ns,
                            )
                        self._count += 1
                        self._first_ns = host_ns if self._first_ns is None else self._first_ns
                        self._last_ns = host_ns
                stream.flush()
            process.stdin.close()
            return_code = process.wait()
            if return_code:
                assert process.stderr is not None
                error = process.stderr.read().decode("utf-8", errors="replace")
                raise RuntimeError(
                    f"ffmpeg camera recording failed ({return_code}): {error.strip()}"
                )
            temporary_video.replace(self.camera_path / "video.mkv")
        except BaseException as exc:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None

    def stop(self, *, aborted: bool = False, timeout_s: float = 15.0) -> dict[str, Any]:
        if not self._stopped:
            self._stopped = True
            deadline = time.monotonic() + min(2.0, timeout_s)
            while self._thread is not None and self._thread.is_alive():
                try:
                    self._queue.put(None, timeout=0.1)
                    break
                except queue.Full:
                    if time.monotonic() >= deadline:
                        discarded = 0
                        while True:
                            try:
                                queued = self._queue.get_nowait()
                            except queue.Empty:
                                break
                            if queued is not None:
                                discarded += 1
                        self._queue.put_nowait(None)
                        with self._lock:
                            self._dropped += discarded
                            if self._error is None:
                                self._error = (
                                    "camera writer queue did not drain; "
                                    f"discarded {discarded} queued frames"
                                )
                        break
        thread = self._thread
        if thread is not None:
            thread.join(timeout_s)
            if thread.is_alive():
                with self._lock:
                    process = self._process
                    if self._error is None:
                        self._error = "camera encoder did not stop before timeout"
                if process is not None and process.poll() is None:
                    process.kill()
                thread.join(min(2.0, timeout_s))
                if thread.is_alive():
                    raise RuntimeError("camera recorder did not stop")
        summary = self.summary()
        summary.update(
            {
                "video_key": self.video_key,
                **self.camera_metadata,
                **(
                    {"capture_timeline": self.shared_timeline.metadata()}
                    if self.shared_timeline is not None
                    else {}
                ),
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "storage": {
                    "container": "matroska",
                    "codec": "h264",
                    "encoder": self.video_encoder,
                    "pixel_format": "yuv444p",
                    "crf": self.video_crf,
                    "video_file": "video.mkv",
                    "frame_metadata_file": "frames.jsonl",
                },
                "stage": "aborted" if aborted else "complete",
                "stopped_at": utc_now(),
            }
        )
        atomic_json(self.camera_path / "camera.json", summary)
        return summary

    def summary(self) -> dict[str, Any]:
        with self._lock:
            first_ns = self._first_ns
            last_ns = self._last_ns
            count = self._count
            dropped = self._dropped
            source_callbacks = self._source_callbacks
            duplicate_source_frames = self._duplicate_source_frames
            rate_limited_source_frames = self._rate_limited_source_frames
            source_frame_number_gaps = self._source_frame_number_gaps
            source_nonmonotonic_frames = self._source_nonmonotonic_frames
            first_source_ns = self._first_source_ns
            last_source_ns = self._last_source_ns
            maximum_gap_ns = self._maximum_gap_ns
            error = self._error
        total_seen = count + dropped
        duration_s = (
            0.0
            if first_ns is None or last_ns is None
            else max(0.0, (last_ns - first_ns) / 1_000_000_000)
        )
        source_duration_s = (
            0.0
            if first_source_ns is None or last_source_ns is None
            else max(0.0, (last_source_ns - first_source_ns) / 1_000_000_000)
        )
        usable_source_frames = max(0, source_callbacks - duplicate_source_frames)
        return {
            "frame_count": count,
            "dropped_queue_frames": dropped,
            "drop_fraction": 0.0 if total_seen == 0 else dropped / total_seen,
            "source_callbacks": source_callbacks,
            "duplicate_source_frames": duplicate_source_frames,
            "duplicate_source_fraction": (
                0.0
                if source_callbacks == 0
                else duplicate_source_frames / source_callbacks
            ),
            "rate_limited_source_frames": rate_limited_source_frames,
            "source_frame_number_gaps": source_frame_number_gaps,
            "source_nonmonotonic_frames": source_nonmonotonic_frames,
            "source_duration_s": source_duration_s,
            "source_effective_fps": (
                0.0
                if source_duration_s <= 0.0
                else max(0, usable_source_frames - 1) / source_duration_s
            ),
            "recording_effective_fps": (
                0.0
                if duration_s <= 0.0
                else max(0, count - 1) / duration_s
            ),
            "queue_depth": self._queue.qsize(),
            "queue_capacity": self._queue.maxsize,
            "first_host_monotonic_ns": first_ns,
            "last_host_monotonic_ns": last_ns,
            "maximum_frame_gap_ms": maximum_gap_ns / 1_000_000.0,
            "duration_s": duration_s,
            "error": error,
        }
