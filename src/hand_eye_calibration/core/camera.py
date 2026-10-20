"""Single-source RealSense RGB capture with exact active-stream intrinsics."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time
import threading
from typing import Any

import numpy as np

from hand_eye_calibration.core.camera_profiles import format_profiles, list_bgr_color_profiles
from hand_eye_calibration.core.schema import CameraIntrinsics


@dataclass(frozen=True)
class CapturedColorFrame:
    """Immutable metadata around one copied RGB frame from the sole pipeline."""

    image: np.ndarray
    sequence: int
    host_monotonic_ns: int
    device_timestamp_ms: float
    frame_number: int


class RealSenseCamera:
    """One RGB pipeline with fan-out through a latest-frame condition.

    The pipeline is read by exactly one capture thread.  Calibration, browser
    preview, and any diagnostics consume copied frames from that source rather
    than each opening a second RealSense pipeline or competing on
    ``wait_for_frames``.
    """

    def __init__(
        self,
        *,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        serial: str | None = None,
        warmup_frames: int = 30,
        name: str = "RealSense",
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.serial = serial
        self.warmup_frames = int(warmup_frames)
        self.name = str(name)
        self.pipeline: Any | None = None
        self.intrinsics: CameraIntrinsics | None = None
        self._capture_stop = threading.Event()
        self._capture_thread: threading.Thread | None = None
        self._frame_condition = threading.Condition()
        self._latest_frame: CapturedColorFrame | None = None
        self._frame_sequence = 0
        self._last_read_sequence = 0
        self._stream_error: BaseException | None = None
        self._capture_times_ns: deque[int] = deque(maxlen=120)

    def _validate_requested_profile(self) -> None:
        """Fail before calibration motion when config asks for a non-native mode."""
        try:
            profiles = list_bgr_color_profiles(self.serial)
        except RuntimeError:
            # ``pipeline.start`` below remains the authority if the SDK cannot
            # enumerate profiles, for example during transient USB recovery.
            return
        requested = (self.width, self.height, self.fps)
        if any(
            (profile.width, profile.height, profile.fps) == requested
            for profile in profiles
        ):
            return
        raise RuntimeError(
            f"{self.name} 不支持 RGB 档位 {self.width}x{self.height}@{self.fps}；"
            f"可用档位：{format_profiles(profiles)}"
        )

    def start(self) -> CameraIntrinsics:
        if self.pipeline is not None:
            if self.intrinsics is None:
                raise RuntimeError(f"{self.name} 已启动但没有相机内参")
            return self.intrinsics
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is required; use the .venv-handeye environment"
            ) from exc
        self._validate_requested_profile()
        pipeline = rs.pipeline()
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )
        try:
            profile = pipeline.start(config)
            stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            values = stream.get_intrinsics()
            device = profile.get_device()
            serial = device.get_info(rs.camera_info.serial_number)
            model = getattr(values.model, "name", str(values.model))
            intrinsics = CameraIntrinsics(
                width=int(values.width),
                height=int(values.height),
                fps=int(stream.fps()),
                camera_matrix=np.asarray(
                    [
                        [values.fx, 0.0, values.ppx],
                        [0.0, values.fy, values.ppy],
                        [0.0, 0.0, 1.0],
                    ],
                    dtype=np.float64,
                ),
                distortion_coefficients=np.asarray(values.coeffs[:5]),
                distortion_model=str(model),
                serial=serial,
            )
            for _ in range(max(0, self.warmup_frames)):
                pipeline.wait_for_frames(5000)
        except RuntimeError as exc:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            raise RuntimeError(
                f"无法独占启动 {self.name}；请先关闭相机视频流，"
                "等待相机服务退出后重试"
            ) from exc
        except BaseException:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            raise

        with self._frame_condition:
            self.pipeline = pipeline
            self.intrinsics = intrinsics
            self._latest_frame = None
            self._frame_sequence = 0
            self._last_read_sequence = 0
            self._stream_error = None
            self._capture_times_ns.clear()
        self._capture_stop.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_worker,
            args=(pipeline,),
            name=f"{self.name}-rgb-source",
            daemon=True,
        )
        self._capture_thread.start()
        print(
            f"HANDEYE_CAMERA_PROFILE {self.name} "
            f"{intrinsics.width}x{intrinsics.height}@{intrinsics.fps} "
            "single-source-rgb",
            flush=True,
        )
        return intrinsics

    def _capture_worker(self, pipeline: Any) -> None:
        failure: BaseException | None = None
        try:
            while not self._capture_stop.is_set():
                frames = pipeline.wait_for_frames(1000)
                color = frames.get_color_frame()
                if not color:
                    continue
                captured_at = time.monotonic_ns()
                image = np.asanyarray(color.get_data()).copy()
                with self._frame_condition:
                    self._frame_sequence += 1
                    self._latest_frame = CapturedColorFrame(
                        image=image,
                        sequence=self._frame_sequence,
                        host_monotonic_ns=captured_at,
                        device_timestamp_ms=float(color.get_timestamp()),
                        frame_number=int(color.get_frame_number()),
                    )
                    self._capture_times_ns.append(captured_at)
                    self._frame_condition.notify_all()
        except BaseException as exc:
            if not self._capture_stop.is_set():
                failure = exc
        finally:
            with self._frame_condition:
                if failure is not None:
                    self._stream_error = failure
                self._frame_condition.notify_all()

    def latest_sequence(self) -> int:
        with self._frame_condition:
            return self._frame_sequence

    def wait_for_frame(
        self,
        after_sequence: int = 0,
        *,
        timeout_s: float = 5.0,
    ) -> CapturedColorFrame:
        """Return a copied frame newer than ``after_sequence``.

        A slow detector receives the most recent frame instead of forcing the
        camera source to queue stale frames.  This makes preview load unable to
        delay high-resolution sampling.
        """
        if timeout_s <= 0.0:
            raise ValueError("frame timeout must be positive")
        deadline = time.monotonic() + timeout_s
        with self._frame_condition:
            while self._frame_sequence <= after_sequence:
                if self._stream_error is not None:
                    raise RuntimeError(f"{self.name} RGB 流读取失败") from self._stream_error
                if self.pipeline is None or self._capture_stop.is_set():
                    raise RuntimeError(f"{self.name} is not started")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(f"等待 {self.name} RGB 帧超时")
                self._frame_condition.wait(remaining)
            frame = self._latest_frame
            if frame is None:
                raise RuntimeError(f"{self.name} 没有可用 RGB 帧")
            return CapturedColorFrame(
                image=frame.image.copy(),
                sequence=frame.sequence,
                host_monotonic_ns=frame.host_monotonic_ns,
                device_timestamp_ms=frame.device_timestamp_ms,
                frame_number=frame.frame_number,
            )

    def read(self) -> np.ndarray:
        """Compatibility API for one direct consumer of the shared source."""
        frame = self.wait_for_frame(self._last_read_sequence)
        self._last_read_sequence = frame.sequence
        return frame.image

    def stream_metrics(self) -> dict[str, float | int | None]:
        """Small live health snapshot for calibration logs and diagnostics."""
        with self._frame_condition:
            timestamps = tuple(self._capture_times_ns)
            sequence = self._frame_sequence
            error = self._stream_error
        capture_fps: float | None = None
        if len(timestamps) >= 2:
            span_s = (timestamps[-1] - timestamps[0]) / 1_000_000_000.0
            if span_s > 0.0:
                capture_fps = (len(timestamps) - 1) / span_s
        return {
            "sequence": sequence,
            "capture_fps": capture_fps,
            "stream_error": None if error is None else str(error),
        }

    def stop(self) -> None:
        pipeline = self.pipeline
        if pipeline is None:
            return
        self._capture_stop.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        thread = self._capture_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        try:
            pipeline.stop()
        except RuntimeError:
            pass
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._frame_condition:
            self.pipeline = None
            self._capture_thread = None
            self._latest_frame = None
            self._frame_condition.notify_all()

    def __enter__(self) -> "RealSenseCamera":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()


# Preserve the original public name for external scripts.
D455Camera = RealSenseCamera
