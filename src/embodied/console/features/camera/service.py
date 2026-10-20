#!/usr/bin/env python3
"""Stream RealSense RGB with optional aligned depth over WebRTC."""

from __future__ import annotations

import argparse
import asyncio
import base64
from dataclasses import dataclass
from fractions import Fraction
import gc
import hmac
import json
import os
from pathlib import Path
import ssl
import threading
import time
from typing import Any, Callable
from urllib.parse import parse_qs, urlsplit

from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiortc.mediastreams import MediaStreamError
from av import VideoFrame
import cv2
import numpy as np
import pyrealsense2 as rs
from websockets.asyncio.server import ServerConnection, serve

from embodied.core.project import port


SIGNAL_PATH = "/api/v1/cameras/d455/webrtc"
COLLECTION_SIGNAL_PATH = "/api/v1/cameras/collection"
COLLECTION_SIGNAL_HOST = "127.0.0.1"
COLLECTION_SIGNAL_PORT = port("collection_camera")
COLLECTION_CAMERA_MESSAGE_TYPES = frozenset(
    {
        "collection_camera_start",
        "collection_camera_stop",
        "collection_camera_abort",
        "collection_camera_preview",
        "collection_camera_status",
    }
)
from websockets.exceptions import ConnectionClosed

from legacy.arx_tools.depth_visualization import DepthVisualizer
from legacy.arx_tools.rgb_cube_detector import RgbCubeDetector
from embodied.data.groot.config import load_collection_config
from embodied.data.groot.raw_episode import RawEpisodeStore
from embodied.data.groot.recording import (
    CameraEpisodeRecorder,
    SharedFrameTimeline,
    warm_camera_video_encoder,
)
from embodied.data.groot.validator import (
    MINIMUM_USABLE_CAMERA_RATE_FRACTION,
)
from legacy.arx_tools.camera_recovery import requires_clean_process_restart
from legacy.arx_tools.runtime_identity import SERVICE_STARTED_AT, SOURCE_REVISION


VIDEO_CLOCK_RATE = 90_000
VIDEO_TIME_BASE = Fraction(1, VIDEO_CLOCK_RATE)
CAMERA_START_TIMEOUT_SECONDS = 10
CAMERA_RESTART_DELAY_SECONDS = 0.4
CAMERA_IDLE_RELEASE_SECONDS = 12.0
CAMERA_CALIBRATION_RESTART_DELAY_SECONDS = 0.25
CAMERA_RESTART_EXIT_CODE = 75
DEPTH_RETRY_INITIAL_SECONDS = 2.0
DEPTH_RETRY_MAX_SECONDS = 30.0
PIPELINE_RELEASE_SETTLE_SECONDS = 0.25
COLOR_AUTO_WARMUP_FRAMES = 30
DEPTH_PRESET_VALUES = {
    "default": 1.0,
    "high-accuracy": 3.0,
    "high-density": 4.0,
}
DEFAULT_D455_SERIAL = "239222300444"
DEFAULT_EYE_IN_HAND_SERIAL = "230322274781"


@dataclass(frozen=True)
class CapturedFrame:
    image: np.ndarray
    pts: int


def _exposure_option_unit_us(option_range: Any) -> float:
    """Infer whether a RealSense exposure option uses us or UVC 100-us ticks."""
    return 100.0 if float(option_range.max) <= 10_000.0 else 1.0


def _encode_rgb_preview(image: np.ndarray) -> bytes:
    resized = cv2.resize(image, (320, 180), interpolation=cv2.INTER_AREA)
    bgr = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)
    encoded, payload = cv2.imencode(
        ".jpg",
        bgr,
        [cv2.IMWRITE_JPEG_QUALITY, 72],
    )
    if not encoded:
        raise RuntimeError("camera preview JPEG encoding failed")
    return payload.tobytes()


def _verified_color_profile(
    profile: Any,
    *,
    width: int,
    height: int,
    fps: int,
) -> tuple[Any, dict[str, Any]]:
    color_profile = profile.get_stream(
        rs.stream.color
    ).as_video_stream_profile()
    resolved = {
        "width": int(color_profile.width()),
        "height": int(color_profile.height()),
        "fps": int(color_profile.fps()),
        "format": str(color_profile.format()),
    }
    expected = {
        "width": int(width),
        "height": int(height),
        "fps": int(fps),
        "format": str(rs.format.rgb8),
    }
    if resolved != expected:
        raise RuntimeError(
            f"resolved color profile {resolved} does not match requested {expected}"
        )
    return color_profile, resolved


def _find_color_sensor(profile: Any, warnings: list[str]) -> Any | None:
    try:
        for sensor in profile.get_device().query_sensors():
            try:
                is_color = any(
                    stream_profile.stream_type() == rs.stream.color
                    for stream_profile in sensor.get_stream_profiles()
                )
            except Exception:
                continue
            if is_color:
                return sensor
    except Exception as exc:
        warnings.append(f"cannot enumerate color sensor: {exc}")
    warnings.append("color sensor was not found")
    return None


def enable_auto_color_controls(profile: Any) -> dict[str, Any]:
    """Let the camera settle in the current scene before recording."""
    requested = {
        "auto_exposure": True,
        "auto_white_balance": True,
        "auto_exposure_priority": False,
    }
    applied: dict[str, Any] = {}
    warnings: list[str] = []
    color_sensor = _find_color_sensor(profile, warnings)
    if color_sensor is None:
        return {
            "mode": "auto_warmup_then_lock",
            "warmup_frames": 0,
            "locked": False,
            "requested": requested,
            "applied": applied,
            "warnings": warnings,
        }

    options = (
        (
            "auto_exposure_priority",
            getattr(rs.option, "auto_exposure_priority", None),
            0.0,
        ),
        (
            "auto_exposure",
            getattr(rs.option, "enable_auto_exposure", None),
            1.0,
        ),
        (
            "auto_white_balance",
            getattr(rs.option, "enable_auto_white_balance", None),
            1.0,
        ),
    )
    for name, option, value in options:
        if option is None:
            continue
        try:
            if not color_sensor.supports(option):
                continue
            color_sensor.set_option(option, value)
            applied[name] = bool(round(float(color_sensor.get_option(option))))
        except Exception as exc:
            warnings.append(f"failed to enable {name}: {exc}")
    return {
        "mode": "auto_warmup_then_lock",
        "warmup_frames": 0,
        "locked": False,
        "requested": requested,
        "applied": applied,
        "warnings": warnings,
    }


def apply_fixed_color_controls(
    profile: Any,
    *,
    exposure_us: int,
    gain: int,
    white_balance_k: int,
) -> dict[str, Any]:
    """Best-effort fixed RGB controls with enough metadata for later review."""
    requested = {
        "auto_exposure": False,
        "exposure_us": int(exposure_us),
        "gain": int(gain),
        "auto_white_balance": False,
        "white_balance_k": int(white_balance_k),
    }
    applied: dict[str, Any] = {}
    warnings: list[str] = []
    color_sensor = _find_color_sensor(profile, warnings)
    if color_sensor is None:
        warnings.append("automatic controls may remain enabled")
        return {"requested": requested, "applied": applied, "warnings": warnings}

    def set_option(name: str, option: Any, value: float) -> None:
        if option is None:
            warnings.append(f"RealSense SDK does not expose {name}")
            return
        try:
            if not color_sensor.supports(option):
                warnings.append(f"color sensor does not support {name}")
                return
            color_sensor.set_option(option, float(value))
            actual = float(color_sensor.get_option(option))
            applied[name] = bool(round(actual)) if name.startswith("auto_") else actual
        except Exception as exc:
            warnings.append(f"failed to set {name}: {exc}")

    set_option(
        "auto_exposure",
        getattr(rs.option, "enable_auto_exposure", None),
        0.0,
    )
    exposure_option = getattr(rs.option, "exposure", None)
    if exposure_option is None:
        warnings.append("RealSense SDK does not expose exposure")
    else:
        try:
            if not color_sensor.supports(exposure_option):
                warnings.append("color sensor does not support exposure")
            else:
                option_range = color_sensor.get_option_range(exposure_option)
                option_unit_us = _exposure_option_unit_us(option_range)
                requested_option_value = float(exposure_us) / option_unit_us
                step = max(float(option_range.step), 1.0)
                option_value = float(option_range.min) + round(
                    (requested_option_value - float(option_range.min)) / step
                ) * step
                option_value = min(
                    float(option_range.max),
                    max(float(option_range.min), option_value),
                )
                color_sensor.set_option(exposure_option, option_value)
                actual_option_value = float(
                    color_sensor.get_option(exposure_option)
                )
                applied["exposure_us"] = actual_option_value * option_unit_us
                applied["exposure_option_value"] = actual_option_value
                applied["exposure_option_unit_us"] = option_unit_us
                if abs(option_value - requested_option_value) > step / 2.0:
                    warnings.append(
                        f"exposure was clamped to {applied['exposure_us']:.0f} us"
                    )
        except Exception as exc:
            warnings.append(f"failed to set exposure: {exc}")
    set_option(
        "gain",
        getattr(rs.option, "gain", None),
        float(gain),
    )
    set_option(
        "auto_white_balance",
        getattr(rs.option, "enable_auto_white_balance", None),
        0.0,
    )
    set_option(
        "white_balance_k",
        getattr(rs.option, "white_balance", None),
        float(white_balance_k),
    )
    return {"requested": requested, "applied": applied, "warnings": warnings}


def lock_configured_color_controls(
    profile: Any,
    *,
    exposure_us: int,
    gain: int,
    white_balance_k: int,
    warmup_frames: int,
) -> dict[str, Any]:
    """Lock stable, camera-specific color controls after sensor warmup."""
    controls = apply_fixed_color_controls(
        profile,
        exposure_us=exposure_us,
        gain=gain,
        white_balance_k=white_balance_k,
    )
    applied = controls.get("applied", {})
    controls["mode"] = "auto_warmup_then_fixed"
    controls["warmup_frames"] = int(warmup_frames)
    controls["locked"] = (
        applied.get("auto_exposure") is False
        and applied.get("auto_white_balance") is False
        and "exposure_us" in applied
        and "gain" in applied
        and "white_balance_k" in applied
    )
    return controls


class RealSenseFrameHub:
    """Capture and filter one D455 frameset for every connected peer."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        frame_rate: int,
        serial: str | None,
        depth_preset: str,
        restart_event: asyncio.Event,
        rgb_only: bool = False,
    ) -> None:
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.serial = serial
        self.depth_preset = depth_preset
        self.rgb_only = bool(rgb_only)
        self._restart_event = restart_event
        self._loop = asyncio.get_running_loop()
        self._start_lock = asyncio.Lock()
        self._startup: asyncio.Future[None] | None = None
        self._thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        self._capture_error: str | None = None
        self._idle_stop_handle: asyncio.TimerHandle | None = None
        self._subscribers: dict[str, set[asyncio.Queue[CapturedFrame | None]]] = {
            "rgb": set(),
            "depth": set(),
        }
        self._latest_depth_units: np.ndarray | None = None
        self._depth_scale = 0.001
        self._color_intrinsics: dict[str, Any] | None = None
        self._color_source_profile: dict[str, Any] | None = None
        self._depth_available = False
        self._depth_error: str | None = None
        self._depth_retry_at: float | None = None
        self._cube_detector = RgbCubeDetector(
            background_required=False,
            require_table_roi=True,
        )
        self._latest_cube_detection: dict[str, Any] = {
            "type": "cube_detection",
            "status": "lost",
            "confidence": 0.0,
            "candidates": [],
            "selected_candidate_id": None,
            "timestamp": time.time(),
        }
        self._cube_roi_polygon: np.ndarray | None = None
        self._cube_perception_enabled = False
        self._frame_lock = threading.Lock()
        self._latest_rgb_image: np.ndarray | None = None
        self._latest_rgb_host_ns: int | None = None
        self._latest_rgb_frame_number: int | None = None
        self._collection_config = load_collection_config()
        self._collection_camera = self._collection_config.camera
        self._color_controls: dict[str, Any] = {}
        self._raw_episode_store = RawEpisodeStore(config=self._collection_config)
        self._episode_recorder: CameraEpisodeRecorder | None = None
        self._episode_session_id: str | None = None
        self._recording_lock = threading.Lock()

    async def ensure_running(self) -> None:
        stopping_thread = self._thread
        if (
            stopping_thread is not None
            and stopping_thread.is_alive()
            and self._stop_event is not None
            and self._stop_event.is_set()
        ):
            await asyncio.to_thread(stopping_thread.join, 6)
            if stopping_thread.is_alive():
                raise RuntimeError("D455 上一采集会话仍在释放，请稍后重试")
        async with self._start_lock:
            if self._thread is None or not self._thread.is_alive():
                self._startup = self._loop.create_future()
                self._stop_event = threading.Event()
                self._capture_error = None
                self._thread = threading.Thread(
                    target=self._capture_worker,
                    args=(self._stop_event,),
                    name="realsense-capture",
                    daemon=True,
                )
                self._thread.start()
            startup = self._startup
        if startup is None:
            raise RuntimeError("RealSense capture did not initialize")
        await asyncio.wait_for(
            asyncio.shield(startup), CAMERA_START_TIMEOUT_SECONDS
        )

    def subscribe(self, stream: str) -> asyncio.Queue[CapturedFrame | None]:
        if stream not in self._subscribers:
            raise ValueError(f"unsupported RealSense stream: {stream}")
        self._cancel_idle_release()
        queue: asyncio.Queue[CapturedFrame | None] = asyncio.Queue(maxsize=1)
        self._subscribers[stream].add(queue)
        return queue

    def unsubscribe(
        self, stream: str, queue: asyncio.Queue[CapturedFrame | None]
    ) -> None:
        self._subscribers[stream].discard(queue)
        self.stop_if_idle()

    def _cancel_idle_release(self) -> None:
        if self._idle_stop_handle is not None:
            self._idle_stop_handle.cancel()
            self._idle_stop_handle = None

    def _release_if_still_idle(self) -> None:
        self._idle_stop_handle = None
        if any(self._subscribers.values()):
            return
        with self._recording_lock:
            if self._episode_recorder is not None:
                return
        if self._stop_event is not None:
            self._stop_event.set()

    def stop_if_idle(self) -> None:
        if any(self._subscribers.values()):
            return
        with self._recording_lock:
            if self._episode_recorder is not None:
                return
        if self._idle_stop_handle is None:
            self._idle_stop_handle = self._loop.call_later(
                CAMERA_IDLE_RELEASE_SECONDS, self._release_if_still_idle
            )

    async def release_for_exclusive_use(self) -> None:
        """Fully release UVC handles before the calibration process opens them."""
        self._cancel_idle_release()
        if self._stop_event is not None:
            self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            await asyncio.to_thread(thread.join, 6)
        if thread is not None and thread.is_alive():
            raise RuntimeError("D455 视频流未能在限定时间内释放")

    def request_clean_restart(self) -> None:
        self._restart_event.set()

    async def shutdown(self) -> None:
        self._cancel_idle_release()
        await asyncio.to_thread(self.abort_episode_recording)
        if self._stop_event is not None:
            self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            await asyncio.to_thread(thread.join, 6)

    @staticmethod
    def _set_option(processing_block: Any, option: Any, value: float) -> None:
        if option in processing_block.get_supported_options():
            processing_block.set_option(option, value)

    def _build_filters(self) -> tuple[Any, ...]:
        threshold = rs.threshold_filter()
        self._set_option(threshold, rs.option.min_distance, 0.2)
        self._set_option(threshold, rs.option.max_distance, 4.0)

        depth_to_disparity = rs.disparity_transform(True)
        spatial = rs.spatial_filter()
        self._set_option(spatial, rs.option.filter_magnitude, 2)
        self._set_option(spatial, rs.option.filter_smooth_alpha, 0.5)
        self._set_option(spatial, rs.option.filter_smooth_delta, 20)
        self._set_option(spatial, rs.option.holes_fill, 0)

        temporal = rs.temporal_filter()
        self._set_option(temporal, rs.option.filter_smooth_alpha, 0.25)
        self._set_option(temporal, rs.option.filter_smooth_delta, 20)
        self._set_option(temporal, rs.option.holes_fill, 0)

        disparity_to_depth = rs.disparity_transform(False)
        return (
            threshold,
            depth_to_disparity,
            spatial,
            temporal,
            disparity_to_depth,
        )

    def _capture_worker(self, stop_event: threading.Event) -> None:
        pipeline: Any | None = None
        profile: Any | None = None
        visualizer: DepthVisualizer | None = None
        align: Any | None = None
        filters: tuple[Any, ...] | None = None
        failure: BaseException | None = None
        depth_retry_delay = DEPTH_RETRY_INITIAL_SECONDS
        next_depth_retry_at = float("inf")
        color_warmup_frames = 0
        color_controls_locked = False

        def start_pipeline(with_depth: bool) -> tuple[Any, Any]:
            nonlocal color_controls_locked, color_warmup_frames
            next_pipeline = rs.pipeline()
            config = rs.config()
            if self.serial:
                config.enable_device(self.serial)
            config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.rgb8,
                self.frame_rate,
            )
            if with_depth:
                config.enable_stream(
                    rs.stream.depth,
                    self.width,
                    self.height,
                    rs.format.z16,
                    self.frame_rate,
                )
            try:
                next_profile = next_pipeline.start(config)
                color_warmup_frames = 0
                color_controls_locked = False
                color_controls = enable_auto_color_controls(next_profile)
                with self._frame_lock:
                    self._color_controls = color_controls
                for warning in color_controls["warnings"]:
                    print(f"D455 自动色彩预热提示：{warning}")
                return next_pipeline, next_profile
            except BaseException:
                try:
                    next_pipeline.stop()
                except RuntimeError:
                    pass
                del next_pipeline
                raise

        def release_pipeline() -> None:
            nonlocal align, filters, pipeline, profile, visualizer
            current_pipeline = pipeline
            current_profile = profile
            current_visualizer = visualizer
            current_align = align
            current_filters = filters
            pipeline = None
            profile = None
            visualizer = None
            align = None
            filters = None
            if current_pipeline is None:
                return
            try:
                current_pipeline.stop()
            except RuntimeError:
                pass
            del (
                current_align,
                current_filters,
                current_pipeline,
                current_profile,
                current_visualizer,
            )
            gc.collect()
            time.sleep(PIPELINE_RELEASE_SETTLE_SECONDS)

        def record_color_intrinsics(
            profile: Any,
            with_depth: bool,
            depth_error: BaseException | str | None = None,
        ) -> None:
            color_profile, source_profile = _verified_color_profile(
                profile,
                width=self.width,
                height=self.height,
                fps=self.frame_rate,
            )
            color_intrinsics = color_profile.get_intrinsics()
            with self._frame_lock:
                self._depth_available = with_depth
                self._depth_error = None if with_depth else (
                    str(depth_error) if depth_error is not None else "深度流不可用"
                )
                if with_depth:
                    self._depth_retry_at = None
                self._latest_depth_units = None
                self._color_source_profile = source_profile
                self._color_intrinsics = {
                    "width": int(color_intrinsics.width),
                    "height": int(color_intrinsics.height),
                    "fx": float(color_intrinsics.fx),
                    "fy": float(color_intrinsics.fy),
                    "ppx": float(color_intrinsics.ppx),
                    "ppy": float(color_intrinsics.ppy),
                    "model": str(color_intrinsics.model),
                    "coeffs": [float(value) for value in color_intrinsics.coeffs],
                }

        def initialize_depth(profile: Any) -> tuple[Any, Any, tuple[Any, ...]]:
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())
            with self._frame_lock:
                self._depth_scale = depth_scale
            if self.depth_preset != "default":
                try:
                    self._set_option(
                        depth_sensor,
                        rs.option.visual_preset,
                        DEPTH_PRESET_VALUES[self.depth_preset],
                    )
                except RuntimeError as preset_error:
                    print(
                        "D455 深度预设不可用，继续使用当前预设："
                        f"{preset_error}"
                    )
            return (
                DepthVisualizer(depth_scale=depth_scale),
                rs.align(rs.stream.color),
                self._build_filters(),
            )

        def schedule_depth_retry(depth_error: BaseException | str) -> None:
            nonlocal depth_retry_delay, next_depth_retry_at
            next_depth_retry_at = time.monotonic() + depth_retry_delay
            with self._frame_lock:
                self._depth_error = str(depth_error)
                self._depth_retry_at = next_depth_retry_at
            depth_retry_delay = min(
                DEPTH_RETRY_MAX_SECONDS,
                depth_retry_delay * 2.0,
            )

        try:
            depth_available = not self.rgb_only
            depth_failure: BaseException | str | None = (
                "已显式启用 RGB-only 模式" if self.rgb_only else None
            )
            try:
                pipeline, profile = start_pipeline(depth_available)
            except BaseException as depth_error:
                if not depth_available:
                    raise
                print(f"D455 深度流不可用，切换到 RGB-only：{depth_error}")
                depth_failure = depth_error
                depth_available = False
                pipeline, profile = start_pipeline(False)
            record_color_intrinsics(profile, depth_available, depth_failure)

            if depth_available:
                try:
                    visualizer, align, filters = initialize_depth(profile)
                except BaseException as depth_error:
                    print(f"D455 深度初始化失败，切换到 RGB-only：{depth_error}")
                    release_pipeline()
                    pipeline, profile = start_pipeline(False)
                    depth_available = False
                    depth_failure = depth_error
                    record_color_intrinsics(profile, False, depth_error)
            if not depth_available and not self.rgb_only:
                schedule_depth_retry(depth_failure or "深度流不可用")

            first_frame = True
            missing_depth_frames = 0
            while not stop_event.is_set():
                with self._recording_lock:
                    recording_active = self._episode_recorder is not None
                if (
                    not depth_available
                    and not self.rgb_only
                    and not recording_active
                    and time.monotonic() >= next_depth_retry_at
                ):
                    print("D455 正在重试深度流。")
                    try:
                        release_pipeline()
                        pipeline, profile = start_pipeline(True)
                        visualizer, align, filters = initialize_depth(profile)
                    except BaseException as depth_error:
                        release_pipeline()
                        if requires_clean_process_restart(depth_error):
                            print(
                                "D455 深度重试遇到 UVC 资源忙，正在重启采集进程："
                                f"{depth_error}"
                            )
                            raise RuntimeError(
                                "D455 UVC 资源仍被当前进程占用，需要干净重启："
                                f"{depth_error}"
                            ) from depth_error
                        print(f"D455 深度重试失败，继续 RGB-only：{depth_error}")
                        pipeline, profile = start_pipeline(False)
                        depth_available = False
                        visualizer = None
                        align = None
                        filters = None
                        record_color_intrinsics(profile, False, depth_error)
                        schedule_depth_retry(depth_error)
                        first_frame = True
                        missing_depth_frames = 0
                        continue
                    depth_available = True
                    depth_retry_delay = DEPTH_RETRY_INITIAL_SECONDS
                    next_depth_retry_at = float("inf")
                    record_color_intrinsics(profile, True)
                    first_frame = True
                    missing_depth_frames = 0
                    print("D455 深度流已恢复。")
                try:
                    frames = pipeline.wait_for_frames(5000 if first_frame else 2000)
                except RuntimeError as frame_error:
                    if not depth_available or not first_frame:
                        raise
                    print(f"D455 深度首帧不可用，切换到 RGB-only：{frame_error}")
                    release_pipeline()
                    pipeline, profile = start_pipeline(False)
                    depth_available = False
                    record_color_intrinsics(profile, False, frame_error)
                    schedule_depth_retry(frame_error)
                    visualizer = None
                    align = None
                    filters = None
                    continue
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                color_warmup_frames += 1
                if not color_controls_locked:
                    if color_warmup_frames < COLOR_AUTO_WARMUP_FRAMES:
                        continue
                    color_controls = lock_configured_color_controls(
                        profile,
                        exposure_us=self._collection_camera.exposure_us,
                        gain=self._collection_camera.gain,
                        white_balance_k=self._collection_camera.white_balance_k,
                        warmup_frames=color_warmup_frames,
                    )
                    color_controls_locked = True
                    with self._frame_lock:
                        self._color_controls = color_controls
                    for warning in color_controls["warnings"]:
                        print(f"D455 色彩参数锁定提示：{warning}")
                    continue
                capture_host_ns = time.monotonic_ns()
                rgb_image = np.asanyarray(color_frame.get_data()).copy()
                color_frame_number = int(color_frame.get_frame_number())
                with self._frame_lock:
                    self._latest_rgb_image = rgb_image
                    self._latest_rgb_host_ns = capture_host_ns
                    self._latest_rgb_frame_number = color_frame_number
                self._capture_episode_frame(
                    rgb_image,
                    host_monotonic_ns=capture_host_ns,
                    device_timestamp_ms=float(color_frame.get_timestamp()),
                    frame_number=color_frame_number,
                )
                process_depth = (
                    bool(self._subscribers["depth"])
                    or self._cube_perception_enabled
                )
                aligned = (
                    align.process(frames)
                    if align is not None and process_depth
                    else frames
                )
                depth_units: np.ndarray | None = None
                depth_image: np.ndarray | None = None
                if (
                    process_depth
                    and depth_available
                    and filters is not None
                    and visualizer is not None
                ):
                    depth_frame = aligned.get_depth_frame()
                    if not depth_frame:
                        missing_depth_frames += 1
                        if missing_depth_frames < 3:
                            continue
                        print("D455 深度帧持续缺失，切换到 RGB-only。")
                        release_pipeline()
                        pipeline, profile = start_pipeline(False)
                        depth_available = False
                        depth_error = RuntimeError("深度帧连续 3 帧缺失")
                        record_color_intrinsics(profile, False, depth_error)
                        schedule_depth_retry(depth_error)
                        visualizer = None
                        align = None
                        filters = None
                        first_frame = True
                        missing_depth_frames = 0
                        continue
                    missing_depth_frames = 0
                    (
                        threshold,
                        depth_to_disparity,
                        spatial,
                        temporal,
                        disparity_to_depth,
                    ) = filters
                    filtered_depth = threshold.process(depth_frame)
                    filtered_depth = depth_to_disparity.process(filtered_depth)
                    filtered_depth = spatial.process(filtered_depth)
                    filtered_depth = temporal.process(filtered_depth)
                    filtered_depth = disparity_to_depth.process(filtered_depth)
                    depth_units = np.asanyarray(filtered_depth.get_data()).copy()
                    depth_image = visualizer.colorize(depth_units)
                if self._subscribers["depth"] and depth_image is None:
                    depth_image = np.zeros_like(rgb_image)
                with self._frame_lock:
                    cube_roi = (
                        None
                        if self._cube_roi_polygon is None
                        else self._cube_roi_polygon.copy()
                    )
                    cube_detection = (
                        self._cube_detector.process(rgb_image, roi_polygon=cube_roi)
                        if self._cube_perception_enabled
                        else None
                    )
                pts = capture_host_ns * VIDEO_CLOCK_RATE // 1_000_000_000
                self._loop.call_soon_threadsafe(
                    self._publish,
                    rgb_image,
                    depth_image,
                    depth_units,
                    pts,
                    cube_detection,
                )
                first_frame = False
        except BaseException as exc:  # Forward SDK failures to the WebRTC tracks.
            failure = exc
        finally:
            # pipeline.start() may fail after opening UVC file descriptors. Stop
            # and destroy it unconditionally before allowing another attempt.
            release_pipeline()
            self._loop.call_soon_threadsafe(
                self._capture_finished, stop_event, failure
            )

    def _capture_episode_frame(
        self,
        image: np.ndarray,
        *,
        host_monotonic_ns: int,
        device_timestamp_ms: float,
        frame_number: int,
    ) -> None:
        with self._recording_lock:
            recorder = self._episode_recorder
        if recorder is not None:
            recorder.capture(
                image,
                host_monotonic_ns=host_monotonic_ns,
                device_timestamp_ms=device_timestamp_ms,
                frame_number=frame_number,
            )

    def collection_recording_status(self) -> dict[str, Any]:
        with self._recording_lock:
            recorder = self._episode_recorder
            session_id = self._episode_session_id
        capture_running = bool(
            self._thread is not None
            and self._thread.is_alive()
            and self._stop_event is not None
            and not self._stop_event.is_set()
        )
        error = None
        if recorder is not None and not capture_running:
            error = self._capture_error or "D455 RGB capture thread stopped"
        response = {
            "type": "collection_camera_status",
            "ok": error is None,
            "active": recorder is not None,
            "session_id": session_id,
            "capture_running": capture_running,
            "summary": None if recorder is None else recorder.summary(),
        }
        if error is not None:
            response["error"] = error
        return response

    def collection_rgb_preview(self) -> dict[str, Any]:
        with self._frame_lock:
            image = self._latest_rgb_image
            host_monotonic_ns = self._latest_rgb_host_ns
            frame_number = self._latest_rgb_frame_number
        if image is None or host_monotonic_ns is None:
            raise RuntimeError("D455 RGB preview is not ready")
        return {
            "video_key": self._collection_camera.video_key,
            "jpeg_base64": base64.b64encode(_encode_rgb_preview(image)).decode(
                "ascii"
            ),
            "host_monotonic_ns": host_monotonic_ns,
            "frame_number": frame_number,
        }

    def start_episode_recording(
        self,
        session_id: Any,
        activation_barrier: threading.Barrier | None = None,
        shared_timeline: SharedFrameTimeline | None = None,
    ) -> dict[str, Any]:
        session_id = str(session_id)
        location = self._raw_episode_store.locate(session_id)
        if location.stage != "inprogress":
            raise RuntimeError("camera recording requires an in-progress raw episode")
        if (
            self.width != self._collection_config.camera.width
            or self.height != self._collection_config.camera.height
            or self.frame_rate != self._collection_config.camera.fps
        ):
            raise RuntimeError(
                "D455 must run at "
                f"{self._collection_config.camera.width}x"
                f"{self._collection_config.camera.height}@"
                f"{self._collection_config.camera.fps} for data collection"
            )
        with self._recording_lock:
            if self._episode_recorder is not None:
                raise RuntimeError(
                    f"camera is already recording {self._episode_session_id}"
                )
        with self._frame_lock:
            intrinsics = (
                None
                if self._color_intrinsics is None
                else dict(self._color_intrinsics)
            )
            source_profile = (
                None
                if self._color_source_profile is None
                else dict(self._color_source_profile)
            )
            color_controls = dict(self._color_controls)
        if intrinsics is None or source_profile is None:
            raise RuntimeError("D455 has not resolved its native RGB profile")
        recorder = CameraEpisodeRecorder(
            location.path,
            video_key=self._collection_config.camera.video_key,
            width=self.width,
            height=self.height,
            fps=self.frame_rate,
            video_crf=self._collection_config.camera.video_crf,
            camera_metadata={
                "camera": "Intel RealSense D455",
                "serial": self.serial,
                "stream": "rgb",
                "pixel_format": "rgb8",
                "bit_depth": 8,
                "source_profile": source_profile,
                "color_controls": color_controls,
                "intrinsics": intrinsics,
            },
            shared_timeline=shared_timeline,
        )
        activated = False
        try:
            recorder.start()
            if activation_barrier is not None:
                activation_barrier.wait(timeout=10.0)
            with self._recording_lock:
                if self._episode_recorder is not None:
                    raise RuntimeError(
                        f"camera is already recording {self._episode_session_id}"
                    )
                self._episode_recorder = recorder
                self._episode_session_id = session_id
                activated = True
        except BaseException:
            if not activated:
                recorder.stop(aborted=True)
            raise
        return self.collection_recording_status()

    def stop_episode_recording(self, session_id: Any) -> dict[str, Any]:
        session_id = str(session_id)
        with self._recording_lock:
            if self._episode_recorder is None:
                raise RuntimeError("camera is not recording")
            if session_id != self._episode_session_id:
                raise RuntimeError("camera recording session does not match")
            recorder = self._episode_recorder
            self._episode_recorder = None
            self._episode_session_id = None
        summary = recorder.stop()
        self._loop.call_soon_threadsafe(self.stop_if_idle)
        return {
            "type": "collection_camera_stop",
            "ok": summary.get("error") is None,
            "session_id": session_id,
            "summary": summary,
        }

    def abort_episode_recording(self, session_id: Any = None) -> dict[str, Any]:
        expected = None if session_id is None else str(session_id)
        with self._recording_lock:
            if self._episode_recorder is None:
                return {
                    "type": "collection_camera_abort",
                    "ok": True,
                    "active": False,
                }
            if expected is not None and expected != self._episode_session_id:
                raise RuntimeError("camera recording session does not match")
            recorder = self._episode_recorder
            active_session_id = self._episode_session_id
            self._episode_recorder = None
            self._episode_session_id = None
        summary = recorder.stop(aborted=True)
        self._loop.call_soon_threadsafe(self.stop_if_idle)
        return {
            "type": "collection_camera_abort",
            "ok": True,
            "active": False,
            "session_id": active_session_id,
            "summary": summary,
        }

    @staticmethod
    def _put_latest(
        queue: asyncio.Queue[CapturedFrame | None], item: CapturedFrame | None
    ) -> None:
        if queue.full():
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        queue.put_nowait(item)

    def _publish(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray | None,
        depth_units: np.ndarray | int | None,
        pts: int | None = None,
        cube_detection: dict[str, Any] | None = None,
    ) -> None:
        # Keep metric aligned depth separate from the colorized video frame.
        # Consumers can request a small patch without receiving the full depth
        # image over the signaling channel.
        # Keep the old three-argument private-call shape usable for small
        # harnesses; live capture always supplies metric depth units explicitly.
        if pts is None:
            pts = int(depth_units)
            metric_depth = None
        else:
            metric_depth = depth_units
        if metric_depth is not None:
            with self._frame_lock:
                self._latest_depth_units = np.asarray(metric_depth).copy()
        if cube_detection is not None:
            with self._frame_lock:
                self._latest_cube_detection = cube_detection
        rgb_frame = CapturedFrame(rgb_image, pts)
        for queue in tuple(self._subscribers["rgb"]):
            self._put_latest(queue, rgb_frame)
        if depth_image is not None:
            depth_frame = CapturedFrame(depth_image, pts)
            for queue in tuple(self._subscribers["depth"]):
                self._put_latest(queue, depth_frame)
        if self._startup is not None and not self._startup.done():
            self._startup.set_result(None)

    def _capture_finished(
        self, stop_event: threading.Event, failure: BaseException | None
    ) -> None:
        if stop_event is not self._stop_event:
            return
        self._capture_error = None if failure is None else str(failure)
        stop_event.set()
        # Keep the Thread object until ensure_running() observes or joins its
        # completed lifecycle; clearing it here races the worker's final return.
        if self._startup is not None and not self._startup.done():
            if failure is None:
                self._startup.set_exception(
                    RuntimeError("RealSense capture stopped before its first frame")
                )
            else:
                self._startup.set_exception(RuntimeError(str(failure)))
        for queues in self._subscribers.values():
            for queue in tuple(queues):
                self._put_latest(queue, None)
        if failure is not None:
            print(f"D455 采集错误：{failure}")
            # librealsense can retain UVC descriptors after pipeline.start()
            # fails. Replacing this camera-only process is the one reliable
            # way to release those kernel handles without touching arm control.
            self._loop.call_later(
                CAMERA_RESTART_DELAY_SECONDS,
                self._restart_event.set,
            )

    def depth_sample(self, x: Any, y: Any, radius: Any = 2) -> dict[str, Any]:
        """Return a robust metric depth patch from the latest aligned frame."""
        try:
            pixel_x = int(round(float(x)))
            pixel_y = int(round(float(y)))
            patch_radius = max(0, min(8, int(round(float(radius)))))
        except (TypeError, ValueError):
            raise ValueError("depth sample coordinates must be finite numbers")
        with self._frame_lock:
            depth_units = None if self._latest_depth_units is None else self._latest_depth_units.copy()
            scale = float(self._depth_scale)
            intrinsics = None if self._color_intrinsics is None else dict(self._color_intrinsics)
            depth_available = self._depth_available
        if not depth_available:
            raise RuntimeError("D455 当前为 RGB-only 模式，深度流不可用")
        if depth_units is None or intrinsics is None:
            raise RuntimeError("D455 尚未产生对齐深度帧")
        height, width = depth_units.shape[:2]
        if not 0 <= pixel_x < width or not 0 <= pixel_y < height:
            raise ValueError("depth sample pixel is outside the RGB frame")
        x0, x1 = max(0, pixel_x - patch_radius), min(width, pixel_x + patch_radius + 1)
        y0, y1 = max(0, pixel_y - patch_radius), min(height, pixel_y + patch_radius + 1)
        values = depth_units[y0:y1, x0:x1].astype(np.float64).reshape(-1)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            return {
                "type": "depth_sample",
                "ok": False,
                "x": pixel_x,
                "y": pixel_y,
                "intrinsics": intrinsics,
            }
        depth_m = float(np.median(values) * scale)
        return {
            "type": "depth_sample",
            "ok": True,
            "x": pixel_x,
            "y": pixel_y,
            "radius": patch_radius,
            "depth_m": depth_m,
            "valid_count": int(values.size),
            "intrinsics": intrinsics,
        }

    def camera_status(self) -> dict[str, Any]:
        with self._frame_lock:
            intrinsics = None if self._color_intrinsics is None else dict(self._color_intrinsics)
            depth_available = self._depth_available
            depth_error = self._depth_error
            depth_retry_at = self._depth_retry_at
        depth_retry_in_ms = (
            None
            if depth_retry_at is None
            else max(0, round((depth_retry_at - time.monotonic()) * 1000))
        )
        return {
            "type": "camera_status",
            "ok": intrinsics is not None,
            "source_revision": SOURCE_REVISION,
            "service_started_at": SERVICE_STARTED_AT,
            "depth_available": depth_available,
            "depth_error": depth_error,
            "depth_retry_in_ms": depth_retry_in_ms,
            "intrinsics": intrinsics,
            "cube_perception_enabled": self._cube_perception_enabled,
        }

    def cube_detection(self) -> dict[str, Any]:
        with self._frame_lock:
            result = dict(self._latest_cube_detection)
            result["candidates"] = [
                dict(candidate)
                for candidate in self._latest_cube_detection.get("candidates", [])
                if isinstance(candidate, dict)
            ]
            intrinsics = None if self._color_intrinsics is None else dict(self._color_intrinsics)
            depth_available = self._depth_available
            depth_error = self._depth_error
            depth_retry_at = self._depth_retry_at
            roi_active = self._cube_roi_polygon is not None
        result["intrinsics"] = intrinsics
        result["source_revision"] = SOURCE_REVISION
        result["service_started_at"] = SERVICE_STARTED_AT
        result["cube_size_m"] = 0.03
        result["depth_available"] = depth_available
        result["depth_error"] = depth_error
        result["depth_retry_in_ms"] = (
            None
            if depth_retry_at is None
            else max(0, round((depth_retry_at - time.monotonic()) * 1000))
        )
        result["table_roi_active"] = roi_active
        result["cube_perception_enabled"] = self._cube_perception_enabled
        center = result.get("center_px")
        if depth_available and result.get("status") == "tracking" and center:
            try:
                # The center of the detected top face is safer than its edges,
                # where a depth patch can mix the cube and tabletop planes.
                result["depth_sample"] = self.depth_sample(
                    center[0], center[1], radius=1
                )
            except (TypeError, ValueError, RuntimeError) as exc:
                result["depth_sample"] = {
                    "type": "depth_sample",
                    "ok": False,
                    "error": str(exc),
                }
        for candidate in result["candidates"]:
            candidate["intrinsics"] = intrinsics
            candidate["cube_size_m"] = result["cube_size_m"]
            center = candidate.get("center_px")
            if depth_available and candidate.get("status") == "tracking" and center:
                try:
                    candidate["depth_sample"] = self.depth_sample(
                        center[0], center[1], radius=1
                    )
                except (TypeError, ValueError, RuntimeError) as exc:
                    candidate["depth_sample"] = {
                        "type": "depth_sample",
                        "ok": False,
                        "error": str(exc),
                    }
        return result

    def set_cube_perception_enabled(self, value: Any) -> dict[str, Any]:
        enabled = value is True
        with self._frame_lock:
            self._cube_perception_enabled = enabled
            self._cube_detector.reset()
            self._latest_cube_detection = {
                "type": "cube_detection", "status": "disabled" if not enabled else "lost",
                "confidence": 0.0, "candidates": [], "selected_candidate_id": None,
                "timestamp": time.time(),
            }
        return {"type": "cube_perception", "ok": True, "enabled": enabled}

    def configure_cube_detection(self, polygon: Any) -> dict[str, Any]:
        if polygon is None:
            with self._frame_lock:
                self._cube_roi_polygon = None
                self._latest_cube_detection = {
                    "type": "cube_detection",
                    "status": "lost",
                    "confidence": 0.0,
                    "candidates": [],
                    "selected_candidate_id": None,
                    "timestamp": time.time(),
                }
                self._cube_detector.reset()
            return {
                "type": "cube_config",
                "ok": True,
                "table_roi_active": False,
            }
        try:
            points = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
        except (TypeError, ValueError) as exc:
            raise ValueError("table ROI must contain pixel coordinate pairs") from exc
        if points.shape[0] != 4 or not np.all(np.isfinite(points)):
            raise ValueError("table ROI must contain four finite corners")
        points[:, 0] = np.clip(points[:, 0], -2 * self.width, 3 * self.width)
        points[:, 1] = np.clip(points[:, 1], -2 * self.height, 3 * self.height)
        first_edge = points[1] - points[0]
        second_edge = points[2] - points[0]
        signed_area = first_edge[0] * second_edge[1] - first_edge[1] * second_edge[0]
        if abs(float(signed_area)) < 4:
            raise ValueError("table ROI is degenerate")
        with self._frame_lock:
            self._cube_roi_polygon = points.copy()
            self._latest_cube_detection = {
                "type": "cube_detection",
                "status": "lost",
                "confidence": 0.0,
                "candidates": [],
                "selected_candidate_id": None,
                "timestamp": time.time(),
            }
            self._cube_detector.reset()
        return {
            "type": "cube_config",
            "ok": True,
            "table_roi_active": True,
        }

    def request_cube_background_capture(self, frame_count: Any = None) -> dict[str, Any]:
        with self._frame_lock:
            result = self._cube_detector.request_background_capture(frame_count)
        result.update({"type": "cube_background", "ok": True})
        return result

    def cube_background_status(self) -> dict[str, Any]:
        with self._frame_lock:
            result = dict(self._cube_detector.background_status())
        result.update({"type": "cube_background", "ok": True})
        return result


class RealSenseRgbdHub:
    """Capture serial-bound eye-in-hand RGB and depth streams."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        frame_rate: int,
        output_frame_rate: int,
        output_width: int,
        output_height: int,
        serial: str,
        restart_event: asyncio.Event,
    ) -> None:
        self.width = width
        self.height = height
        self.frame_rate = frame_rate
        self.output_frame_rate = output_frame_rate
        self.output_size = (output_width, output_height)
        self.serial = serial
        self._restart_event = restart_event
        self._loop = asyncio.get_running_loop()
        self._start_lock = asyncio.Lock()
        self._startup: asyncio.Future[None] | None = None
        self._thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        self._capture_error: str | None = None
        self._idle_stop_handle: asyncio.TimerHandle | None = None
        self._subscribers: dict[
            str, set[asyncio.Queue[CapturedFrame | None]]
        ] = {
            "rgb": set(),
            "depth": set(),
        }
        self._frame_lock = threading.Lock()
        self._color_intrinsics: dict[str, Any] | None = None
        self._color_source_profile: dict[str, Any] | None = None
        self._latest_rgb_image: np.ndarray | None = None
        self._latest_rgb_host_ns: int | None = None
        self._latest_rgb_frame_number: int | None = None
        self._collection_config = load_collection_config()
        self._collection_camera = self._collection_config.wrist_camera
        self._color_controls: dict[str, Any] = {}
        self._raw_episode_store = RawEpisodeStore(config=self._collection_config)
        self._episode_recorder: CameraEpisodeRecorder | None = None
        self._episode_session_id: str | None = None
        self._recording_lock = threading.Lock()

    async def ensure_running(self) -> None:
        stopping_thread = self._thread
        if (
            stopping_thread is not None
            and stopping_thread.is_alive()
            and self._stop_event is not None
            and self._stop_event.is_set()
        ):
            await asyncio.to_thread(stopping_thread.join, 6)
            if stopping_thread.is_alive():
                raise RuntimeError("眼在手相机上一采集会话仍在释放，请稍后重试")
        async with self._start_lock:
            if self._thread is None or not self._thread.is_alive():
                self._startup = self._loop.create_future()
                self._stop_event = threading.Event()
                self._capture_error = None
                self._thread = threading.Thread(
                    target=self._capture_worker,
                    args=(self._stop_event,),
                    name="eye-in-hand-capture",
                    daemon=True,
                )
                self._thread.start()
            startup = self._startup
        if startup is None:
            raise RuntimeError("眼在手相机采集未初始化")
        await asyncio.wait_for(
            asyncio.shield(startup), CAMERA_START_TIMEOUT_SECONDS
        )

    def subscribe(self, stream: str) -> asyncio.Queue[CapturedFrame | None]:
        if stream not in self._subscribers:
            raise ValueError(f"unsupported eye-in-hand stream: {stream}")
        self._cancel_idle_release()
        queue: asyncio.Queue[CapturedFrame | None] = asyncio.Queue(maxsize=1)
        self._subscribers[stream].add(queue)
        return queue

    def unsubscribe(
        self, stream: str, queue: asyncio.Queue[CapturedFrame | None]
    ) -> None:
        self._subscribers[stream].discard(queue)
        self.stop_if_idle()

    def _cancel_idle_release(self) -> None:
        if self._idle_stop_handle is not None:
            self._idle_stop_handle.cancel()
            self._idle_stop_handle = None

    def _release_if_still_idle(self) -> None:
        self._idle_stop_handle = None
        if any(self._subscribers.values()):
            return
        with self._recording_lock:
            if self._episode_recorder is not None:
                return
        if self._stop_event is not None:
            self._stop_event.set()

    def stop_if_idle(self) -> None:
        if any(self._subscribers.values()):
            return
        with self._recording_lock:
            if self._episode_recorder is not None:
                return
        if self._idle_stop_handle is None:
            self._idle_stop_handle = self._loop.call_later(
                CAMERA_IDLE_RELEASE_SECONDS, self._release_if_still_idle
            )

    async def release_for_exclusive_use(self) -> None:
        """Fully release UVC handles before the calibration process opens them."""
        self._cancel_idle_release()
        if self._stop_event is not None:
            self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            await asyncio.to_thread(thread.join, 6)
        if thread is not None and thread.is_alive():
            raise RuntimeError("D405 视频流未能在限定时间内释放")

    async def shutdown(self) -> None:
        self._cancel_idle_release()
        await asyncio.to_thread(self.abort_episode_recording)
        if self._stop_event is not None:
            self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            await asyncio.to_thread(thread.join, 6)

    def _capture_worker(self, stop_event: threading.Event) -> None:
        pipeline = rs.pipeline()
        failure: BaseException | None = None
        try:
            config = rs.config()
            config.enable_device(self.serial)
            config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.rgb8,
                self.frame_rate,
            )
            config.enable_stream(
                rs.stream.depth,
                self.width,
                self.height,
                rs.format.z16,
                self.frame_rate,
            )
            profile = pipeline.start(config)
            color_controls = enable_auto_color_controls(profile)
            with self._frame_lock:
                self._color_controls = color_controls
            for warning in color_controls["warnings"]:
                print(f"D405 自动色彩预热提示：{warning}")
            color_profile, source_profile = _verified_color_profile(
                profile,
                width=self.width,
                height=self.height,
                fps=self.frame_rate,
            )
            color_intrinsics = color_profile.get_intrinsics()
            with self._frame_lock:
                self._color_source_profile = source_profile
                self._color_intrinsics = {
                    "width": int(color_intrinsics.width),
                    "height": int(color_intrinsics.height),
                    "fx": float(color_intrinsics.fx),
                    "fy": float(color_intrinsics.fy),
                    "ppx": float(color_intrinsics.ppx),
                    "ppy": float(color_intrinsics.ppy),
                    "model": str(color_intrinsics.model),
                    "coeffs": [
                        float(value) for value in color_intrinsics.coeffs
                    ],
                }
            depth_scale = float(
                profile.get_device().first_depth_sensor().get_depth_scale()
            )
            visualizer = DepthVisualizer(
                depth_scale=depth_scale,
                min_distance_m=0.07,
                max_distance_m=1.0,
                min_valid_frames=2,
                min_consistent_neighbors=2,
            )
            align = rs.align(rs.stream.color)
            color_warmup_frames = 0
            color_controls_locked = False
            while not stop_event.is_set():
                frames = pipeline.wait_for_frames(2000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                color_warmup_frames += 1
                if not color_controls_locked:
                    if color_warmup_frames < COLOR_AUTO_WARMUP_FRAMES:
                        continue
                    color_controls = lock_configured_color_controls(
                        profile,
                        exposure_us=self._collection_camera.exposure_us,
                        gain=self._collection_camera.gain,
                        white_balance_k=self._collection_camera.white_balance_k,
                        warmup_frames=color_warmup_frames,
                    )
                    color_controls_locked = True
                    with self._frame_lock:
                        self._color_controls = color_controls
                    for warning in color_controls["warnings"]:
                        print(f"D405 色彩参数锁定提示：{warning}")
                    continue
                capture_host_ns = time.monotonic_ns()
                rgb_image = np.asanyarray(color_frame.get_data()).copy()
                color_frame_number = int(color_frame.get_frame_number())
                with self._frame_lock:
                    self._latest_rgb_image = rgb_image
                    self._latest_rgb_host_ns = capture_host_ns
                    self._latest_rgb_frame_number = color_frame_number
                self._capture_episode_frame(
                    rgb_image,
                    host_monotonic_ns=capture_host_ns,
                    device_timestamp_ms=float(color_frame.get_timestamp()),
                    frame_number=color_frame_number,
                )
                process_depth = bool(self._subscribers["depth"])
                depth_image: np.ndarray | None = None
                if process_depth:
                    aligned = align.process(frames)
                    depth_frame = aligned.get_depth_frame()
                    if not depth_frame:
                        continue
                    depth_units = np.asanyarray(depth_frame.get_data()).copy()
                    depth_image = visualizer.colorize(depth_units)
                pts = capture_host_ns * VIDEO_CLOCK_RATE // 1_000_000_000
                self._loop.call_soon_threadsafe(
                    self._publish,
                    rgb_image,
                    depth_image,
                    pts,
                )
        except BaseException as exc:
            failure = exc
        finally:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            self._loop.call_soon_threadsafe(
                self._capture_finished, stop_event, failure
            )

    def _capture_episode_frame(
        self,
        image: np.ndarray,
        *,
        host_monotonic_ns: int,
        device_timestamp_ms: float,
        frame_number: int,
    ) -> None:
        with self._recording_lock:
            recorder = self._episode_recorder
        if recorder is not None:
            recorder.capture(
                image,
                host_monotonic_ns=host_monotonic_ns,
                device_timestamp_ms=device_timestamp_ms,
                frame_number=frame_number,
            )

    def collection_recording_status(self) -> dict[str, Any]:
        with self._recording_lock:
            recorder = self._episode_recorder
            session_id = self._episode_session_id
        capture_running = bool(
            self._thread is not None
            and self._thread.is_alive()
            and self._stop_event is not None
            and not self._stop_event.is_set()
        )
        error = None
        if recorder is not None and not capture_running:
            error = self._capture_error or "D405 RGB capture thread stopped"
        response = {
            "type": "collection_camera_status",
            "ok": error is None,
            "active": recorder is not None,
            "session_id": session_id,
            "capture_running": capture_running,
            "summary": None if recorder is None else recorder.summary(),
        }
        if error is not None:
            response["error"] = error
        return response

    def collection_rgb_preview(self) -> dict[str, Any]:
        with self._frame_lock:
            image = self._latest_rgb_image
            host_monotonic_ns = self._latest_rgb_host_ns
            frame_number = self._latest_rgb_frame_number
        if image is None or host_monotonic_ns is None:
            raise RuntimeError("D405 RGB preview is not ready")
        return {
            "video_key": self._collection_camera.video_key,
            "jpeg_base64": base64.b64encode(_encode_rgb_preview(image)).decode(
                "ascii"
            ),
            "host_monotonic_ns": host_monotonic_ns,
            "frame_number": frame_number,
        }

    def start_episode_recording(
        self,
        session_id: Any,
        activation_barrier: threading.Barrier | None = None,
        shared_timeline: SharedFrameTimeline | None = None,
    ) -> dict[str, Any]:
        session_id = str(session_id)
        location = self._raw_episode_store.locate(session_id)
        if location.stage != "inprogress":
            raise RuntimeError("camera recording requires an in-progress raw episode")
        camera = self._collection_config.wrist_camera
        if (
            self.width != camera.width
            or self.height != camera.height
            or self.frame_rate != camera.fps
        ):
            raise RuntimeError(
                f"D405 must run at {camera.width}x{camera.height}@{camera.fps} "
                "for data collection"
            )
        with self._recording_lock:
            if self._episode_recorder is not None:
                raise RuntimeError(
                    f"camera is already recording {self._episode_session_id}"
                )
        with self._frame_lock:
            intrinsics = (
                None
                if self._color_intrinsics is None
                else dict(self._color_intrinsics)
            )
            source_profile = (
                None
                if self._color_source_profile is None
                else dict(self._color_source_profile)
            )
            color_controls = dict(self._color_controls)
        if intrinsics is None or source_profile is None:
            raise RuntimeError("D405 has not resolved its native RGB profile")
        recorder = CameraEpisodeRecorder(
            location.path,
            video_key=camera.video_key,
            width=self.width,
            height=self.height,
            fps=self.frame_rate,
            video_crf=camera.video_crf,
            camera_metadata={
                "camera": "Intel RealSense D405",
                "serial": self.serial,
                "stream": "rgb",
                "pixel_format": "rgb8",
                "bit_depth": 8,
                "source_profile": source_profile,
                "color_controls": color_controls,
                "intrinsics": intrinsics,
            },
            shared_timeline=shared_timeline,
        )
        activated = False
        try:
            recorder.start()
            if activation_barrier is not None:
                activation_barrier.wait(timeout=10.0)
            with self._recording_lock:
                if self._episode_recorder is not None:
                    raise RuntimeError(
                        f"camera is already recording {self._episode_session_id}"
                    )
                self._episode_recorder = recorder
                self._episode_session_id = session_id
                activated = True
        except BaseException:
            if not activated:
                recorder.stop(aborted=True)
            raise
        return self.collection_recording_status()

    def stop_episode_recording(self, session_id: Any) -> dict[str, Any]:
        session_id = str(session_id)
        with self._recording_lock:
            if self._episode_recorder is None:
                raise RuntimeError("camera is not recording")
            if session_id != self._episode_session_id:
                raise RuntimeError("camera recording session does not match")
            recorder = self._episode_recorder
            self._episode_recorder = None
            self._episode_session_id = None
        summary = recorder.stop()
        self._loop.call_soon_threadsafe(self.stop_if_idle)
        return {
            "type": "collection_camera_stop",
            "ok": summary.get("error") is None,
            "session_id": session_id,
            "summary": summary,
        }

    def abort_episode_recording(self, session_id: Any = None) -> dict[str, Any]:
        expected = None if session_id is None else str(session_id)
        with self._recording_lock:
            if self._episode_recorder is None:
                return {
                    "type": "collection_camera_abort",
                    "ok": True,
                    "active": False,
                }
            if expected is not None and expected != self._episode_session_id:
                raise RuntimeError("camera recording session does not match")
            recorder = self._episode_recorder
            active_session_id = self._episode_session_id
            self._episode_recorder = None
            self._episode_session_id = None
        summary = recorder.stop(aborted=True)
        self._loop.call_soon_threadsafe(self.stop_if_idle)
        return {
            "type": "collection_camera_abort",
            "ok": True,
            "active": False,
            "session_id": active_session_id,
            "summary": summary,
        }

    def _publish(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray | None,
        pts: int,
    ) -> None:
        rgb_frame = CapturedFrame(rgb_image, pts)
        for queue in tuple(self._subscribers["rgb"]):
            RealSenseFrameHub._put_latest(queue, rgb_frame)
        if depth_image is not None:
            depth_frame = CapturedFrame(depth_image, pts)
            for queue in tuple(self._subscribers["depth"]):
                RealSenseFrameHub._put_latest(queue, depth_frame)
        if self._startup is not None and not self._startup.done():
            self._startup.set_result(None)

    def _capture_finished(
        self, stop_event: threading.Event, failure: BaseException | None
    ) -> None:
        if stop_event is not self._stop_event:
            return
        self._capture_error = None if failure is None else str(failure)
        stop_event.set()
        if self._startup is not None and not self._startup.done():
            message = (
                "眼在手相机在首帧前停止"
                if failure is None
                else f"眼在手相机 {self.serial} 启动失败：{failure}"
            )
            self._startup.set_exception(RuntimeError(message))
        for queues in self._subscribers.values():
            for queue in tuple(queues):
                RealSenseFrameHub._put_latest(queue, None)
        if failure is not None:
            print(f"眼在手相机 {self.serial} 采集错误：{failure}")
            self._loop.call_later(
                CAMERA_RESTART_DELAY_SECONDS,
                self._restart_event.set,
            )


class RealSenseVideoTrack(VideoStreamTrack):
    def __init__(
        self,
        hub: RealSenseFrameHub | RealSenseRgbdHub,
        stream: str,
        output_frame_rate: int | None = None,
        output_size: tuple[int, int] | None = None,
        label: str = "video",
        on_first_frame: Callable[[str], None] | None = None,
    ) -> None:
        super().__init__()
        self._hub = hub
        self._stream = stream
        self._queue = hub.subscribe(stream)
        self._subscribed = True
        self._output_frame_rate = output_frame_rate
        self._output_size = output_size
        self._last_token_pts: int | None = None
        self._label = label
        self._on_first_frame = on_first_frame
        self._first_frame_delivered = False
        # Keep this generic limiter for explicit lower-output profiles.  The
        # default source/output profile is now native 30 FPS, so every frame
        # passes and no cadence conversion is performed.
        self._output_tokens = 1.0

    async def recv(self) -> VideoFrame:
        while True:
            captured = await self._queue.get()
            if captured is None:
                raise MediaStreamError
            if self._allow_output_frame(captured.pts):
                break
        image = captured.image
        if self._output_size is not None:
            output_width, output_height = self._output_size
            source_height, source_width = image.shape[:2]
            if (source_width, source_height) != self._output_size:
                if (
                    source_width % output_width == 0
                    and source_height % output_height == 0
                ):
                    image = image[
                        :: source_height // output_height,
                        :: source_width // output_width,
                    ]
                else:
                    # av's scaler keeps profile support local to the media
                    # track and avoids a second RealSense/UVC stream.
                    source = VideoFrame.from_ndarray(image, format="rgb24")
                    image = source.reformat(
                        width=output_width,
                        height=output_height,
                        format="rgb24",
                    ).to_ndarray(format="rgb24")
        frame = VideoFrame.from_ndarray(image, format="rgb24")
        frame.pts = captured.pts
        frame.time_base = VIDEO_TIME_BASE
        if not self._first_frame_delivered:
            self._first_frame_delivered = True
            if self._on_first_frame is not None:
                self._on_first_frame(self._label)
        return frame

    def _allow_output_frame(self, pts: int) -> bool:
        rate = self._output_frame_rate
        if rate is None:
            return True
        previous = self._last_token_pts
        self._last_token_pts = pts
        if previous is None:
            self._output_tokens = 0.0
            return True
        elapsed_pts = max(0, pts - previous)
        self._output_tokens = min(
            2.0,
            self._output_tokens
            + elapsed_pts * float(rate) / float(VIDEO_CLOCK_RATE),
        )
        if self._output_tokens + 1e-9 < 1.0:
            return False
        self._output_tokens -= 1.0
        return True

    def stop(self) -> None:
        if self._subscribed:
            self._hub.unsubscribe(self._stream, self._queue)
            self._subscribed = False
        super().stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D455 WebRTC RGB-D 视频服务")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=port("camera_webrtc"))
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--origin", action="append", dest="origins")
    parser.add_argument("--token", help="局域网配对令牌")
    parser.add_argument(
        "--serial",
        default=os.environ.get("ARX5_REALSENSE_SERIAL", DEFAULT_D455_SERIAL),
        help="指定 RealSense 序列号（也可设置 ARX5_REALSENSE_SERIAL）",
    )
    parser.add_argument(
        "--eye-in-hand-serial",
        default=os.environ.get(
            "ARX5_EYE_IN_HAND_SERIAL", DEFAULT_EYE_IN_HAND_SERIAL
        ),
        help="指定眼在手 RealSense 序列号（也可设置 ARX5_EYE_IN_HAND_SERIAL）",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--eye-in-hand-width", type=int, default=1280)
    parser.add_argument("--eye-in-hand-height", type=int, default=720)
    parser.add_argument("--eye-in-hand-fps", type=int, default=30)
    parser.add_argument("--eye-in-hand-output-width", type=int, default=640)
    parser.add_argument("--eye-in-hand-output-height", type=int, default=360)
    parser.add_argument(
        "--eye-in-hand-output-fps",
        type=int,
        default=30,
        help="D405 WebRTC 预览上限；RGBD 采集仍使用 --eye-in-hand-fps",
    )
    parser.add_argument(
        "--rgb-only",
        action="store_true",
        default=os.environ.get("ARX5_CAMERA_RGB_ONLY", "").lower() in {"1", "true", "yes"},
        help="只启动 RGB；用于验证检测完全不依赖深度流",
    )
    parser.add_argument(
        "--depth-preset",
        choices=tuple(DEPTH_PRESET_VALUES),
        default=os.environ.get("ARX5_DEPTH_PRESET", "default"),
        help="D455 深度预设（也可设置 ARX5_DEPTH_PRESET）",
    )
    args = parser.parse_args()
    if (args.cert is None) != (args.key is None):
        parser.error("--cert 和 --key 必须同时提供")
    if any(value <= 0 for value in (
        args.width,
        args.height,
        args.fps,
        args.eye_in_hand_width,
        args.eye_in_hand_height,
        args.eye_in_hand_fps,
        args.eye_in_hand_output_width,
        args.eye_in_hand_output_height,
        args.eye_in_hand_output_fps,
    )):
        parser.error("--width、--height 和 --fps 必须为正整数")
    if args.eye_in_hand_output_fps > args.eye_in_hand_fps:
        parser.error("--eye-in-hand-output-fps 不能高于 --eye-in-hand-fps")
    if (
        args.eye_in_hand_output_width > args.eye_in_hand_width
        or args.eye_in_hand_output_height > args.eye_in_hand_height
    ):
        parser.error("D405 WebRTC 输出尺寸不能大于原始采集尺寸")
    return args


def authorized(websocket: ServerConnection, token: str | None) -> bool:
    if token is None:
        return True
    request = websocket.request
    if request is None:
        return False
    values = parse_qs(urlsplit(request.path).query).get("token", [])
    return len(values) == 1 and hmac.compare_digest(values[0], token)


def canonical_path(websocket: ServerConnection) -> bool:
    request = websocket.request
    return request is not None and urlsplit(request.path).path == SIGNAL_PATH


def prefer_h264(peer: RTCPeerConnection) -> None:
    codecs = RTCRtpSender.getCapabilities("video").codecs
    ordered = [codec for codec in codecs if codec.mimeType == "video/H264"]
    ordered.extend(codec for codec in codecs if codec.mimeType != "video/H264")
    for transceiver in peer.getTransceivers():
        if transceiver.kind == "video":
            transceiver.setCodecPreferences(ordered)


async def collection_camera_response(
    hub: RealSenseFrameHub,
    eye_in_hand_hub: RealSenseRgbdHub,
    message: dict[str, Any],
) -> dict[str, Any]:
    """Start, stop, and report one atomic dual-RGB episode recording."""
    message_type = message.get("type")
    if message_type not in COLLECTION_CAMERA_MESSAGE_TYPES:
        raise ValueError("unsupported collection camera request")
    if message_type == "collection_camera_preview":
        video_key = str(message.get("video_key") or "")
        preview_hub = {
            "exterior_rgb": hub,
            "wrist_rgb": eye_in_hand_hub,
        }.get(video_key)
        if preview_hub is None:
            raise ValueError("unsupported collection camera preview")
        await preview_hub.ensure_running()
        response = await asyncio.to_thread(preview_hub.collection_rgb_preview)
        response.update(
            {
                "type": message_type,
                "ok": True,
                "id": message.get("id"),
            }
        )
        return response
    if message_type == "collection_camera_start":
        await asyncio.gather(hub.ensure_running(), eye_in_hand_hub.ensure_running())
    elif message_type == "collection_camera_status":
        exterior_active = hub.collection_recording_status().get("active")
        wrist_active = eye_in_hand_hub.collection_recording_status().get("active")
        if not exterior_active and not wrist_active:
            await asyncio.gather(hub.ensure_running(), eye_in_hand_hub.ensure_running())
    if message_type == "collection_camera_start":
        activation_barrier = threading.Barrier(2)
        shared_timeline = SharedFrameTimeline(
            hub._collection_camera.fps,
            master_video_key=eye_in_hand_hub._collection_camera.video_key,
        )

        async def start_recorder(recording_hub: Any) -> dict[str, Any]:
            try:
                return await asyncio.to_thread(
                    recording_hub.start_episode_recording,
                    message.get("session_id"),
                    activation_barrier,
                    shared_timeline,
                )
            except BaseException:
                activation_barrier.abort()
                raise

        start_results = await asyncio.gather(
            start_recorder(hub),
            start_recorder(eye_in_hand_hub),
            return_exceptions=True,
        )
        failures = [
            str(result)
            for result in start_results
            if isinstance(result, BaseException)
        ]
        if failures:
            await asyncio.gather(
                asyncio.to_thread(hub.abort_episode_recording),
                asyncio.to_thread(eye_in_hand_hub.abort_episode_recording),
            )
            raise RuntimeError("; ".join(failures))
        exterior, wrist = start_results
    elif message_type == "collection_camera_stop":
        stop_results = await asyncio.gather(
            asyncio.to_thread(
                hub.stop_episode_recording,
                message.get("session_id"),
            ),
            asyncio.to_thread(
                eye_in_hand_hub.stop_episode_recording,
                message.get("session_id"),
            ),
            return_exceptions=True,
        )
        results: list[dict[str, Any]] = []
        failures: list[str] = []
        for label, result in zip(("D455", "D405"), stop_results, strict=True):
            if isinstance(result, BaseException):
                failures.append(f"{label}: {result}")
            else:
                results.append(result)
        if failures:
            raise RuntimeError("; ".join(failures))
        exterior, wrist = results
    elif message_type == "collection_camera_abort":
        exterior, wrist = await asyncio.gather(
            asyncio.to_thread(
                hub.abort_episode_recording,
                message.get("session_id"),
            ),
            asyncio.to_thread(
                eye_in_hand_hub.abort_episode_recording,
                message.get("session_id"),
            ),
        )
    else:
        exterior = hub.collection_recording_status()
        wrist = eye_in_hand_hub.collection_recording_status()
    summaries = {
        "exterior_rgb": exterior.get("summary"),
        "wrist_rgb": wrist.get("summary"),
    }
    recorder_errors = [
        f"{label}: {response['error']}"
        for label, response in (("D455", exterior), ("D405", wrist))
        if response.get("error")
    ] + [
        f"{label}: {summary['error']}"
        for label, summary in (
            ("D455", summaries["exterior_rgb"]),
            ("D405", summaries["wrist_rgb"]),
        )
        if isinstance(summary, dict) and summary.get("error")
    ]
    for label, summary, camera in (
        ("D455", summaries["exterior_rgb"], hub._collection_camera),
        ("D405", summaries["wrist_rgb"], eye_in_hand_hub._collection_camera),
    ):
        if not isinstance(summary, dict):
            continue
        source_nonmonotonic = int(summary.get("source_nonmonotonic_frames") or 0)
        if source_nonmonotonic:
            recorder_errors.append(
                f"{label}: source frame sequence reset {source_nonmonotonic} times"
            )
        recording_duration_s = float(summary.get("duration_s") or 0.0)
        recording_effective_fps = float(
            summary.get("recording_effective_fps")
            or summary.get("source_effective_fps")
            or 0.0
        )
        minimum_usable_fps = camera.fps * MINIMUM_USABLE_CAMERA_RATE_FRACTION
        if (
            recording_duration_s >= 3.0
            and recording_effective_fps < minimum_usable_fps
        ):
            recorder_errors.append(
                f"{label}: effective recording frame rate "
                f"{recording_effective_fps:.2f} FPS "
                f"is below minimum usable {minimum_usable_fps:.2f} FPS"
            )
    session_ids = {
        str(response["session_id"])
        for response in (exterior, wrist)
        if response.get("session_id") is not None
    }
    active = bool(exterior.get("active") or wrist.get("active"))
    response = {
        "type": message_type,
        "ok": bool(
            exterior.get("ok", True)
            and wrist.get("ok", True)
            and not recorder_errors
        ),
        "active": active,
        "session_id": next(iter(session_ids)) if len(session_ids) == 1 else None,
        "cameras": summaries,
        "summary": summaries["exterior_rgb"],
    }
    if active and len(session_ids) != 1:
        response["ok"] = False
        response["error"] = "D455 and D405 recording sessions are inconsistent"
    elif recorder_errors:
        response["error"] = "; ".join(recorder_errors)
    response["id"] = message.get("id")
    return response


async def handle_collection_client(
    websocket: ServerConnection,
    hub: RealSenseFrameHub,
    eye_in_hand_hub: RealSenseRgbdHub,
    token: str | None,
) -> None:
    request = websocket.request
    if (
        request is None
        or urlsplit(request.path).path != COLLECTION_SIGNAL_PATH
        or not authorized(websocket, token)
    ):
        await websocket.close(code=1008, reason="invalid collection client")
        return
    camera_session_id: str | None = None
    print("主机采集相机控制已连接。")
    try:
        async for raw_message in websocket:
            message: Any = None
            try:
                message = json.loads(raw_message)
                response = await collection_camera_response(
                    hub, eye_in_hand_hub, message
                )
                message_type = message.get("type")
                if message_type == "collection_camera_start":
                    camera_session_id = str(message.get("session_id"))
                elif message_type in {
                    "collection_camera_stop",
                    "collection_camera_abort",
                }:
                    camera_session_id = None
            except (
                TypeError,
                ValueError,
                RuntimeError,
                json.JSONDecodeError,
            ) as exc:
                response = {
                    "type": (
                        message.get("type", "camera_error")
                        if isinstance(message, dict)
                        else "camera_error"
                    ),
                    "id": (
                        message.get("id") if isinstance(message, dict) else None
                    ),
                    "ok": False,
                    "error": str(exc),
                }
            await websocket.send(json.dumps(response, separators=(",", ":")))
    except ConnectionClosed:
        pass
    finally:
        if camera_session_id is not None:
            try:
                await asyncio.gather(
                    asyncio.to_thread(
                        hub.abort_episode_recording, camera_session_id
                    ),
                    asyncio.to_thread(
                        eye_in_hand_hub.abort_episode_recording,
                        camera_session_id,
                    ),
                )
            except RuntimeError:
                pass
        hub.stop_if_idle()
        eye_in_hand_hub.stop_if_idle()
        print("主机采集相机控制已断开。")


async def handle_client(
    websocket: ServerConnection,
    hub: RealSenseFrameHub,
    eye_in_hand_hub: RealSenseRgbdHub,
    peers: set[RTCPeerConnection],
    token: str | None,
) -> None:
    if not canonical_path(websocket):
        await websocket.close(code=1008, reason="unsupported signaling path")
        return
    if not authorized(websocket, token):
        await websocket.close(code=1008, reason="invalid pairing token")
        return

    peer = RTCPeerConnection(configuration=RTCConfiguration(iceServers=[]))
    peers.add(peer)
    tracks: list[RealSenseVideoTrack] = []
    offer_handled = False
    # aiortc starts __connect in an unowned background task after each SDP
    # description. Keep that task so teardown can cancel and retrieve it.
    peer_connect_tasks: set[asyncio.Task[Any]] = set()
    remote = websocket.remote_address
    connected_at = time.monotonic()
    print(f"相机浏览器已连接：{remote}")

    def log_peer_event(event: str) -> None:
        elapsed_ms = (time.monotonic() - connected_at) * 1000.0
        print(f"WebRTC 时序 {remote} +{elapsed_ms:.0f}ms {event}")

    def remember_peer_connect_tasks(before: set[asyncio.Task[Any]]) -> None:
        for task in asyncio.all_tasks():
            if task in before:
                continue
            coroutine = task.get_coro()
            if getattr(coroutine, "__qualname__", "") == "RTCPeerConnection.__connect":
                peer_connect_tasks.add(task)

    @peer.on("connectionstatechange")
    async def on_connectionstatechange() -> None:
        log_peer_event(f"Peer={peer.connectionState}")
        # A closed peer is often caused by WebSocket teardown. Closing the
        # signaling socket again races SDP/ICE setup in aiortc.
        if peer.connectionState == "failed":
            await websocket.close()

    @peer.on("iceconnectionstatechange")
    def on_iceconnectionstatechange() -> None:
        log_peer_event(f"ICE={peer.iceConnectionState}")

    @peer.on("datachannel")
    def on_datachannel(channel: Any) -> None:
        if channel.label != "depth":
            return

        async def handle_camera_message(raw_message: Any) -> None:
            message: Any = None
            try:
                message = json.loads(raw_message)
                message_type = message.get("type")
                if message_type == "depth_sample":
                    response = hub.depth_sample(
                        message.get("x"), message.get("y"), message.get("radius", 2)
                    )
                elif message_type == "camera_release_for_calibration":
                    # This is intentionally an explicit release, unlike the
                    # reconnect grace period used for ordinary browser reloads.
                    await hub.release_for_exclusive_use()
                    await eye_in_hand_hub.release_for_exclusive_use()
                    # Librealsense can retain UVC descriptors after pipeline.stop().
                    # Exit this process so the kernel closes every descriptor; the
                    # supervisor starts a fresh, idle server before calibration.
                    response = {
                        "type": "camera_release_for_calibration",
                        "ok": True,
                        "clean_restart": True,
                    }
                elif message_type == "cube_perception":
                    response = hub.set_cube_perception_enabled(message.get("enabled"))
                elif message_type == "cube_detection":
                    response = hub.cube_detection()
                elif message_type == "camera_status":
                    response = hub.camera_status()
                elif message_type == "cube_config":
                    response = hub.configure_cube_detection(
                        message.get("table_polygon_px")
                    )
                elif message_type == "cube_background_capture":
                    response = hub.request_cube_background_capture(
                        message.get("frames")
                    )
                elif message_type == "cube_background_status":
                    response = hub.cube_background_status()
                else:
                    raise ValueError("unsupported camera data request")
                response["id"] = message.get("id")
            except (TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
                response = {
                    "type": message.get("type", "camera_error")
                    if isinstance(message, dict)
                    else "camera_error",
                    "id": message.get("id") if isinstance(message, dict) else None,
                    "ok": False,
                    "error": str(exc),
                }
            if channel.readyState == "open":
                channel.send(json.dumps(response, separators=(",", ":")))
            if response.get("clean_restart"):
                self_loop = asyncio.get_running_loop()
                self_loop.call_later(
                    CAMERA_CALIBRATION_RESTART_DELAY_SECONDS,
                    hub.request_clean_restart,
                )

        @channel.on("message")
        def on_camera_message(raw_message: Any) -> None:
            asyncio.create_task(handle_camera_message(raw_message))

    try:
        async for raw_message in websocket:
            try:
                message = json.loads(raw_message)
            except (json.JSONDecodeError, TypeError):
                await websocket.send(
                    json.dumps({"type": "error", "error": "bad json"})
                )
                continue

            message_type = message.get("type")
            if message_type == "close":
                break
            if message_type != "offer" or not isinstance(message.get("sdp"), str):
                await websocket.send(
                    json.dumps({"type": "error", "error": "expected offer"})
                )
                continue
            if offer_handled:
                await websocket.send(
                    json.dumps({"type": "error", "error": "offer already handled"})
                )
                continue

            try:
                offer_handled = True
                send_video = message.get("video", True) is not False
                await hub.ensure_running()
                if send_video:
                    await eye_in_hand_hub.ensure_running()
                    log_peer_event("D455/D405 采集首帧就绪")
                else:
                    log_peer_event("D455 采集就绪（仅数据通道）")
                tasks_before = set(asyncio.all_tasks())
                await peer.setRemoteDescription(
                    RTCSessionDescription(sdp=message["sdp"], type="offer")
                )
                remember_peer_connect_tasks(tasks_before)
                log_peer_event("浏览器 SDP 已接收")
                if send_video:
                    tracks = [
                        RealSenseVideoTrack(
                            hub,
                            "rgb",
                            label="D455-RGB",
                            on_first_frame=lambda label: log_peer_event(
                                f"编码器首帧={label}"
                            ),
                        ),
                        RealSenseVideoTrack(
                            hub,
                            "depth",
                            label="D455-depth",
                            on_first_frame=lambda label: log_peer_event(
                                f"编码器首帧={label}"
                            ),
                        ),
                        RealSenseVideoTrack(
                            eye_in_hand_hub,
                            "rgb",
                            eye_in_hand_hub.output_frame_rate,
                            eye_in_hand_hub.output_size,
                            label="D405-RGB",
                            on_first_frame=lambda label: log_peer_event(
                                f"编码器首帧={label}"
                            ),
                        ),
                        RealSenseVideoTrack(
                            eye_in_hand_hub,
                            "depth",
                            eye_in_hand_hub.output_frame_rate,
                            eye_in_hand_hub.output_size,
                            label="D405-depth",
                            on_first_frame=lambda label: log_peer_event(
                                f"编码器首帧={label}"
                            ),
                        ),
                    ]
                for track in tracks:
                    peer.addTrack(track)
                if tracks:
                    prefer_h264(peer)
                answer = await peer.createAnswer()
                tasks_before = set(asyncio.all_tasks())
                await peer.setLocalDescription(answer)
                remember_peer_connect_tasks(tasks_before)
                await websocket.send(
                    json.dumps(
                        {
                            "type": "answer",
                            "sdp": peer.localDescription.sdp,
                        }
                    )
                )
                log_peer_event("SDP answer sent")
            except ConnectionClosed:
                break
            except Exception as exc:
                # Browser navigation/reload may close signaling while SDP/ICE
                # negotiation is pending. That is a normal disconnect, not a
                # server error worth surfacing as an unhandled handler failure.
                try:
                    await websocket.send(
                        json.dumps({"type": "error", "error": str(exc)})
                    )
                except ConnectionClosed:
                    pass
                break
    except ConnectionClosed:
        pass
    finally:
        for track in tracks:
            track.stop()
        # Prevent aiortc's unowned __connect task from starting an ICE
        # transport after peer.close() has already closed it.
        for task in peer_connect_tasks:
            if not task.done():
                task.cancel()
        if peer_connect_tasks:
            await asyncio.gather(*peer_connect_tasks, return_exceptions=True)
        await peer.close()
        peers.discard(peer)
        hub.stop_if_idle()
        eye_in_hand_hub.stop_if_idle()
        print(f"相机浏览器已断开：{remote}")


async def run_server(args: argparse.Namespace) -> bool:
    origins = args.origins or [
        f"http://localhost:{port('console')}",
        f"http://127.0.0.1:{port('console')}",
    ]
    ssl_context = None
    if args.cert is not None:
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.minimum_version = ssl.TLSVersion.TLSv1_2
        ssl_context.load_cert_chain(args.cert, args.key)

    video_encoder = await asyncio.to_thread(warm_camera_video_encoder)
    print(f"采集视频编码器已预热：{video_encoder}")

    restart_event = asyncio.Event()
    hub = RealSenseFrameHub(
        width=args.width,
        height=args.height,
        frame_rate=args.fps,
        serial=args.serial,
        depth_preset=args.depth_preset,
        restart_event=restart_event,
        rgb_only=args.rgb_only,
    )
    eye_in_hand_hub = RealSenseRgbdHub(
        width=args.eye_in_hand_width,
        height=args.eye_in_hand_height,
        frame_rate=args.eye_in_hand_fps,
        output_frame_rate=args.eye_in_hand_output_fps,
        output_width=args.eye_in_hand_output_width,
        output_height=args.eye_in_hand_output_height,
        serial=args.eye_in_hand_serial,
        restart_event=restart_event,
    )
    peers: set[RTCPeerConnection] = set()
    try:
        async with (
            serve(
                lambda websocket: handle_client(
                    websocket, hub, eye_in_hand_hub, peers, args.token
                ),
                args.host,
                args.port,
                origins=origins,
                ssl=ssl_context,
                max_size=2 * 1024 * 1024,
                max_queue=2,
                ping_interval=10,
                ping_timeout=10,
            ),
            serve(
                lambda websocket: handle_collection_client(
                    websocket, hub, eye_in_hand_hub, args.token
                ),
                COLLECTION_SIGNAL_HOST,
                COLLECTION_SIGNAL_PORT,
                max_size=256 * 1024,
                max_queue=2,
                ping_interval=10,
                ping_timeout=10,
            ),
        ):
            scheme = "wss" if ssl_context is not None else "ws"
            print(
                f"D455 WebRTC 信令监听 "
                f"{scheme}://{args.host}:{args.port}{SIGNAL_PATH}"
            )
            print(
                "主机采集控制监听 "
                f"ws://{COLLECTION_SIGNAL_HOST}:{COLLECTION_SIGNAL_PORT}"
                f"{COLLECTION_SIGNAL_PATH}"
            )
            print(
                "相机绑定："
                f"D455={args.serial}，眼在手 D405={args.eye_in_hand_serial} "
                f"采集={args.eye_in_hand_width}x{args.eye_in_hand_height}@"
                f"{args.eye_in_hand_fps}，预览={args.eye_in_hand_output_width}x"
                f"{args.eye_in_hand_output_height}@{args.eye_in_hand_output_fps}fps"
            )
            print(
                "桌面 ROI 颜色方块感知已启用；深度流"
                + ("已禁用。" if args.rgb_only else f"可用时同步采集（preset={args.depth_preset}）。")
            )
            await restart_event.wait()
            print("D455 采集进程正在自动恢复。")
    finally:
        await asyncio.gather(*(peer.close() for peer in peers))
        await asyncio.gather(hub.shutdown(), eye_in_hand_hub.shutdown())
    return restart_event.is_set()


def main() -> int:
    args = parse_args()
    try:
        restart_requested = asyncio.run(run_server(args))
    except KeyboardInterrupt:
        print("\nD455 WebRTC 服务已停止。")
        return 0
    if restart_requested:
        # Exit completely so librealsense's non-CLOEXEC UVC descriptors are
        # closed by the kernel. camera_webrtc_supervisor.sh starts a clean PID.
        return CAMERA_RESTART_EXIT_CODE
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
