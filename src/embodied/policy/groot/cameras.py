from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Any

import numpy as np
import pyrealsense2 as rs

from embodied.data.groot.config import load_collection_config


DEFAULT_D455_SERIAL = "239222300444"
DEFAULT_D405_SERIAL = "230322274781"
MAXIMUM_SKEW_NS = 50_000_000
MAXIMUM_AGE_NS = 200_000_000
AUTO_WARMUP_FRAMES = 30


class CameraError(RuntimeError):
    """Raised when both current RGB observations are not trustworthy."""


@dataclass(frozen=True)
class CameraSpec:
    name: str
    serial: str
    width: int
    height: int
    fps: int
    exposure_us: int
    gain: int
    white_balance_k: int


@dataclass(frozen=True)
class RgbFrame:
    image: np.ndarray
    timestamp_ns: int
    device_timestamp_ms: float
    number: int

    def __post_init__(self) -> None:
        image = np.asarray(self.image)
        if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
            raise ValueError("RGB frame must be an HxWx3 uint8 array")
        if self.timestamp_ns < 1 or self.number < 0:
            raise ValueError("RGB frame timestamp or number is invalid")
        if not math.isfinite(self.device_timestamp_ms):
            raise ValueError("RGB device timestamp is invalid")
        copied = np.ascontiguousarray(image).copy()
        copied.setflags(write=False)
        object.__setattr__(self, "image", copied)


@dataclass(frozen=True)
class SynchronizedFrames:
    exterior: RgbFrame | None
    wrist: RgbFrame


class DualFrameBuffer:
    def __init__(self, capacity: int = 12) -> None:
        if capacity < 2:
            raise ValueError("frame buffer capacity must be at least two")
        self._frames: dict[str, deque[RgbFrame]] = {
            "exterior": deque(maxlen=capacity),
            "wrist": deque(maxlen=capacity),
        }
        self._condition = threading.Condition()

    def add(self, name: str, frame: RgbFrame) -> None:
        if name not in self._frames:
            raise ValueError(f"unknown camera name: {name}")
        with self._condition:
            frames = self._frames[name]
            if frames and frame.number == frames[-1].number:
                return
            frames.append(frame)
            self._condition.notify_all()

    def ready(self, names: tuple[str, ...] = ("exterior", "wrist")) -> bool:
        with self._condition:
            return all(self._frames[name] for name in names)

    def wait_ready(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not all(self._frames.values()):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def latest_pair(self, now_ns: int | None = None) -> SynchronizedFrames:
        now = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._condition:
            if not all(self._frames.values()):
                raise CameraError("双相机尚未产生 RGB 帧")
            exterior = tuple(self._frames["exterior"])
            wrist = tuple(self._frames["wrist"])
        common_time = min(exterior[-1].timestamp_ns, wrist[-1].timestamp_ns)
        ext = min(exterior, key=lambda frame: abs(frame.timestamp_ns - common_time))
        hand = min(wrist, key=lambda frame: abs(frame.timestamp_ns - common_time))
        skew_ns = abs(ext.timestamp_ns - hand.timestamp_ns)
        if skew_ns > MAXIMUM_SKEW_NS:
            raise CameraError(f"双相机帧偏差 {skew_ns / 1e6:.1f} ms 超过 50 ms")
        age_ns = now - min(ext.timestamp_ns, hand.timestamp_ns)
        if age_ns < 0 or age_ns > MAXIMUM_AGE_NS:
            raise CameraError(f"双相机帧已过期：{age_ns / 1e6:.1f} ms")
        return SynchronizedFrames(ext, hand)

    def latest_frame(self, name: str, now_ns: int | None = None) -> RgbFrame:
        if name not in self._frames:
            raise ValueError(f"unknown camera name: {name}")
        now = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._condition:
            if not self._frames[name]:
                raise CameraError(f"{name} 相机尚未产生 RGB 帧")
            frame = self._frames[name][-1]
        age_ns = now - frame.timestamp_ns
        if age_ns < 0 or age_ns > MAXIMUM_AGE_NS:
            raise CameraError(f"{name} 相机帧已过期：{age_ns / 1e6:.1f} ms")
        return frame


def default_camera_specs() -> tuple[CameraSpec, CameraSpec]:
    config = load_collection_config()
    exterior = config.camera
    wrist = config.wrist_camera
    return (
        CameraSpec(
            "exterior",
            DEFAULT_D455_SERIAL,
            exterior.width,
            exterior.height,
            exterior.fps,
            exterior.exposure_us,
            exterior.gain,
            exterior.white_balance_k,
        ),
        CameraSpec(
            "wrist",
            DEFAULT_D405_SERIAL,
            wrist.width,
            wrist.height,
            wrist.fps,
            wrist.exposure_us,
            wrist.gain,
            wrist.white_balance_k,
        ),
    )


def _find_color_sensor(profile: Any) -> Any:
    for sensor in profile.get_device().query_sensors():
        for stream_profile in sensor.get_stream_profiles():
            if stream_profile.stream_type() == rs.stream.color:
                return sensor
    raise CameraError("RealSense color sensor was not found")


def _set_supported(sensor: Any, option: Any, value: float, name: str) -> None:
    if option is None or not sensor.supports(option):
        raise CameraError(f"RealSense color sensor does not support {name}")
    sensor.set_option(option, float(value))


def _enable_auto_controls(profile: Any) -> None:
    sensor = _find_color_sensor(profile)
    _set_supported(
        sensor,
        getattr(rs.option, "enable_auto_exposure", None),
        1.0,
        "auto exposure",
    )
    _set_supported(
        sensor,
        getattr(rs.option, "enable_auto_white_balance", None),
        1.0,
        "auto white balance",
    )
    priority = getattr(rs.option, "auto_exposure_priority", None)
    if priority is not None and sensor.supports(priority):
        sensor.set_option(priority, 0.0)


def _lock_fixed_controls(profile: Any, spec: CameraSpec) -> None:
    sensor = _find_color_sensor(profile)
    _set_supported(
        sensor,
        getattr(rs.option, "enable_auto_exposure", None),
        0.0,
        "auto exposure",
    )
    exposure = getattr(rs.option, "exposure", None)
    if exposure is None or not sensor.supports(exposure):
        raise CameraError("RealSense color sensor does not support exposure")
    exposure_range = sensor.get_option_range(exposure)
    unit_us = 100.0 if float(exposure_range.max) <= 10_000.0 else 1.0
    requested = float(spec.exposure_us) / unit_us
    step = max(float(exposure_range.step), 1.0)
    value = float(exposure_range.min) + round(
        (requested - float(exposure_range.min)) / step
    ) * step
    value = min(float(exposure_range.max), max(float(exposure_range.min), value))
    sensor.set_option(exposure, value)
    _set_supported(sensor, getattr(rs.option, "gain", None), spec.gain, "gain")
    _set_supported(
        sensor,
        getattr(rs.option, "enable_auto_white_balance", None),
        0.0,
        "auto white balance",
    )
    _set_supported(
        sensor,
        getattr(rs.option, "white_balance", None),
        spec.white_balance_k,
        "white balance",
    )


def _verify_profile(profile: Any, spec: CameraSpec) -> None:
    video = profile.get_stream(rs.stream.color).as_video_stream_profile()
    actual = (video.width(), video.height(), video.fps(), video.format())
    expected = (spec.width, spec.height, spec.fps, rs.format.rgb8)
    if actual != expected:
        raise CameraError(
            f"{spec.name} resolved RGB profile {actual} does not match {expected}"
        )


class DualRgbCamera:
    def __init__(self, specs: tuple[CameraSpec, ...] | None = None) -> None:
        self.specs = default_camera_specs() if specs is None else specs
        names = tuple(spec.name for spec in self.specs)
        if names not in (("exterior", "wrist"), ("wrist",)):
            raise ValueError("camera specs must contain exterior and wrist, or wrist only")
        self._names = names
        self.frames = DualFrameBuffer()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._errors: dict[str, BaseException] = {}
        self._threads: list[threading.Thread] = []

    def start(self, timeout_s: float = 10.0) -> None:
        if self._threads:
            raise CameraError("dual RGB camera is already started")
        for spec in self.specs:
            thread = threading.Thread(
                target=self._capture,
                args=(spec,),
                name=f"arx5-gr00t-{spec.name}-rgb",
                daemon=True,
            )
            self._threads.append(thread)
            thread.start()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            failure: tuple[str, BaseException] | None = None
            with self._lock:
                if self._errors:
                    failure = next(iter(self._errors.items()))
            if failure is not None:
                name, error = failure
                self.close()
                raise CameraError(f"{name} camera failed: {error}") from error
            if self.frames.ready(self._names):
                self.latest()
                return
            time.sleep(0.02)
        self.close()
        raise CameraError("RGB 相机启动超时")

    def _capture(self, spec: CameraSpec) -> None:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(spec.serial)
        config.enable_stream(
            rs.stream.color,
            spec.width,
            spec.height,
            rs.format.rgb8,
            spec.fps,
        )
        try:
            profile = pipeline.start(config)
            _verify_profile(profile, spec)
            _enable_auto_controls(profile)
            for _ in range(AUTO_WARMUP_FRAMES):
                if self._stop.is_set():
                    return
                pipeline.wait_for_frames(2000)
            _lock_fixed_controls(profile, spec)
            while not self._stop.is_set():
                frames = pipeline.wait_for_frames(2000)
                color = frames.get_color_frame()
                if not color:
                    continue
                self.frames.add(
                    spec.name,
                    RgbFrame(
                        np.asanyarray(color.get_data()),
                        time.monotonic_ns(),
                        float(color.get_timestamp()),
                        int(color.get_frame_number()),
                    ),
                )
        except BaseException as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._errors[spec.name] = exc
        finally:
            try:
                pipeline.stop()
            except RuntimeError:
                pass

    def latest(self) -> SynchronizedFrames:
        with self._lock:
            if self._errors:
                name, error = next(iter(self._errors.items()))
                raise CameraError(f"{name} camera failed: {error}") from error
        if self._names == ("wrist",):
            return SynchronizedFrames(None, self.frames.latest_frame("wrist"))
        return self.frames.latest_pair()

    def close(self) -> None:
        self._stop.set()
        current = threading.current_thread()
        alive: list[str] = []
        for thread in self._threads:
            if thread is not current:
                thread.join(3.0)
                if thread.is_alive():
                    alive.append(thread.name)
        self._threads.clear()
        if alive:
            raise CameraError(f"camera threads did not stop: {', '.join(alive)}")

    def __enter__(self) -> DualRgbCamera:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
