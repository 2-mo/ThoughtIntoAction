"""RealSense RGB profile discovery and reproducible benchmark helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, order=True)
class ColorStreamProfile:
    """A native BGR colour stream advertised by one RealSense device."""

    width: int
    height: int
    fps: int

    def label(self) -> str:
        return f"{self.width}x{self.height}@{self.fps}"


def _realsense() -> Any:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError("需要 pyrealsense2 才能枚举 RealSense 视频档位") from exc
    return rs


def find_device(serial: str | None = None) -> Any:
    """Return the selected connected RealSense device without opening a stream."""
    rs = _realsense()
    devices = list(rs.context().query_devices())
    if serial is not None:
        for device in devices:
            if device.get_info(rs.camera_info.serial_number) == str(serial):
                return device
        raise RuntimeError(f"未找到 RealSense 序列号 {serial}")
    if not devices:
        raise RuntimeError("未检测到 RealSense 相机")
    return devices[0]


def list_bgr_color_profiles(serial: str | None = None) -> tuple[ColorStreamProfile, ...]:
    """List native BGR profiles only; this does not claim a UVC stream."""
    rs = _realsense()
    device = find_device(serial)
    profiles: set[ColorStreamProfile] = set()
    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            try:
                video = profile.as_video_stream_profile()
            except RuntimeError:
                continue
            if video.stream_type() != rs.stream.color or video.format() != rs.format.bgr8:
                continue
            profiles.add(
                ColorStreamProfile(
                    width=int(video.width()),
                    height=int(video.height()),
                    fps=int(video.fps()),
                )
            )
    if not profiles:
        serial_text = "默认设备" if serial is None else str(serial)
        raise RuntimeError(f"{serial_text} 没有可用的 BGR 彩色流")
    return tuple(sorted(profiles, key=lambda value: (value.width * value.height, value.width, value.height, value.fps)))


def supports_bgr_color_profile(
    width: int,
    height: int,
    fps: int,
    *,
    serial: str | None = None,
) -> bool:
    requested = ColorStreamProfile(int(width), int(height), int(fps))
    return requested in set(list_bgr_color_profiles(serial))


def format_profiles(
    profiles: tuple[ColorStreamProfile, ...] | list[ColorStreamProfile],
) -> str:
    """Compact human-readable grouping used in errors and the benchmark CLI."""
    grouped: dict[tuple[int, int], list[int]] = {}
    for profile in profiles:
        grouped.setdefault((profile.width, profile.height), []).append(profile.fps)
    return ", ".join(
        f"{width}x{height}@{'/'.join(str(fps) for fps in sorted(set(fps_values)))}"
        for (width, height), fps_values in sorted(
            grouped.items(), key=lambda item: (item[0][0] * item[0][1], item[0])
        )
    )
