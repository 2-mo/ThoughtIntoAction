#!/usr/bin/env python3
"""Enumerate and benchmark D405 RGB profiles without touching the arm.

Profile listing only reads camera capabilities.  ``--benchmark`` opens D405
RGB temporarily, so the normal WebRTC D405 stream must be disconnected first.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

# Keep the executable form usable as well as ``python -m`` from ``./arx5``.
if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from embodied.calibration.hand_eye.core.camera_profiles import (
    ColorStreamProfile,
    find_device,
    format_profiles,
    list_bgr_color_profiles,
)

DEFAULT_D405_SERIAL = "230322274781"
DEFAULT_PROFILES = ("424x240@15", "1280x720@30")


def parse_profile(value: str) -> ColorStreamProfile:
    try:
        size, fps_text = value.lower().split("@", 1)
        width_text, height_text = size.split("x", 1)
        profile = ColorStreamProfile(int(width_text), int(height_text), int(fps_text))
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("档位格式应为 WIDTHxHEIGHT@FPS") from exc
    if profile.width <= 0 or profile.height <= 0 or profile.fps <= 0:
        raise argparse.ArgumentTypeError("宽、高和 FPS 都必须为正数")
    return profile


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return float(ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))])


def benchmark_profile(
    serial: str,
    requested: ColorStreamProfile,
    *,
    duration_s: float,
    preview_width: int,
) -> dict[str, Any]:
    """Measure the capture path, detector, and independent preview branch."""
    try:
        import cv2
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError("基准需要 .venv-handeye 中的 OpenCV 和 pyrealsense2") from exc

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color,
        requested.width,
        requested.height,
        rs.format.bgr8,
        requested.fps,
    )
    intervals_ms: list[float] = []
    copy_ms: list[float] = []
    detect_ms: list[float] = []
    preview_ms: list[float] = []
    jpeg_sizes: list[int] = []
    previous_frame_at: float | None = None
    profile: Any | None = None
    try:
        profile = pipeline.start(config)
        for _ in range(max(8, requested.fps)):
            pipeline.wait_for_frames(5000)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(dictionary)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            frames = pipeline.wait_for_frames(5000)
            frame_at = time.monotonic()
            color = frames.get_color_frame()
            if not color:
                continue
            if previous_frame_at is not None:
                intervals_ms.append((frame_at - previous_frame_at) * 1000.0)
            previous_frame_at = frame_at

            started = time.perf_counter()
            image = np.asanyarray(color.get_data()).copy()
            copy_ms.append((time.perf_counter() - started) * 1000.0)

            started = time.perf_counter()
            detector.detectMarkers(image)
            detect_ms.append((time.perf_counter() - started) * 1000.0)

            started = time.perf_counter()
            height, width = image.shape[:2]
            scale = min(1.0, preview_width / float(width))
            preview = image
            if scale < 1.0:
                preview = cv2.resize(
                    image,
                    (max(1, round(width * scale)), max(1, round(height * scale))),
                    interpolation=cv2.INTER_AREA,
                )
            encoded_ok, encoded = cv2.imencode(
                ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 55]
            )
            preview_ms.append((time.perf_counter() - started) * 1000.0)
            if encoded_ok:
                jpeg_sizes.append(int(encoded.size))

        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        active = ColorStreamProfile(
            int(stream.width()), int(stream.height()), int(stream.fps())
        )
        mean_interval = statistics.fmean(intervals_ms) if intervals_ms else math.nan
        return {
            "requested": asdict(requested),
            "active": asdict(active),
            "sample_count": len(copy_ms),
            "capture_fps": None if not math.isfinite(mean_interval) else 1000.0 / mean_interval,
            "frame_interval_p50_ms": percentile(intervals_ms, 0.50),
            "frame_interval_p95_ms": percentile(intervals_ms, 0.95),
            "copy_p50_ms": percentile(copy_ms, 0.50),
            "copy_p95_ms": percentile(copy_ms, 0.95),
            "aruco_detect_p50_ms": percentile(detect_ms, 0.50),
            "aruco_detect_p95_ms": percentile(detect_ms, 0.95),
            "preview_resize_jpeg_p50_ms": percentile(preview_ms, 0.50),
            "preview_resize_jpeg_p95_ms": percentile(preview_ms, 0.95),
            "preview_jpeg_mean_kib": (
                None if not jpeg_sizes else statistics.fmean(jpeg_sizes) / 1024.0
            ),
        }
    finally:
        try:
            pipeline.stop()
        except RuntimeError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description="D405 RGB 档位探测与性能基准")
    parser.add_argument("--serial", default=DEFAULT_D405_SERIAL)
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="独占打开 D405 并实测；运行前请断开网页 D405 视频",
    )
    parser.add_argument(
        "--profile",
        action="append",
        type=parse_profile,
        help="要实测的档位，可重复；默认旧档位和推荐高分档",
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--preview-width", type=int, default=640)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1.0 <= args.duration <= 60.0:
        parser.error("--duration 必须在 1..60 秒")
    if args.preview_width < 160:
        parser.error("--preview-width 至少为 160")

    device = find_device(args.serial)
    available = list_bgr_color_profiles(args.serial)
    print(
        json.dumps(
            {
                "device": device.get_info(__import__("pyrealsense2").camera_info.name),
                "serial": args.serial,
                "available_bgr_profiles": format_profiles(available),
                "recommended_calibration_profile": asdict(
                    ColorStreamProfile(1280, 720, 30)
                ),
                "note": "25 FPS 不是 D405 RGB 原生档位；默认采集与预览统一使用 30 FPS。",
            },
            ensure_ascii=False,
        )
    )
    if not args.benchmark:
        print("仅完成无侵入档位探测；加 --benchmark 执行独占性能测试。")
        return 0

    requested = args.profile or [parse_profile(value) for value in DEFAULT_PROFILES]
    available_set = set(available)
    invalid = [profile.label() for profile in requested if profile not in available_set]
    if invalid:
        parser.error(
            "D405 不支持："
            + ", ".join(invalid)
            + "；可用档位："
            + format_profiles(available)
        )
    for profile in requested:
        try:
            report = benchmark_profile(
                args.serial,
                profile,
                duration_s=args.duration,
                preview_width=args.preview_width,
            )
        except RuntimeError as exc:
            raise RuntimeError(
                "无法独占启动 D405；网页视频仍可能占用相机。"
                "请关闭 D405 WebRTC 画面后重试："
                f"{exc}"
            ) from exc
        print(json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"D405 基准失败：{exc}", file=sys.stderr)
        raise SystemExit(2)
