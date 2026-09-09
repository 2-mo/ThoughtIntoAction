#!/usr/bin/env python3
"""Enumerate the configured RealSense camera without starting a video stream."""

import argparse
import os


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default=os.environ.get("QUEST_CAMERA_SERIAL", ""))
    args = parser.parse_args()
    import pyrealsense2 as rs

    context = rs.context()
    devices = list(context.query_devices())
    selected = []
    for device in devices:
        serial = device.get_info(rs.camera_info.serial_number)
        name = device.get_info(rs.camera_info.name)
        print(f"{name}: serial={serial}")
        if not args.serial or args.serial == serial:
            selected.append(device)
    if len(selected) != 1:
        print(f"Expected one configured camera; found {len(selected)}. Set cameras.wrist_rgb.serial in configs/local.toml.")
        return 1
    profiles = set()
    for sensor in selected[0].query_sensors():
        for profile in sensor.get_stream_profiles():
            if profile.stream_type() == rs.stream.color:
                video = profile.as_video_stream_profile()
                profiles.add((video.width(), video.height(), profile.fps(), str(profile.format())))
    for width, height, fps, pixel_format in sorted(profiles):
        print(f"RGB profile: {width}x{height}@{fps} {pixel_format}")
    print("Enumeration only; no pipeline started. Hand-eye calibration is not configured.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
