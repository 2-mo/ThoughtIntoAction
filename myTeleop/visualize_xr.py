#!/usr/bin/env python3
"""Read-only browser visualizer for the Quest right controller."""

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import xrobotoolkit_sdk as xrt


HTML_PATH = Path(__file__).with_name("xr_visualizer.html")


def normalize_quaternion(values: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in values]


def quaternion_multiply(left: list[float], right: list[float]) -> list[float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


class XRState:
    def __init__(self, sample_interval: float) -> None:
        self.sample_interval = sample_interval
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.origin_position: Optional[list[float]] = None
        self.origin_quaternion: Optional[list[float]] = None
        self.latest: dict[str, object] = {
            "valid": False,
            "fresh": False,
            "timestamp_ns": 0,
            "pose": [0.0] * 7,
            "relative_position": [0.0] * 3,
            "relative_quaternion": [0.0, 0.0, 0.0, 1.0],
            "grip": 0.0,
            "trigger": 0.0,
            "stream_hz": 0.0,
            "age_ms": None,
        }
        self.last_timestamp = 0illen
        self.last_change_time = 0.0
        self.stream_hz = 0.0

    def reset_origin(self) -> None:
        with self.lock:
            pose = self.latest["pose"]
            if self.latest["valid"] and isinstance(pose, list):
                self.origin_position = [float(value) for value in pose[:3]]
                self.origin_quaternion = normalize_quaternion(
                    [float(value) for value in pose[3:7]]
                )

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            snapshot = dict(self.latest)
            if self.last_change_time:
                age_ms = (time.monotonic() - self.last_change_time) * 1000.0
                snapshot["age_ms"] = age_ms
                snapshot["fresh"] = age_ms < 250.0
            return snapshot

    def sample_loop(self) -> None:
        while not self.stop_event.is_set():
            started = time.monotonic()
            timestamp = int(xrt.get_time_stamp_ns())
            pose = [float(value) for value in xrt.get_right_controller_pose()]
            grip = float(xrt.get_right_grip())
            trigger = float(xrt.get_right_trigger())
            valid = (
                timestamp > 0
                and len(pose) == 7
                and all(math.isfinite(value) for value in pose)
            )

            with self.lock:
                if timestamp > 0 and timestamp != self.last_timestamp:
                    if self.last_timestamp > 0:
                        dt = (timestamp - self.last_timestamp) / 1e9
                        if 0.001 <= dt <= 1.0:
                            instant_hz = 1.0 / dt
                            self.stream_hz = (
                                instant_hz
                                if self.stream_hz == 0.0
                                else 0.9 * self.stream_hz + 0.1 * instant_hz
                            )
                    self.last_timestamp = timestamp
                    self.last_change_time = time.monotonic()

                relative_position = [0.0] * 3
                relative_quaternion = [0.0, 0.0, 0.0, 1.0]
                if valid:
                    position = pose[:3]
                    quaternion = normalize_quaternion(pose[3:7])
                    if self.origin_position is None or self.origin_quaternion is None:
                        self.origin_position = position.copy()
                        self.origin_quaternion = quaternion.copy()
                    relative_position = [
                        position[index] - self.origin_position[index]
                        for index in range(3)
                    ]
                    origin_inverse = [
                        -self.origin_quaternion[0],
                        -self.origin_quaternion[1],
                        -self.origin_quaternion[2],
                        self.origin_quaternion[3],
                    ]
                    relative_quaternion = normalize_quaternion(
                        quaternion_multiply(origin_inverse, quaternion)
                    )

                self.latest = {
                    "valid": valid,
                    "fresh": False,
                    "timestamp_ns": timestamp,
                    "pose": pose,
                    "relative_position": relative_position,
                    "relative_quaternion": relative_quaternion,
                    "grip": grip,
                    "trigger": trigger,
                    "stream_hz": self.stream_hz,
                    "age_ms": None,
                }

            elapsed = time.monotonic() - started
            self.stop_event.wait(max(0.0, self.sample_interval - elapsed))


def make_handler(state: XRState, html: bytes):
    class Handler(BaseHTTPRequestHandler):
        def send_bytes(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/" or self.path == "/index.html":
                self.send_bytes(200, "text/html; charset=utf-8", html)
            elif self.path == "/api/state":
                body = json.dumps(state.snapshot(), separators=(",", ":")).encode()
                self.send_bytes(200, "application/json", body)
            else:
                self.send_bytes(404, "text/plain; charset=utf-8", b"Not found\n")

        def do_POST(self) -> None:  # noqa: N802
            if self.path == "/api/reset":
                state.reset_origin()
                self.send_bytes(204, "text/plain", b"")
            else:
                self.send_bytes(404, "text/plain; charset=utf-8", b"Not found\n")

        def log_message(self, _format: str, *_args: object) -> None:
            return

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only web visualizer for Quest controller data."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--sample-rate", type=float, default=100.0)
    args = parser.parse_args()
    if args.sample_rate <= 0:
        parser.error("--sample-rate must be positive")

    html = HTML_PATH.read_bytes()
    state = XRState(sample_interval=1.0 / args.sample_rate)
    xrt.init()
    sampler = threading.Thread(target=state.sample_loop, name="xr-sampler")
    sampler.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state, html))
    print("XR read-only visualizer started (no CAN or robot commands).")
    print(f"Open: http://{args.host}:{args.port}")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        print("\nStopping XR visualizer...")
    finally:
        server.shutdown()
        server.server_close()
        state.stop_event.set()
        sampler.join(timeout=2.0)
        xrt.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
