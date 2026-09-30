#!/usr/bin/env python3
"""HTTP(S) server for the ARX5 console and its host-side camera stream."""

from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import selectors
import shutil
import ssl
import subprocess
from urllib.parse import parse_qs, urlsplit


DEFAULT_RGB_CAMERA_PATTERN = (
    "/dev/v4l/by-id/"
    "usb-Intel_R__RealSense_TM__Depth_Camera_455*video-index0"
)
DEFAULT_DEPTH_CAMERA_PATTERN = (
    "/dev/v4l/by-path/*-usb-*:1.0-video-index0"
)
RGB_STREAM_PATH = "/camera/rgb.mjpeg"
DEPTH_STREAM_PATH = "/camera/depth.mjpeg"
LEGACY_CAMERA_STREAM_PATH = "/camera/stream.mjpeg"
CAMERA_BOUNDARY = "arx5frame"
CAMERA_START_TIMEOUT_SECONDS = 8
CAMERA_IDLE_TIMEOUT_SECONDS = 5
# Console sources that change during development. Large immutable assets
# (.task models, .wasm) stay freely cacheable.
REVALIDATED_SUFFIXES = frozenset({".js", ".mjs", ".css", ".html", ".svg", ".json"})
DEPTH_COLOR_FILTER = (
    "bilateral=sigmaS=1:sigmaR=0.003,"
    "tmedian=radius=1,"
    "lut=y='clip((val-200)*maxval/3800,0,maxval)',"
    "pseudocolor=preset=turbo"
)


def discover_rgb_camera_device(requested: Path | None = None) -> Path:
    """Return the D455 RGB V4L2 node, preferring its stable by-id link."""
    if requested is not None:
        return requested
    matches = sorted(
        Path("/dev/v4l/by-id").glob(Path(DEFAULT_RGB_CAMERA_PATTERN).name)
    )
    if matches:
        return matches[0]
    return Path("/dev/video4")


def discover_depth_camera_device(requested: Path | None = None) -> Path:
    """Return the D455 Z16 depth node, preferring its stable USB interface link."""
    if requested is not None:
        return requested
    matches = sorted(
        path
        for path in Path("/dev/v4l/by-path").glob(
            Path(DEFAULT_DEPTH_CAMERA_PATTERN).name
        )
        if "usbv" not in path.name
    )
    if matches:
        return matches[0]
    return Path("/dev/video0")


def ffmpeg_camera_command(
    device: Path,
    *,
    input_format: str,
    width: int,
    height: int,
    frame_rate: int,
    video_filter: str | None = None,
) -> list[str]:
    command = [
        "ffmpeg",
        "-hide_banner",
        "-nostdin",
        "-nostats",
        "-loglevel",
        "error",
        "-f",
        "v4l2",
        "-input_format",
        input_format,
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(frame_rate),
        "-i",
        str(device),
        "-an",
    ]
    if video_filter is not None:
        command.extend(["-vf", video_filter])
    command.extend(
        [
            "-c:v",
            "mjpeg",
            "-q:v",
            "5",
            "-f",
            "mpjpeg",
            "-boundary_tag",
            CAMERA_BOUNDARY,
            "pipe:1",
        ]
    )
    return command


class ConsoleHandler(SimpleHTTPRequestHandler):
    ca_certificate: Path | None = None
    pairing_token: str | None = None
    rgb_camera_device: Path
    depth_camera_device: Path
    camera_width: int
    camera_height: int
    camera_frame_rate: int
    spa_fallback: bool = False

    def log_request(self, code: int | str = "-", size: int | str = "-") -> None:
        """Log requests without query strings, which can contain pairing tokens."""
        parts = self.requestline.split(" ", 2)
        if len(parts) == 3:
            parts[1] = urlsplit(parts[1]).path
        self.log_message('"%s" %s %s', " ".join(parts), str(code), str(size))

    def end_headers(self) -> None:
        """Force revalidation of console sources so edits land on reload.

        This is what replaces hand-maintained ``?v=N`` query strings: the
        browser still caches, but must revalidate against ``Last-Modified``
        before reusing an entry. Versioned URLs additionally risked loading a
        module twice under two URLs, since ES module identity includes the
        query string.
        """
        path = urlsplit(self.path).path
        if path != "/arx5-local-ca.crt":
            if path in {"", "/", "/index.html"}:
                self.send_header("Cache-Control", "no-store")
            elif Path(path).suffix in REVALIDATED_SUFFIXES:
                self.send_header("Cache-Control", "no-cache, must-revalidate")
        super().end_headers()

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path in {RGB_STREAM_PATH, DEPTH_STREAM_PATH, LEGACY_CAMERA_STREAM_PATH}:
            if not self.camera_authorized():
                self.send_error(403, "invalid pairing token")
                return
            if path == DEPTH_STREAM_PATH:
                self.serve_camera_stream(
                    device=self.depth_camera_device,
                    input_format="gray16le",
                    video_filter=DEPTH_COLOR_FILTER,
                    display_name="D455 depth",
                )
            else:
                self.serve_camera_stream(
                    device=self.rgb_camera_device,
                    input_format="yuyv422",
                    video_filter=None,
                    display_name="D455 RGB",
                )
            return
        if path == "/arx5-local-ca.crt" and self.ca_certificate is not None:
            data = self.ca_certificate.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/x-x509-ca-cert")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
            return
        if self.spa_fallback and path != "/" and not Path(path).suffix:
            requested = Path(self.directory) / path.lstrip("/")
            if not requested.exists():
                self.path = "/index.html"
        super().do_GET()

    def camera_authorized(self) -> bool:
        if self.pairing_token is None:
            return True
        query = parse_qs(urlsplit(self.path).query)
        return query.get("token") == [self.pairing_token]

    def serve_camera_stream(
        self,
        *,
        device: Path,
        input_format: str,
        video_filter: str | None,
        display_name: str,
    ) -> None:
        if shutil.which("ffmpeg") is None:
            self.send_error(503, "ffmpeg is not installed")
            return
        if not device.exists():
            self.send_error(503, f"camera not found: {device}")
            return

        command = ffmpeg_camera_command(
            device,
            input_format=input_format,
            width=self.camera_width,
            height=self.camera_height,
            frame_rate=self.camera_frame_rate,
            video_filter=video_filter,
        )
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=None,
                env={**os.environ, "AV_LOG_FORCE_NOCOLOR": "1"},
            )
        except OSError as exc:
            self.send_error(503, f"could not start camera: {exc}")
            return

        selector = selectors.DefaultSelector()
        try:
            assert process.stdout is not None
            selector.register(process.stdout, selectors.EVENT_READ)
            if not selector.select(CAMERA_START_TIMEOUT_SECONDS):
                self.send_error(503, f"{display_name} opened but produced no frame")
                return
            first_chunk = process.stdout.read1(64 * 1024)
            if not first_chunk:
                self.send_error(503, f"{display_name} exited before its first frame")
                return

            self.send_response(200)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={CAMERA_BOUNDARY}",
            )
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            self.wfile.write(first_chunk)
            self.wfile.flush()

            while selector.select(CAMERA_IDLE_TIMEOUT_SECONDS):
                chunk = process.stdout.read1(64 * 1024)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ssl.SSLEOFError):
            pass
        finally:
            selector.close()
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARX5 网页与 D455 摄像头服务")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--ca", type=Path)
    parser.add_argument("--token")
    parser.add_argument(
        "--camera",
        "--rgb-camera",
        dest="rgb_camera",
        type=Path,
        default=Path(
            os.environ.get("ARX5_RGB_CAMERA_DEVICE")
            or os.environ["ARX5_CAMERA_DEVICE"]
        )
        if os.environ.get("ARX5_RGB_CAMERA_DEVICE")
        or os.environ.get("ARX5_CAMERA_DEVICE")
        else None,
    )
    parser.add_argument(
        "--depth-camera",
        type=Path,
        default=Path(os.environ["ARX5_DEPTH_CAMERA_DEVICE"])
        if os.environ.get("ARX5_DEPTH_CAMERA_DEVICE")
        else None,
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=15)
    parser.add_argument(
        "--spa",
        action="store_true",
        help="serve index.html for extensionless frontend routes",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    tls_paths = (args.cert, args.key, args.ca)
    if any(tls_paths) and not all(tls_paths):
        raise SystemExit("--cert、--key 和 --ca 必须一起提供")
    required_paths = (args.directory, *(path for path in tls_paths if path is not None))
    for path in required_paths:
        if not path.exists():
            raise SystemExit(f"路径不存在：{path}")
    if args.camera_width <= 0 or args.camera_height <= 0 or args.camera_fps <= 0:
        raise SystemExit("摄像头宽、高和帧率必须为正整数")

    ConsoleHandler.ca_certificate = args.ca
    ConsoleHandler.pairing_token = args.token
    ConsoleHandler.rgb_camera_device = discover_rgb_camera_device(args.rgb_camera)
    ConsoleHandler.depth_camera_device = discover_depth_camera_device(args.depth_camera)
    ConsoleHandler.camera_width = args.camera_width
    ConsoleHandler.camera_height = args.camera_height
    ConsoleHandler.camera_frame_rate = args.camera_fps
    ConsoleHandler.spa_fallback = args.spa
    handler = partial(ConsoleHandler, directory=str(args.directory))
    server = ThreadingHTTPServer((args.host, args.port), handler)
    scheme = "http"
    if args.cert is not None and args.key is not None:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(args.cert, args.key)
        server.socket = context.wrap_socket(server.socket, server_side=True)
        scheme = "https"
    print(f"网页监听 {scheme}://{args.host}:{args.port}")
    print(f"D455 RGB：{ConsoleHandler.rgb_camera_device}")
    print(f"D455 depth：{ConsoleHandler.depth_camera_device}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
