"""One owner per configured RealSense; independent feeds shared by monitor and recording."""
import base64
from contextlib import closing
import signal
import threading
import time
import uuid

from embodied.sensors.realsense import RealSenseCamera
from embodied.sensors.client import socket_path
from embodied.contracts.device_lease import DeviceLease
from embodied.core.local_server import LocalServer
from embodied.sensors.transport import FrameServer
from embodied.core.project import load_config


class CameraFeed:
    def __init__(self, options=None, camera_id="wrist_rgb"):
        self.options = options or {}
        self.camera_id = camera_id
        self.enabled = False
        self.control_lock = threading.RLock()
        self.data_lock = threading.RLock()
        self.capture_stop = threading.Event()
        self.thread = None
        self.frame = None
        self.preview = None
        self.depth_preview = None
        self.metadata = None
        self.error = "Camera starting"

    def set_enabled(self, enabled):
        from embodied.sensors.control import recording_guard, check_legacy_session
        if type(enabled) is not bool:
            raise ValueError("Expected enabled boolean")
        with self.control_lock:
            if enabled == self.enabled:
                return self.dispatch({"command": "monitor"})
            guard = None
            if not enabled:
                check_legacy_session()
                try:
                    guard = recording_guard()
                except RuntimeError as exc:
                    raise RuntimeError("正在录制，不能关闭相机") from exc
            try:
                if enabled:
                    self.capture_stop = threading.Event()
                    self.error = "Camera starting"
                    self.enabled = True
                    self.thread = threading.Thread(target=self.capture, args=(self.options, self.capture_stop), daemon=True)
                    self.thread.start()
                else:
                    self.capture_stop.set()
                    if self.thread:
                        self.thread.join(timeout=5)
                        if self.thread.is_alive():
                            raise RuntimeError("相机仍在停止，请稍后重试")
                    self.thread = None
                    self.enabled = False
                    with self.data_lock:
                        self.frame = self.preview = self.depth_preview = self.metadata = None
                        self.error = "相机已关闭"
                return self.dispatch({"command": "monitor"})
            finally:
                if guard:
                    guard.close()

    def close(self):
        self.capture_stop.set()
        if self.thread:
            self.thread.join(timeout=5)

    def dispatch(self, payload):
        if payload.get("command") == "set_enabled":
            return self.set_enabled(payload.get("enabled"))
        if payload.get("command") == "frame":
            with self.data_lock:
                frame = self.frame
            if frame and "image" in frame:
                return {**{k: v for k, v in frame.items() if k != "image"},
                        "bgr_base64": base64.b64encode(frame["image"]).decode("ascii")}
            return frame
        if payload.get("command") != "monitor":
            raise ValueError("Unknown camera request")
        with self.data_lock:
            return self.snapshot()

    def snapshot(self):
        now = time.monotonic_ns()
        frame, preview, depth = self.frame, self.preview, self.depth_preview
        imu = None if frame is None else frame.get("imu")
        if imu:
            imu = {**imu, **{key: {**imu[key], "age_s": (now - imu[key]["host_received_monotonic_ns"]) / 1e9}
                            for key in ("accel", "gyro") if imu.get(key)}}
        return {"imu": imu,
                "camera": {"id": self.camera_id, "label": self.options.get("label", self.camera_id),
                           "serial": self.options.get("serial", ""), "model": self.options.get("model", ""),
                           "preview_fps": 5, "metadata": self.metadata, "error": self.error,
                           "enabled": self.enabled, "switch_protocol": 1,
                           "age_s": None if frame is None else (now - frame["host_received_monotonic_ns"]) / 1e9},
                "depth_preview": {"jpeg_base64": None if depth is None else depth[1],
                                  "age_s": None if depth is None else (now - depth[0]) / 1e9,
                                  "range_m": [0.2, 2.0], "aligned_to_rgb": False},
                "preview": {"jpeg_base64": None if preview is None else preview[1],
                            "age_s": None if preview is None else (now - preview[0]) / 1e9}}

    def capture(self, options, stop):
        while not stop.is_set():
            try:
                import cv2
                with closing(DeviceLease("camera:" + options["serial"])), RealSenseCamera(
                        options["serial"], options["width"], options["height"], options["fps"],
                        expected_model=options.get("model"), color_options=options.get("color", {})) as camera:
                    # A reconnect starts a new stream even when device frame numbers repeat.
                    stream_id = uuid.uuid4().hex
                    metadata = {**camera.metadata, "stream_id": stream_id, "zmq_frame_protocol": 1}
                    last_preview = 0
                    preview = depth_preview = None
                    while not stop.is_set():
                        frame = camera.read()
                        image = frame["image"]
                        encoded_frame = {**{k: v for k, v in frame.items() if k not in ("depth_image", "depth_scale_m")},
                                         "stream_id": stream_id}
                        if time.monotonic() - last_preview >= .2:
                            size = (min(640, image.shape[1]), max(1, round(image.shape[0] * min(640, image.shape[1]) / image.shape[1])))
                            ok, jpeg = cv2.imencode(".jpg", cv2.resize(image, size), [cv2.IMWRITE_JPEG_QUALITY, 75])
                            if ok:
                                preview = (frame["host_received_monotonic_ns"], base64.b64encode(jpeg).decode("ascii"))
                            depth = frame.get("depth_image")
                            depth_preview = None
                            if depth is not None:
                                visual = colorize_depth(depth, frame["depth_scale_m"])
                                ok, jpeg = cv2.imencode(".jpg", cv2.resize(visual, size, interpolation=cv2.INTER_NEAREST), [cv2.IMWRITE_JPEG_QUALITY, 75])
                                if ok:
                                    depth_preview = (frame["host_received_monotonic_ns"], base64.b64encode(jpeg).decode("ascii"))
                            last_preview = time.monotonic()
                        with self.data_lock:
                            self.frame, self.metadata, self.error = encoded_frame, metadata, None
                            self.preview, self.depth_preview = preview, depth_preview
            except Exception as exc:
                with self.data_lock:
                    self.error = f"{type(exc).__name__}: {exc}"
                    self.frame = self.preview = self.depth_preview = self.metadata = None
                stop.wait(2)


class CameraService:
    """Route named feeds; requests without an ID retain the wrist recording contract."""
    def __init__(self, cameras, color_options=None):
        if "wrist_rgb" not in cameras:
            raise ValueError("cameras.wrist_rgb is required for the recording channel")
        serials = set()
        self.feeds = {}
        for camera_id, supplied in cameras.items():
            options = {"enabled": False, "driver": "realsense", "serial": "",
                       "width": 640, "height": 480, "fps": 30, **supplied}
            options["color"] = {**(color_options or {}), **supplied.get("color", {})}
            if options["driver"] != "realsense":
                raise ValueError(f"Unsupported camera driver: {camera_id}")
            serial = options["serial"]
            if not isinstance(serial, str) or type(options["enabled"]) is not bool:
                raise ValueError(f"Invalid camera configuration: {camera_id}")
            if serial and serial in serials:
                raise ValueError(f"Duplicate camera serial: {serial}")
            if serial:
                serials.add(serial)
            for key in ("width", "height", "fps"):
                if type(options[key]) is not int or options[key] <= 0:
                    raise ValueError(f"Invalid {camera_id}.{key}")
            self.feeds[camera_id] = CameraFeed(options, camera_id)

    def start(self):
        for feed in self.feeds.values():
            if feed.options["enabled"]:
                feed.set_enabled(True)
            else:
                feed.error = "相机已关闭"

    def close(self):
        for feed in self.feeds.values():
            feed.capture_stop.set()
        for feed in self.feeds.values():
            feed.close()

    def dispatch(self, payload):
        camera_id = payload.get("camera_id", "wrist_rgb")
        if not isinstance(camera_id, str) or camera_id not in self.feeds:
            raise ValueError(f"Unknown camera: {camera_id}")
        if payload.get("command") == "monitor" and "camera_id" not in payload:
            cameras = {key: feed.dispatch(payload) for key, feed in self.feeds.items()}
            return {**cameras["wrist_rgb"], "cameras": cameras, "default_camera_id": "wrist_rgb"}
        return self.feeds[camera_id].dispatch(payload)


def colorize_depth(depth, scale):
    """Fixed metric scale across frames; invalid pixels are black."""
    import cv2
    import numpy as np
    meters = depth.astype(np.float32) * scale
    levels = (np.clip((meters - .2) / 1.8, 0., 1.) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(levels, cv2.COLORMAP_TURBO)
    colored[depth == 0] = 0
    return colored


def main():
    import argparse
    argparse.ArgumentParser(description=__doc__).parse_args()
    config = load_config()
    stopping = False
    def stop_requested(*_):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop_requested)
    server = frames = None
    try:
        feed = CameraService(config["cameras"], config.get("camera_service", {}).get("color", {}))
        server = LocalServer(socket_path(), feed.dispatch)
        frames = FrameServer(socket_path(), feed.feeds)
        feed.start()
        print(f"Camera service ready: {socket_path()}", flush=True)
        # Keep the signal handler free of locks (Event.set can re-enter a held lock).
        while not stopping:
            if frames.error:
                raise RuntimeError('Camera frame transport failed') from frames.error
            time.sleep(.2)
    finally:
        if "feed" in locals():
            feed.close()
        if frames:
            frames.close()
        if server:
            server.close()


if __name__ == "__main__":
    main()
