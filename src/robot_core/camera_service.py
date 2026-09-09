"""Independent single-owner RGB service; monitor and recording share original frames."""
import base64
import signal
import threading
import time

from .camera import RealSenseCamera
from .camera_client import socket_path
from .device_lease import DeviceLease
from .local_server import LocalServer
from .project import load_config


class CameraFeed:
    def __init__(self):
        self.frame = None
        self.preview = None
        self.depth_preview = None
        self.metadata = None
        self.error = "Camera starting"

    def dispatch(self, payload):
        if payload.get("command") == "frame":
            return self.frame
        if payload.get("command") != "monitor":
            raise ValueError("Unknown camera request")
        now = time.monotonic_ns()
        frame, preview, depth = self.frame, self.preview, self.depth_preview
        return {"camera": {"metadata": self.metadata, "error": self.error,
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
                with RealSenseCamera(options["serial"], options["width"], options["height"], options["fps"]) as camera:
                    self.metadata = camera.metadata
                    last_preview = 0
                    while not stop.is_set():
                        frame = camera.read()
                        image = frame["image"]
                        self.frame = {**{k: v for k, v in frame.items() if k not in ("image", "depth_image", "depth_scale_m")},
                                      "bgr_base64": base64.b64encode(image.tobytes()).decode("ascii")}
                        self.error = None
                        if time.monotonic() - last_preview >= .2:
                            ok, jpeg = cv2.imencode(".jpg", cv2.resize(image, (320, 240)), [cv2.IMWRITE_JPEG_QUALITY, 75])
                            if ok:
                                self.preview = (frame["host_received_monotonic_ns"], base64.b64encode(jpeg).decode("ascii"))
                            depth = frame.get("depth_image")
                            self.depth_preview = None
                            if depth is not None:
                                visual = colorize_depth(depth, frame["depth_scale_m"])
                                ok, jpeg = cv2.imencode(".jpg", cv2.resize(visual, (320, 240), interpolation=cv2.INTER_NEAREST))
                                if ok:
                                    self.depth_preview = (frame["host_received_monotonic_ns"], base64.b64encode(jpeg).decode("ascii"))
                            last_preview = time.monotonic()
            except Exception as exc:
                self.error = f"{type(exc).__name__}: {exc}"
                self.frame = self.preview = self.depth_preview = self.metadata = None
                stop.wait(2)


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
    options = load_config()["cameras"]["wrist_rgb"]
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    lease = DeviceLease("camera:" + options["serial"])
    server = None
    thread = None
    try:
        feed = CameraFeed()
        server = LocalServer(socket_path(), feed.dispatch)
        if options["enabled"]:
            thread = threading.Thread(target=feed.capture, args=(options, stop), daemon=True)
            thread.start()
        else:
            feed.error = "Wrist camera disabled"
        print(f"Camera service ready: {socket_path()}", flush=True)
        stop.wait()
    finally:
        stop.set()
        if thread:
            thread.join(timeout=5)
        if server:
            server.close()
        lease.close()


if __name__ == "__main__":
    main()
