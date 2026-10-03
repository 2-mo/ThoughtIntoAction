"""Read-only clients of the independent RGB owner; no camera SDK imports."""
import base64
import subprocess
import time

from .project import load_config, resolve_path, PROJECT_ROOT
from .session_protocol import request
from .device_lease import DeviceLease


def socket_path():
    return resolve_path(load_config().get("camera_service", {}).get("socket", "outputs/runtime/camera.sock"))


def ensure_service():
    path = socket_path()
    try:
        request(path, "monitor", timeout=.3)
        return
    except (OSError, RuntimeError):
        pass
    # Serializes launch attempts from concurrent viewers/recorders.
    deadline = time.monotonic() + 5
    while True:
        try:
            with_launch = DeviceLease("camera-launch:" + str(path))
            break
        except RuntimeError:
            if time.monotonic() >= deadline:
                raise RuntimeError("Another camera service launch is still in progress")
            time.sleep(.1)
    try:
        try:
            request(path, "monitor", timeout=.3)
            return
        except (OSError, RuntimeError):
            pass
        log = resolve_path(load_config()["paths"]["log_dir"])
        log.mkdir(parents=True, exist_ok=True)
        with (log / "camera-service.log").open("ab", buffering=0) as output:
            subprocess.Popen([str(PROJECT_ROOT / "run.sh"), "camera-service"],
                             stdin=subprocess.DEVNULL, stdout=output, stderr=output, start_new_session=True)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                request(path, "monitor", timeout=.3)
                return
            except (OSError, RuntimeError):
                time.sleep(.1)
        raise RuntimeError("Camera service did not start; see camera-service.log")
    finally:
        with_launch.close()


class SharedCamera:
    def __init__(self, serial, width=640, height=480, fps=30, *, camera_id=None):
        self.expected = (serial, width, height, fps)
        self.arguments = {} if camera_id is None else {"camera_id": camera_id}
        self.last = None

    def __enter__(self):
        ensure_service()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            state = request(socket_path(), "monitor", timeout=1, **self.arguments)
            meta = state["camera"]["metadata"]
            if meta:
                if tuple(meta[k] for k in ("serial", "width", "height", "fps")) != self.expected:
                    raise RuntimeError("Camera service settings differ; restart camera-service with the new configuration")
                self.metadata = meta
                return self
            time.sleep(.1)
        raise RuntimeError(state["camera"].get("error") or "No camera frames")

    def read(self):
        import numpy as np
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            frame = request(socket_path(), "frame", timeout=1, **self.arguments)
            if frame and frame["host_received_monotonic_ns"] != self.last:
                if time.monotonic_ns() - frame["host_received_monotonic_ns"] > 500_000_000:
                    raise RuntimeError("Camera service frame is stale")
                self.last = frame["host_received_monotonic_ns"]
                raw = base64.b64decode(frame.pop("bgr_base64"), validate=True)
                frame["image"] = np.frombuffer(raw, dtype=np.uint8).reshape(
                    self.metadata["height"], self.metadata["width"], 3).copy()
                return frame
            time.sleep(.005)
        raise RuntimeError("Camera service stopped producing frames")

    def __exit__(self, *exc):
        pass  # Shared camera lifetime is independent of each reader.
