"""Independent MuJoCo feedback viewer and offline model snapshot."""

from pathlib import Path
import signal
import struct
import time
import zlib
import mujoco
import numpy as np
from embodied.devices.robots.arx5.kinematics import X5Model
from embodied.control.settings import load_settings
from embodied.control.ipc.services import create_node, open_service, make_subscriber, drain_latest
from embodied.control.ipc.types import JointVector15, ParallelVector2


def setup_camera(camera):
    camera.lookat[:] = [.03, 0, .22]
    camera.distance = 1.15
    camera.azimuth = 135
    camera.elevation = -22


def run(prefix: str, config_path: str | None):
    import mujoco.viewer
    config = load_settings(config_path).robot
    x5 = X5Model(config)
    data = mujoco.MjData(x5.model)
    data.qpos[:6] = config.park_q
    data.qpos[6:] = config.gripper_max_width / 2
    node = create_node()
    q_sub = make_subscriber(open_service(node, f"{prefix}/robot/arm/states/joint_position", JointVector15))
    width_sub = make_subscriber(open_service(node, f"{prefix}/robot/gripper/states/parallel_position", ParallelVector2))
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with mujoco.viewer.launch_passive(x5.model, data, show_left_ui=False, show_right_ui=False) as viewer:
        setup_camera(viewer.cam)
        while viewer.is_running() and not stopped:
            q, width = drain_latest(q_sub), drain_latest(width_sub)
            with viewer.lock():
                if q is not None and q.len == 6:
                    data.qpos[:6] = list(q.values)[:6]
                if width is not None and width.len == 1:
                    data.qpos[6:] = width.values[0] / 2
                mujoco.mj_forward(x5.model, data)
            viewer.sync()
            time.sleep(1 / 60)


def snapshot(path: Path, config_path: str | None):
    config = load_settings(config_path).robot
    x5 = X5Model(config)
    data = mujoco.MjData(x5.model)
    data.qpos[:6] = config.park_q
    data.qpos[6:] = config.gripper_max_width / 2
    mujoco.mj_forward(x5.model, data)
    camera = mujoco.MjvCamera()
    setup_camera(camera)
    x5.model.vis.global_.offwidth = 960
    x5.model.vis.global_.offheight = 720
    with mujoco.Renderer(x5.model, height=720, width=960) as renderer:
        renderer.update_scene(data, camera)
        pixels = renderer.render()
    # Standard RGB PNG encoding avoids an extra image-processing dependency.
    height, width = pixels.shape[:2]
    def chunk(kind, contents):
        return struct.pack(">I", len(contents)) + kind + contents + struct.pack(">I", zlib.crc32(kind + contents))
    scanlines = b"".join(b"\0" + row.tobytes() for row in pixels)
    encoded = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    encoded += chunk(b"IDAT", zlib.compress(scanlines)) + chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)
    return path


class Viewer:
    run = staticmethod(run)
    snapshot = staticmethod(snapshot)
