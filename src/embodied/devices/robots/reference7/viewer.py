"""Independent, read-only MuJoCo viewer for the Reference7 plugin."""
from pathlib import Path
import signal
import struct
import time
import zlib

import mujoco

from embodied.control.settings import load_settings
from .model import Model


def camera(view):
    view.lookat[:] = [0., 0., .6]
    view.distance, view.azimuth, view.elevation = 1.8, 130, -20


class Viewer:
    @staticmethod
    def run(prefix, config_path):
        import mujoco.viewer
        from embodied.control.ipc.services import create_node, open_service, make_subscriber, drain_latest
        from embodied.control.ipc.types import JointVector15
        config = load_settings(config_path).robot
        model = Model(config).model
        data = mujoco.MjData(model)
        data.qpos[:] = config.park_q
        node = create_node()
        sub = make_subscriber(open_service(node, f'{prefix}/robot/arm/states/joint_position', JointVector15))
        stopped = False
        def stop(*_):
            nonlocal stopped
            stopped = True
        previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                camera(viewer.cam)
                while viewer.is_running() and not stopped:
                    q = drain_latest(sub)
                    with viewer.lock():
                        if q is not None and q.len == config.joint_count:
                            data.qpos[:] = list(q.values)[:config.joint_count]
                        mujoco.mj_forward(model, data)
                    viewer.sync()
                    time.sleep(1 / 60)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)

    @staticmethod
    def snapshot(path, config_path):
        config = load_settings(config_path).robot
        model = Model(config).model
        data = mujoco.MjData(model)
        data.qpos[:] = config.park_q
        mujoco.mj_forward(model, data)
        view = mujoco.MjvCamera()
        camera(view)
        model.vis.global_.offwidth, model.vis.global_.offheight = 640, 480
        with mujoco.Renderer(model, height=480, width=640) as renderer:
            renderer.update_scene(data, camera=view)
            pixels = renderer.render()
        def chunk(kind, content):
            return struct.pack('>I', len(content)) + kind + content + struct.pack('>I', zlib.crc32(kind + content))
        encoded = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 640, 480, 8, 2, 0, 0, 0))
        encoded += chunk(b'IDAT', zlib.compress(b''.join(b'\0' + row.tobytes() for row in pixels))) + chunk(b'IEND', b'')
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(encoded)
        return path
