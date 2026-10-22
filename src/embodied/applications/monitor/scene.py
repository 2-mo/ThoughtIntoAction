"""Shared, read-only Three.js scene. Kinematics always come from the supplied URDF."""

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import IPv4Address
import hashlib
import json
import mimetypes
from pathlib import Path
import threading
import time
import uuid
from urllib.parse import parse_qs, urlsplit

import numpy as np

from embodied.core.telemetry import SCHEMA, path_data, scalar_torque

WEB_ROOT = Path(__file__).with_name("web")


def joint_values(value, count, *, optional=False):
    if value is None and optional:
        return None
    values = np.asarray(value, dtype=float)
    if values.shape != (count,) or not np.isfinite(values).all():
        raise ValueError(f'Expected {count} finite joint values')
    return values.tolist()


def gripper_feedback(value):
    if value is None:
        return None
    actual, target, maximum = value.get('actual'), value.get('target'), value.get('max')
    if type(actual) not in (int, float) or not np.isfinite(actual):
        raise ValueError('Invalid gripper feedback')
    if target is not None and (type(target) not in (int, float) or not np.isfinite(target)):
        raise ValueError('Invalid gripper target')
    if maximum is not None and (type(maximum) not in (int, float) or not np.isfinite(maximum) or maximum <= 0):
        raise ValueError('Invalid gripper range')
    return {'actual': actual, 'target': target, 'max': maximum}


class Scene:
    def __init__(self, robot, port=0, *, simulation=None, runtime_monitor=None, host='127.0.0.1'):
        import pinocchio as pin

        address = IPv4Address(host)
        if address.is_unspecified:
            raise ValueError('Monitor requires an explicit IPv4 address, not 0.0.0.0')
        host = str(address)
        self.robot = robot
        self.closed = False
        self.connection_probe = None
        self.simulation = simulation
        self.runtime_monitor = runtime_monitor
        from embodied.applications.monitor.review import ReviewStore
        self.review_store = ReviewStore(runtime_monitor) if runtime_monitor is not None else None
        self._lock = threading.RLock()
        self._state = None
        self._stream = None
        self._stream_error = None
        self._paths = {}
        self._sequence = 0
        self.history = deque(maxlen=500)
        self.set_model(robot, simulation)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.respond(body=True)

            def do_POST(self):
                if urlsplit(self.path).path not in ("/api/replay", "/api/camera", "/api/simulation", "/api/annotations/review"):
                    self.send_error(404); return
                if self.headers.get("Origin") not in (None, owner.url.rstrip("/"),
                        owner.url.rstrip("/").replace("127.0.0.1", "localhost")):
                    self.send_error(403); return
                try:
                    if owner.runtime_monitor is not None and urlsplit(self.path).path not in ('/api/camera', '/api/annotations/review'):
                        raise ValueError('统一内核监看为只读；录制请使用总控 C / collect run，旧回放控制未接入。')
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 4096: raise ValueError("Invalid request size")
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict): raise ValueError("Expected object")
                    result = {"ok": True}
                    if urlsplit(self.path).path == '/api/annotations/review':
                        if owner.review_store is None:
                            raise ValueError('标注复核只支持统一内核采集记录')
                        result['annotations'] = owner.review_store.review(data)
                    elif urlsplit(self.path).path == '/api/simulation':
                        if owner.simulation is None:
                            raise ValueError('当前页面不是仿真')
                        owner.simulation.command(data.get('action'))
                    elif urlsplit(self.path).path == "/api/camera":
                        from embodied.devices.sensors.control import set_enabled
                        set_enabled(data.get("enabled"), camera_id=data.get("camera_id"))
                    else:
                        if owner.simulation is not None:
                            raise ValueError('仿真页不操作真机采集或回放')
                        action = data.get("action")
                        if action not in ("load_last", "load", "play", "pause", "seek", "speed"):
                            raise ValueError("Only web review controls are supported")
                        from embodied.core.project import load_config, resolve_path
                        from embodied.core.session_protocol import request
                        command = "collection.replay_previous" if action == "load_last" else "collection.replay_episode" if action == "load" else "replay.control"
                        request(resolve_path(load_config()["runtime"]["socket"]), command,
                                action=action, position_s=data.get("position_s"), speed=data.get("speed"), episode_id=data.get("episode_id"), timeout=5)
                    content = json.dumps(result, allow_nan=False).encode()
                except (OSError, ValueError, RuntimeError, TypeError, ImportError) as exc:
                    content = json.dumps({"error": str(exc)}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try: self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError): pass

            def do_HEAD(self):
                self.respond(body=False)

            def respond(self, body):
                path = urlsplit(self.path).path
                status = 200
                mesh_digest = None
                if path == '/api/simulation':
                    content = json.dumps(owner.simulation.status() if owner.simulation else {}).encode()
                    mime = 'application/json'
                elif path == '/api/replay' and (owner.simulation is not None or owner.runtime_monitor is not None):
                    content, mime = b'{}', 'application/json'
                elif path == "/api/replay":
                    from embodied.core.project import load_config, resolve_path
                    from embodied.core.session_protocol import request
                    try:
                        payload = request(resolve_path(load_config()["runtime"]["socket"]),
                                          "replay.list" if urlsplit(self.path).query == "list=1" else "replay.frame", timeout=1)
                        frame = payload.get("replay")
                        if frame:
                            if frame["model_hash"] != owner.robot.metadata()["urdf_sha256"]:
                                raise ValueError("回放与网页模型不一致")
                            with owner._lock:
                                frame["pose"] = owner._pose(owner.robot.joints(frame["q"]))
                                target = frame.get("target_q")
                                frame["target_pose"] = owner._pose(owner.robot.joints(target)) if target is not None else None
                                frame["eef_velocity"] = owner._velocity(frame["q"], frame.get("actual_vel"))
                    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
                        payload = {"error": str(exc)}
                    content = json.dumps(payload, allow_nan=False).encode()
                    mime = "application/json"
                elif path == "/api/review":
                    try:
                        if owner.review_store is None:
                            raise ValueError('任务时间轴与技能复核需要统一内核监看页')
                        query = parse_qs(urlsplit(self.path).query)
                        episode_id = query.get('episode_id', [None])[0]
                        payload = {'tasks': owner.review_store.tasks()}
                        if query.get('tasks_only') != ['1']:
                            payload.update(episodes=owner.review_store.episodes(),
                                           annotations=owner.review_store.annotations(episode_id) if episode_id else None)
                    except (OSError, ValueError, TypeError, ImportError) as exc:
                        payload = {'error': str(exc)}
                    content = json.dumps(payload, allow_nan=False).encode()
                    mime = 'application/json'
                elif path == "/api/sensors":
                    from embodied.core.project import load_config, resolve_path
                    from embodied.core.session_protocol import request
                    try:
                        if owner.runtime_monitor is not None:
                            payload = owner.runtime_monitor.monitor()
                        elif owner.simulation is not None:
                            payload = owner.simulation.monitor()
                        else:
                            payload = request(resolve_path(load_config()["runtime"]["socket"]), "monitor", timeout=.3)
                        for metric in (payload.get("timing") or {}).values():
                            if metric:
                                metric["age_s"] = max(0., (time.monotonic_ns() - metric["sample_monotonic_ns"]) / 1e9)
                        limit = payload.get("joint_limit")
                        if limit:
                            limit["age_s"] = max(0., (time.monotonic_ns() - limit["sample_monotonic_ns"]) / 1e9)
                        diagnostic = payload.get("motion_diagnostics")
                        if diagnostic:
                            diagnostic["age_s"] = max(0., (time.monotonic_ns() - diagnostic["sample_monotonic_ns"]) / 1e9)
                        sample = (payload.get("input") or {}).get("sample_monotonic_ns")
                        payload["input_age_s"] = None if sample is None else max(0., (time.monotonic_ns() - sample) / 1e9)
                    except (OSError, ValueError, RuntimeError, KeyError, TypeError):
                        payload = {"error": "采集会话未连接", "input_error": "遥操会话未连接"}
                    try:
                        from embodied.devices.sensors.client import socket_path
                        camera = request(socket_path(), "monitor", timeout=.3)
                        payload.update(camera)
                        payload.pop("error", None)
                    except (OSError, ValueError, RuntimeError) as exc:
                        payload["camera_error"] = f"相机服务未连接：{exc}"
                    from embodied.applications.monitor.quality import camera_quality
                    payload['camera_quality'] = camera_quality(payload)
                    payload["connections"] = owner.connection_probe.snapshot() if owner.connection_probe else {}
                    content = json.dumps(payload, allow_nan=False).encode()
                    mime = "application/json"
                elif path in {"/api/model", "/api/state"}:
                    with owner._lock:
                        payload = owner._manifest if path == "/api/model" else owner.snapshot()
                    content = json.dumps(payload, allow_nan=False).encode()
                    mime = "application/json"
                elif (file := owner._files.get(path)) is not None:
                    try:
                        content = file.read_bytes()
                        mime = mimetypes.guess_type(file.name)[0] or "application/octet-stream"
                        if path.startswith('/meshes/'):
                            mesh_digest = hashlib.sha256(content).hexdigest()
                    except OSError as exc:
                        status = 404 if isinstance(exc, FileNotFoundError) else 503
                        content = json.dumps({'error': '模型或页面资源不可用；检查资源文件，更新后请重启网页监看服务。',
                                              'resource': path, 'detail': str(exc)}, ensure_ascii=False).encode()
                        mime = 'application/json; charset=utf-8'
                else:
                    self.send_error(404)
                    return
                try:
                    self.send_response(status)
                    self.send_header("Content-Type", mime)
                    self.send_header("Content-Length", str(len(content)))
                    if mesh_digest is not None:
                        self.send_header("X-Content-SHA256", mesh_digest)
                    # Reloads must never show an old viewer or a cached robot state.
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.end_headers()
                    if body:
                        self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # Closing/reloading a browser does not affect the host.

            def log_message(self, *args):
                pass

        # A fresh port identifies this process; another scene cannot masquerade as it.
        self._server = ThreadingHTTPServer((host, port), Handler)
        self.url = f"http://{host}:{self._server.server_port}/"
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        kwargs={"poll_interval": .05},
                                        name="robot-scene-http", daemon=True)
        try:
            self._thread.start()
        except BaseException:
            self._server.server_close()
            raise
        print(f"Scene (Three.js · {robot.urdf.name if robot else '等待运行时模型'}): {self.url}", flush=True)

    def set_model(self, robot, simulation=None):
        """Replace display state without restarting HTTP, camera or connection services."""
        import pinocchio as pin
        with self._lock:
            self.robot, self.simulation = robot, simulation
            self._state = self._stream = self._stream_error = None
            self._paths = {}
            self._sequence = 0
            self.history.clear()
            self._files = {"/" + str(p.relative_to(WEB_ROOT)): p for p in WEB_ROOT.rglob("*") if p.is_file()}
            self._files["/"] = WEB_ROOT / "index.html"
            if robot is None:
                self._manifest = {'robot': {}, 'model_file': '等待运行时模型', 'viewer_id': uuid.uuid4().hex,
                    'meshes': [], 'matrix_order': 'column-major', 'joint_limits': {'lower': [], 'upper': []},
                    'velocity_limits': [], 'torque_limits': [], 'data_contract': None, 'reference': None,
                    'runtime_monitor': self.runtime_monitor is not None}
                return
            # Own FK buffers; never mutate the controller's model data.
            self._data = robot.model.createData()
            self._visual = pin.buildGeomFromUrdf(
                robot.model, str(robot.urdf), pin.GeometryType.VISUAL,
                package_dirs=[str(robot.urdf.parent)],
            )
            self._geometry_data = pin.GeometryData(self._visual)
            meshes = []
            for index, geometry in enumerate(self._visual.geometryObjects):
                spec = {'name': geometry.name, 'link': robot.model.frames[geometry.parentFrame].name,
                        'scale': geometry.meshScale.tolist()}
                mesh = Path(geometry.meshPath).resolve()
                if mesh.is_file() and mesh.suffix.lower() == '.stl':
                    spec['url'] = f'/meshes/{index}.stl'
                    self._files[spec['url']] = mesh
                else:
                    shape = geometry.geometry
                    kind = type(shape).__name__
                    if kind == 'Box':
                        spec['primitive'] = {'kind': 'box', 'size': (2 * shape.halfSide).tolist()}
                    elif kind == 'Sphere':
                        spec['primitive'] = {'kind': 'sphere', 'radius': float(shape.radius)}
                    elif kind == 'Cylinder':
                        spec['primitive'] = {'kind': 'cylinder', 'radius': float(shape.radius), 'length': 2 * float(shape.halfLength)}
                    else:
                        raise ValueError(f'Unsupported display geometry: {kind} ({geometry.meshPath})')
                meshes.append(spec)
            if not meshes:
                raise ValueError("Robot has no visual meshes")
            self._manifest = {"robot": robot.metadata(), "model_file": robot.urdf.name,
                              "viewer_id": uuid.uuid4().hex,
                              "meshes": meshes, "matrix_order": "column-major",
                              "joint_limits": {"lower": robot.lower.tolist(), "upper": robot.upper.tolist()},
                              "data_contract": getattr(robot, "data_contract", None)}
            if simulation is not None:
                self._manifest['simulation'] = {'engine': 'arx-televr / MuJoCo', 'control_hz': 250}
            if self.runtime_monitor is not None:
                self._manifest['runtime_monitor'] = True
            from embodied.core.project import load_config
            gripper_max = load_config().get("telemetry", {}).get("gripper_torque_max_nm")
            if gripper_max is not None and (not np.isfinite(gripper_max) or gripper_max <= 0):
                raise ValueError("Invalid gripper display torque limit")
            velocity = robot.model.velocityLimit
            if simulation is not None and getattr(self, "_hardware_velocity", None) is not None:
                # The kernel URDF carries placeholder limits (1000 / 10 rad/s); chart against the real motors.
                velocity = np.minimum(velocity, self._hardware_velocity)
            elif simulation is None:
                self._hardware_velocity = velocity.copy()
            self._manifest["velocity_limits"] = velocity.tolist()
            contract = self._manifest['data_contract']
            has_gripper = contract is None or contract.get('gripper') is not None
            self._manifest["torque_limits"] = [*robot.model.effortLimit.tolist(), *([gripper_max] if has_gripper else [])]
            reference = pin.neutral(robot.model)
            self._manifest["reference"] = {"sequence": 0, "actual": self._pose(reference),
                                           "target": self._pose(reference), "target_visible": False, "paths": {}}

    def snapshot(self):
        with self._lock:
            stream = dict(self._stream or {})
            if self._stream_error:
                stream.update(status="error", message=self._stream_error)
            elif self._stream is None:
                stream.update(status="waiting", message="Waiting for robot state")
            else:
                now = time.monotonic_ns()
                age = (now - min(stream["sample_monotonic_ns"], stream["emitted_monotonic_ns"])) / 1e9
                stream.update(status="stale" if age > 1. else "live", age_s=max(0., age))
            return {**(self._state or {"actual": None, "target": None, "paths": {}}), "stream": stream,
                    "viewer_id": self._manifest['viewer_id']}

    def set_stream_error(self, message):
        with self._lock:
            self._stream_error = message

    def accept(self, packet):
        """Validate a full state snapshot before allowing it to affect the scene."""
        if not isinstance(packet, dict):
            raise ValueError("Expected a robot state object")
        if packet["schema"] != SCHEMA or packet["robot"] != self._manifest["robot"]:
            raise ValueError("Robot model/frame or state schema mismatch")
        count = self.robot.model.nq
        actual = joint_values(packet['actual'], count)
        target = joint_values(packet['target'], count)
        gripper = gripper_feedback(packet.get('gripper'))
        torque = joint_values(packet.get('actual_torque'), count, optional=True)
        grip_torque = scalar_torque(packet.get("gripper_actual_torque"))
        velocity = joint_values(packet.get("actual_vel"), count, optional=True)
        target_velocity = joint_values(packet.get("target_vel"), count, optional=True)
        grip_velocity = scalar_torque(packet.get("gripper_actual_vel"))
        stream_id, source = packet["stream_id"], packet["source"]
        if not isinstance(stream_id, str) or not stream_id or not isinstance(source, dict):
            raise ValueError("Invalid robot state source")
        sample, emitted = packet["sample_monotonic_ns"], packet["emitted_monotonic_ns"]
        sequence = packet["sequence"]
        if not all(isinstance(x, int) and x > 0 for x in (sample, emitted, sequence)):
            raise ValueError("Invalid robot state timestamps/sequence")
        if sample > emitted or emitted > time.monotonic_ns() + 500_000_000:
            raise ValueError("Robot state timestamp is not from the local monotonic clock")
        if not isinstance(packet["paths"], dict) or not all(isinstance(value, dict) for value in packet["paths"].values()):
            raise ValueError("Invalid robot state paths")
        paths = {str(name): path_data(value["points"], value["color"])
                 for name, value in packet["paths"].items()}
        with self._lock:
            changed_source = self._stream is None or stream_id != self._stream["stream_id"]
            previous = self._stream
            self._stream = {"stream_id": stream_id, "source": source, "sequence": sequence,
                            "sample_monotonic_ns": sample, "emitted_monotonic_ns": emitted}
            self._stream_error = None
            if changed_source:
                self.history.clear()
                self._paths = {}
            if changed_source or previous["sequence"] != sequence or self._paths_without_history() != paths:
                self._paths = paths
                self.update(actual, target)
                self._state["motor_diagnostics"] = packet.get("motor_diagnostics")
                self._state["command_details"] = packet.get("command_details")
                self._state["gripper"] = gripper
                self._state["actual_torque"] = torque
                self._state["gripper_actual_torque"] = grip_torque
                self._state.update(actual_vel=velocity, target_vel=target_velocity,
                                   gripper_actual_vel=grip_velocity, eef_velocity=self._velocity(actual, velocity))

    def _paths_without_history(self):
        return {name: value for name, value in self._paths.items() if name != "playback_path"}

    def _velocity(self, q, velocity):
        if velocity is None:
            return None
        import pinocchio as pin
        jacobian = pin.computeFrameJacobian(self.robot.model, self._data, np.asarray(q),
                                            self.robot.frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        twist = jacobian @ np.asarray(velocity)
        return {"linear": twist[:3].tolist(), "angular": twist[3:].tolist(),
                "speed": float(np.linalg.norm(twist[:3])), "angular_speed": float(np.linalg.norm(twist[3:]))}

    def _pose(self, q):
        import pinocchio as pin
        pin.framesForwardKinematics(self.robot.model, self._data, q)
        pin.updateGeometryPlacements(self.robot.model, self._data, self._visual, self._geometry_data)
        return {"q": q.tolist(),
                "eef_quaternion": pin.Quaternion(self._data.oMf[self.robot.frame].rotation).coeffs().tolist(),
                "meshes": [pose.homogeneous.flatten(order="F").tolist()
                           for pose in self._geometry_data.oMg],
                "eef": self._data.oMf[self.robot.frame].homogeneous.flatten(order="F").tolist()}

    def path(self, name, points, color):
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            points = np.empty((0, 3))
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("Expected finite Nx3 path points")
        if not isinstance(color, int) or not 0 <= color <= 0xFFFFFF:
            raise ValueError("Expected RGB path color")
        with self._lock:
            if self.closed:
                raise RuntimeError("Viewer is closed")
            self._paths = {**self._paths, str(name): {"points": points.tolist(), "color": color}}
            if self._state is not None:
                self._state = {**self._state, "paths": self._paths}

    def update(self, actual, target=None):
        actual = np.asarray(actual, dtype=float)
        target = actual if target is None else np.asarray(target, dtype=float)
        if actual.shape != (self.robot.model.nq,) or target.shape != actual.shape or not np.isfinite([actual, target]).all():
            raise ValueError("Viewer joint vectors disagree with the selected model")
        with self._lock:
            if self.closed:
                raise RuntimeError("Viewer is closed")
            # Monitoring includes finite feedback just outside limits; no clipping.
            measured, desired = self._pose(actual), self._pose(target)
            position = measured["eef"][12:15]
            if not self.history or np.linalg.norm(np.asarray(position) - self.history[-1]) > 1e-5:
                self.history.append(position)
            self._paths = {**self._paths, "playback_path": {
                "points": list(self.history), "color": 0x38A7FF}}
            self._sequence += 1
            self._state = {"sequence": self._sequence, "actual": measured, "target": desired,
                           # Sub-degree solver noise should not tint the actual mesh.
                           "target_visible": bool(np.max(np.abs(actual - target)) > .005),
                           "paths": self._paths}

    def close(self):
        with self._lock:
            if self.closed:
                return
            self.closed = True
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2)
