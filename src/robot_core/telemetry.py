"""One-way local robot state stream. Publishers never depend on a viewer process."""

import json
from copy import deepcopy
import re
import threading
import time
import uuid

import numpy as np
import zmq

from .project import load_config

SCHEMA = 1
MAX_MESSAGE = 4_000_000


def command_details(state):
    """Raw SDK output and gains; absent fields stay absent for older recordings."""
    keys = ("sdk_cmd_pos", "sdk_cmd_vel", "sdk_cmd_torque", "kp", "kd",
            "gripper_sdk_cmd_pos", "gripper_sdk_cmd_vel", "gripper_sdk_cmd_torque",
            "gripper_kp", "gripper_kd", "gripper_mode", "feedback_timestamp",
            "command_timestamp", "controller_timestamp", "submission")
    return {key: deepcopy(state[key]) for key in keys if key in state}


def endpoint(value=None):
    value = value or load_config()["telemetry"]["endpoint"]
    if not re.fullmatch(r"tcp://127\.0\.0\.1:[0-9]+", value):
        raise ValueError("Telemetry endpoint must be tcp://127.0.0.1:PORT")
    if not 1 <= int(value.rsplit(":", 1)[1]) <= 65535:
        raise ValueError("Invalid telemetry port")
    return value


def joint_pair(actual, target=None):
    actual = np.asarray(actual, dtype=float)
    target = actual if target is None else np.asarray(target, dtype=float)
    if actual.shape != (6,) or target.shape != (6,) or not np.isfinite([actual, target]).all():
        raise ValueError("State requires six finite actual and target joint angles")
    return actual.tolist(), target.tolist()


def gripper_data(value):
    if value is None:
        return None
    actual, target, maximum = value["actual"], value.get("target"), value["max"]
    if not all(type(v) in (int, float) and np.isfinite(v) for v in (actual, maximum)) or maximum <= 0:
        raise ValueError("Invalid gripper feedback/range")
    if target is not None and (type(target) not in (int, float) or not np.isfinite(target)):
        raise ValueError("Invalid gripper target")
    return {"actual": float(actual), "target": target, "max": float(maximum)}


def scalar_torque(value):
    if value is not None and (type(value) not in (int, float) or not np.isfinite(value)):
        raise ValueError("Expected finite gripper feedback torque")
    return value


def torque_data(value):
    if value is None:
        return None
    values = np.asarray(value, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("Expected six finite joint feedback torques")
    return values.tolist()


def path_data(points, color):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        points = np.empty((0, 3))
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Expected finite Nx3 path points")
    if not isinstance(color, int) or not 0 <= color <= 0xFFFFFF:
        raise ValueError("Expected RGB path color")
    return {"points": points.tolist(), "color": color}


class LocalPublisher:
    """A bounded latest-state buffer and a PUB socket owned by its worker thread."""

    def __init__(self, robot, source=None, address=None):
        self.address = endpoint(address)
        self.robot = robot
        self._base = {"schema": SCHEMA, "stream_id": uuid.uuid4().hex,
                      "robot": robot.metadata(), "source": dict(source or {})}
        self._lock = threading.Lock()
        self._state = None
        self._paths = {}
        self._sequence = 0
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._error = None
        self.closed = False
        self._thread = threading.Thread(target=self._publish, name="robot-state-publisher", daemon=True)
        self._thread.start()
        if not self._ready.wait(3):
            self.close()
            raise RuntimeError("Telemetry publisher startup timed out")
        if self._error is not None:
            self.close()
            raise RuntimeError(f"Cannot publish robot state at {self.address}: {self._error}")
        print(f"Robot state publisher: {self.address}", flush=True)

    def _publish(self):
        try:
            with zmq.Context() as context, context.socket(zmq.PUB) as sock:
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.SNDHWM, 1)
                # TCP binding enforces one publisher per channel, before any arm opens.
                sock.bind(self.address)
                self._ready.set()
                while not self._stop.is_set():
                    with self._lock:
                        state = self._state
                        paths = self._paths
                        base = self._base
                    if state is not None:
                        packet = {**base, **state, "paths": paths,
                                  "emitted_monotonic_ns": time.monotonic_ns()}
                        raw = json.dumps(packet, allow_nan=False).encode()
                        if len(raw) <= MAX_MESSAGE:
                            try:
                                sock.send(raw, flags=zmq.DONTWAIT)
                            except zmq.Again:
                                pass  # A slow or absent monitor never delays control.
                    self._stop.wait(.05)
        except Exception as exc:
            self._error = exc
            self._ready.set()

    def update(self, actual, target=None, *, gripper=None, actual_torque=None, gripper_actual_torque=None, actual_vel=None, target_vel=None, gripper_actual_vel=None, motor_diagnostics=None, command_details=None):
        actual, target = joint_pair(actual, target)
        gripper = gripper_data(gripper)
        actual_torque = torque_data(actual_torque)
        gripper_actual_torque = scalar_torque(gripper_actual_torque)
        actual_vel = torque_data(actual_vel)
        target_vel = torque_data(target_vel)
        gripper_actual_vel = scalar_torque(gripper_actual_vel)
        with self._lock:
            if self.closed:
                raise RuntimeError("State publisher is closed")
            self._sequence += 1
            self._state = {"sequence": self._sequence, "actual": actual, "target": target,
                           "sample_monotonic_ns": time.monotonic_ns(), "gripper": gripper, "actual_torque": actual_torque,
                           "gripper_actual_torque": gripper_actual_torque, "actual_vel": actual_vel,
                           "target_vel": target_vel, "gripper_actual_vel": gripper_actual_vel,
                           "command_details": command_details,
                           "motor_diagnostics": {k: v for k, v in motor_diagnostics.items() if k != "events"} if isinstance(motor_diagnostics, dict) else None}

    def path(self, name, points, color):
        data = path_data(points, color)
        with self._lock:
            if self.closed:
                raise RuntimeError("State publisher is closed")
            paths = {**self._paths, str(name): data}
            if len(json.dumps(paths).encode()) > MAX_MESSAGE - 16384:
                raise ValueError("Telemetry paths exceed the 4 MB message budget")
            self._paths = paths

    def set_source(self, source):
        """The service stays alive while input ownership changes."""
        with self._lock:
            self._base = {**self._base, "source": dict(source)}
            self._paths = {}

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._stop.set()
        self._thread.join(timeout=2)


class NullPublisher:
    def __init__(self, robot, source=None):
        pass

    def update(self, actual, target=None, *, gripper=None, actual_torque=None, gripper_actual_torque=None, actual_vel=None, target_vel=None, gripper_actual_vel=None, motor_diagnostics=None, command_details=None):
        pass

    def path(self, name, points, color):
        pass

    def close(self):
        pass


class StateSubscriber:
    """Used only by monitoring; SUB cannot send a motion command to the publisher."""

    def __init__(self, address=None):
        address = endpoint(address)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.MAXMSGSIZE, MAX_MESSAGE)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.connect(address)

    def receive(self, timeout_ms=100):
        if not self.socket.poll(timeout_ms):
            return None
        return json.loads(self.socket.recv())

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
