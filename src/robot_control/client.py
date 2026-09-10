"""Local arm-service client. No SDK, device access or state publisher."""

import json
import socket
import threading
import time
from types import SimpleNamespace

import numpy as np

from robot_core.project import load_config, resolve_path

MAX_RPC = 65536


class ArmClient:
    def __init__(self, robot, *, path=None, hardware=True, actuate=False, input_name="terminal", speed=None, motion=None):
        self.path = str(resolve_path(path or load_config()["arm_service"]["socket"]))
        self.robot, self.hardware, self.actuate = robot, hardware, actuate
        self.input_name, self.speed = input_name, speed
        self.motion = dict(motion or {})
        self.service_id = self.token = None
        self.closed = False
        self._sequence = 0
        self._pending_q = None
        self._pending_profile = "teleop"
        self._lock = threading.RLock()
        self.info = self._rpc("describe")
        if self.info.get("trajectory_protocol") != 1:
            raise RuntimeError("机械臂服务版本较旧，请退出遥操并重启 启动机械臂服务.sh，再启动遥操")
        from .online_motion import DEFAULTS
        missing = (set(self.motion) & set(DEFAULTS)) - set(self.info.get("effective_motion_limits", {}))
        if missing:
            raise RuntimeError("机械臂服务版本较旧，不支持参数 " + ", ".join(sorted(missing))
                + "；请退出遥操并重启 启动机械臂服务.sh，再启动遥操")
        self.service_id = self.info["service_id"]
        if self.info["robot"] != robot.metadata():
            raise ValueError("Robot model/frame differs from arm service")
        if self.info["hardware"] is not hardware:
            raise ValueError("Expected hardware/virtual service differs from the running service")
        self.settings = self.info["settings"]
        self.robot_config = SimpleNamespace(joint_pos_min=robot.lower.copy(),
                                            joint_pos_max=robot.upper.copy(), joint_dof=6)
        self._check_state(self.info)

    def _rpc(self, op, **args):
        with self._lock:
            if self.closed:
                raise RuntimeError("Arm client is closed")
            request = {"op": op, "service_id": self.service_id, "token": self.token,
                       "sent_monotonic_ns": time.monotonic_ns(), **args}
            try:
                # One bounded request per connection; no cross-thread socket sharing,
                # reconnect replay, automatic lease acquisition or queued motion retries.
                with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
                    connection.settimeout(.3)
                    connection.connect(self.path)
                    connection.sendall(json.dumps(request, allow_nan=False).encode())
                    raw, _, flags, _ = connection.recvmsg(MAX_RPC)
                if flags & socket.MSG_TRUNC or not raw:
                    raise RuntimeError("Incomplete arm-service response")
                response = json.loads(raw)
            except (OSError, ValueError) as exc:
                raise RuntimeError(f"Arm service unavailable at {self.path}; start ./run.sh arm-service ({exc})") from exc
            if not response.get("ok"):
                raise RuntimeError(response.get("error", "Arm service rejected request"))
            if op == "command":
                finished = time.monotonic_ns()
                self.command_timing = {"sample_monotonic_ns": finished,
                                       "duration_ms": (finished - request["sent_monotonic_ns"]) / 1e6}
            return response["result"]

    @staticmethod
    def _check_state(info):
        state = info["state"]
        if state is None or not 0 <= (time.monotonic_ns() - state["host_monotonic_ns"]) / 1e9 < .25:
            raise RuntimeError("Arm service feedback is stale")
        if not np.isfinite([*state["actual_pos"], *state["actual_vel"], state["gripper_actual_pos"]]).all():
            raise RuntimeError("Arm service returned non-finite feedback")
        return state

    def snapshot(self):
        self.info = self._rpc("snapshot")
        return self._check_state(self.info)

    def get_joint_positions(self):
        state = self.snapshot()
        return np.r_[state["actual_pos"], state["gripper_actual_pos"]]

    def get_joint_velocities(self):
        state = self.snapshot()
        return np.r_[state["actual_vel"], state["gripper_actual_vel"]]

    def get_runtime_diagnostics(self):
        state = self.snapshot()
        return {key: np.asarray(value) if isinstance(value, list) else value for key, value in state.items()}

    def recording_samples(self, after=0):
        return self._rpc("samples", after=after)

    def get_recording_snapshot(self):
        return {**self.snapshot(), "arm_service_id": self.service_id}

    def start_control(self):
        with self._lock:
            if self.token is None:
                args = {"robot": self.robot.metadata(), "hardware": self.hardware,
                        "actuate": self.actuate, "input": self.input_name, "motion": self.motion}
                if self.speed is not None:
                    args["speed"] = self.speed
                self.token = self._rpc("acquire", **args)["token"]

    def set_joint_positions(self, positions, *, profile="teleop"):
        with self._lock:
            self._pending_q = self.robot.joints(positions).tolist()
            self._pending_profile = profile
        return True

    def set_catch_pos(self, position):
        with self._lock:
            if not np.isfinite(position) or not 0 <= position <= self.settings["gripper_width"]:
                raise ValueError("Invalid gripper target")
            self.start_control()
            self._sequence += 1
            self.info = self._rpc("command", q=self._pending_q, gripper=float(position), sequence=self._sequence, profile=self._pending_profile)
            self._pending_q = None
        return True

    def stop_control(self, *, release=False):
        """Normal jerk-limited stop; service retains hold after voluntary release."""
        with self._lock:
            if self.token is not None:
                self.info = self._rpc("stop", release_after_stop=release)
                if release:
                    self.token = None
            self._pending_q = None

    def set_to_damping(self):
        with self._lock:
            if self.token is not None:
                try:
                    self._rpc("release")
                finally:
                    self.token = None
                    self._pending_q = None

    def close(self):
        if self.closed:
            return
        try:
            self.set_to_damping()
        finally:
            self.closed = True
