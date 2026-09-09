"""Independent X5 device owner, command arbitration and continuous feedback."""

import argparse
from contextlib import ExitStack, contextmanager
import json
import math
from pathlib import Path
import selectors
import signal
import socket
import stat
import time
import uuid

from robot_core.device_lease import DeviceLease
from robot_core.model import RobotModel
from robot_core.plugins import Registry
from robot_core.project import load_config, resolve_path
from robot_core.telemetry import LocalPublisher

MAX_RPC = 65536
REQUEST_AGE_S = .25


def service_path(value=None):
    return resolve_path(value or load_config()["arm_service"]["socket"])


class ArmService:
    """All backend calls run on one thread. Only a live lease may submit targets."""

    def __init__(self, robot, arm, publisher, *, backend, hardware, settings, lease_timeout_s=.5):
        self.robot, self.arm, self.publisher = robot, arm, publisher
        self.backend, self.hardware, self.settings = backend, hardware, dict(settings)
        self.lease_timeout_s = lease_timeout_s
        self.service_id = uuid.uuid4().hex
        self.owner = None
        self.pending = None
        self.state = None
        self.target = None
        self.applied_command = None
        self._last_feedback = None
        self._feedback_progress = time.monotonic()
        self._source()

    def _source(self):
        self.publisher.set_source({
            "kind": "hardware" if self.hardware else "virtual", "backend": self.backend,
            "input": self.owner["input"] if self.owner else "待机",
            "mode": "control" if self.owner and self.owner["actuate"] else "damping",
            "controller": "arm-service",
        })

    def release(self):
        # Clear authority before the backend transition, including on failures.
        self.owner = self.pending = self.target = None
        self.applied_command = None
        self.arm.set_control_enabled(False)
        self._source()

    def expire(self):
        if self.owner and time.monotonic() > self.owner["deadline"]:
            self.release()

    def tick(self):
        self.expire()
        pending, self.pending = self.pending, None
        if pending and self.owner and self.owner["actuate"]:
            q, gripper, command = pending
            if q is not None:
                self.arm.set_joint_positions(q)
            self.arm.set_catch_pos(gripper)
            self.applied_command = command
        self.sample()

    def sample(self):
        """Read cached SDK feedback now, also for camera-bracketed RPC reads."""
        if self.hardware:
            state = self.arm.get_recording_snapshot()
            stamp = state["feedback_timestamp"]
            if not math.isfinite(stamp):
                raise RuntimeError("Invalid SDK feedback timestamp")
            if self._last_feedback is None or stamp > self._last_feedback:
                self._last_feedback, self._feedback_progress = stamp, time.monotonic()
            elif stamp < self._last_feedback or time.monotonic() - self._feedback_progress > .25:
                raise RuntimeError("SDK feedback stopped progressing")
        else:
            q, dq = self.arm.get_joint_positions(), self.arm.get_joint_velocities()
            state = {"actual_pos": q[:6].tolist(), "actual_vel": dq[:6].tolist(),
                     "gripper_actual_pos": float(q[6]), "gripper_actual_vel": float(dq[6]),
                     "feedback_timestamp": time.monotonic()}
        self.state = {**state, "host_monotonic_ns": time.monotonic_ns(),
                      "service_command": self.applied_command,
                      "actuate": bool(self.owner and self.owner["actuate"])}
        self.publisher.update(state["actual_pos"], self.target)

    def describe(self):
        return {"service_id": self.service_id, "robot": self.robot.metadata(),
                "backend": self.backend, "hardware": self.hardware, "settings": self.settings,
                "effective_max_joint_speed_rad_s": self.owner["speed"] if self.owner else self.settings["max_joint_speed_rad_s"],
                "mode": "control" if self.owner and self.owner["actuate"] else "damping",
                "owner": self.owner["input"] if self.owner else None, "state": self.state}

    def request(self, request):
        self.expire()
        if not isinstance(request, dict):
            raise ValueError("Expected an RPC object")
        sent = request.get("sent_monotonic_ns")
        if type(sent) is not int or not 0 <= (time.monotonic_ns() - sent) / 1e9 <= REQUEST_AGE_S:
            raise ValueError("Expired or invalid request timestamp")
        op = request.get("op")
        if op == "describe":
            return self.describe()
        if request.get("service_id") != self.service_id:
            raise ValueError("Arm service restarted; reconnect explicitly")
        if op == "snapshot":
            self.sample()
        if self.state is None or (time.monotonic_ns() - self.state["host_monotonic_ns"]) / 1e9 > .25:
            raise ValueError("Arm feedback is stale")
        if op == "acquire":
            if self.owner:
                raise ValueError(f"Arm control is owned by {self.owner['input']}")
            if request.get("robot") != self.robot.metadata():
                raise ValueError("Robot model/frame mismatch")
            if request.get("hardware") is not self.hardware:
                raise ValueError("Expected hardware/virtual service differs from the running service")
            actuate, name = request.get("actuate"), request.get("input")
            if type(actuate) is not bool or not isinstance(name, str) or not 0 < len(name) <= 80:
                raise ValueError("Invalid control intent or input name")
            speed = request.get("speed", self.settings["max_joint_speed_rad_s"])
            if type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
                raise ValueError("Invalid control speed")
            self.arm.set_motion_limit(speed)
            self.arm.set_control_enabled(actuate)
            self.owner = {"token": uuid.uuid4().hex, "input": name, "actuate": actuate, "speed": speed,
                          "deadline": time.monotonic() + self.lease_timeout_s, "sequence": 0}
            self.target = None
            self._source()
            return {"token": self.owner["token"]}
        if op == "snapshot" and request.get("token") is None:
            return self.describe()
        if not self.owner or request.get("token") != self.owner["token"]:
            raise ValueError("Control lease is absent or expired; restart the control program")
        if op == "release":
            self.release()
        elif op == "snapshot":
            pass
        elif op == "command":
            sequence = request.get("sequence")
            if type(sequence) is not int or sequence <= self.owner["sequence"]:
                raise ValueError("Repeated or out-of-order command")
            value = request.get("q")
            q = None if value is None else self.robot.joints(value)
            grip = request.get("gripper")
            if type(grip) not in (float, int) or not math.isfinite(grip) or not 0 <= grip <= self.settings["gripper_width"]:
                raise ValueError("Invalid gripper width")
            grip = min(grip, self.settings["gripper_open_limit_m"])
            # Latest command wins; the service applies at most one per control tick.
            self.pending = (q, grip, {"sequence": sequence, "received_monotonic_ns": time.monotonic_ns(),
                                     "q": None if q is None else q.tolist(), "gripper": grip,
                                     "input": self.owner["input"]})
            if q is not None:
                self.target = q.tolist()
            self.owner["sequence"] = sequence
        else:
            raise ValueError(f"Unknown arm service operation: {op}")
        if self.owner:
            self.owner["deadline"] = time.monotonic() + self.lease_timeout_s
        return self.describe()


@contextmanager
def rpc_listener(path):
    """Reserve the socket before constructing any device; preserve non-socket files."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as resources:
        lease = DeviceLease("arm-service:" + str(path.resolve()))
        resources.callback(lease.close)
        if path.exists() or path.is_symlink():
            if not stat.S_ISSOCK(path.lstat().st_mode):
                raise ValueError(f"Refusing to replace a non-socket: {path}")
            path.unlink()
        server = resources.enter_context(socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET))
        server.bind(str(path))
        path.chmod(0o600)
        resources.callback(path.unlink, missing_ok=True)
        server.listen(16)
        server.setblocking(False)
        yield server


def serve(service, path, *, ready=None, stop=None, listener=None):
    """Private local RPC socket. Never block device sampling on an RPC client."""
    with ExitStack() as resources:
        server = listener or resources.enter_context(rpc_listener(path))
        selector = resources.enter_context(selectors.DefaultSelector())
        selector.register(server, selectors.EVENT_READ)
        clients = {}
        def drop(client):
            selector.unregister(client)
            clients.pop(client, None)
            client.close()
        resources.callback(lambda: [drop(client) for client in list(clients)])
        service.tick()
        if ready:
            ready.set()
        print(f"Arm service ready: {path} ({service.backend}; damping standby)", flush=True)
        dt, next_tick = 1. / service.settings["control_rate_hz"], time.monotonic()
        while stop is None or not stop.is_set():
            now = time.monotonic()
            if now >= next_tick:
                service.tick()
                next_tick = time.monotonic() + dt
            for key, _ in selector.select(max(0., min(.02, next_tick - time.monotonic()))):
                client = key.fileobj
                if client is server:
                    conn, _ = server.accept()
                    if len(clients) >= 16:
                        conn.close()
                        continue
                    conn.setblocking(False)
                    clients[conn] = time.monotonic()
                    selector.register(conn, selectors.EVENT_READ)
                    continue
                try:
                    raw, _, flags, _ = client.recvmsg(MAX_RPC)
                    if not raw:
                        continue
                    try:
                        if flags & socket.MSG_TRUNC:
                            raise ValueError("RPC request too large")
                        result = service.request(json.loads(raw))
                        reply = {"ok": True, "result": result}
                    except (ValueError, TypeError, KeyError, OverflowError) as exc:
                        reply = {"ok": False, "error": str(exc)}
                    client.send(json.dumps(reply, allow_nan=False).encode())
                except (BrokenPipeError, ConnectionResetError, BlockingIOError):
                    pass
                finally:
                    drop(client)
            for client, connected in list(clients.items()):
                if time.monotonic() - connected > REQUEST_AGE_S:
                    drop(client)


def main(argv=None):
    config = load_config()
    defaults = config["arm_service"]
    parser = argparse.ArgumentParser(prog="arm-service", description=__doc__)
    parser.add_argument("--backend", default=defaults["backend"], help="Arm plugin with service_device capability")
    parser.add_argument("--socket", default=str(service_path()))
    parser.add_argument("--endpoint", help="Local telemetry endpoint")
    parser.add_argument("--urdf", default=str(resolve_path(config["robot"]["urdf"])))
    parser.add_argument("--can-port", default=config["robot"]["can_port"])
    parser.add_argument("--q", nargs=6, type=float, help="Initial virtual pose only")
    for name in ("control_rate_hz", "max_joint_speed_rad_s", "joint_soft_limit_margin_rad",
                 "preview_time_s", "gripper_width", "gripper_open_readout", "gripper_open_limit_m",
                 "gripper_contact_torque", "gripper_hold_torque", "gripper_contact_cycles",
                 "gripper_release_margin_m"):
        parser.add_argument("--" + name.replace("_", "-"),
                            type=int if name in {"control_rate_hz", "gripper_contact_cycles"} else float,
                            default=defaults[name])
    args = parser.parse_args(argv)
    spec = Registry().spec("arm", args.backend)
    if "service_device" not in spec.capabilities:
        parser.error(f"Backend {args.backend!r} does not support independent service ownership")
    settings = {name: getattr(args, name) for name in defaults if hasattr(args, name)
                and name not in {"socket", "backend"}}
    for key, value in settings.items():
        if not math.isfinite(value) or (key != "gripper_open_readout" and value < 0):
            parser.error(f"Invalid {key}")
    if min(settings[name] for name in ("control_rate_hz", "max_joint_speed_rad_s", "gripper_width")) <= 0:
        parser.error("Rate, speed and gripper width must be positive")
    if not 0 < settings["gripper_open_limit_m"] <= settings["gripper_width"]:
        parser.error("gripper_open_limit_m must be positive and no greater than gripper_width")
    if spec.hardware and args.q is not None:
        parser.error("--q is a virtual initialization setting")
    robot = RobotModel(args.urdf)
    q = robot.joints(args.q or [0, 1.5, 1.45, -1.52, 0, 0]) if not spec.hardware else None
    def interrupted(*_):
        raise KeyboardInterrupt
    previous = signal.signal(signal.SIGTERM, interrupted)
    service = None
    try:
        with ExitStack() as resources:
            # Reserve both endpoints before opening a device. A second service fails early.
            listener = resources.enter_context(rpc_listener(service_path(args.socket)))
            publisher = LocalPublisher(robot, address=args.endpoint)
            resources.callback(publisher.close)
            if args.backend == "stanford":
                print("Opening X5 SDK: enables motors and enters damping; this is active communication, not passive CAN reading.", flush=True)
            arm = Registry().create("arm", args.backend, initial_q=q, settings={
                **settings, "can_port": args.can_port, "actuate": False,
                "dynamics_urdf_path": str(robot.urdf), "gripper_force_hold": True})
            resources.callback(arm.close)
            service = ArmService(robot, arm, publisher, backend=args.backend,
                                 hardware=spec.hardware, settings=settings)
            resources.callback(service.release)
            serve(service, service_path(args.socket), listener=listener)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    main()
