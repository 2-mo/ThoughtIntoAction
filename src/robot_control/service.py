"""Independent X5 device owner, command arbitration and continuous feedback."""

import argparse
from collections import deque
from contextlib import ExitStack, contextmanager
import json
import math
from pathlib import Path
import selectors
import signal
import socket
import stat
import subprocess
import sys
import time
import uuid

import numpy as np
from .online_motion import OnlineMotion, DEFAULTS as TRAJECTORY_DEFAULTS

from robot_core.device_lease import DeviceLease
from robot_core.model import RobotModel
from robot_core.loop_rate import LoopRate
from robot_core.plugins import Registry
from robot_core.project import load_config, resolve_path
from robot_core.telemetry import LocalPublisher

MAX_RPC = 65536
REQUEST_AGE_S = .25


def service_path(value=None):
    return resolve_path(value or load_config()["arm_service"]["socket"])


class ArmService:
    """All backend calls run on one thread. Only a live lease may submit targets."""

    def __init__(self, robot, arm, publisher, *, backend, hardware, settings, lease_timeout_s=.5, safety=None):
        self.robot, self.arm, self.publisher = robot, arm, publisher
        self.backend, self.hardware, self.settings = backend, hardware, {**TRAJECTORY_DEFAULTS, **settings}
        self._tcp_model = RobotModel(robot.urdf, eef="eef_link")
        from robot_core.safety import TablePlane
        self.safety = dict(load_config()["arm_safety"] if safety is None else safety)
        if type(self.safety.get("table_enabled")) is not bool:
            raise ValueError("table_enabled must be boolean")
        self._table_plane = TablePlane(self.safety["table_z_m"] if self.safety["table_enabled"] else None,
                                       self.safety["table_clearance_m"])
        self._table_report = (None if self._table_plane.minimum_z is None else
                              {"minimum_z_m": self._table_plane.minimum_z, "blocked": False})
        self.sample_sequence = 0
        self.sample_history = deque(maxlen=max(256, self.settings["control_rate_hz"] * 3))
        self._loop_rate = LoopRate(self.settings["control_rate_hz"])
        self.motion = None
        self.holding = self.release_after_stop = False
        self.gripper = None
        self.profile = "teleop"
        self.ik_step = 0.
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
            "mode": "stopping" if self.release_after_stop else "hold" if self.holding else "control" if self.owner and self.owner["actuate"] else "damping",
            "controller": "arm-service",
        })

    def release(self):
        # Clear authority before the backend transition, including on failures.
        self.owner = self.pending = self.target = None
        self.motion = None
        self.holding = self.release_after_stop = False
        self.applied_command = None
        self.arm.set_control_enabled(False)
        self._source()

    def expire(self):
        if self.owner and not self.release_after_stop and time.monotonic() > self.owner["deadline"]:
            self.release()

    def tick(self):
        self._loop_rate.tick()
        self.expire()
        pending, self.pending = self.pending, None
        if pending and self.owner and self.owner["actuate"] and not self.release_after_stop:
            q, self.gripper, command = pending
            floor = self.owner["motion"].get("tcp_min_z_m")
            if q is not None and floor is not None:
                same = self.target is not None and np.array_equal(q, self.target) and command["profile"] == self.profile
                safe = same or self.motion.path_above_floor(q, command["profile"], self._tcp_model, floor)
                self._table_report = {"minimum_z_m": floor, "blocked": not safe}
                if not safe:
                    # Continue the previously accepted path instead of abruptly
                    # freezing moving joints or accepting an unsafe new target.
                    q = None
                    command = None
            if command is None:
                self.pending = None
            else:
                self._accept_pending(q, command)
        self._advance_motion()
        self.sample()
        self.sample_sequence += 1
        self.sample_history.append({**self.state, "sample_sequence": self.sample_sequence})

    def _accept_pending(self, q, command):
        previous = self.applied_command.get("q") if self.applied_command else None
        self.ik_step = float(np.max(np.abs(q - np.array(previous)))) if q is not None and previous is not None else 0.
        self.profile = command["profile"]
        if q is None:
            self.target = None
            self.motion.stop()
        else:
            self.target = q.tolist()
        self.applied_command = command

    def _advance_motion(self):
        if self.motion is not None and (self.holding or self.owner and self.owner["actuate"]):
            if self.target is not None:
                target = np.array(self.target)
                lead = self.motion.settings["max_command_lead_rad"]
                if lead > 0:
                    actual = np.array(self.state["actual_pos"])
                    target = np.clip(target, actual - lead, actual + lead)
                floor = self.motion.settings.get("tcp_min_z_m")
                safe = (lead <= 0 or floor is None or
                        self.motion.path_above_floor(target, self.profile, self._tcp_model, floor))
                if safe:
                    self.motion.target(target, self.profile)
                else:
                    self._table_report = {"minimum_z_m": floor, "blocked": True}
            q, velocity = self.motion.step()
            # Use the planner's actual profile: switching to Home first brakes
            # under the previous limits before adopting the Home profile.
            settings = self.motion.settings
            preview = settings["teleop_preview_time_s"] if self.motion.profile == "teleop" else settings["preview_time_s"]
            if preview != self._current_preview_time_s:
                self.arm.set_motion_limit(self._motion_speed, preview_time_s=preview,
                    max_command_lead_rad=settings["max_command_lead_rad"],
                    joint_soft_limit_margin_rad=settings["joint_soft_limit_margin_rad"])
                self._current_preview_time_s = preview
            self.arm.set_planned_joint_state(q, velocity)
            self.arm.set_catch_pos(self.gripper)
            settled = (self.state is not None and np.max(np.abs(self.state["actual_vel"])) < .03
                       and np.max(np.abs(np.asarray(self.state["actual_pos"]) - q)) < .03)
            if self.release_after_stop and self.motion.finished and settled:
                self.owner = None
                self.release_after_stop = False
                self.holding = True
                self._source()

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
        if self.motion is not None:
            actual = np.asarray(state["actual_pos"])
            planned = np.array(self.motion.input.current_position)
            desired = np.asarray(self.target) if self.target is not None else planned
            state["motion_diagnostics"] = {"sample_monotonic_ns": time.monotonic_ns(),
                "q_actual": actual.tolist(), "q_des": desired.tolist(), "q_safe": planned.tolist(),
                "peak_ik_step": self.ik_step, "ik_backlog": float(np.max(np.abs(desired - actual))),
                "command_tracking_error": float(np.max(np.abs(planned - actual)))}
        self.state = {**state, "host_monotonic_ns": time.monotonic_ns(),
                      "service_command": self.applied_command,
                      "loop_rates": {"arm_service": self._loop_rate.snapshot()},
                      "table_safety": self._table_report,
                      "actuate": bool(self.holding or self.owner and self.owner["actuate"]),
                      "trajectory": self.motion.snapshot() if self.motion else None}
        self.publisher.update(state["actual_pos"], self.target, actual_torque=state.get("actual_torque"),
                              gripper_actual_torque=state.get("gripper_actual_torque"), gripper={
            "actual": state["gripper_actual_pos"],
            "target": self.gripper if self.state["actuate"] else None,
            "max": self.settings["gripper_width"],
        })

    def describe(self):
        hardware_speed = np.asarray(self.arm.robot_config.joint_vel_max, dtype=float)
        speed = self.owner["speed"] if self.owner else self.settings["max_joint_speed_rad_s"]
        return {"sample_protocol": 1, "trajectory_protocol": 1, "service_id": self.service_id, "robot": self.robot.metadata(),
                "backend": self.backend, "hardware": self.hardware, "settings": self.settings,
                "safety": self.safety,
                "hardware_joint_speed_limits_rad_s": hardware_speed.tolist(),
                "effective_joint_speed_limits_rad_s": (hardware_speed if speed == 0 else np.minimum(hardware_speed, speed)).tolist(),
                "effective_motion_limits": self.owner["motion"] if self.owner else {key: self.settings.get(key) for key in ("max_command_lead_rad", "preview_time_s", "joint_soft_limit_margin_rad", *TRAJECTORY_DEFAULTS)},
                "effective_max_joint_speed_rad_s": self.owner["speed"] if self.owner else self.settings["max_joint_speed_rad_s"],
                "stopped_hold_recovery_protocol": 1,
                "mode": "stopping" if self.release_after_stop else "hold" if self.holding else "control" if self.owner and self.owner["actuate"] else "damping",
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
        if op == "samples":
            after = request.get("after")
            if type(after) is not int or after < 0:
                raise ValueError("Invalid sample cursor")
            oldest = self.sample_history[0]["sample_sequence"] if self.sample_history else self.sample_sequence + 1
            if after and after < oldest - 1:
                raise ValueError("Recording sample buffer overflow; data is incomplete")
            rows = [row for row in self.sample_history if row["sample_sequence"] > after][:4] if after else []
            return {"states": rows, "latest": self.sample_sequence}
        if op == "snapshot":
            self.sample()
        if self.state is None or (time.monotonic_ns() - self.state["host_monotonic_ns"]) / 1e9 > .25:
            raise ValueError("Arm feedback is stale")
        if op == "recover_hold":
            # A stopped, abandoned voluntary release may never reach its old planned
            # position. Explicit recovery holds measured position without disabling torque.
            if (not self.owner or self.owner['input'] != request.get('expected_owner')
                    or self.owner['input'] not in ('pi05-sft', 'pi05-rollout', 'zero-shot-pregrasp')
                    or not self.release_after_stop or self.motion is None
                    or not self.motion.finished or time.monotonic() <= self.owner['deadline']):
                raise ValueError('Recovery requires an abandoned, finished PI05 stop')
            cutoff = time.monotonic_ns() - 500_000_000
            recent = [s for s in self.sample_history if s['host_monotonic_ns'] >= cutoff]
            if (not recent or recent[0]['host_monotonic_ns'] > cutoff + 50_000_000
                    or any(not np.isfinite([*s['actual_pos'], *s['actual_vel']]).all()
                           or np.max(np.abs(s['actual_vel'])) >= .03 for s in recent)
                    or np.max(np.ptp([s['actual_pos'] for s in recent], axis=0)) > .01):
                raise ValueError('Recovery requires stationary feedback for 0.5 seconds')
            q = self.robot.joints(self.state['actual_pos'])
            previous_error = float(np.max(np.abs(q - self.motion.input.current_position)))
            planner = OnlineMotion(q, np.zeros(6), self.arm.robot_config.joint_vel_max,
                                   self.owner['speed'], self.motion.settings,
                                   1/self.settings['control_rate_hz'], self.robot.lower, self.robot.upper)
            planner.stop()
            self.motion = planner
            self.pending = self.target = self.applied_command = None
            self.owner = None
            self.release_after_stop = False
            self.holding = True
            self._source()
            return {'recovered': True, 'held_rad': q.tolist(), 'previous_tracking_error_rad': previous_error}
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
            if actuate and np.max(np.abs(self.state["actual_vel"])) > .05:
                raise ValueError("Wait for the arm to stop before acquiring control")
            speed = request.get("speed", self.settings["max_joint_speed_rad_s"])
            keys = ("max_command_lead_rad", "preview_time_s", "joint_soft_limit_margin_rad", *TRAJECTORY_DEFAULTS)
            overrides = request.get("motion", {})
            if not isinstance(overrides, dict) or set(overrides) - set(keys):
                raise ValueError("Invalid motion settings")
            motion = {key: overrides.get(key, self.settings.get(key)) for key in keys}
            from .trajectory import validate_motion_limits
            validate_motion_limits(speed, motion["max_command_lead_rad"], motion["preview_time_s"], motion["joint_soft_limit_margin_rad"])
            for key in TRAJECTORY_DEFAULTS:
                if type(motion[key]) not in (int, float) or not math.isfinite(motion[key]) or motion[key] < 0 or (motion[key] == 0 and key not in {"preset_speed_rad_s", "home_final_speed_rad_s"}):
                    raise ValueError(f"Invalid trajectory limit {key}; preset/final Home speed may be zero")
            floor = self._table_plane.minimum_z
            motion["tcp_min_z_m"] = floor
            if floor is not None:
                if type(floor) not in (int, float) or not math.isfinite(floor):
                    raise ValueError("Invalid TCP floor height")
                if self._tcp_model.fk(self.state["actual_pos"])[2, 3] < floor:
                    raise ValueError("当前夹指尖端低于桌面安全高度；请先检查桌面设置与机械臂位置")
            self._table_report = None if floor is None else {"minimum_z_m": floor, "blocked": False}
            planner = OnlineMotion(self.state["actual_pos"], self.state["actual_vel"],
                                   self.arm.robot_config.joint_vel_max, speed, motion,
                                   1/self.settings["control_rate_hz"], self.robot.lower, self.robot.upper)
            self.arm.set_motion_limit(speed, **{k: motion[k] for k in ("max_command_lead_rad", "preview_time_s", "joint_soft_limit_margin_rad")})
            self._current_preview_time_s = motion["preview_time_s"]
            self._motion_speed = speed
            self.arm.set_control_enabled(actuate)
            self.owner = {"token": uuid.uuid4().hex, "input": name, "actuate": actuate, "speed": speed, "motion": motion,
                          "deadline": time.monotonic() + self.lease_timeout_s, "sequence": 0}
            self.target = None
            self.motion = planner if actuate else None
            if self.motion:
                self.motion.stop()
            self.holding = self.release_after_stop = False
            self.gripper = self.state["gripper_actual_pos"]
            self._source()
            return {"token": self.owner["token"]}
        if op == "snapshot" and request.get("token") is None:
            return self.describe()
        if not self.owner or request.get("token") != self.owner["token"]:
            raise ValueError("Control lease is absent or expired; restart the control program")
        if op == "release":
            self.release()
        elif op == "stop":
            if type(request.get("release_after_stop", False)) is not bool:
                raise ValueError("Invalid stop intent")
            self.pending = self.target = None
            if self.motion is not None:
                self.motion.stop()
                self.release_after_stop = request.get("release_after_stop", False)
            elif request.get("release_after_stop"):
                self.release()
        elif op == "snapshot":
            pass
        elif op == "command":
            if self.release_after_stop:
                raise ValueError("Control is stopping; wait for standby")
            profile = request.get("profile", "teleop")
            if profile not in ("teleop", "home", "preset"):
                raise ValueError("Invalid trajectory profile")
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
                                     "input": self.owner["input"], "profile": profile})
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
                next_tick += dt
                if next_tick < time.monotonic():
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
    defaults = {**TRAJECTORY_DEFAULTS, **config["arm_service"]}
    parser = argparse.ArgumentParser(prog="arm-service", description=__doc__)
    parser.add_argument("--interactive", action=argparse.BooleanOptionalAction, default=False,
                        help="显示带常驻快捷键栏的机械臂 TUI")
    parser.add_argument("--prepare-can", action=argparse.BooleanOptionalAction, default=False,
                        help="真机启动前准备 CAN 接口，必要时通过 sudo 请求密码")
    parser.add_argument("--backend", default=defaults["backend"], help="Arm plugin with service_device capability")
    parser.add_argument("--socket", default=str(service_path()))
    parser.add_argument("--endpoint", help="Local telemetry endpoint")
    parser.add_argument("--urdf", default=str(resolve_path(config["robot"]["urdf"])))
    parser.add_argument("--can-port", default=config["robot"]["can_port"])
    parser.add_argument("--q", nargs=6, type=float, help="Initial virtual pose only")
    for name in ("control_rate_hz", "max_joint_speed_rad_s", "max_command_lead_rad", "joint_soft_limit_margin_rad",
                 "preview_time_s", "gripper_width", "gripper_open_readout", "gripper_open_limit_m",
                 "gripper_contact_torque", "gripper_hold_torque", "gripper_contact_cycles",
                 "gripper_release_margin_m", *TRAJECTORY_DEFAULTS):
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
    if min(settings[name] for name in ("control_rate_hz", "gripper_width")) <= 0:
        parser.error("Rate and gripper width must be positive")
    if not 0 < settings["gripper_open_limit_m"] <= settings["gripper_width"]:
        parser.error("gripper_open_limit_m must be positive and no greater than gripper_width")
    if spec.hardware and args.q is not None:
        parser.error("--q is a virtual initialization setting")
    if args.prepare_can and spec.hardware:
        result = subprocess.run([
            "bash", str(resolve_path("scripts/setup/can_up.sh")),
            "--interface", args.can_port,
            "--device", str(resolve_path(config["robot"]["can_device"])),
        ])
        if result.returncode:
            parser.exit(1, "CAN 接口准备失败，机械臂服务未启动。请检查 USB-CAN 连接及 sudo 权限。\n")
    if args.interactive:
        from .console import run_console
        return run_console(args, spec.hardware,
                           [*list(sys.argv[1:] if argv is None else argv), "--no-prepare-can"])
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
