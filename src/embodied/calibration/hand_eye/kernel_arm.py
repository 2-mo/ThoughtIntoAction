"""The ArmController subset the dual-camera hand-eye collector uses, on the arx kernel (todo C-06).

`ManualEyeToHandCollector(config, session, controller=KernelArm(run))` runs unchanged without
opening CAN. FK and IK use the kernel's own URDF and TCP (`tcp_link`, link6 + 0.145 m along +x,
the legacy `eef_link`), so stored `T_base_eef` and solved transforms keep their frames. Moves are
kernel previews the kernel executes; damping, teaching and the gripper are short-TTL kernel targets
kept alive by one thread. Differences from the SDK controller, to confirm at real-arm acceptance:
"damping" is kernel free-drive; an emergency cancels and holds instead of dropping the arm; the
gripper is position controlled, so the clamping force after `force_close_gripper` comes from a
target CLAMP_SQUEEZE_M inside the contact width rather than a torque feedforward.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pinocchio as pin

from embodied.contracts.runtime.control import ControlSession, live_runs
from embodied.core.model import RobotModel
from embodied.core.project import PROJECT_ROOT
from legacy.arx5_control import (ArmLimits, ArmSnapshot, ControlError, ControllerMode, MotionResult,
                                 MotionSettleError, Pose, SafetyConfig, SafetyError)

KERNEL_URDF = PROJECT_ROOT / 'control/models/x5/x5_2025.urdf'
TTL_MS = 200
KEEPALIVE_S = .08
MAX_SPEED = .5  # rad/s, the kernel planner's bound
CLOSE_SPEED_M_S = .02
CONTACT_LAG_M = .002  # measured width this far above the closing target means the fingers are blocked
CLAMP_SQUEEZE_M = .0015


@dataclass(frozen=True)
class KernelPlan:
    target_joint_position: np.ndarray
    path_xyz_m: np.ndarray
    duration_s: float
    preview: dict


def pose6(transform):
    return np.r_[transform[:3, 3], pin.rpy.matrixToRpy(transform[:3, :3])]


def quaternion_matrix(xyzw):
    x, y, z, w = xyzw
    return pin.Quaternion(w, x, y, z).normalized().toRotationMatrix()


class KernelArm:
    """Acquires its own lease on `directory` (default: newest running kernel), or uses a caller's `session`."""

    def __init__(self, directory: Path | None = None, source: str = 'handeye', *,
                 session: ControlSession | None = None) -> None:
        self.shared = session
        runs = [session.directory] if session else [Path(directory)] if directory else live_runs(PROJECT_ROOT / 'control/run')
        if not runs:
            raise RuntimeError('没有运行中的内核：先运行 ./run.sh kernel sim 或 ./run.sh kernel real。')
        self.directory, self.source = runs[-1], source
        self.model = RobotModel(urdf=KERNEL_URDF, eef='tcp_link')
        self.safety = SafetyConfig(max_commanded_velocity_rad_s=MAX_SPEED, minimum_motion_duration_s=.4)
        self.mode = ControllerMode.DISCONNECTED
        self.session: ControlSession | None = None
        self.robot: dict = {}
        self.fault: str | None = None
        self.gripper_enabled = False
        self.clamp: float | None = None  # latched clamp target after a force close
        self.stream: tuple | None = None
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.keepalive: threading.Thread | None = None

    # Lease and modes

    def connect(self, stabilize: bool = False) -> None:
        if self.session is None:
            session = self.shared or ControlSession(self.directory, source=self.source, purpose='operation')
            if self.shared is None:
                session.__enter__()
            self.session, self.fault = session, None
            self.robot = session.describe()['robot']
            self.stop.clear()
            self.keepalive = threading.Thread(target=self._keepalive, name='handeye-kernel', daemon=True)
            self.keepalive.start()
            print(f'手眼标定经内核 {session.identity["runtime_id"]}（{session.identity["backend"]}）。', flush=True)
        if stabilize:
            self.hold_current()
        elif self.mode is ControllerMode.DISCONNECTED:
            self.damping()

    def close(self, emergency: bool = False) -> None:
        if emergency:
            self.emergency_damping('hand-eye close')
        self.stop.set()
        if self.keepalive is not None:
            self.keepalive.join(timeout=1)
            self.keepalive = None
        if self.session is not None:
            session, self.session = self.session, None
            if self.shared is None:
                session.close()  # The kernel holds the measured pose.
            elif self.fault is None:
                session.command('hold')  # Hand the held arm back to the lease owner.
        self.stream = None
        self.mode = ControllerMode.CLOSED

    def _require_session(self):
        if self.session is None:
            raise ControlError('机械臂未连接。')
        if self.fault is not None:
            raise ControlError(f'故障：{self.fault}')

    def _set_stream(self, stream):
        with self.lock:
            self.stream = stream
            if stream is None:
                self.session.command('hold')
            else:
                self._send(stream)

    def _send(self, stream):
        action, width = stream
        fields = {} if width is None else {'gripper_width': float(width)}
        self.session.command(action, ttl_ms=TTL_MS, sent_monotonic_ns=time.monotonic_ns(), **fields)

    def _keepalive(self):
        while not self.stop.wait(KEEPALIVE_S):
            try:
                with self.lock:
                    if self.stream is not None and self.fault is None:
                        self._send(self.stream)
                if self.session is not None and self.session.error:
                    raise RuntimeError(f'控制租约已失效：{self.session.error}')
            except Exception as exc:
                self.emergency_damping(str(exc))

    def _gripper_width(self):
        if self.clamp is not None:
            return self.clamp
        return float(self.session.status()['gripper_width'])

    def hold_current(self) -> ArmSnapshot:
        self._require_session()
        self._set_stream(('gripper', self._gripper_width()) if self.gripper_enabled else None)
        self.mode = ControllerMode.ACTIVE
        return self.get_state()

    def teach_with_gripper_hold(self) -> ArmSnapshot:
        """Arm free, gripper held at the clamp or its measured width; the mode stays ACTIVE."""
        self._require_session()
        if self.mode is not ControllerMode.ACTIVE:
            raise ControlError('示教前先保持当前位置。')
        self.gripper_enabled = True
        self._set_stream(('free-drive', self._gripper_width()))
        return self.get_state()

    def activate_gripper(self) -> ArmSnapshot:
        self._require_session()
        if self.mode is not ControllerMode.ACTIVE:
            self.hold_current()
        if not self.gripper_enabled:
            self.gripper_enabled = True
            self._set_stream(('gripper', self._gripper_width()))
        return self.get_state()

    def damping(self) -> None:
        self._require_session()
        self.gripper_enabled, self.clamp = False, None
        self._set_stream(('free-drive', None))
        self.mode = ControllerMode.DAMPING

    def emergency_damping(self, reason: str) -> None:
        """From any thread: latch a fault, cancel the operation and hold (the kernel's handover policy)."""
        with self.lock:
            first = self.fault is None
            self.fault, self.stream, self.clamp = reason, None, None
            self.mode = ControllerMode.FAULT
        if first:
            print(f'手眼标定急停：{reason}；内核取消运动并保持当前位置。', flush=True)
        if self.session is not None:
            try:
                self.session.command('cancel')
            except Exception:
                pass  # Lease expiry also makes the kernel hold.

    def reconfigure_safety(self, safety: SafetyConfig) -> None:
        self.safety = safety

    # State and model

    def get_state(self) -> ArmSnapshot:
        self._require_session()
        status = self.session.status()
        pose = np.asarray(status['pose'], dtype=float)
        rotation = quaternion_matrix(pose[3:])
        return ArmSnapshot(timestamp=status['feedback_monotonic_ns'] / 1e9,
                           joint_position=np.asarray(status['q'], dtype=float),
                           joint_velocity=np.asarray(status['dq'], dtype=float), joint_torque=np.zeros(6),
                           gripper_position=float(status['gripper_width']), gripper_velocity=0.,
                           gripper_torque=0., eef_pose_6d=np.r_[pose[:3], pin.rpy.matrixToRpy(rotation)])

    def get_limits(self) -> ArmLimits:
        self._require_session()
        return ArmLimits(np.asarray(self.robot['joint_min'], dtype=float), np.asarray(self.robot['joint_max'], dtype=float),
                         float(self.robot['gripper_max_width']))

    def forward_kinematics(self, joint_position) -> np.ndarray:
        return pose6(self.model.fk(joint_position))

    def minimum_duration(self, delta_rad: float) -> float:
        return math.pi / 2 * abs(float(delta_rad)) / self.safety.max_commanded_velocity_rad_s

    def minimum_gripper_duration(self, delta_m: float) -> float:
        return math.pi / 2 * abs(float(delta_m)) / self.safety.max_commanded_gripper_velocity_m_s

    # Moves: kernel previews from the live state, executed and settled by the kernel.

    def _target(self, target, seed):
        if isinstance(target, Pose):
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = quaternion_matrix(target.orientation_xyzw), target.position_xyz_m
            try:
                return self.model.ik(transform, seed)
            except ValueError as exc:
                raise SafetyError(str(exc)) from exc
        return np.asarray(target, dtype=float)

    def _preview(self, kind, velocity_scale, **fields):
        if not 0 < velocity_scale <= 1:
            raise SafetyError('velocity_scale 必须在 (0, 1]。')
        speed = float(np.clip(velocity_scale * self.safety.max_commanded_velocity_rad_s, .01, MAX_SPEED))
        try:
            preview = self.session.preview(kind, speed=speed, gripper_width=self._gripper_width(), **fields)
        except RuntimeError as exc:
            raise SafetyError(f'内核拒绝规划：{exc}') from exc
        q = np.asarray(preview['q'])
        return KernelPlan(q[-1], np.asarray(preview['tcp'])[:, :3], float(preview['times'][-1]), preview)

    def plan_movej(self, target, *, velocity_scale: float = 1., allow_large_motion: bool = False) -> KernelPlan:
        self._require_session()
        current = self.get_state().joint_position
        goal = self._target(target, current)
        if not allow_large_motion and np.max(np.abs(goal - current)) > self.safety.max_relative_motion_rad:
            raise SafetyError('关节运动超过单次允许范围。')
        margin = self.safety.joint_limit_margin_rad
        if np.any(goal < np.asarray(self.robot['joint_min']) + margin) or np.any(goal > np.asarray(self.robot['joint_max']) - margin):
            raise SafetyError('目标关节超出限位余量。')
        return self._preview('movej', velocity_scale, q=goal.tolist())

    def plan_movel(self, target: Pose, *, unrestricted: bool = False, velocity_scale: float = 1.) -> KernelPlan:
        self._require_session()
        start = self.get_state().eef_pose_6d[:3]
        delta = np.asarray(target.position_xyz_m) - start
        if not unrestricted and (np.max(np.abs(delta)) > self.safety.max_cartesian_axis_motion_m
                                 or np.linalg.norm(delta) > self.safety.max_cartesian_motion_m):
            raise SafetyError('直线运动超过单次允许范围。')
        return self._preview('movel', velocity_scale,
                             pose=[*map(float, target.position_xyz_m), *map(float, target.orientation_xyzw)])

    def movej(self, target, *, velocity_scale: float = 1., allow_large_motion: bool = False) -> MotionResult:
        self._require_session()
        if self.mode is not ControllerMode.ACTIVE:
            raise ControlError('运动前先保持当前位置。')
        start = self.get_state().joint_position
        plan = self.plan_movej(target, velocity_scale=velocity_scale, allow_large_motion=allow_large_motion)
        with self.lock:  # Serialized with emergency_damping: either it lands first, or its cancel stops this move.
            if self.fault is not None:
                raise ControlError(f'运动已中止：{self.fault}')
            self.stream = None
            operation = self.session.execute(plan.preview)['operation_id']
        deadline, peak = time.monotonic() + plan.duration_s + 10, np.zeros(6)
        while True:
            if self.fault is not None:
                raise ControlError(f'运动已中止：{self.fault}')
            status = self.session.status()
            peak = np.maximum(peak, np.abs(status['dq']))
            # The execute ACK can precede the first snapshot that lists the operation.
            entry = next((op for op in status['operations'] if op['operation_id'] == operation), None)
            if entry is not None and entry['status'] != 'running':
                break
            if time.monotonic() > deadline:
                self.emergency_damping('运动超时')
                raise SafetyError('运动超时，已取消并保持。')
            time.sleep(.02)
        if entry['status'] != 'completed':
            reason = entry.get('reason') or entry['status']
            if 'settle' in reason:
                self.hold_current()
                raise MotionSettleError('joint did not settle at the trajectory target')
            self.emergency_damping(reason)
            raise SafetyError(f'内核运动{entry["status"]}：{reason}')
        if self.gripper_enabled:
            self._set_stream(('gripper', self._gripper_width()))
        return MotionResult(start, plan.target_joint_position, np.asarray(status['q']), plan.duration_s, peak, np.zeros(6))

    # Gripper: position targets; the arm holds while they stream.

    def _gripper_until(self, target_of, seconds, done=lambda status: False):
        start = time.monotonic()
        while True:
            if self.fault is not None:
                raise ControlError(f'夹爪已中止：{self.fault}')
            elapsed = time.monotonic() - start
            with self.lock:
                self.stream = ('gripper', target_of(elapsed))
                self._send(self.stream)
            status = self.session.status()
            if elapsed >= seconds or done(status):
                return status
            time.sleep(.02)

    def move_gripper(self, target_width_m: float, duration_s: float):
        self._require_session()
        if self.mode is not ControllerMode.ACTIVE:
            self.hold_current()
        limit = float(self.robot['gripper_max_width'])
        width = float(target_width_m)
        if not 0 <= width <= limit:
            raise SafetyError(f'夹爪宽度必须在 0..{limit:.3f} m。')
        start = float(self.session.status()['gripper_width'])
        minimum = max(self.safety.minimum_motion_duration_s, self.minimum_gripper_duration(width - start))
        if not minimum <= duration_s <= 300:
            raise SafetyError(f'夹爪运动时长必须在 {minimum:.2f}..300 s。')
        self.clamp, self.gripper_enabled = None, True
        profile = lambda t: start + (width - start) * (1 - math.cos(math.pi * min(1., t / duration_s))) / 2
        self._gripper_until(profile, duration_s)
        status = self._gripper_until(lambda t: width, 1.,
                                     lambda s: abs(s['gripper_width'] - width) < self.safety.gripper_position_tolerance_m)
        return SimpleNamespace(final_width_m=float(status['gripper_width']), target_width_m=width)

    def force_close_gripper(self, closing_torque_nm: float = .15, *, minimum_width_m: float = 0., timeout_s: float = 5.,
                            contact_hold_s: float = .75, position_hold_after: bool = True, torque_hold_after: bool = True):
        """Close until the fingers stop following (contact), the minimum width, or the timeout."""
        self._require_session()
        if self.mode is not ControllerMode.ACTIVE:
            self.hold_current()
        self.clamp, self.gripper_enabled = None, True
        start = float(self.session.status()['gripper_width'])
        floor = max(0., float(minimum_width_m))
        began, blocked = time.monotonic(), {'since': None}
        closing = lambda t: max(floor, start - CLOSE_SPEED_M_S * t)

        def contact(status):
            target = closing(time.monotonic() - began)
            lagging = status['gripper_width'] - target > CONTACT_LAG_M
            blocked['since'] = (blocked['since'] or time.monotonic()) if lagging else None
            settled = target <= floor and abs(status['gripper_width'] - floor) < CONTACT_LAG_M
            return settled or (blocked['since'] is not None
                               and time.monotonic() - blocked['since'] >= self.safety.force_close_stall_time_s)
        status = self._gripper_until(closing, timeout_s, contact)
        width = float(status['gripper_width'])
        reason = 'contact' if blocked['since'] is not None else 'minimum_width' if width - floor < CONTACT_LAG_M else 'timeout'
        if reason == 'contact' and (position_hold_after or torque_hold_after):
            self.clamp = max(floor, width - CLAMP_SQUEEZE_M)
            self._gripper_until(lambda t: self.clamp, contact_hold_s)
        else:
            self._set_stream(('gripper', width))
        return SimpleNamespace(stop_reason=reason, final_width_m=width, commanded_closing_torque_nm=float(closing_torque_nm),
                               max_abs_torque_nm=0.)
