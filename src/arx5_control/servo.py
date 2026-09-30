"""Jerk-limited online joint planning for Cartesian drag servoing."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Protocol

import numpy as np

from .controller import (
    ArmSnapshot,
    ControlError,
    ControllerMode,
    SafetyError,
)
from .motion.limits import scaled_derivative_limits
from .motion.online import JointRuckigPlanner, JointServoCommand
from .motion.contracts import Pose


class ServoArm(Protocol):
    """The subset of :class:`ArmController` used by the worker thread."""

    mode: ControllerMode

    def get_state(self) -> ArmSnapshot: ...

    def get_limits(self): ...

    def _plan_cartesian_servo_delta(
        self,
        start_pose_6d: np.ndarray,
        start_joint_position: np.ndarray,
        delta_xyz_m: np.ndarray,
        seed_joint_position: np.ndarray,
    ): ...

    def plan_servop_target(
        self,
        target: Pose,
        seed_joint_position: np.ndarray,
    ): ...

    def _command_joint_servo(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        gripper_position: float,
    ) -> ArmSnapshot: ...

    def emergency_damping(self, reason: str) -> None: ...


@dataclass(frozen=True)
class CartesianServoStatus:
    active: bool
    stopping: bool
    stop_reason: str | None
    error: str | None
    update_age_s: float
    processed_generation: int
    target_generation: int
    target_error: str | None
    target_joint_position: np.ndarray | None
    target_delta_xyz_m: np.ndarray | None
    feedback_snapshot: ArmSnapshot | None
    target_tcp_pose: Pose | None


class CartesianServo:
    """Fixed-period server-side servo that follows the newest Cartesian target."""

    def __init__(
        self,
        controller: ServoArm,
        *,
        rate_hz: float,
        max_velocity_rad_s: float,
        max_acceleration_rad_s2: float,
        max_jerk_rad_s3: float,
        joint_limit_margin_rad: float,
        watchdog_timeout_s: float,
        state_stale_timeout_s: float,
        velocity_scale: float = 1.0,
    ) -> None:
        values = (
            rate_hz,
            max_velocity_rad_s,
            max_acceleration_rad_s2,
            max_jerk_rad_s3,
            joint_limit_margin_rad,
            watchdog_timeout_s,
            state_stale_timeout_s,
            velocity_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("Cartesian servo configuration must be positive")
        if velocity_scale > 1.0:
            raise ValueError("Cartesian servo velocity_scale must not exceed 1")
        self._controller = controller
        self._period_s = 1.0 / float(rate_hz)
        (
            self._max_velocity,
            self._max_acceleration,
            self._max_jerk,
        ) = scaled_derivative_limits(
            max_velocity_rad_s,
            max_acceleration_rad_s2,
            max_jerk_rad_s3,
            velocity_scale,
        )
        self._joint_limit_margin = float(joint_limit_margin_rad)
        self._watchdog_timeout_s = float(watchdog_timeout_s)
        self._state_stale_timeout_s = float(state_stale_timeout_s)
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._start_snapshot: ArmSnapshot | None = None
        self._latest_delta = np.zeros(3, dtype=np.float64)
        self._latest_pose: Pose | None = None
        self._latest_velocity_scale = 1.0
        self._update_generation = 0
        self._processed_generation = 0
        self._planned_generation = 0
        self._target_error: str | None = None
        self._target_joint_position: np.ndarray | None = None
        self._target_delta_xyz_m: np.ndarray | None = None
        self._target_tcp_pose: Pose | None = None
        self._latest_feedback: ArmSnapshot | None = None
        self._last_update_time = 0.0
        self._stop_requested = False
        self._stop_reason: str | None = None
        self._error: str | None = None
        self._active = False
        self._stopping = False
        self._stop_limit_scale = 1.0

    def start(self) -> ArmSnapshot:
        with self._condition:
            if self._thread is not None:
                raise ControlError("Cartesian servo is already started")
        if self._controller.mode != ControllerMode.ACTIVE:
            raise ControlError("Cartesian servo requires active position control")
        snapshot = self._controller.get_state()
        if np.max(np.abs(snapshot.joint_velocity)) > self._max_velocity + 1e-9:
            raise SafetyError("joint velocity is too high to start Cartesian servo")
        now = time.monotonic()
        with self._condition:
            self._start_snapshot = snapshot
            self._latest_feedback = snapshot
            self._target_joint_position = snapshot.joint_position.copy()
            self._target_delta_xyz_m = np.zeros(3, dtype=np.float64)
            self._last_update_time = now
            self._active = True
            self._thread = threading.Thread(
                target=self._run,
                name="arx5-cartesian-servo",
                daemon=True,
            )
            self._thread.start()
        return snapshot

    def update_target(
        self,
        delta_xyz_m: np.ndarray,
        *,
        velocity_scale: float = 1.0,
    ) -> int:
        delta = np.asarray(delta_xyz_m, dtype=np.float64).copy()
        if delta.shape != (3,) or not np.all(np.isfinite(delta)):
            raise ValueError("servo XYZ target must contain three finite values")
        self._validate_velocity_scale(velocity_scale)
        with self._condition:
            if not self._active or self._stopping:
                raise ControlError("Cartesian servo is not accepting targets")
            self._latest_delta = delta
            self._latest_pose = None
            self._latest_velocity_scale = float(velocity_scale)
            self._update_generation += 1
            self._last_update_time = time.monotonic()
            self._condition.notify_all()
            return self._update_generation

    def update_pose_target(
        self,
        target: Pose,
        *,
        velocity_scale: float = 1.0,
    ) -> int:
        if not isinstance(target, Pose):
            raise TypeError("servop target must be a Pose")
        self._validate_velocity_scale(velocity_scale)
        with self._condition:
            if not self._active or self._stopping:
                raise ControlError("Cartesian servo is not accepting targets")
            self._latest_pose = target
            self._latest_velocity_scale = float(velocity_scale)
            self._update_generation += 1
            self._last_update_time = time.monotonic()
            self._condition.notify_all()
            return self._update_generation

    @staticmethod
    def _validate_velocity_scale(velocity_scale: float) -> None:
        if (
            not math.isfinite(velocity_scale)
            or not 0.0 < velocity_scale <= 1.0
        ):
            raise ValueError("servo velocity scale must be in (0, 1]")

    def stop(
        self,
        reason: str = "client stop",
        *,
        emergency: bool = False,
        timeout_s: float = 3.0,
        deceleration_scale: float = 1.0,
    ) -> None:
        if (
            not math.isfinite(deceleration_scale)
            or not 0.0 < deceleration_scale <= 1.0
        ):
            raise ValueError("servo deceleration scale must be in (0, 1]")
        with self._condition:
            thread = self._thread
            if thread is None:
                return
            self._stop_requested = True
            self._stop_reason = reason
            self._stop_limit_scale = float(deceleration_scale)
            self._condition.notify_all()
        if emergency:
            self._controller.emergency_damping(reason)
        if thread is not threading.current_thread():
            thread.join(timeout_s)
            if thread.is_alive():
                self._controller.emergency_damping(
                    f"Cartesian servo did not stop within {timeout_s:.1f}s"
                )
                thread.join(1.0)

    def status(self) -> CartesianServoStatus:
        with self._condition:
            return self._status_locked()

    def wait_for_target(
        self,
        generation: int,
        timeout_s: float,
    ) -> CartesianServoStatus:
        """Wait for the worker to accept or reject a target for UI preview."""
        if not isinstance(generation, int) or generation < 1:
            raise ValueError("servo target generation must be positive")
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("servo preview timeout must be non-negative")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while (
                self._processed_generation < generation
                and self._active
                and not self._stopping
                and self._error is None
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
            return self._status_locked()

    def _status_locked(self) -> CartesianServoStatus:
        age = (
            max(0.0, time.monotonic() - self._last_update_time)
            if self._last_update_time > 0.0
            else 0.0
        )
        return CartesianServoStatus(
            active=self._active,
            stopping=self._stopping,
            stop_reason=self._stop_reason,
            error=self._error,
            update_age_s=age,
            processed_generation=self._processed_generation,
            target_generation=self._planned_generation,
            target_error=self._target_error,
            target_joint_position=(
                None
                if self._target_joint_position is None
                else self._target_joint_position.copy()
            ),
            target_delta_xyz_m=(
                None
                if self._target_delta_xyz_m is None
                else self._target_delta_xyz_m.copy()
            ),
            feedback_snapshot=self._copy_snapshot(self._latest_feedback),
            target_tcp_pose=self._target_tcp_pose,
        )

    @staticmethod
    def _copy_snapshot(snapshot: ArmSnapshot | None) -> ArmSnapshot | None:
        if snapshot is None:
            return None
        return ArmSnapshot(
            timestamp=float(snapshot.timestamp),
            joint_position=np.asarray(snapshot.joint_position).copy(),
            joint_velocity=np.asarray(snapshot.joint_velocity).copy(),
            joint_torque=np.asarray(snapshot.joint_torque).copy(),
            gripper_position=float(snapshot.gripper_position),
            gripper_velocity=float(snapshot.gripper_velocity),
            gripper_torque=float(snapshot.gripper_torque),
            eef_pose_6d=np.asarray(snapshot.eef_pose_6d).copy(),
            joint_acceleration=(
                None
                if getattr(snapshot, "joint_acceleration", None) is None
                else np.asarray(snapshot.joint_acceleration).copy()
            ),
        )

    def _run(self) -> None:
        snapshot = self._start_snapshot
        if snapshot is None:  # pragma: no cover - guarded by start()
            return
        try:
            limits = self._controller.get_limits()
            minimum = np.asarray(limits.joint_position_min, dtype=np.float64)
            maximum = np.asarray(limits.joint_position_max, dtype=np.float64)
            minimum += self._joint_limit_margin
            maximum -= self._joint_limit_margin
            joint_velocity_max = getattr(limits, "joint_velocity_max", None)
            if joint_velocity_max is None:
                joint_velocity_max = self._max_velocity
            else:
                joint_velocity_max = np.minimum(
                    np.asarray(joint_velocity_max, dtype=np.float64),
                    self._max_velocity,
                )
            planner = JointRuckigPlanner(
                snapshot.joint_position,
                snapshot.joint_velocity,
                initial_acceleration=getattr(snapshot, "joint_acceleration", None),
                period_s=self._period_s,
                max_velocity=joint_velocity_max,
                max_acceleration=self._max_acceleration,
                max_jerk=self._max_jerk,
                min_position=minimum,
                max_position=maximum,
            )
            processed_generation = 0
            seed = snapshot.joint_position.copy()
            deadline = time.monotonic()
            last_feedback_timestamp = float(snapshot.timestamp)
            last_feedback_progress = time.monotonic()
            while True:
                now = time.monotonic()
                with self._condition:
                    delta = self._latest_delta.copy()
                    pose_target = self._latest_pose
                    velocity_scale = self._latest_velocity_scale
                    generation = self._update_generation
                    update_age = now - self._last_update_time
                    stop_requested = self._stop_requested
                    stop_reason = self._stop_reason

                if self._controller.mode != ControllerMode.ACTIVE:
                    raise ControlError("Cartesian servo lost active controller mode")
                if stop_requested or update_age > self._watchdog_timeout_s:
                    if not planner.braking:
                        planner.set_brake_scale(self._stop_limit_scale)
                        planner.brake()
                        with self._condition:
                            self._stopping = True
                            if stop_reason is None:
                                self._stop_reason = "target watchdog timeout"
                elif generation != processed_generation:
                    try:
                        if pose_target is not None:
                            plan = self._controller.plan_servop_target(
                                pose_target,
                                seed,
                            )
                            seed = plan.target_joint_position.copy()
                        elif np.linalg.norm(delta) < 0.0001:
                            seed = snapshot.joint_position.copy()
                        else:
                            plan = self._controller._plan_cartesian_servo_delta(
                                snapshot.eef_pose_6d,
                                snapshot.joint_position,
                                delta,
                                seed,
                            )
                            seed = plan.target_joint_position.copy()
                    except (ControlError, SafetyError) as exc:
                        processed_generation = generation
                        with self._condition:
                            self._processed_generation = generation
                            self._target_error = str(exc)
                            self._condition.notify_all()
                    else:
                        planner.set_limit_scale(
                            velocity_scale
                            * float(getattr(plan, "singularity_speed_scale", 1.0))
                        )
                        planner.set_target(seed)
                        processed_generation = generation
                        with self._condition:
                            self._processed_generation = generation
                            self._planned_generation = generation
                            self._target_error = None
                            self._target_joint_position = seed.copy()
                            self._target_delta_xyz_m = delta.copy()
                            self._target_tcp_pose = pose_target
                            self._condition.notify_all()

                command = planner.step()
                feedback = self._controller._command_joint_servo(
                    command.position,
                    command.velocity,
                    snapshot.gripper_position,
                )
                with self._condition:
                    self._latest_feedback = feedback
                feedback_timestamp = float(feedback.timestamp)
                progress_time = time.monotonic()
                if feedback_timestamp > last_feedback_timestamp + 1e-6:
                    last_feedback_timestamp = feedback_timestamp
                    last_feedback_progress = progress_time
                elif progress_time - last_feedback_progress > self._state_stale_timeout_s:
                    raise SafetyError("Cartesian servo feedback timestamp stopped")
                if planner.braking and command.finished:
                    break

                deadline += self._period_s
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                elif remaining < -2.0 * self._period_s:
                    deadline = time.monotonic()
        except BaseException as exc:
            with self._condition:
                self._error = str(exc)
                self._stop_reason = self._stop_reason or "servo fault"
            if self._controller.mode == ControllerMode.ACTIVE:
                self._controller.emergency_damping(f"Cartesian servo failed: {exc}")
        finally:
            with self._condition:
                self._active = False
                self._stopping = False
                self._condition.notify_all()


@dataclass(frozen=True)
class JointServoStatus:
    active: bool
    stopping: bool
    stop_reason: str | None
    error: str | None
    update_age_s: float
    processed_generation: int
    target_generation: int
    target_joint_position: np.ndarray | None
    feedback_snapshot: ArmSnapshot | None


class JointServo:
    """Fixed-period jerk-limited servo for absolute joint targets."""

    def __init__(
        self,
        controller: ServoArm,
        *,
        rate_hz: float,
        max_velocity_rad_s: float,
        max_acceleration_rad_s2: float,
        max_jerk_rad_s3: float,
        joint_limit_margin_rad: float,
        watchdog_timeout_s: float,
        state_stale_timeout_s: float,
        velocity_scale: float = 1.0,
    ) -> None:
        values = (
            rate_hz,
            max_velocity_rad_s,
            max_acceleration_rad_s2,
            max_jerk_rad_s3,
            joint_limit_margin_rad,
            watchdog_timeout_s,
            state_stale_timeout_s,
            velocity_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("joint servo configuration must be positive")
        if velocity_scale > 1.0:
            raise ValueError("joint servo velocity_scale must not exceed 1")
        self._controller = controller
        self._period_s = 1.0 / float(rate_hz)
        self._max_velocity, self._max_acceleration, self._max_jerk = (
            scaled_derivative_limits(
                max_velocity_rad_s,
                max_acceleration_rad_s2,
                max_jerk_rad_s3,
                velocity_scale,
            )
        )
        self._joint_limit_margin = float(joint_limit_margin_rad)
        self._watchdog_timeout_s = float(watchdog_timeout_s)
        self._state_stale_timeout_s = float(state_stale_timeout_s)
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._start_snapshot: ArmSnapshot | None = None
        self._latest_target: np.ndarray | None = None
        self._latest_velocity_scale = 1.0
        self._latest_feedback: ArmSnapshot | None = None
        self._update_generation = 0
        self._processed_generation = 0
        self._last_update_time = 0.0
        self._stop_requested = False
        self._stop_reason: str | None = None
        self._stop_limit_scale = 1.0
        self._error: str | None = None
        self._active = False
        self._stopping = False

    def start(self) -> ArmSnapshot:
        with self._condition:
            if self._thread is not None:
                raise ControlError("joint servo is already started")
        if self._controller.mode != ControllerMode.ACTIVE:
            raise ControlError("joint servo requires active position control")
        snapshot = self._controller.get_state()
        if np.max(np.abs(snapshot.joint_velocity)) > self._max_velocity + 1e-9:
            raise SafetyError("joint velocity is too high to start joint servo")
        now = time.monotonic()
        with self._condition:
            self._start_snapshot = snapshot
            self._latest_feedback = snapshot
            self._latest_target = snapshot.joint_position.copy()
            self._last_update_time = now
            self._active = True
            self._thread = threading.Thread(
                target=self._run,
                name="arx5-joint-servo",
                daemon=True,
            )
            self._thread.start()
        return snapshot

    def update_target(
        self,
        target_joint_position: np.ndarray,
        *,
        velocity_scale: float = 1.0,
    ) -> int:
        target = np.asarray(target_joint_position, dtype=np.float64).copy()
        if target.ndim != 1 or not np.all(np.isfinite(target)):
            raise ValueError("servoj target must be a finite joint vector")
        if (
            not math.isfinite(velocity_scale)
            or not 0.0 < velocity_scale <= 1.0
        ):
            raise ValueError("servo velocity scale must be in (0, 1]")
        with self._condition:
            if not self._active or self._stopping:
                raise ControlError("joint servo is not accepting targets")
            start = self._start_snapshot
            if start is None or target.shape != start.joint_position.shape:
                raise ValueError("servoj target has the wrong joint count")
            self._latest_target = target
            self._latest_velocity_scale = float(velocity_scale)
            self._update_generation += 1
            self._last_update_time = time.monotonic()
            self._condition.notify_all()
            return self._update_generation

    def wait_for_target(
        self,
        generation: int,
        timeout_s: float,
    ) -> JointServoStatus:
        if not isinstance(generation, int) or generation < 1:
            raise ValueError("servo target generation must be positive")
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("servo target timeout must be non-negative")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while (
                self._processed_generation < generation
                and self._active
                and not self._stopping
                and self._error is None
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
            return self._status_locked()

    def stop(
        self,
        reason: str = "client stop",
        *,
        emergency: bool = False,
        timeout_s: float = 3.0,
        deceleration_scale: float = 1.0,
    ) -> None:
        if (
            not math.isfinite(deceleration_scale)
            or not 0.0 < deceleration_scale <= 1.0
        ):
            raise ValueError("servo deceleration scale must be in (0, 1]")
        with self._condition:
            thread = self._thread
            if thread is None:
                return
            self._stop_requested = True
            self._stop_reason = reason
            self._stop_limit_scale = float(deceleration_scale)
            self._condition.notify_all()
        if emergency:
            self._controller.emergency_damping(reason)
        if thread is not threading.current_thread():
            thread.join(timeout_s)
            if thread.is_alive():
                self._controller.emergency_damping(
                    f"joint servo did not stop within {timeout_s:.1f}s"
                )
                thread.join(1.0)

    def status(self) -> JointServoStatus:
        with self._condition:
            return self._status_locked()

    def _status_locked(self) -> JointServoStatus:
        age = (
            max(0.0, time.monotonic() - self._last_update_time)
            if self._last_update_time > 0.0
            else 0.0
        )
        return JointServoStatus(
            active=self._active,
            stopping=self._stopping,
            stop_reason=self._stop_reason,
            error=self._error,
            update_age_s=age,
            processed_generation=self._processed_generation,
            target_generation=self._update_generation,
            target_joint_position=(
                None if self._latest_target is None else self._latest_target.copy()
            ),
            feedback_snapshot=CartesianServo._copy_snapshot(self._latest_feedback),
        )

    def _run(self) -> None:
        snapshot = self._start_snapshot
        if snapshot is None:  # pragma: no cover - guarded by start()
            return
        try:
            limits = self._controller.get_limits()
            minimum = np.asarray(limits.joint_position_min, dtype=np.float64)
            maximum = np.asarray(limits.joint_position_max, dtype=np.float64)
            minimum += self._joint_limit_margin
            maximum -= self._joint_limit_margin
            joint_velocity_max = getattr(limits, "joint_velocity_max", None)
            if joint_velocity_max is None:
                joint_velocity_max = self._max_velocity
            else:
                joint_velocity_max = np.minimum(
                    np.asarray(joint_velocity_max, dtype=np.float64),
                    self._max_velocity,
                )
            planner = JointRuckigPlanner(
                snapshot.joint_position,
                snapshot.joint_velocity,
                initial_acceleration=getattr(snapshot, "joint_acceleration", None),
                period_s=self._period_s,
                max_velocity=joint_velocity_max,
                max_acceleration=self._max_acceleration,
                max_jerk=self._max_jerk,
                min_position=minimum,
                max_position=maximum,
            )
            processed_generation = 0
            deadline = time.monotonic()
            last_feedback_timestamp = float(snapshot.timestamp)
            last_feedback_progress = time.monotonic()
            while True:
                now = time.monotonic()
                with self._condition:
                    target = (
                        None
                        if self._latest_target is None
                        else self._latest_target.copy()
                    )
                    generation = self._update_generation
                    velocity_scale = self._latest_velocity_scale
                    update_age = now - self._last_update_time
                    stop_requested = self._stop_requested
                    stop_reason = self._stop_reason

                if self._controller.mode != ControllerMode.ACTIVE:
                    raise ControlError("joint servo lost active controller mode")
                if stop_requested or update_age > self._watchdog_timeout_s:
                    if not planner.braking:
                        planner.set_brake_scale(self._stop_limit_scale)
                        planner.brake()
                        with self._condition:
                            self._stopping = True
                            if stop_reason is None:
                                self._stop_reason = "target watchdog timeout"
                elif generation != processed_generation and target is not None:
                    planner.set_limit_scale(velocity_scale)
                    planner.set_target(target)
                    processed_generation = generation
                    with self._condition:
                        self._processed_generation = generation
                        self._condition.notify_all()

                command = planner.step()
                feedback = self._controller._command_joint_servo(
                    command.position,
                    command.velocity,
                    snapshot.gripper_position,
                )
                with self._condition:
                    self._latest_feedback = feedback
                feedback_timestamp = float(feedback.timestamp)
                progress_time = time.monotonic()
                if feedback_timestamp > last_feedback_timestamp + 1e-6:
                    last_feedback_timestamp = feedback_timestamp
                    last_feedback_progress = progress_time
                elif progress_time - last_feedback_progress > self._state_stale_timeout_s:
                    raise SafetyError("joint servo feedback timestamp stopped")
                if planner.braking and command.finished:
                    break

                deadline += self._period_s
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                elif remaining < -2.0 * self._period_s:
                    deadline = time.monotonic()
        except BaseException as exc:
            with self._condition:
                self._error = str(exc)
                self._stop_reason = self._stop_reason or "servo fault"
            if self._controller.mode == ControllerMode.ACTIVE:
                self._controller.emergency_damping(f"joint servo failed: {exc}")
        finally:
            with self._condition:
                self._active = False
                self._stopping = False
                self._condition.notify_all()


@dataclass(frozen=True)
class GripperServoStatus:
    active: bool
    stopping: bool
    stop_reason: str | None
    error: str | None
    update_age_s: float
    direction: int
    feedback_snapshot: ArmSnapshot | None


class GripperServo:
    """Hold-to-move gripper control with jerk-limited start and stop."""

    def __init__(
        self,
        controller: ServoArm,
        *,
        rate_hz: float,
        max_velocity_m_s: float,
        max_acceleration_m_s2: float,
        max_jerk_m_s3: float,
        watchdog_timeout_s: float,
        state_stale_timeout_s: float,
        velocity_scale: float = 1.0,
    ) -> None:
        values = (
            rate_hz,
            max_velocity_m_s,
            max_acceleration_m_s2,
            max_jerk_m_s3,
            watchdog_timeout_s,
            state_stale_timeout_s,
            velocity_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("gripper servo configuration must be positive")
        if velocity_scale > 1.0:
            raise ValueError("gripper servo velocity_scale must not exceed 1")
        self._controller = controller
        self._period_s = 1.0 / float(rate_hz)
        (
            self._max_velocity,
            self._max_acceleration,
            self._max_jerk,
        ) = scaled_derivative_limits(
            max_velocity_m_s,
            max_acceleration_m_s2,
            max_jerk_m_s3,
            velocity_scale,
        )
        self._watchdog_timeout_s = float(watchdog_timeout_s)
        self._state_stale_timeout_s = float(state_stale_timeout_s)
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._start_snapshot: ArmSnapshot | None = None
        self._latest_feedback: ArmSnapshot | None = None
        self._direction = 0
        self._closing_torque_nm = 0.15
        self._minimum_width_m = 0.0
        self._last_update_time = 0.0
        self._stop_requested = False
        self._stop_reason: str | None = None
        self._error: str | None = None
        self._active = False
        self._stopping = False

    @staticmethod
    def _validate_direction(direction: int) -> int:
        if isinstance(direction, bool) or direction not in (-1, 1):
            raise ValueError("gripper servo direction must be -1 or 1")
        return int(direction)

    def start(
        self,
        direction: int,
        *,
        closing_torque_nm: float = 0.15,
        minimum_width_m: float = 0.0,
    ) -> ArmSnapshot:
        direction = self._validate_direction(direction)
        if not math.isfinite(closing_torque_nm) or closing_torque_nm <= 0.0:
            raise ValueError("gripper closing torque must be positive")
        with self._condition:
            if self._thread is not None:
                raise ControlError("gripper servo is already started")
        if self._controller.mode != ControllerMode.ACTIVE:
            raise ControlError("gripper servo requires active position control")
        snapshot = self._controller.get_state()
        limits = self._controller.get_limits()
        maximum_width = float(limits.gripper_width_m)
        if not math.isfinite(minimum_width_m) or not 0.0 <= minimum_width_m <= maximum_width:
            raise ValueError("minimum gripper width is outside its configured range")
        if not 0.0 <= snapshot.gripper_position <= maximum_width:
            raise SafetyError("current gripper position is outside its configured range")
        now = time.monotonic()
        with self._condition:
            self._start_snapshot = snapshot
            self._latest_feedback = snapshot
            self._direction = direction
            self._closing_torque_nm = float(closing_torque_nm)
            self._minimum_width_m = float(minimum_width_m)
            self._last_update_time = now
            self._active = True
            self._thread = threading.Thread(
                target=self._run,
                name="arx5-gripper-servo",
                daemon=True,
            )
            self._thread.start()
        return snapshot

    def update_direction(self, direction: int) -> None:
        direction = self._validate_direction(direction)
        with self._condition:
            if not self._active or self._stopping:
                raise ControlError("gripper servo is not accepting updates")
            self._direction = direction
            self._last_update_time = time.monotonic()
            self._condition.notify_all()

    def stop(
        self,
        reason: str = "client stop",
        *,
        emergency: bool = False,
        timeout_s: float = 3.0,
    ) -> None:
        with self._condition:
            thread = self._thread
            if thread is None:
                return
            self._stop_requested = True
            self._stop_reason = reason
            self._condition.notify_all()
        if emergency:
            self._controller.emergency_damping(reason)
        if thread is not threading.current_thread():
            thread.join(timeout_s)
            if thread.is_alive():
                self._controller.emergency_damping(
                    f"gripper servo did not stop within {timeout_s:.1f}s"
                )
                thread.join(1.0)

    def status(self) -> GripperServoStatus:
        with self._condition:
            age = (
                max(0.0, time.monotonic() - self._last_update_time)
                if self._last_update_time > 0.0
                else 0.0
            )
            return GripperServoStatus(
                active=self._active,
                stopping=self._stopping,
                stop_reason=self._stop_reason,
                error=self._error,
                update_age_s=age,
                direction=self._direction,
                feedback_snapshot=CartesianServo._copy_snapshot(self._latest_feedback),
            )

    def _run(self) -> None:
        snapshot = self._start_snapshot
        if snapshot is None:  # pragma: no cover - guarded by start()
            return
        try:
            maximum_width = float(self._controller.get_limits().gripper_width_m)
            minimum_width = self._minimum_width_m
            planner_minimum = min(minimum_width, snapshot.gripper_position)
            initial_velocity = float(np.clip(
                snapshot.gripper_velocity,
                -self._max_velocity,
                self._max_velocity,
            ))
            planner = JointRuckigPlanner(
                np.array([snapshot.gripper_position], dtype=np.float64),
                np.array([initial_velocity], dtype=np.float64),
                period_s=self._period_s,
                max_velocity=self._max_velocity,
                max_acceleration=self._max_acceleration,
                max_jerk=self._max_jerk,
                min_position=np.array([planner_minimum]),
                max_position=np.array([maximum_width]),
            )
            last_direction = 0
            deadline = time.monotonic()
            last_feedback_timestamp = float(snapshot.timestamp)
            last_feedback_progress = time.monotonic()
            while True:
                now = time.monotonic()
                with self._condition:
                    direction = self._direction
                    update_age = now - self._last_update_time
                    stop_requested = self._stop_requested

                if self._controller.mode != ControllerMode.ACTIVE:
                    raise ControlError("gripper servo lost active controller mode")

                contact = direction < 0 and abs(snapshot.gripper_torque) >= self._closing_torque_nm
                velocity = float(planner.velocity[0])
                distance = (
                    maximum_width - float(planner.position[0])
                    if direction > 0
                    else float(planner.position[0]) - minimum_width
                )
                stopping_distance = velocity * velocity / (2.0 * self._max_acceleration)
                at_limit = distance <= stopping_distance + 0.0002
                watchdog_expired = update_age > self._watchdog_timeout_s
                should_stop = stop_requested or watchdog_expired or contact or at_limit

                if should_stop and not planner.braking:
                    planner.brake()
                    with self._condition:
                        self._stopping = True
                        if self._stop_reason is None:
                            if contact:
                                self._stop_reason = "closing torque reached"
                            elif at_limit:
                                self._stop_reason = "gripper travel limit"
                            elif watchdog_expired:
                                self._stop_reason = "target watchdog timeout"
                            else:
                                self._stop_reason = "client stop"
                elif not should_stop and direction != last_direction:
                    planner.set_velocity_target(
                        np.array([direction * self._max_velocity], dtype=np.float64)
                    )
                    last_direction = direction

                command = planner.step()
                gripper_position = float(
                    np.clip(command.position[0], planner_minimum, maximum_width)
                )
                feedback = self._controller._command_joint_servo(
                    snapshot.joint_position,
                    np.zeros_like(snapshot.joint_position),
                    gripper_position,
                )
                snapshot = feedback
                with self._condition:
                    self._latest_feedback = feedback

                feedback_timestamp = float(feedback.timestamp)
                progress_time = time.monotonic()
                if feedback_timestamp > last_feedback_timestamp + 1e-6:
                    last_feedback_timestamp = feedback_timestamp
                    last_feedback_progress = progress_time
                elif progress_time - last_feedback_progress > self._state_stale_timeout_s:
                    raise SafetyError("gripper servo feedback timestamp stopped")
                if planner.braking and command.finished:
                    break

                deadline += self._period_s
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                elif remaining < -2.0 * self._period_s:
                    deadline = time.monotonic()
        except BaseException as exc:
            with self._condition:
                self._error = str(exc)
                self._stop_reason = self._stop_reason or "gripper servo fault"
            if self._controller.mode == ControllerMode.ACTIVE:
                self._controller.emergency_damping(f"gripper servo failed: {exc}")
        finally:
            with self._condition:
                self._active = False
                self._stopping = False
                self._condition.notify_all()
