"""Unified fixed-rate arm and gripper teleoperation worker."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
import time
from typing import Protocol

import numpy as np

from legacy.arx5_control.cartesian import rpy_to_quaternion_xyzw
from legacy.arx5_control.controller import ArmSnapshot, ControlError, ControllerMode, SafetyError
from embodied.motion.online import JointRuckigPlanner
from legacy.arx5_control.teleoperation import (
    GripperState,
    TeleoperationConfig,
    TeleoperationTarget,
    limit_teleoperation_target_step,
    quaternion_distance_rad,
    quaternion_multiply_xyzw,
    quaternion_slerp_xyzw,
)


class TeleoperationArm(Protocol):
    mode: ControllerMode

    def get_state(self) -> ArmSnapshot: ...

    def get_limits(self): ...

    def activate_gripper(self) -> ArmSnapshot: ...

    def _plan_cartesian_servo_delta(
        self,
        start_pose_6d: np.ndarray,
        start_joint_position: np.ndarray,
        delta_xyz_m: np.ndarray,
        seed_joint_position: np.ndarray,
        *,
        target_orientation_xyzw: np.ndarray | None = None,
    ): ...

    def _command_joint_servo(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        gripper_position: float,
    ) -> ArmSnapshot: ...

    def emergency_damping(self, reason: str) -> None: ...


@dataclass(frozen=True)
class TeleoperationServoStatus:
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
    target_orientation_xyzw: np.ndarray | None
    gripper_state: GripperState
    gripper_contact_latched: bool
    workspace_speed_scale: float
    cycle_period_s: float
    feedback_snapshot: ArmSnapshot | None


class TeleoperationServo:
    """Track one relative 6DoF pose and gripper state in one command loop."""

    def __init__(
        self,
        controller: TeleoperationArm,
        *,
        config: TeleoperationConfig,
        profile_name: str,
        rate_hz: float,
        watchdog_timeout_s: float,
        state_stale_timeout_s: float,
        gripper_velocity_m_s: float,
        gripper_acceleration_m_s2: float,
        gripper_jerk_m_s3: float,
        startup_gripper_tolerance_m: float = 0.0,
    ) -> None:
        if profile_name not in config.profiles:
            raise ValueError("unknown teleoperation profile")
        values = (
            rate_hz,
            watchdog_timeout_s,
            state_stale_timeout_s,
            gripper_velocity_m_s,
            gripper_acceleration_m_s2,
            gripper_jerk_m_s3,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("teleoperation servo configuration must be positive")
        if (
            not math.isfinite(startup_gripper_tolerance_m)
            or startup_gripper_tolerance_m < 0.0
        ):
            raise ValueError("startup gripper tolerance must be non-negative")
        self._controller = controller
        self._config = config
        self._profile = config.profiles[profile_name]
        self._period_s = 1.0 / rate_hz
        self._watchdog_timeout_s = watchdog_timeout_s
        self._state_stale_timeout_s = state_stale_timeout_s
        self._gripper_limits = (
            gripper_velocity_m_s,
            gripper_acceleration_m_s2,
            gripper_jerk_m_s3,
        )
        self._startup_gripper_tolerance_m = float(startup_gripper_tolerance_m)
        self._condition = threading.Condition()
        self._latest_target = TeleoperationTarget(
            np.zeros(3),
            np.array([0.0, 0.0, 0.0, 1.0]),
            GripperState.OPEN,
        )
        self._update_generation = 0
        self._processed_generation = 0
        self._planned_generation = 0
        self._last_update_time = 0.0
        self._active = False
        self._stopping = False
        self._stop_requested = False
        self._stop_reason: str | None = None
        self._error: str | None = None
        self._target_error: str | None = None
        self._thread: threading.Thread | None = None
        self._start_snapshot: ArmSnapshot | None = None
        self._latest_feedback: ArmSnapshot | None = None
        self._target_joint_position: np.ndarray | None = None
        self._target_delta_xyz_m: np.ndarray | None = None
        self._target_orientation_xyzw: np.ndarray | None = None
        self._gripper_state = GripperState.OPEN
        self._gripper_contact_latched = False
        self._workspace_speed_scale = 1.0
        self._cycle_period_s = 0.0
        self._open_width_m = 0.0
        self._closing_torque_nm = 0.15

    def start(
        self,
        *,
        open_width_m: float,
        closing_torque_nm: float,
    ) -> ArmSnapshot:
        with self._condition:
            if self._thread is not None:
                raise ControlError("teleoperation servo is already started")
        if self._controller.mode != ControllerMode.ACTIVE:
            raise ControlError("teleoperation servo requires active position control")
        if not math.isfinite(closing_torque_nm) or closing_torque_nm <= 0.0:
            raise ValueError("closing torque must be positive")
        limits = self._controller.get_limits()
        maximum_width = float(limits.gripper_width_m)
        if not math.isfinite(open_width_m) or not 0.0 <= open_width_m <= maximum_width:
            raise ValueError("open gripper width is outside its configured range")
        snapshot = self._controller.get_state()
        if not self._config.workspace.contains(snapshot.eef_pose_6d[:3]):
            raise SafetyError("current TCP is outside the configured workspace")
        tolerance = self._startup_gripper_tolerance_m
        if not -tolerance <= snapshot.gripper_position <= maximum_width + tolerance:
            raise SafetyError("current gripper position is outside its configured range")
        snapshot = self._controller.activate_gripper()
        if not -tolerance <= snapshot.gripper_position <= maximum_width + tolerance:
            raise SafetyError("current gripper position is outside its configured range")
        snapshot = replace(
            snapshot,
            gripper_position=float(
                np.clip(snapshot.gripper_position, 0.0, maximum_width)
            ),
        )
        now = time.monotonic()
        with self._condition:
            self._start_snapshot = snapshot
            self._latest_feedback = snapshot
            self._target_joint_position = snapshot.joint_position.copy()
            self._target_delta_xyz_m = np.zeros(3, dtype=np.float64)
            self._target_orientation_xyzw = np.array([0.0, 0.0, 0.0, 1.0])
            self._gripper_state = GripperState.OPEN
            self._gripper_contact_latched = False
            self._workspace_speed_scale = self._config.workspace.speed_scale(
                snapshot.eef_pose_6d[:3]
            )
            self._open_width_m = float(open_width_m)
            self._closing_torque_nm = float(closing_torque_nm)
            self._last_update_time = now
            self._active = True
            self._thread = threading.Thread(
                target=self._run,
                name="arx5-teleoperation-servo",
                daemon=True,
            )
            self._thread.start()
        return snapshot

    def update_target(self, target: TeleoperationTarget) -> int:
        if not isinstance(target, TeleoperationTarget):
            raise TypeError("teleoperation target has an invalid type")
        copied = TeleoperationTarget(
            target.delta_xyz_m,
            target.delta_orientation_xyzw,
            target.gripper_state,
        )
        with self._condition:
            if not self._active or self._stopping:
                raise ControlError("teleoperation servo is not accepting targets")
            self._latest_target = copied
            self._update_generation += 1
            self._last_update_time = time.monotonic()
            self._condition.notify_all()
            return self._update_generation

    def wait_for_target(
        self,
        generation: int,
        timeout_s: float,
    ) -> TeleoperationServoStatus:
        if not isinstance(generation, int) or generation < 1:
            raise ValueError("teleoperation target generation must be positive")
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("teleoperation target timeout must be non-negative")
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
                    f"teleoperation servo did not stop within {timeout_s:.1f}s"
                )
                thread.join(1.0)

    def status(self) -> TeleoperationServoStatus:
        with self._condition:
            return self._status_locked()

    def _status_locked(self) -> TeleoperationServoStatus:
        age = (
            max(0.0, time.monotonic() - self._last_update_time)
            if self._last_update_time > 0.0
            else 0.0
        )
        return TeleoperationServoStatus(
            active=self._active,
            stopping=self._stopping,
            stop_reason=self._stop_reason,
            error=self._error,
            update_age_s=age,
            processed_generation=self._processed_generation,
            target_generation=self._planned_generation,
            target_error=self._target_error,
            target_joint_position=self._copy_array(self._target_joint_position),
            target_delta_xyz_m=self._copy_array(self._target_delta_xyz_m),
            target_orientation_xyzw=self._copy_array(
                self._target_orientation_xyzw
            ),
            gripper_state=self._gripper_state,
            gripper_contact_latched=self._gripper_contact_latched,
            workspace_speed_scale=self._workspace_speed_scale,
            cycle_period_s=self._cycle_period_s,
            feedback_snapshot=self._copy_snapshot(self._latest_feedback),
        )

    @staticmethod
    def _copy_array(value: np.ndarray | None) -> np.ndarray | None:
        return None if value is None else np.asarray(value).copy()

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
            maximum_gripper_width = float(limits.gripper_width_m)
            controller_safety = getattr(self._controller, "safety", None)
            joint_velocity_max = np.minimum(
                np.asarray(limits.joint_velocity_max, dtype=np.float64),
                min(
                    self._profile.joint_velocity_rad_s,
                    float(
                        getattr(
                            controller_safety,
                            "servo_max_velocity_rad_s",
                            math.inf,
                        )
                    ),
                ),
            )
            joint_acceleration_max = min(
                self._profile.joint_acceleration_rad_s2,
                float(
                    getattr(
                        controller_safety,
                        "servo_max_acceleration_rad_s2",
                        math.inf,
                    )
                ),
            )
            joint_jerk_max = min(
                self._profile.joint_jerk_rad_s3,
                float(
                    getattr(
                        controller_safety,
                        "servo_max_jerk_rad_s3",
                        math.inf,
                    )
                ),
            )
            arm_planner = JointRuckigPlanner(
                snapshot.joint_position,
                snapshot.joint_velocity,
                initial_acceleration=getattr(snapshot, "joint_acceleration", None),
                period_s=self._period_s,
                max_velocity=joint_velocity_max,
                max_acceleration=joint_acceleration_max,
                max_jerk=joint_jerk_max,
                min_position=np.asarray(limits.joint_position_min),
                max_position=np.asarray(limits.joint_position_max),
            )
            gripper_velocity, gripper_acceleration, gripper_jerk = (
                self._gripper_limits
            )
            gripper_planner = JointRuckigPlanner(
                np.array([snapshot.gripper_position]),
                np.array(
                    [
                        np.clip(
                            snapshot.gripper_velocity,
                            -gripper_velocity,
                            gripper_velocity,
                        )
                    ]
                ),
                period_s=self._period_s,
                max_velocity=gripper_velocity,
                max_acceleration=gripper_acceleration,
                max_jerk=gripper_jerk,
                min_position=np.array([0.0]),
                max_position=np.array([maximum_gripper_width]),
            )
            start_pose = np.asarray(snapshot.eef_pose_6d).copy()
            start_joint = np.asarray(snapshot.joint_position).copy()
            start_orientation = rpy_to_quaternion_xyzw(start_pose[3:])
            seed = start_joint.copy()
            last_accepted_target = TeleoperationTarget(
                np.zeros(3),
                np.array([0.0, 0.0, 0.0, 1.0]),
                GripperState.OPEN,
            )
            last_target_process_time = time.monotonic()
            processed_generation = 0
            commanded_gripper_state: GripperState | None = None
            active_gripper_state: GripperState | None = None
            deadline = time.monotonic()
            last_cycle_start: float | None = None
            last_feedback_timestamp = float(snapshot.timestamp)
            last_feedback_progress = time.monotonic()

            while True:
                now = time.monotonic()
                with self._condition:
                    target = self._latest_target
                    generation = self._update_generation
                    update_age = now - self._last_update_time
                    stop_requested = self._stop_requested
                    stop_reason = self._stop_reason
                    if last_cycle_start is not None:
                        self._cycle_period_s = now - last_cycle_start
                    last_cycle_start = now

                if self._controller.mode != ControllerMode.ACTIVE:
                    raise ControlError("teleoperation servo lost active controller mode")

                watchdog_expired = update_age > self._watchdog_timeout_s
                global_stop = stop_requested or watchdog_expired
                if global_stop:
                    if not arm_planner.braking:
                        arm_planner.brake()
                    if not gripper_planner.braking:
                        gripper_planner.brake()
                    with self._condition:
                        self._stopping = True
                        if stop_reason is None and self._stop_reason is None:
                            self._stop_reason = "target watchdog timeout"
                elif generation != processed_generation:
                    active_gripper_state = target.gripper_state
                    commanded_gripper_state = self._apply_gripper_state(
                        gripper_planner,
                        active_gripper_state,
                        commanded_gripper_state,
                    )
                    with self._condition:
                        self._gripper_state = active_gripper_state
                    try:
                        requested_position = start_pose[:3] + target.delta_xyz_m
                        if not self._config.workspace.contains(requested_position):
                            raise SafetyError(
                                "teleoperation target is outside the configured workspace"
                            )
                        bounded_target = limit_teleoperation_target_step(
                            last_accepted_target,
                            target,
                            self._profile,
                            now - last_target_process_time,
                        )
                        target_position = (
                            start_pose[:3] + bounded_target.delta_xyz_m
                        )
                        workspace_scale = self._config.workspace.speed_scale(
                            target_position
                        )
                        if workspace_scale <= 0.0:
                            raise SafetyError(
                                "teleoperation target reaches the workspace boundary"
                            )
                        is_hold_target = (
                            float(np.linalg.norm(bounded_target.delta_xyz_m)) < 0.0001
                            and quaternion_distance_rad(
                                bounded_target.delta_orientation_xyzw,
                                np.array([0.0, 0.0, 0.0, 1.0]),
                            )
                            < math.radians(0.05)
                        )
                        plan = None
                        if not is_hold_target:
                            orientation_error: SafetyError | None = None
                            orientation_fractions = (1.0, 0.75, 0.5, 0.25, 0.0)
                            plan_cartesian = (
                                self._controller._plan_cartesian_servo_delta
                            )
                            for orientation_fraction in orientation_fractions:
                                relative_orientation = quaternion_slerp_xyzw(
                                    last_accepted_target.delta_orientation_xyzw,
                                    bounded_target.delta_orientation_xyzw,
                                    orientation_fraction,
                                )
                                target_orientation = quaternion_multiply_xyzw(
                                    relative_orientation,
                                    start_orientation,
                                )
                                try:
                                    plan = plan_cartesian(
                                        start_pose,
                                        start_joint,
                                        bounded_target.delta_xyz_m,
                                        seed,
                                        target_orientation_xyzw=target_orientation,
                                    )
                                except SafetyError as exc:
                                    if not str(exc).startswith(
                                        "IK orientation residual is too large"
                                    ):
                                        raise
                                    orientation_error = exc
                                    continue
                                bounded_target = replace(
                                    bounded_target,
                                    delta_orientation_xyzw=relative_orientation,
                                )
                                break
                            if plan is None:
                                assert orientation_error is not None
                                raise orientation_error
                            arm_planner.set_limit_scale(
                                min(
                                    workspace_scale,
                                    float(
                                        getattr(plan, "singularity_speed_scale", 1.0)
                                    ),
                                )
                            )
                            arm_planner.set_target(plan.target_joint_position)
                    except (ControlError, SafetyError, ValueError) as exc:
                        processed_generation = generation
                        with self._condition:
                            self._processed_generation = generation
                            self._target_error = str(exc)
                            self._condition.notify_all()
                    else:
                        if plan is not None:
                            seed = plan.target_joint_position.copy()
                        last_accepted_target = bounded_target
                        last_target_process_time = now
                        processed_generation = generation
                        with self._condition:
                            self._processed_generation = generation
                            self._planned_generation = generation
                            self._target_error = None
                            self._target_joint_position = seed.copy()
                            self._target_delta_xyz_m = (
                                bounded_target.delta_xyz_m.copy()
                            )
                            self._target_orientation_xyzw = (
                                bounded_target.delta_orientation_xyzw.copy()
                            )
                            self._workspace_speed_scale = workspace_scale

                if active_gripper_state is not None:
                    commanded_gripper_state = self._apply_gripper_state(
                        gripper_planner,
                        active_gripper_state,
                        commanded_gripper_state,
                    )
                if (
                    not global_stop
                    and active_gripper_state == GripperState.CLOSE
                    and not self._gripper_contact_latched
                ):
                    contact = (
                        abs(snapshot.gripper_torque) >= self._closing_torque_nm
                    )
                    velocity = float(gripper_planner.velocity[0])
                    stopping_distance = (
                        velocity * velocity / (2.0 * gripper_acceleration)
                    )
                    at_limit = (
                        float(gripper_planner.position[0])
                        <= stopping_distance + 0.0002
                    )
                    if contact or at_limit:
                        gripper_planner.brake()
                        with self._condition:
                            self._gripper_contact_latched = True

                arm_command = arm_planner.step()
                gripper_command = gripper_planner.step()
                gripper_position = float(
                    np.clip(
                        gripper_command.position[0],
                        0.0,
                        maximum_gripper_width,
                    )
                )
                feedback = self._controller._command_joint_servo(
                    arm_command.position,
                    arm_command.velocity,
                    gripper_position,
                )
                snapshot = feedback
                with self._condition:
                    self._latest_feedback = feedback
                    if generation == processed_generation:
                        self._condition.notify_all()

                feedback_timestamp = float(feedback.timestamp)
                progress_time = time.monotonic()
                if feedback_timestamp > last_feedback_timestamp + 1e-6:
                    last_feedback_timestamp = feedback_timestamp
                    last_feedback_progress = progress_time
                elif (
                    progress_time - last_feedback_progress
                    > self._state_stale_timeout_s
                ):
                    raise SafetyError(
                        "teleoperation servo feedback timestamp stopped"
                    )
                if (
                    global_stop
                    and arm_planner.braking
                    and gripper_planner.braking
                    and arm_command.finished
                    and gripper_command.finished
                ):
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
                self._stop_reason = self._stop_reason or "teleoperation servo fault"
            if self._controller.mode == ControllerMode.ACTIVE:
                self._controller.emergency_damping(
                    f"teleoperation servo failed: {exc}"
                )
        finally:
            with self._condition:
                self._active = False
                self._stopping = False
                self._condition.notify_all()

    def _apply_gripper_state(
        self,
        planner: JointRuckigPlanner,
        state: GripperState,
        previous: GripperState | None,
    ) -> GripperState:
        if state == previous:
            return state
        if state == GripperState.OPEN:
            planner.set_target(np.array([self._open_width_m]))
            with self._condition:
                self._gripper_contact_latched = False
        elif not self._gripper_contact_latched:
            planner.set_velocity_target(
                np.array([-self._gripper_limits[0]], dtype=np.float64)
            )
        return state


__all__ = ["TeleoperationServo", "TeleoperationServoStatus"]
