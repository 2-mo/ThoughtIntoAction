"""Persistent, safety-oriented controller for one ARX X5 arm.

This layer intentionally omits homing and calibration.  It connects once,
keeps the SDK object alive between commands, and changes position stiffness
gradually to avoid repeated active/damping mode shocks.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
import fcntl
import math
import os
from pathlib import Path
import threading
import time
from typing import Sequence

import arx5_interface as arx5
import numpy as np

from .cartesian import (
    CartesianPlan,
    CartesianPlanner,
    CartesianPlanningError,
    quaternion_xyzw_to_rpy,
    rpy_to_quaternion_xyzw,
)
from .errors import BusyError, ControlError, MotionSettleError, SafetyError
from .kdl_urdf import kdl_compatible_urdf_path
from .motion import (
    RuckigJointTrajectory,
    TrajectorySample,
    analyze_sampled_motion,
    cosine_fraction,
    cosine_segment,
    minimum_cosine_duration,
    plan_blended_joint_trajectory,
    plan_ruckig_joint_trajectory,
    plan_scalar_progress,
    resample_quintic_hermite,
    PrimitiveObstacle,
    CartesianGeometryPath,
    CartesianPathMode,
    CartesianPathSegment,
    CartesianWaypoint,
    LinkCapsule,
    MotionContext,
    MotionPlan,
    MotionType,
    Payload,
    Pose,
    ReferenceFrame,
    ToolCenterPoint,
    build_cartesian_path,
    build_circular_path,
    check_tcp_path,
    concatenate_cartesian_paths,
    round_trip,
    scaled_derivative_limits,
    via_point,
)


class ControllerMode(str, Enum):
    DISCONNECTED = "disconnected"
    DAMPING = "damping"
    ACTIVE = "active"
    FAULT = "fault"
    CLOSED = "closed"


@dataclass(frozen=True)
class SafetyConfig:
    """Conservative application-level limits for early hardware tests."""

    max_relative_motion_rad: float = math.radians(30.0)
    max_commanded_velocity_rad_s: float = 0.25
    max_commanded_cartesian_velocity_m_s: float = 0.020
    max_commanded_cartesian_acceleration_m_s2: float = 0.060
    max_commanded_cartesian_jerk_m_s3: float = 0.60
    cartesian_waypoint_rate_hz: float = 20.0
    max_cartesian_axis_motion_m: float = 0.060
    max_cartesian_motion_m: float = 0.080
    max_cartesian_joint_delta_rad: float = math.radians(30.0)
    cartesian_fk_position_tolerance_m: float = 0.001
    cartesian_fk_orientation_tolerance_rad: float = math.radians(1.2)
    cartesian_jacobian_step_rad: float = 1e-5
    cartesian_singularity_warn: float = 0.030
    cartesian_singularity_stop: float = 0.005
    cartesian_singularity_min_speed_scale: float = 0.25
    cartesian_geometry_step_m: float = 0.003
    cartesian_orientation_step_rad: float = math.radians(2.0)
    cartesian_max_ik_step_rad: float = math.radians(12.0)
    link_collision_clearance_m: float = 0.003
    link_collision_joint_step_rad: float = math.radians(0.5)
    feedback_velocity_limit_rad_s: float = 0.60
    torque_limit_ratio: float = 0.80
    max_commanded_gripper_velocity_m_s: float = 0.06
    feedback_gripper_velocity_limit_m_s: float = 0.12
    gripper_servo_max_acceleration_m_s2: float = 0.60
    gripper_servo_max_jerk_m_s3: float = 6.0
    gripper_position_tolerance_m: float = 0.005
    gripper_torque_limit_ratio: float = 0.80
    max_force_close_torque_nm: float = 0.30
    force_close_position_margin_m: float = 0.001
    force_close_stall_velocity_m_s: float = 0.002
    force_close_stall_time_s: float = 0.25
    force_ramp_s: float = 0.40
    joint_limit_margin_rad: float = math.radians(1.0)
    non_target_drift_rad: float = math.radians(5.0)
    target_overshoot_margin_rad: float = math.radians(3.0)
    state_stale_timeout_s: float = 0.50
    trajectory_rate_hz: float = 50.0
    monitor_period_s: float = 0.02
    preview_time_s: float = 0.20
    settle_time_s: float = 0.30
    trajectory_final_hold_s: float = 0.20
    terminal_position_tolerance_rad: float = math.radians(1.0)
    terminal_velocity_tolerance_rad_s: float = 0.05
    terminal_stability_window_s: float = 0.12
    terminal_position_stability_rad: float = math.radians(0.15)
    minimum_motion_duration_s: float = 2.0
    servo_rate_hz: float = 100.0
    servo_max_velocity_rad_s: float = 0.15
    servo_max_acceleration_rad_s2: float = 0.30
    servo_max_jerk_rad_s3: float = 1.00
    servo_tracking_error_rad: float = math.radians(7.5)
    servo_watchdog_timeout_s: float = 0.20

    def __post_init__(self) -> None:
        positive = {
            "max_relative_motion_rad": self.max_relative_motion_rad,
            "max_commanded_velocity_rad_s": self.max_commanded_velocity_rad_s,
            "max_commanded_cartesian_velocity_m_s": (
                self.max_commanded_cartesian_velocity_m_s
            ),
            "max_commanded_cartesian_acceleration_m_s2": (
                self.max_commanded_cartesian_acceleration_m_s2
            ),
            "max_commanded_cartesian_jerk_m_s3": (
                self.max_commanded_cartesian_jerk_m_s3
            ),
            "cartesian_waypoint_rate_hz": self.cartesian_waypoint_rate_hz,
            "max_cartesian_axis_motion_m": self.max_cartesian_axis_motion_m,
            "max_cartesian_motion_m": self.max_cartesian_motion_m,
            "max_cartesian_joint_delta_rad": self.max_cartesian_joint_delta_rad,
            "cartesian_fk_position_tolerance_m": (
                self.cartesian_fk_position_tolerance_m
            ),
            "cartesian_fk_orientation_tolerance_rad": (
                self.cartesian_fk_orientation_tolerance_rad
            ),
            "cartesian_jacobian_step_rad": self.cartesian_jacobian_step_rad,
            "cartesian_singularity_warn": self.cartesian_singularity_warn,
            "cartesian_singularity_stop": self.cartesian_singularity_stop,
            "cartesian_geometry_step_m": self.cartesian_geometry_step_m,
            "cartesian_orientation_step_rad": self.cartesian_orientation_step_rad,
            "cartesian_max_ik_step_rad": self.cartesian_max_ik_step_rad,
            "link_collision_joint_step_rad": self.link_collision_joint_step_rad,
            "feedback_velocity_limit_rad_s": self.feedback_velocity_limit_rad_s,
            "max_commanded_gripper_velocity_m_s": (
                self.max_commanded_gripper_velocity_m_s
            ),
            "feedback_gripper_velocity_limit_m_s": (
                self.feedback_gripper_velocity_limit_m_s
            ),
            "gripper_servo_max_acceleration_m_s2": (
                self.gripper_servo_max_acceleration_m_s2
            ),
            "gripper_servo_max_jerk_m_s3": self.gripper_servo_max_jerk_m_s3,
            "gripper_position_tolerance_m": self.gripper_position_tolerance_m,
            "max_force_close_torque_nm": self.max_force_close_torque_nm,
            "force_close_position_margin_m": (
                self.force_close_position_margin_m
            ),
            "force_close_stall_velocity_m_s": (
                self.force_close_stall_velocity_m_s
            ),
            "force_close_stall_time_s": self.force_close_stall_time_s,
            "force_ramp_s": self.force_ramp_s,
            "joint_limit_margin_rad": self.joint_limit_margin_rad,
            "non_target_drift_rad": self.non_target_drift_rad,
            "target_overshoot_margin_rad": self.target_overshoot_margin_rad,
            "state_stale_timeout_s": self.state_stale_timeout_s,
            "trajectory_rate_hz": self.trajectory_rate_hz,
            "monitor_period_s": self.monitor_period_s,
            "preview_time_s": self.preview_time_s,
            "settle_time_s": self.settle_time_s,
            "trajectory_final_hold_s": self.trajectory_final_hold_s,
            "terminal_position_tolerance_rad": (
                self.terminal_position_tolerance_rad
            ),
            "terminal_velocity_tolerance_rad_s": (
                self.terminal_velocity_tolerance_rad_s
            ),
            "terminal_stability_window_s": self.terminal_stability_window_s,
            "terminal_position_stability_rad": (
                self.terminal_position_stability_rad
            ),
            "minimum_motion_duration_s": self.minimum_motion_duration_s,
            "servo_rate_hz": self.servo_rate_hz,
            "servo_max_velocity_rad_s": self.servo_max_velocity_rad_s,
            "servo_max_acceleration_rad_s2": self.servo_max_acceleration_rad_s2,
            "servo_max_jerk_rad_s3": self.servo_max_jerk_rad_s3,
            "servo_tracking_error_rad": self.servo_tracking_error_rad,
            "servo_watchdog_timeout_s": self.servo_watchdog_timeout_s,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if not 0.0 < self.torque_limit_ratio <= 1.0:
            raise ValueError("torque_limit_ratio must be in (0, 1]")
        if not 0.0 < self.gripper_torque_limit_ratio <= 1.0:
            raise ValueError("gripper_torque_limit_ratio must be in (0, 1]")
        if not 0.0 < self.cartesian_singularity_min_speed_scale <= 1.0:
            raise ValueError("cartesian_singularity_min_speed_scale must be in (0, 1]")
        if self.cartesian_singularity_warn <= self.cartesian_singularity_stop:
            raise ValueError("cartesian singularity warn threshold must exceed stop threshold")
        if (
            not math.isfinite(self.link_collision_clearance_m)
            or self.link_collision_clearance_m < 0.0
        ):
            raise ValueError("link_collision_clearance_m must be non-negative")
        if self.servo_max_velocity_rad_s > self.max_commanded_velocity_rad_s:
            raise ValueError(
                "servo_max_velocity_rad_s must not exceed the commanded velocity limit"
            )
        if self.servo_max_velocity_rad_s >= self.feedback_velocity_limit_rad_s:
            raise ValueError(
                "servo velocity limit must stay below the feedback abort limit"
            )
        if self.terminal_stability_window_s > self.settle_time_s:
            raise ValueError(
                "terminal stability window must not exceed settle time"
            )


@dataclass(frozen=True)
class ArmSnapshot:
    timestamp: float
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    joint_torque: np.ndarray
    gripper_position: float
    gripper_velocity: float
    gripper_torque: float
    eef_pose_6d: np.ndarray
    # SDK publishes position and velocity only. This is a finite-difference
    # estimate when two advancing feedback timestamps are available.
    joint_acceleration: np.ndarray | None = None


@dataclass(frozen=True)
class ArmLimits:
    joint_position_min: np.ndarray
    joint_position_max: np.ndarray
    gripper_width_m: float
    joint_velocity_max: np.ndarray | None = None


@dataclass(frozen=True)
class MotionResult:
    start_position: np.ndarray
    target_position: np.ndarray
    final_position: np.ndarray
    commanded_duration_s: float
    max_abs_velocity_rad_s: np.ndarray
    max_abs_torque_nm: np.ndarray

    @property
    def final_error_rad(self) -> np.ndarray:
        return self.final_position - self.target_position


@dataclass(frozen=True)
class JointTrajectoryPreview:
    """A checked, non-executing joint trajectory and its TCP path."""

    start_joint_position: np.ndarray
    target_joint_position: np.ndarray
    joint_positions: np.ndarray
    path_xyz_m: np.ndarray
    duration_s: float
    smooth_trajectory: RuckigJointTrajectory | None = None
    cartesian_plan: CartesianPlan | None = None

    @property
    def max_joint_delta_rad(self) -> float:
        return float(
            np.max(np.abs(self.target_joint_position - self.start_joint_position))
        )


@dataclass(frozen=True)
class CartesianMotionResult:
    """Result of a preplanned straight XYZ path executed as one trajectory."""

    plan: CartesianPlan
    joint_motion: MotionResult
    final_pose_6d: np.ndarray
    target_tcp_pose_6d: np.ndarray | None = None

    @property
    def final_position_error_m(self) -> float:
        target = (
            self.plan.target_pose_6d
            if self.target_tcp_pose_6d is None
            else self.target_tcp_pose_6d
        )
        return float(
            np.linalg.norm(self.final_pose_6d[:3] - target[:3])
        )


@dataclass(frozen=True)
class CartesianWaypointTrajectoryPreview:
    """Fully checked blended Cartesian path ready for SDK submission."""

    plan: CartesianPlan
    geometry: CartesianGeometryPath
    path_xyz_m: np.ndarray
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    sample_times_s: np.ndarray
    duration_s: float
    canonical_joint_positions: np.ndarray
    canonical_sample_times_s: np.ndarray


@dataclass(frozen=True)
class GripperMotionResult:
    start_width_m: float
    excursion_width_m: float
    target_width_m: float
    final_width_m: float
    commanded_duration_s: float
    min_measured_width_m: float
    max_measured_width_m: float
    max_abs_velocity_m_s: float
    max_abs_torque_nm: float

    @property
    def final_error_m(self) -> float:
        return self.final_width_m - self.target_width_m


@dataclass(frozen=True)
class GripperForceResult:
    start_width_m: float
    final_width_m: float
    minimum_width_m: float
    commanded_closing_torque_nm: float
    elapsed_s: float
    stop_reason: str
    max_abs_velocity_m_s: float
    max_abs_torque_nm: float
    position_hold_enabled: bool


class _InterfaceLock:
    """Same device ownership boundary as the default arx runtime."""
    def __init__(self, interface: str) -> None:
        self.interface = interface
        self._lease = None

    def acquire(self) -> None:
        if self._lease is None:
            from embodied_contracts.device_lease import DeviceLease
            try:
                self._lease = DeviceLease(self.interface)
            except RuntimeError as exc:
                raise BusyError(str(exc)) from exc

    def release(self) -> None:
        if self._lease is not None:
            self._lease.close()
            self._lease = None


class ArmController:
    """One long-lived ARX X5 joint controller.

    Normal motion completion leaves the controller active and holding its last
    target.  Only ``damping()`` or ``close()`` releases position stiffness.
    """

    GAIN_UPDATE_RATE_HZ = 100.0
    EXTENDED_MOTION_MAX_DURATION_S = 300.0

    def __init__(
        self,
        model: str = "X5",
        interface: str = "can0",
        *,
        gain_scale: float = 0.50,
        gain_ramp_s: float = 1.50,
        gripper_gain_scale: float = 0.50,
        gripper_gain_ramp_s: float = 1.00,
        gripper_open_readout: float | None = None,
        safety: SafetyConfig | None = None,
        tcp_collision_obstacles: Sequence[PrimitiveObstacle] = (),
        tcp_collision_clearance_m: float = 0.0,
        enable_link_collision: bool = False,
        link_collision_capsules: Sequence[LinkCapsule] | None = None,
    ) -> None:
        if not 0.1 <= gain_scale <= 1.0:
            raise ValueError("gain_scale must be in [0.1, 1.0]")
        if not 0.5 <= gain_ramp_s <= 5.0:
            raise ValueError("gain_ramp_s must be in [0.5, 5.0]")
        if not 0.1 <= gripper_gain_scale <= 1.0:
            raise ValueError("gripper_gain_scale must be in [0.1, 1.0]")
        if not 0.5 <= gripper_gain_ramp_s <= 5.0:
            raise ValueError("gripper_gain_ramp_s must be in [0.5, 5.0]")
        if gripper_open_readout is not None:
            if (
                not math.isfinite(gripper_open_readout)
                or not 0.5 <= abs(gripper_open_readout) <= 10.0
            ):
                raise ValueError(
                    "gripper_open_readout magnitude must be in [0.5, 10.0]"
                )

        self.model = model
        self.interface = interface
        self.gain_scale = float(gain_scale)
        self.gain_ramp_s = float(gain_ramp_s)
        self.gripper_gain_scale = float(gripper_gain_scale)
        self.gripper_gain_ramp_s = float(gripper_gain_ramp_s)
        self.gripper_open_readout = (
            None
            if gripper_open_readout is None
            else float(gripper_open_readout)
        )
        self.safety = safety or SafetyConfig()
        if (
            not math.isfinite(tcp_collision_clearance_m)
            or tcp_collision_clearance_m < 0.0
        ):
            raise ValueError("tcp_collision_clearance_m must be non-negative and finite")
        self._tcp_collision_obstacles = tuple(tcp_collision_obstacles)
        self._tcp_collision_clearance_m = float(tcp_collision_clearance_m)
        self._enable_link_collision = bool(enable_link_collision)
        self._link_collision_capsules = (
            None
            if link_collision_capsules is None
            else tuple(link_collision_capsules)
        )

        self._sdk: arx5.Arx5JointController | None = None
        self._robot_config: arx5.RobotConfig | None = None
        self._controller_config: arx5.ControllerConfig | None = None
        self._cartesian_planner: CartesianPlanner | None = None
        self._lock = _InterfaceLock(interface)
        self._mode = ControllerMode.DISCONNECTED
        self._current_arm_gain_scale = 0.0
        self._current_gripper_gain_scale = 0.0
        self._gripper_force_hold_position_m: float | None = None
        self._gripper_force_hold_motor_torque_nm = 0.0
        self._fault_reason: str | None = None
        self._last_feedback_for_acceleration: tuple[float, np.ndarray] | None = None
        self._snapshot_lock = threading.Lock()
        self._latest_snapshot: ArmSnapshot | None = None
        # Web emergency damping may run from a different worker thread.  This
        # lock makes mode checks and outgoing SDK commands one atomic action,
        # so an interrupted operation cannot issue another command afterward.
        self._mode_lock = threading.RLock()
        self._fault_generation = 0
        self._motion_context = MotionContext()
        self._motion_in_progress = False
        self._active_servo_kind: str | None = None
        self._servo_context_revision: int | None = None
        self._joint_servo = None
        self._cartesian_servo = None
        self._trajectory_stop_kind: str | None = None
        self._trajectory_stop_scale = 1.0
        self._trajectory_stop_requested = threading.Event()
        self._trajectory_stop_completed = threading.Event()

    @property
    def mode(self) -> ControllerMode:
        return self._mode

    @property
    def fault_reason(self) -> str | None:
        return self._fault_reason

    @property
    def is_connected(self) -> bool:
        return self._sdk is not None

    @property
    def kinematics_backend(self) -> str:
        planner = self._cartesian_planner
        return "disconnected" if planner is None else planner.kinematics_backend

    @property
    def collision_backend(self) -> str:
        planner = self._cartesian_planner
        return "disconnected" if planner is None else planner.collision_backend

    @property
    def current_gain_scale(self) -> float:
        """Backward-compatible alias for the arm position gain scale."""
        return self._current_arm_gain_scale

    @property
    def current_arm_gain_scale(self) -> float:
        return self._current_arm_gain_scale

    @property
    def current_gripper_gain_scale(self) -> float:
        return self._current_gripper_gain_scale

    @property
    def gripper_force_hold_active(self) -> bool:
        return self._gripper_force_hold_position_m is not None

    @property
    def motion_context(self) -> MotionContext:
        return self._motion_context

    def _set_motion_context(self, **changes: object) -> MotionContext:
        with self._mode_lock:
            self._clear_finished_servo_sessions_locked()
            if self._motion_in_progress:
                raise BusyError("cannot change motion context during motion")
            self._motion_context = self._motion_context.updated(**changes)
            return self._motion_context

    def set_tcp(self, tcp: ToolCenterPoint) -> MotionContext:
        if not isinstance(tcp, ToolCenterPoint):
            raise TypeError("tcp must be a ToolCenterPoint")
        return self._set_motion_context(tcp=tcp)

    def set_payload(self, payload: Payload) -> MotionContext:
        if not isinstance(payload, Payload):
            raise TypeError("payload must be a Payload")
        return self._set_motion_context(payload=payload)

    def set_reference_frame(self, frame: ReferenceFrame) -> MotionContext:
        if not isinstance(frame, ReferenceFrame):
            raise TypeError("frame must be a ReferenceFrame")
        return self._set_motion_context(reference_frame=frame)

    def set_speed_override(self, override: float) -> MotionContext:
        if not math.isfinite(override) or not 0.0 < override <= 1.0:
            raise ValueError("speed override must be in (0, 1]")
        return self._set_motion_context(speed_override=float(override))

    def reconfigure_safety(self, safety: SafetyConfig) -> None:
        """Atomically apply an idle-time safety envelope.

        Some Cartesian planner limits are copied when the planner is created.
        Assigning ``controller.safety`` directly after ``connect()`` therefore
        used to leave the planner enforcing an older joint-limit margin.  That
        is especially harmful for a taught hand-eye center near, but still
        inside, a joint limit.  Rebuild only while idle so the runtime safety
        object and the planner always describe the same envelope.
        """
        if not isinstance(safety, SafetyConfig):
            raise TypeError("safety must be a SafetyConfig")
        with self._mode_lock:
            self._clear_finished_servo_sessions_locked()
            if self._motion_in_progress:
                raise BusyError("cannot change safety limits during motion")
            self.safety = safety
            if self._robot_config is not None:
                self._cartesian_planner = self._build_cartesian_planner(
                    self._robot_config
                )

    def _build_cartesian_planner(
        self,
        robot_config: arx5.RobotConfig,
    ) -> CartesianPlanner:
        """Create a planner from the current, already validated safety config."""
        return CartesianPlanner(
            robot_config,
            joint_limit_margin_rad=self.safety.joint_limit_margin_rad,
            max_axis_motion_m=self.safety.max_cartesian_axis_motion_m,
            max_motion_m=self.safety.max_cartesian_motion_m,
            max_joint_delta_rad=self.safety.max_cartesian_joint_delta_rad,
            position_tolerance_m=self.safety.cartesian_fk_position_tolerance_m,
            orientation_tolerance_rad=(
                self.safety.cartesian_fk_orientation_tolerance_rad
            ),
            jacobian_step_rad=self.safety.cartesian_jacobian_step_rad,
            singularity_warn=self.safety.cartesian_singularity_warn,
            singularity_stop=self.safety.cartesian_singularity_stop,
            singularity_min_speed_scale=(
                self.safety.cartesian_singularity_min_speed_scale
            ),
            max_ik_step_rad=self.safety.cartesian_max_ik_step_rad,
            tcp_obstacles=self._tcp_collision_obstacles,
            tcp_clearance_m=self._tcp_collision_clearance_m,
            use_pinocchio=True,
            enable_link_collision=self._enable_link_collision,
            link_collision_clearance_m=self.safety.link_collision_clearance_m,
            link_collision_joint_step_rad=(
                self.safety.link_collision_joint_step_rad
            ),
            link_collision_capsules=self._link_collision_capsules,
        )

    @staticmethod
    def _pose_from_pose_6d(pose_6d: np.ndarray) -> Pose:
        value = np.asarray(pose_6d, dtype=np.float64)
        if value.shape != (6,) or not np.all(np.isfinite(value)):
            raise SafetyError("controller returned an invalid flange pose")
        return Pose(value[:3], rpy_to_quaternion_xyzw(value[3:]))

    @staticmethod
    def _pose_to_pose_6d(pose: Pose) -> np.ndarray:
        if not isinstance(pose, Pose):
            raise TypeError("pose must be a Pose")
        return np.concatenate(
            (pose.position_xyz_m, quaternion_xyzw_to_rpy(pose.orientation_xyzw))
        )

    def _tcp_pose_from_snapshot(
        self,
        snapshot: ArmSnapshot,
        context: MotionContext,
    ) -> Pose:
        flange = self._pose_from_pose_6d(snapshot.eef_pose_6d)
        return context.flange_pose_to_tcp(flange)

    def get_tcp_pose(self, *, in_reference_frame: bool = True) -> Pose:
        context = self._motion_context
        base_tcp = self._tcp_pose_from_snapshot(self.get_state(), context)
        if not in_reference_frame:
            return base_tcp
        return context.reference_frame.pose_in_base.inverse().compose(base_tcp)

    def servoj_status(self):
        with self._mode_lock:
            self._clear_finished_servo_sessions_locked()
            return None if self._joint_servo is None else self._joint_servo.status()

    def servop_status(self):
        with self._mode_lock:
            self._clear_finished_servo_sessions_locked()
            return (
                None
                if self._cartesian_servo is None
                else self._cartesian_servo.status()
            )

    def _gripper_hold_command(
        self, fallback_position_m: float
    ) -> tuple[float, float]:
        position = self._gripper_force_hold_position_m
        if position is None:
            return float(fallback_position_m), 0.0
        return position, self._gripper_force_hold_motor_torque_nm

    def _clear_gripper_force_hold(self) -> None:
        self._gripper_force_hold_position_m = None
        self._gripper_force_hold_motor_torque_nm = 0.0

    def connect(self, *, stabilize: bool = False) -> ArmSnapshot:
        """Connect in damping, optionally holding the first measured arm pose.

        ``stabilize`` is intended for interactive control surfaces.  It closes
        the otherwise unbounded damping window after SDK initialization, where
        gravity feedforward can move a joint before the operator enables hold.
        Calibration and teaching callers keep the default damping behavior.
        """
        if self._mode not in (ControllerMode.DISCONNECTED,):
            raise ControlError(f"cannot connect while mode is {self._mode.value}")

        self._lock.acquire()
        try:
            robot_config = arx5.RobotConfigFactory.get_instance().get_config(
                self.model
            )
            # The vendor model puts an inertia on the KDL tree root. Use a
            # generated massless fixed parent so KDL keeps that inertia and
            # does not emit its misleading root-inertia warning. This changes
            # neither the configured base frame nor any X5 dynamics result.
            robot_config.urdf_path = kdl_compatible_urdf_path(
                robot_config.urdf_path,
                robot_config.base_link_name,
            )
            if self.gripper_open_readout is not None:
                robot_config.gripper_open_readout = self.gripper_open_readout
            controller_config = (
                arx5.ControllerConfigFactory.get_instance().get_config(
                    "joint_controller", robot_config.joint_dof
                )
            )
            controller_config.background_send_recv = True
            # Cubic SDK interpolation consumes waypoint velocities. Linear
            # mode would discard Ruckig's derivative profile between points.
            controller_config.interpolation_method = "cubic"

            self._robot_config = robot_config
            self._controller_config = controller_config
            self._cartesian_planner = self._build_cartesian_planner(robot_config)
            self._sdk = arx5.Arx5JointController(
                robot_config, controller_config, self.interface
            )
            snapshot = self._read_snapshot()
            self._validate_snapshot(snapshot)
            self._mode = ControllerMode.DAMPING
            self._current_arm_gain_scale = 0.0
            self._current_gripper_gain_scale = 0.0
            if stabilize:
                # Use this first validated feedback as the hold target.  Do not
                # wait for another read: even a short damping dwell can let J4
                # move under the SDK's additive gravity feedforward.
                self._set_joint_cmd_in_mode(
                    self._make_joint_state(
                        snapshot.joint_position,
                        snapshot.gripper_position,
                        0.0,
                    ),
                    ControllerMode.DAMPING,
                )
                self._ramp_gains(
                    0.0,
                    self.gain_scale,
                    0.0,
                    0.0,
                    self.gain_ramp_s,
                )
                with self._mode_lock:
                    if self._mode != ControllerMode.DAMPING:
                        raise ControlError("startup stabilization was interrupted")
                    self._mode = ControllerMode.ACTIVE
                snapshot = self.get_state()
            return snapshot
        except BaseException:
            sdk = self._sdk
            self._sdk = None
            if sdk is not None:
                try:
                    sdk.set_to_damping()
                except Exception:
                    pass
                del sdk
            self._robot_config = None
            self._controller_config = None
            self._cartesian_planner = None
            self._last_feedback_for_acceleration = None
            self._mode = ControllerMode.DISCONNECTED
            self._current_arm_gain_scale = 0.0
            self._current_gripper_gain_scale = 0.0
            self._lock.release()
            raise

    def get_state(self) -> ArmSnapshot:
        snapshot = self._read_snapshot()
        self._validate_snapshot(snapshot)
        self._remember_snapshot(snapshot)
        return snapshot

    def get_cached_state(self) -> ArmSnapshot | None:
        """Return monitor-owned feedback without issuing another SDK read."""
        with self._snapshot_lock:
            return self._latest_snapshot

    def get_limits(self) -> ArmLimits:
        """Return copies of the configured position limits."""
        config = self._require_robot_config()
        return ArmLimits(
            joint_position_min=config.joint_pos_min.copy(),
            joint_position_max=config.joint_pos_max.copy(),
            gripper_width_m=float(config.gripper_width),
            joint_velocity_max=np.asarray(config.joint_vel_max, dtype=np.float64).copy(),
        )

    def activate(self) -> ArmSnapshot:
        if self._mode == ControllerMode.ACTIVE:
            return self.get_state()
        if self._mode == ControllerMode.FAULT:
            raise ControlError("controller is faulted; run clear_fault() first")
        if self._mode != ControllerMode.DAMPING:
            raise ControlError(f"cannot activate while mode is {self._mode.value}")

        snapshot = self.get_state()
        self._set_joint_cmd_in_mode(
            self._make_joint_state(
                snapshot.joint_position, snapshot.gripper_position, 0.0
            ),
            ControllerMode.DAMPING,
        )
        self._ramp_gains(
            0.0,
            self.gain_scale,
            self._current_gripper_gain_scale,
            self._current_gripper_gain_scale,
            self.gain_ramp_s,
        )
        with self._mode_lock:
            if self._mode != ControllerMode.DAMPING:
                raise ControlError("activation was interrupted")
            self._mode = ControllerMode.ACTIVE
        return self.get_state()

    def activate_gripper(self) -> ArmSnapshot:
        """Softly enable gripper stiffness while holding the measured pose."""
        self._ensure_active()
        if math.isclose(
            self._current_gripper_gain_scale,
            self.gripper_gain_scale,
            abs_tol=1e-12,
        ):
            return self.get_state()

        snapshot = self.get_state()
        gripper_position, gripper_torque = self._gripper_hold_command(
            snapshot.gripper_position
        )
        self._set_joint_cmd_in_mode(
            self._make_joint_state(
                snapshot.joint_position,
                gripper_position,
                0.0,
                gripper_torque=gripper_torque,
            ),
            ControllerMode.ACTIVE,
        )
        self._ramp_gains(
            self._current_arm_gain_scale,
            self._current_arm_gain_scale,
            self._current_gripper_gain_scale,
            self.gripper_gain_scale,
            self.gripper_gain_ramp_s,
        )
        return self.get_state()

    def teach_with_gripper_hold(self) -> ArmSnapshot:
        """Release arm position stiffness while retaining gripper position hold."""
        self._require_active()
        if self._current_gripper_gain_scale <= 0.0:
            raise ControlError("gripper hold must be active before teach mode")
        snapshot = self.get_state()
        gripper_position, gripper_torque = self._gripper_hold_command(
            snapshot.gripper_position
        )
        self._set_joint_cmd_in_mode(
            self._make_joint_state(
                snapshot.joint_position,
                gripper_position,
                0.0,
                gripper_torque=gripper_torque,
            ),
            ControllerMode.ACTIVE,
        )
        self._ramp_gains(
            self._current_arm_gain_scale,
            0.0,
            self._current_gripper_gain_scale,
            self._current_gripper_gain_scale,
            self.gain_ramp_s,
        )
        return self.get_state()

    def hold_current(self) -> ArmSnapshot:
        self._stop_online_servos("hold current")
        if self._mode == ControllerMode.DAMPING:
            return self.activate()
        self._require_active()
        snapshot = self.get_state()
        gripper_position, gripper_torque = self._gripper_hold_command(
            snapshot.gripper_position
        )
        self._set_joint_cmd_in_mode(
            self._make_joint_state(
                snapshot.joint_position,
                gripper_position,
                0.0,
                gripper_torque=gripper_torque,
            ),
            ControllerMode.ACTIVE,
        )
        if not math.isclose(
            self._current_arm_gain_scale,
            self.gain_scale,
            abs_tol=1e-12,
        ):
            self._ramp_gains(
                self._current_arm_gain_scale,
                self.gain_scale,
                self._current_gripper_gain_scale,
                self._current_gripper_gain_scale,
                self.gain_ramp_s,
            )
        return self.get_state()

    def damping(self) -> ArmSnapshot:
        """Softly release position stiffness, then enter SDK damping mode."""
        if self._mode == ControllerMode.FAULT:
            raise ControlError("controller is faulted; run clear_fault() first")
        if self._mode == ControllerMode.DAMPING:
            return self.get_state()
        self._stop_online_servos("enter damping")
        self._require_active()

        # Track the latest measured pose while stiffness is released.  This
        # avoids pulling toward an old target during the ramp-down.
        snapshot = self.get_state()
        self._clear_gripper_force_hold()
        self._set_joint_cmd_in_mode(
            self._make_joint_state(
                snapshot.joint_position, snapshot.gripper_position, 0.0
            ),
            ControllerMode.ACTIVE,
        )
        self._ramp_gains(
            self._current_arm_gain_scale,
            0.0,
            self._current_gripper_gain_scale,
            0.0,
            max(self.gain_ramp_s, self.gripper_gain_ramp_s),
        )
        with self._mode_lock:
            if self._mode != ControllerMode.ACTIVE:
                raise ControlError("damping transition was interrupted")
            self._require_sdk().set_to_damping()
            self._current_arm_gain_scale = 0.0
            self._current_gripper_gain_scale = 0.0
            self._mode = ControllerMode.DAMPING
        return self.get_state()

    def emergency_damping(self, reason: str) -> None:
        """Immediately remove position stiffness and latch a fault."""
        with self._mode_lock:
            self._fault_generation += 1
            sdk = self._sdk
            if sdk is not None:
                try:
                    sdk.set_to_damping()
                except Exception:
                    pass
            self._current_arm_gain_scale = 0.0
            self._current_gripper_gain_scale = 0.0
            self._clear_gripper_force_hold()
            self._fault_reason = reason
            if self._mode not in (
                ControllerMode.CLOSED,
                ControllerMode.DISCONNECTED,
            ):
                self._mode = ControllerMode.FAULT

    def clear_fault(self) -> ArmSnapshot:
        """Validate feedback and move from FAULT to DAMPING without activating."""
        with self._mode_lock:
            if self._mode != ControllerMode.FAULT:
                raise ControlError("controller is not faulted")
            fault_generation = self._fault_generation
        snapshot = self.get_state()
        with self._mode_lock:
            if (
                self._mode != ControllerMode.FAULT
                or self._fault_generation != fault_generation
            ):
                raise ControlError("fault clearing was interrupted")
            self._require_sdk().set_to_damping()
            self._fault_reason = None
            self._current_arm_gain_scale = 0.0
            self._current_gripper_gain_scale = 0.0
            self._clear_gripper_force_hold()
            self._mode = ControllerMode.DAMPING
        return snapshot

    def _plan_joint_trajectory_preview(
        self,
        target_position: np.ndarray,
        *,
        velocity_scale: float = 1.0,
        acceleration_scale: float = 1.0,
        jerk_scale: float = 1.0,
        allow_large_motion: bool = False,
        start: ArmSnapshot | None = None,
        cartesian_plan: CartesianPlan | None = None,
    ) -> JointTrajectoryPreview:
        """Plan a synchronized jerk-limited joint move without moving."""
        scales = (velocity_scale, acceleration_scale, jerk_scale)
        if not all(math.isfinite(value) and 0.0 < value <= 1.0 for value in scales):
            raise SafetyError("smooth joint derivative scales must be in (0, 1]")
        start = self.get_state() if start is None else start
        # Point-to-point commands are serialized and start from a commanded
        # zero-velocity boundary. The X5 velocity estimate can remain noisy
        # after its measured position has already stabilized.
        stationary_velocity = np.zeros_like(start.joint_velocity)
        target = np.asarray(target_position, dtype=np.float64).copy()
        robot_config = self._require_robot_config()
        margin = self.safety.joint_limit_margin_rad
        velocity_limit, acceleration_limit, jerk_limit = (
            self._scaled_joint_derivative_limits(1.0)
        )
        try:
            trajectory = plan_ruckig_joint_trajectory(
                start.joint_position,
                target,
                stationary_velocity,
                current_acceleration=np.zeros_like(start.joint_velocity),
                min_position=robot_config.joint_pos_min + margin,
                max_position=robot_config.joint_pos_max - margin,
                max_velocity=velocity_limit * velocity_scale,
                max_acceleration=acceleration_limit * acceleration_scale,
                max_jerk=jerk_limit * jerk_scale,
                sample_rate_hz=self.safety.trajectory_rate_hz,
                minimum_duration_s=(
                    self.safety.minimum_motion_duration_s / velocity_scale
                ),
            )
        except (ValueError, RuntimeError) as exc:
            raise SafetyError(f"smooth joint planning failed: {exc}") from exc
        max_duration_s = (
            self.EXTENDED_MOTION_MAX_DURATION_S if allow_large_motion else 30.0
        )
        self._validate_motion(
            start.joint_position,
            target,
            target - start.joint_position,
            (trajectory.duration_s,),
            allow_large_motion=allow_large_motion,
            max_duration_s=max_duration_s,
            require_cosine_duration=False,
        )
        samples = trajectory.samples(0.0)
        self._validate_trajectory_samples(
            start.joint_position,
            samples,
            0.0,
            start_velocity=stationary_velocity,
            allow_large_motion=allow_large_motion,
        )
        joint_positions = np.vstack([start.joint_position, trajectory.positions])
        path_xyz = self._require_cartesian_planner().forward_kinematics_path(
            joint_positions
        )[:, :3]
        return JointTrajectoryPreview(
            start_joint_position=start.joint_position.copy(),
            target_joint_position=target,
            joint_positions=joint_positions,
            path_xyz_m=path_xyz,
            duration_s=trajectory.duration_s,
            smooth_trajectory=trajectory,
            cartesian_plan=cartesian_plan,
        )

    def _plan_joint_pose_endpoint(
        self,
        target: Pose,
        start: ArmSnapshot,
        context: MotionContext,
        *,
        unrestricted: bool,
    ) -> CartesianPlan:
        """Resolve one active-TCP pose to a checked joint-space endpoint."""
        planner = self._require_cartesian_planner()
        target_tcp_in_base = context.command_pose_to_base(target)
        target_flange = context.tcp_target_to_flange(target)
        start_tcp_in_base = self._tcp_pose_from_snapshot(start, context)
        collision = check_tcp_path(
            np.vstack(
                (
                    start_tcp_in_base.position_xyz_m,
                    target_tcp_in_base.position_xyz_m,
                )
            ),
            self._tcp_collision_obstacles,
            clearance_m=self._tcp_collision_clearance_m,
        )
        if not collision.safe:
            raise SafetyError(collision.reason or "movej TCP path is unsafe")
        target_flange_6d = self._pose_to_pose_6d(target_flange)
        try:
            return planner.plan_relative_xyz(
                start.eef_pose_6d,
                start.joint_position,
                target_flange_6d[:3] - start.eef_pose_6d[:3],
                seed_joint_position=start.joint_position,
                target_orientation_xyzw=target_flange.orientation_xyzw,
                unrestricted=unrestricted,
            )
        except CartesianPlanningError as exc:
            raise SafetyError(str(exc)) from exc

    def plan_movej(
        self,
        target_joint_position: Pose | np.ndarray,
        *,
        velocity_scale: float = 1.0,
        allow_large_motion: bool = False,
        start_snapshot: ArmSnapshot | None = None,
    ) -> MotionPlan:
        """Plan a synchronized joint move to joint values or a TCP pose."""
        context = self._motion_context
        effective_scale = context.effective_velocity_scale(velocity_scale)
        start = start_snapshot
        cartesian_plan = None
        target_tcp_pose_in_base = None
        start_tcp_pose_in_base = None
        resolved_target = target_joint_position
        if isinstance(target_joint_position, Pose):
            start = self.get_state() if start is None else start
            cartesian_plan = self._plan_joint_pose_endpoint(
                target_joint_position,
                start,
                context,
                unrestricted=allow_large_motion,
            )
            resolved_target = cartesian_plan.target_joint_position
            target_tcp_pose_in_base = context.command_pose_to_base(
                target_joint_position
            )
            start_tcp_pose_in_base = self._tcp_pose_from_snapshot(start, context)
        planner_args = {
            "velocity_scale": effective_scale,
            "acceleration_scale": effective_scale**2,
            "jerk_scale": effective_scale**3,
            "allow_large_motion": allow_large_motion,
        }
        if start is None:
            preview = self._plan_joint_trajectory_preview(
                resolved_target,
                **planner_args,
            )
        else:
            preview = self._plan_joint_trajectory_preview(
                resolved_target,
                start=start,
                cartesian_plan=cartesian_plan,
                **planner_args,
            )
        trajectory = preview.smooth_trajectory
        if trajectory is None:  # pragma: no cover - guaranteed by planner
            raise SafetyError("joint planner returned no trajectory")
        return MotionPlan(
            motion_type=MotionType.MOVEJ,
            context=context,
            start_joint_position=preview.start_joint_position,
            target_joint_position=preview.target_joint_position,
            joint_positions=trajectory.positions,
            joint_velocities=trajectory.velocities,
            sample_times_s=trajectory.sample_times_s,
            path_xyz_m=preview.path_xyz_m,
            duration_s=trajectory.duration_s,
            backend_plan=preview,
            target_tcp_pose_in_base=target_tcp_pose_in_base,
            start_tcp_pose_in_base=start_tcp_pose_in_base,
            unrestricted=allow_large_motion,
        )

    def plan_movej_waypoints(
        self,
        targets: Sequence[Pose],
        *,
        velocity_scale: float = 1.0,
        allow_large_motion: bool = False,
        start_snapshot: ArmSnapshot | None = None,
    ) -> MotionPlan:
        """Plan exact TCP key poses as one continuous joint-space trajectory."""
        poses = tuple(targets)
        if not poses:
            raise ValueError("movej waypoint path requires at least one target")
        if not all(isinstance(target, Pose) for target in poses):
            raise TypeError("movej waypoint targets must be Pose values")

        context = self._motion_context
        effective_scale = context.effective_velocity_scale(velocity_scale)
        start = self.get_state() if start_snapshot is None else start_snapshot
        virtual = start
        endpoint_plans: list[CartesianPlan] = []
        anchor_positions = [start.joint_position.copy()]
        for target in poses:
            endpoint = self._plan_joint_pose_endpoint(
                target,
                virtual,
                context,
                unrestricted=allow_large_motion,
            )
            endpoint_plans.append(endpoint)
            anchor_positions.append(endpoint.target_joint_position.copy())
            virtual = replace(
                virtual,
                joint_position=endpoint.target_joint_position.copy(),
                joint_velocity=np.zeros_like(virtual.joint_velocity),
                joint_acceleration=np.zeros_like(virtual.joint_position),
                eef_pose_6d=endpoint.solved_pose_6d.copy(),
            )

        velocity_limit, acceleration_limit, jerk_limit = (
            self._scaled_joint_derivative_limits(1.0)
        )
        maximum_duration_s = (
            self.EXTENDED_MOTION_MAX_DURATION_S if allow_large_motion else 30.0
        )
        try:
            trajectory = plan_blended_joint_trajectory(
                np.asarray(anchor_positions, dtype=np.float64),
                max_velocity=velocity_limit * effective_scale,
                max_acceleration=acceleration_limit * effective_scale**2,
                max_jerk=jerk_limit * effective_scale**3,
                sample_rate_hz=self.safety.trajectory_rate_hz,
                minimum_duration_s=(
                    self.safety.minimum_motion_duration_s / effective_scale
                ),
                maximum_duration_s=maximum_duration_s,
            )
        except (ValueError, RuntimeError) as exc:
            raise SafetyError(f"smooth waypoint planning failed: {exc}") from exc

        target_joint_position = endpoint_plans[-1].target_joint_position.copy()
        self._validate_motion(
            start.joint_position,
            target_joint_position,
            target_joint_position - start.joint_position,
            (trajectory.duration_s,),
            allow_large_motion=allow_large_motion,
            max_duration_s=maximum_duration_s,
            require_cosine_duration=False,
        )
        self._validate_trajectory_samples(
            start.joint_position,
            trajectory.samples(0.0),
            0.0,
            start_velocity=np.zeros_like(start.joint_velocity),
            allow_large_motion=allow_large_motion,
        )
        all_joint_positions = np.vstack(
            (start.joint_position, trajectory.positions)
        )
        flange_path = self._require_cartesian_planner().forward_kinematics_path(
            all_joint_positions
        )
        path_xyz = np.asarray(
            [
                context.flange_pose_to_tcp(
                    self._pose_from_pose_6d(flange_pose)
                ).position_xyz_m
                for flange_pose in flange_path
            ],
            dtype=np.float64,
        )
        preview = JointTrajectoryPreview(
            start_joint_position=start.joint_position.copy(),
            target_joint_position=target_joint_position,
            joint_positions=all_joint_positions,
            path_xyz_m=path_xyz,
            duration_s=trajectory.duration_s,
            smooth_trajectory=trajectory,
            cartesian_plan=endpoint_plans[-1],
        )
        return MotionPlan(
            motion_type=MotionType.MOVEJ,
            context=context,
            start_joint_position=start.joint_position.copy(),
            target_joint_position=target_joint_position,
            joint_positions=trajectory.positions,
            joint_velocities=trajectory.velocities,
            sample_times_s=trajectory.sample_times_s,
            path_xyz_m=path_xyz,
            duration_s=trajectory.duration_s,
            backend_plan=preview,
            target_tcp_pose_in_base=context.command_pose_to_base(poses[-1]),
            start_tcp_pose_in_base=self._tcp_pose_from_snapshot(start, context),
            unrestricted=allow_large_motion,
        )

    def movej(
        self,
        target_joint_position: Pose | np.ndarray,
        *,
        velocity_scale: float = 1.0,
        allow_large_motion: bool = False,
    ) -> MotionResult | CartesianMotionResult:
        """Plan and execute one canonical joint-space point move."""
        preview = self.plan_movej(
            target_joint_position,
            velocity_scale=velocity_scale,
            allow_large_motion=allow_large_motion,
        )
        result = self.execute_motion_plan(preview)
        expected_type = (
            CartesianMotionResult
            if isinstance(target_joint_position, Pose)
            else MotionResult
        )
        if not isinstance(result, expected_type):  # pragma: no cover - invariant
            raise ControlError("movej returned an unexpected result type")
        return result

    def validate_motion_plan(
        self,
        plan: MotionPlan,
        *,
        start_tolerance_rad: float = math.radians(0.25),
    ) -> ArmSnapshot:
        """Revalidate immutable plan context and start state before execution."""
        if not isinstance(plan, MotionPlan):
            raise TypeError("plan must be a MotionPlan")
        if (
            not math.isfinite(start_tolerance_rad)
            or start_tolerance_rad <= 0.0
        ):
            raise ValueError("plan start tolerance must be positive and finite")
        if plan.context.revision != self._motion_context.revision:
            raise SafetyError("motion context changed after planning; plan again")
        start = self.get_state()
        start_error = float(
            np.max(np.abs(start.joint_position - plan.start_joint_position))
        )
        if start_error > start_tolerance_rad:
            raise SafetyError("joint position changed after planning; plan again")
        relative_samples = [
            TrajectorySample(
                timestamp=float(timestamp),
                position=position,
                velocity=velocity,
            )
            for timestamp, position, velocity in zip(
                plan.sample_times_s,
                plan.joint_positions,
                plan.joint_velocities,
            )
        ]
        self._validate_trajectory_samples(
            start.joint_position,
            relative_samples,
            0.0,
            start_velocity=np.zeros_like(start.joint_velocity),
            allow_large_motion=plan.unrestricted,
        )
        return start

    def execute_motion_plan(
        self,
        plan: MotionPlan,
        *,
        start_tolerance_rad: float = math.radians(0.25),
    ) -> MotionResult | CartesianMotionResult:
        """Execute exact samples from a previously returned canonical plan."""
        self._ensure_active()
        self.validate_motion_plan(plan, start_tolerance_rad=start_tolerance_rad)
        if plan.motion_type == MotionType.MOVEJ:
            preview = plan.backend_plan
            if not isinstance(preview, JointTrajectoryPreview):
                raise SafetyError("movej plan has an invalid backend")
            motion = self._execute_joint_trajectory_preview(
                preview,
                allow_large_motion=plan.unrestricted,
                start_tolerance_rad=start_tolerance_rad,
            )
            if preview.cartesian_plan is None:
                return motion
            final = self.get_state()
            final_tcp = self._tcp_pose_from_snapshot(final, plan.context)
            target_tcp_pose_6d = (
                None
                if plan.target_tcp_pose_in_base is None
                else self._pose_to_pose_6d(plan.target_tcp_pose_in_base)
            )
            return CartesianMotionResult(
                plan=preview.cartesian_plan,
                joint_motion=motion,
                final_pose_6d=self._pose_to_pose_6d(final_tcp),
                target_tcp_pose_6d=target_tcp_pose_6d,
            )
        preview = plan.backend_plan
        if not isinstance(preview, CartesianWaypointTrajectoryPreview):
            raise SafetyError("Cartesian motion plan has an invalid backend")
        return self._execute_cartesian_preview(
            preview,
            context=plan.context,
            target_tcp_pose_in_base=plan.target_tcp_pose_in_base,
            unrestricted=plan.unrestricted,
            start_tolerance_rad=start_tolerance_rad,
        )

    def _execute_joint_trajectory_preview(
        self,
        preview: JointTrajectoryPreview,
        *,
        allow_large_motion: bool = False,
        start_tolerance_rad: float = math.radians(0.25),
    ) -> MotionResult:
        """Execute the exact jerk-limited samples validated by a preview."""
        self._ensure_active()
        trajectory = preview.smooth_trajectory
        if trajectory is None:
            raise SafetyError("joint preview does not contain a smooth trajectory")
        start = self.get_state()
        start_error = float(
            np.max(
                np.abs(
                    start.joint_position
                    - np.asarray(preview.start_joint_position, dtype=np.float64)
                )
            )
        )
        if start_error > start_tolerance_rad:
            raise SafetyError("joint position changed after preview; preview again")
        max_duration_s = (
            self.EXTENDED_MOTION_MAX_DURATION_S if allow_large_motion else 30.0
        )
        return self._run_motion(
            start=start,
            excursion_target=preview.target_joint_position,
            final_target=preview.target_joint_position,
            samples_factory=lambda timestamp: self._trajectory_samples_with_final_hold(
                trajectory,
                timestamp,
            ),
            commanded_duration_s=(
                trajectory.duration_s + self.safety.trajectory_final_hold_s
            ),
            allow_large_motion=allow_large_motion,
            max_duration_s=max_duration_s,
            require_cosine_duration=False,
            trajectory_start_velocity=np.zeros_like(start.joint_velocity),
        )

    @staticmethod
    def _coerce_pose(
        target: Pose | np.ndarray,
        orientation_xyzw: np.ndarray | None = None,
    ) -> Pose:
        if isinstance(target, Pose):
            if orientation_xyzw is not None:
                raise ValueError("orientation must be omitted when target is a Pose")
            return target
        if orientation_xyzw is None:
            raise ValueError("Cartesian XYZ target requires an orientation")
        return Pose(np.asarray(target, dtype=np.float64), orientation_xyzw)

    def _build_motion_path_geometry(
        self,
        start: ArmSnapshot,
        segments: tuple[CartesianPathSegment, ...],
        context: MotionContext,
    ) -> CartesianGeometryPath:
        start_tcp = self._tcp_pose_from_snapshot(start, context)
        if all(segment.mode == CartesianPathMode.LINEAR for segment in segments):
            waypoints = [
                CartesianWaypoint(
                    start_tcp.position_xyz_m,
                    start_tcp.orientation_xyzw,
                )
            ]
            for segment in segments:
                target = context.command_pose_to_base(segment.target)
                waypoints.append(
                    CartesianWaypoint(
                        target.position_xyz_m,
                        target.orientation_xyzw,
                        blend_radius_m=segment.blend_radius_m,
                        name=segment.name,
                    )
                )
            try:
                return build_cartesian_path(
                    waypoints,
                    max_step_m=self.safety.cartesian_geometry_step_m,
                    max_orientation_step_rad=(
                        self.safety.cartesian_orientation_step_rad
                    ),
                )
            except ValueError as exc:
                raise SafetyError(
                    f"Cartesian geometry planning failed: {exc}"
                ) from exc

        if any(segment.blend_radius_m > 0.0 for segment in segments):
            raise SafetyError(
                "explicit movec segments cannot be combined with blend radii"
            )
        paths: list[CartesianGeometryPath] = []
        current = start_tcp
        for segment in segments:
            target = context.command_pose_to_base(segment.target)
            start_waypoint = CartesianWaypoint(
                current.position_xyz_m,
                current.orientation_xyzw,
            )
            target_waypoint = CartesianWaypoint(
                target.position_xyz_m,
                target.orientation_xyzw,
                name=segment.name,
            )
            try:
                if segment.mode == CartesianPathMode.CIRCULAR:
                    if segment.via is None:  # pragma: no cover - contract invariant
                        raise SafetyError("movec segment has no via pose")
                    via = context.command_pose_to_base(segment.via)
                    geometry = build_circular_path(
                        start_waypoint,
                        CartesianWaypoint(
                            via.position_xyz_m,
                            via.orientation_xyzw,
                        ),
                        target_waypoint,
                        max_step_m=self.safety.cartesian_geometry_step_m,
                        max_orientation_step_rad=(
                            self.safety.cartesian_orientation_step_rad
                        ),
                    )
                else:
                    geometry = build_cartesian_path(
                        (start_waypoint, target_waypoint),
                        max_step_m=self.safety.cartesian_geometry_step_m,
                        max_orientation_step_rad=(
                            self.safety.cartesian_orientation_step_rad
                        ),
                    )
            except ValueError as exc:
                raise SafetyError(
                    f"Cartesian geometry planning failed: {exc}"
                ) from exc
            paths.append(geometry)
            current = target
        try:
            return concatenate_cartesian_paths(paths)
        except ValueError as exc:  # pragma: no cover - constructed contiguously
            raise SafetyError(f"Cartesian path concatenation failed: {exc}") from exc

    def _plan_cartesian_segments(
        self,
        segments: tuple[CartesianPathSegment, ...],
        *,
        motion_type: MotionType,
        velocity_scale: float,
        unrestricted: bool,
        start_snapshot: ArmSnapshot | None = None,
    ) -> MotionPlan:
        if not segments:
            raise ValueError("move_path requires at least one segment")
        context = self._motion_context
        effective_scale = context.effective_velocity_scale(velocity_scale)
        start = self.get_state() if start_snapshot is None else start_snapshot
        geometry = self._build_motion_path_geometry(start, segments, context)
        preview = self._prepare_cartesian_geometry_trajectory(
            start,
            geometry,
            duration_s=None,
            velocity_scale=effective_scale,
            unrestricted=unrestricted,
            context=context,
        )
        target_tcp = context.command_pose_to_base(segments[-1].target)
        return MotionPlan(
            motion_type=motion_type,
            context=context,
            start_joint_position=preview.plan.start_joint_position,
            target_joint_position=preview.plan.target_joint_position,
            joint_positions=preview.joint_positions,
            joint_velocities=preview.joint_velocities,
            sample_times_s=preview.sample_times_s,
            path_xyz_m=preview.path_xyz_m,
            duration_s=preview.duration_s,
            backend_plan=preview,
            target_tcp_pose_in_base=target_tcp,
            start_tcp_pose_in_base=context.flange_pose_to_tcp(
                self._pose_from_pose_6d(start.eef_pose_6d)
            ),
            unrestricted=unrestricted,
        )

    def plan_movel(
        self,
        target: Pose | np.ndarray,
        target_orientation_xyzw: np.ndarray | None = None,
        *,
        velocity_scale: float = 1.0,
        unrestricted: bool = False,
    ) -> MotionPlan:
        """Plan an absolute straight active-TCP move in the active frame."""
        pose = self._coerce_pose(target, target_orientation_xyzw)
        return self._plan_cartesian_segments(
            (CartesianPathSegment(pose),),
            motion_type=MotionType.MOVEL,
            velocity_scale=velocity_scale,
            unrestricted=unrestricted,
        )

    def movel(
        self,
        target: Pose | np.ndarray,
        target_orientation_xyzw: np.ndarray | None = None,
        *,
        velocity_scale: float = 1.0,
        unrestricted: bool = False,
    ) -> CartesianMotionResult:
        plan = self.plan_movel(
            target,
            target_orientation_xyzw,
            velocity_scale=velocity_scale,
            unrestricted=unrestricted,
        )
        result = self.execute_motion_plan(plan)
        if not isinstance(result, CartesianMotionResult):  # pragma: no cover
            raise ControlError("movel returned a non-Cartesian result")
        return result

    def plan_movec(
        self,
        via: Pose,
        target: Pose,
        *,
        velocity_scale: float = 1.0,
        unrestricted: bool = False,
    ) -> MotionPlan:
        """Plan an active-TCP circular arc through ``via`` to ``target``."""
        segment = CartesianPathSegment(
            target=target,
            mode=CartesianPathMode.CIRCULAR,
            via=via,
        )
        return self._plan_cartesian_segments(
            (segment,),
            motion_type=MotionType.MOVEC,
            velocity_scale=velocity_scale,
            unrestricted=unrestricted,
        )

    def movec(
        self,
        via: Pose,
        target: Pose,
        *,
        velocity_scale: float = 1.0,
        unrestricted: bool = False,
    ) -> CartesianMotionResult:
        plan = self.plan_movec(
            via,
            target,
            velocity_scale=velocity_scale,
            unrestricted=unrestricted,
        )
        result = self.execute_motion_plan(plan)
        if not isinstance(result, CartesianMotionResult):  # pragma: no cover
            raise ControlError("movec returned a non-Cartesian result")
        return result

    def plan_move_path(
        self,
        path: Sequence[CartesianPathSegment | Pose],
        *,
        velocity_scale: float = 1.0,
        unrestricted: bool = False,
        start_snapshot: ArmSnapshot | None = None,
    ) -> MotionPlan:
        """Plan one all-or-nothing Cartesian sequence with optional blends."""
        segments = tuple(
            item if isinstance(item, CartesianPathSegment) else CartesianPathSegment(item)
            for item in path
        )
        if not all(isinstance(segment, CartesianPathSegment) for segment in segments):
            raise TypeError("move_path entries must be Pose or CartesianPathSegment")
        return self._plan_cartesian_segments(
            segments,
            motion_type=MotionType.MOVE_PATH,
            velocity_scale=velocity_scale,
            unrestricted=unrestricted,
            start_snapshot=start_snapshot,
        )

    def move_path(
        self,
        path: Sequence[CartesianPathSegment | Pose],
        *,
        velocity_scale: float = 1.0,
        unrestricted: bool = False,
    ) -> CartesianMotionResult:
        plan = self.plan_move_path(
            path,
            velocity_scale=velocity_scale,
            unrestricted=unrestricted,
        )
        result = self.execute_motion_plan(plan)
        if not isinstance(result, CartesianMotionResult):  # pragma: no cover
            raise ControlError("move_path returned a non-Cartesian result")
        return result

    def _prepare_cartesian_geometry_trajectory(
        self,
        start: ArmSnapshot,
        geometry: CartesianGeometryPath,
        *,
        duration_s: float | None,
        velocity_scale: float,
        unrestricted: bool,
        context: MotionContext | None = None,
    ) -> CartesianWaypointTrajectoryPreview:
        if not math.isfinite(velocity_scale) or not 0.01 <= velocity_scale <= 1.0:
            raise SafetyError("Cartesian velocity scale must be in [0.01, 1.0]")
        if duration_s is not None and (
            not math.isfinite(duration_s) or duration_s <= 0.0
        ):
            raise SafetyError("Cartesian duration must be positive and finite")
        duration_limit_s = (
            self.EXTENDED_MOTION_MAX_DURATION_S if unrestricted else 30.0
        )
        canonical_minimum_duration = self.safety.minimum_motion_duration_s
        if duration_s is not None:
            canonical_minimum_duration = max(
                canonical_minimum_duration,
                float(duration_s) * velocity_scale,
            )
        progress = plan_scalar_progress(
            geometry.length_m,
            max_velocity_m_s=self.safety.max_commanded_cartesian_velocity_m_s,
            max_acceleration_m_s2=(
                self.safety.max_commanded_cartesian_acceleration_m_s2
            ),
            max_jerk_m_s3=self.safety.max_commanded_cartesian_jerk_m_s3,
            sample_rate_hz=self.safety.cartesian_waypoint_rate_hz,
            minimum_duration_s=canonical_minimum_duration,
        )
        fixed_progress = geometry.progress[1:]
        positions_xyz = geometry.positions_xyz_m[1:]
        orientations = geometry.orientations_xyzw[1:]
        if context is not None:
            flange_poses = tuple(
                Pose(position, orientation).compose(
                    context.tcp.pose_in_flange.inverse()
                )
                for position, orientation in zip(positions_xyz, orientations)
            )
            positions_xyz = np.asarray(
                [pose.position_xyz_m for pose in flange_poses],
                dtype=np.float64,
            )
            orientations = np.asarray(
                [pose.orientation_xyzw for pose in flange_poses],
                dtype=np.float64,
            )
        poses = np.column_stack(
            (
                positions_xyz,
                np.asarray(
                    [quaternion_xyzw_to_rpy(value) for value in orientations],
                    dtype=np.float64,
                ),
            )
        )
        planner = self._require_cartesian_planner()
        try:
            path = planner.plan_pose_path(
                start.eef_pose_6d,
                start.joint_position,
                poses,
                fixed_progress,
                unrestricted=unrestricted,
            )
        except CartesianPlanningError as exc:
            raise SafetyError(str(exc)) from exc

        all_positions = np.vstack([start.joint_position, path.joint_positions])
        solved_flange_poses = planner.forward_kinematics_path(all_positions)
        if context is None:
            path_xyz = solved_flange_poses[:, :3]
        else:
            path_xyz = np.asarray(
                [
                    context.flange_pose_to_tcp(
                        self._pose_from_pose_6d(flange_pose)
                    ).position_xyz_m
                    for flange_pose in solved_flange_poses
                ],
                dtype=np.float64,
            )
            tcp_collision = check_tcp_path(
                path_xyz,
                self._tcp_collision_obstacles,
                clearance_m=self._tcp_collision_clearance_m,
            )
            if not tcp_collision.safe:
                raise SafetyError(
                    "active TCP collision with "
                    f"{tcp_collision.obstacle_name} near waypoint "
                    f"{tcp_collision.waypoint_index}"
                )
        progress_values = np.concatenate(([0.0], progress.progress))
        progress_times = np.concatenate(([0.0], progress.sample_times_s))
        base_times = np.interp(
            geometry.progress,
            progress_values,
            progress_times,
        )
        target_cartesian_velocity = (
            self.safety.max_commanded_cartesian_velocity_m_s
            * path.final_plan.singularity_speed_scale
        )
        canonical_time_scale = 1.0 / path.final_plan.singularity_speed_scale
        relative_times = base_times * canonical_time_scale
        canonical_dynamics = None
        canonical_accelerations = None
        for _ in range(8):
            if relative_times[-1] / velocity_scale > duration_limit_s:
                raise SafetyError(
                    "Cartesian waypoint path would require more than "
                    f"{duration_limit_s:.0f} seconds"
                )
            intervals = np.diff(relative_times)
            tcp_velocity = np.linalg.norm(
                np.diff(path_xyz, axis=0) / intervals[:, None],
                axis=1,
            )
            tcp_scale = (
                float(np.max(tcp_velocity)) / target_cartesian_velocity
                if tcp_velocity.size
                else 1.0
            )
            if tcp_scale > 1.0 + 1e-6:
                canonical_time_scale *= 1.02 * tcp_scale
                relative_times = base_times * canonical_time_scale
                continue
            dynamics = analyze_sampled_motion(all_positions, relative_times)
            duration_multiplier = dynamics.required_time_scale(
                velocity_limit=self.safety.max_commanded_velocity_rad_s,
                acceleration_limit=self.safety.servo_max_acceleration_rad_s2,
                jerk_limit=self.safety.servo_max_jerk_rad_s3,
            )
            if duration_multiplier > 1.0 + 1e-6:
                canonical_time_scale *= 1.02 * duration_multiplier
                relative_times = base_times * canonical_time_scale
                continue
            accelerations = np.gradient(
                dynamics.command_velocities,
                relative_times,
                axis=0,
                edge_order=2 if relative_times.size > 2 else 1,
            )
            accelerations[0] = 0.0
            accelerations[-1] = 0.0
            (
                canonical_sample_times,
                canonical_sample_positions,
                canonical_sample_velocities,
                _,
            ) = resample_quintic_hermite(
                relative_times,
                all_positions,
                dynamics.command_velocities,
                accelerations,
                speed_scale=1.0,
                sample_rate_hz=self.safety.trajectory_rate_hz,
            )
            sampled_dynamics = analyze_sampled_motion(
                canonical_sample_positions,
                canonical_sample_times,
                command_velocities=canonical_sample_velocities,
            )
            sampled_multiplier = sampled_dynamics.required_time_scale(
                velocity_limit=self.safety.max_commanded_velocity_rad_s,
                acceleration_limit=self.safety.servo_max_acceleration_rad_s2,
                jerk_limit=self.safety.servo_max_jerk_rad_s3,
            )
            if sampled_multiplier > 1.0 + 1e-6:
                canonical_time_scale *= 1.02 * sampled_multiplier
                relative_times = base_times * canonical_time_scale
                continue
            canonical_dynamics = dynamics
            canonical_accelerations = accelerations
            break
        else:
            raise SafetyError("unable to satisfy Cartesian waypoint limits")
        if canonical_dynamics is None or canonical_accelerations is None:
            raise SafetyError("Cartesian waypoint path has no velocity commands")
        scaled_times, scaled_positions, scaled_velocities, _ = (
            resample_quintic_hermite(
                relative_times,
                all_positions,
                canonical_dynamics.command_velocities,
                canonical_accelerations,
                speed_scale=velocity_scale,
                sample_rate_hz=self.safety.trajectory_rate_hz,
            )
        )
        planned_duration_s = float(scaled_times[-1])
        if planned_duration_s > duration_limit_s:
            raise SafetyError(
                "Cartesian waypoint path would require more than "
                f"{duration_limit_s:.0f} seconds"
            )
        return CartesianWaypointTrajectoryPreview(
            plan=path.final_plan,
            geometry=geometry,
            path_xyz_m=path_xyz,
            joint_positions=scaled_positions[1:],
            joint_velocities=scaled_velocities[1:],
            sample_times_s=scaled_times[1:],
            duration_s=float(planned_duration_s),
            canonical_joint_positions=all_positions,
            canonical_sample_times_s=relative_times,
        )

    def _execute_cartesian_preview(
        self,
        preview: CartesianWaypointTrajectoryPreview,
        *,
        context: MotionContext | None,
        target_tcp_pose_in_base: Pose | None,
        unrestricted: bool,
        start_tolerance_rad: float = math.radians(0.25),
    ) -> CartesianMotionResult:
        """Submit exact samples from a validated Cartesian preview."""
        self._ensure_active()
        start = self.get_state()
        start_error = float(
            np.max(
                np.abs(start.joint_position - preview.plan.start_joint_position)
            )
        )
        if start_error > start_tolerance_rad:
            raise SafetyError("joint position changed after planning; plan again")

        def path_samples(start_timestamp: float) -> list[TrajectorySample]:
            samples = [
                TrajectorySample(
                    timestamp=start_timestamp + float(relative_time),
                    position=position.copy(),
                    velocity=velocity.copy(),
                )
                for relative_time, position, velocity in zip(
                    preview.sample_times_s,
                    preview.joint_positions,
                    preview.joint_velocities,
                )
            ]
            samples.append(
                TrajectorySample(
                    timestamp=(
                        start_timestamp
                        + preview.duration_s
                        + self.safety.trajectory_final_hold_s
                    ),
                    position=preview.plan.target_joint_position.copy(),
                    velocity=np.zeros_like(start.joint_velocity),
                )
            )
            return samples

        duration_limit_s = (
            self.EXTENDED_MOTION_MAX_DURATION_S if unrestricted else 30.0
        )
        motion = self._run_motion(
            start=start,
            excursion_target=preview.plan.target_joint_position,
            final_target=preview.plan.target_joint_position,
            samples_factory=path_samples,
            commanded_duration_s=(
                preview.duration_s + self.safety.trajectory_final_hold_s
            ),
            allow_large_motion=unrestricted,
            max_duration_s=duration_limit_s,
            require_cosine_duration=False,
            trajectory_start_velocity=np.zeros_like(start.joint_velocity),
            stop_kind="cartesian",
        )
        final = self.get_state()
        final_pose_6d = final.eef_pose_6d.copy()
        target_pose_6d = None
        if context is not None:
            final_pose_6d = self._pose_to_pose_6d(
                self._tcp_pose_from_snapshot(final, context)
            )
            if target_tcp_pose_in_base is not None:
                target_pose_6d = self._pose_to_pose_6d(target_tcp_pose_in_base)
        return CartesianMotionResult(
            plan=preview.plan,
            joint_motion=motion,
            final_pose_6d=final_pose_6d,
            target_tcp_pose_6d=target_pose_6d,
        )

    def _plan_cartesian_servo_delta(
        self,
        start_pose_6d: np.ndarray,
        start_joint_position: np.ndarray,
        delta_xyz_m: np.ndarray,
        seed_joint_position: np.ndarray,
        *,
        target_orientation_xyzw: np.ndarray | None = None,
    ) -> CartesianPlan:
        """Resolve one live-drag target while preserving its gesture origin."""
        try:
            return self._require_cartesian_planner().plan_relative_xyz(
                np.asarray(start_pose_6d, dtype=np.float64),
                np.asarray(start_joint_position, dtype=np.float64),
                np.asarray(delta_xyz_m, dtype=np.float64),
                target_orientation_xyzw=target_orientation_xyzw,
                seed_joint_position=np.asarray(
                    seed_joint_position, dtype=np.float64
                ),
                unrestricted=True,
            )
        except CartesianPlanningError as exc:
            raise SafetyError(str(exc)) from exc

    def plan_servop_target(
        self,
        target: Pose,
        seed_joint_position: np.ndarray,
    ) -> CartesianPlan:
        """Resolve an absolute active-TCP servo target through the active context."""
        if not isinstance(target, Pose):
            raise TypeError("servop target must be a Pose")
        seed = np.asarray(seed_joint_position, dtype=np.float64).copy()
        with self._mode_lock:
            self._require_mode(ControllerMode.ACTIVE)
            if self._active_servo_kind != "servop":
                raise ControlError("servop session is not active")
            context = self._motion_context
            if self._servo_context_revision != context.revision:
                raise SafetyError("motion context changed during servop")
        planner = self._require_cartesian_planner()
        try:
            seed_pose_6d = planner.forward_kinematics_path(seed[None, :])[0]
            target_flange = context.tcp_target_to_flange(target)
            target_flange_6d = self._pose_to_pose_6d(target_flange)
            start_tcp = context.flange_pose_to_tcp(
                self._pose_from_pose_6d(seed_pose_6d)
            )
            collision = check_tcp_path(
                np.vstack(
                    (
                        start_tcp.position_xyz_m,
                        context.command_pose_to_base(target).position_xyz_m,
                    )
                ),
                self._tcp_collision_obstacles,
                clearance_m=self._tcp_collision_clearance_m,
            )
            if not collision.safe:
                raise SafetyError(collision.reason or "servop TCP path is unsafe")
            return planner.plan_relative_xyz(
                seed_pose_6d,
                seed,
                target_flange_6d[:3] - seed_pose_6d[:3],
                seed_joint_position=seed,
                target_orientation_xyzw=target_flange.orientation_xyzw,
                unrestricted=False,
            )
        except CartesianPlanningError as exc:
            raise SafetyError(str(exc)) from exc

    def _clear_finished_servo_sessions_locked(self) -> None:
        if self._joint_servo is not None and not self._joint_servo.status().active:
            self._joint_servo = None
            if self._active_servo_kind == "servoj":
                self._active_servo_kind = None
        if (
            self._cartesian_servo is not None
            and not self._cartesian_servo.status().active
        ):
            self._cartesian_servo = None
            if self._active_servo_kind == "servop":
                self._active_servo_kind = None
        if self._active_servo_kind is None and self._servo_context_revision is not None:
            self._servo_context_revision = None
            self._motion_in_progress = False

    def _start_servo_session(self, kind: str):
        self._ensure_active()
        with self._mode_lock:
            self._clear_finished_servo_sessions_locked()
            existing = (
                self._joint_servo if kind == "servoj" else self._cartesian_servo
            )
            if existing is not None:
                return existing
            if self._motion_in_progress:
                raise BusyError("another motion is already in progress")
            from .servo import CartesianServo, JointServo

            servo_type = JointServo if kind == "servoj" else CartesianServo
            servo = servo_type(
                self,
                rate_hz=self.safety.servo_rate_hz,
                max_velocity_rad_s=self.safety.servo_max_velocity_rad_s,
                max_acceleration_rad_s2=self.safety.servo_max_acceleration_rad_s2,
                max_jerk_rad_s3=self.safety.servo_max_jerk_rad_s3,
                joint_limit_margin_rad=self.safety.joint_limit_margin_rad,
                watchdog_timeout_s=self.safety.servo_watchdog_timeout_s,
                state_stale_timeout_s=self.safety.state_stale_timeout_s,
            )
            self._motion_in_progress = True
            self._active_servo_kind = kind
            self._servo_context_revision = self._motion_context.revision
            if kind == "servoj":
                self._joint_servo = servo
            else:
                self._cartesian_servo = servo
            try:
                servo.start()
            except BaseException:
                if kind == "servoj":
                    self._joint_servo = None
                else:
                    self._cartesian_servo = None
                self._active_servo_kind = None
                self._servo_context_revision = None
                self._motion_in_progress = False
                raise
            return servo

    def servoj(
        self,
        target_joint_position: np.ndarray,
        *,
        velocity_scale: float = 1.0,
        wait_timeout_s: float = 0.05,
    ):
        """Update a jerk-limited online absolute joint target."""
        target = np.asarray(target_joint_position, dtype=np.float64).copy()
        limits = self.get_limits()
        lower = limits.joint_position_min + self.safety.joint_limit_margin_rad
        upper = limits.joint_position_max - self.safety.joint_limit_margin_rad
        if (
            target.shape != lower.shape
            or not np.all(np.isfinite(target))
            or np.any(target < lower)
            or np.any(target > upper)
        ):
            raise SafetyError("servoj target is invalid or approaches a joint limit")
        effective_scale = self._motion_context.effective_velocity_scale(
            velocity_scale
        )
        servo = self._start_servo_session("servoj")
        generation = servo.update_target(
            target,
            velocity_scale=effective_scale,
        )
        status = servo.wait_for_target(generation, wait_timeout_s)
        if status.error is not None:
            raise ControlError(status.error)
        return status

    def servop(
        self,
        target: Pose | np.ndarray,
        target_orientation_xyzw: np.ndarray | None = None,
        *,
        velocity_scale: float = 1.0,
        wait_timeout_s: float = 0.05,
    ):
        """Update a jerk-limited online absolute active-TCP target."""
        pose = self._coerce_pose(target, target_orientation_xyzw)
        effective_scale = self._motion_context.effective_velocity_scale(
            velocity_scale
        )
        servo = self._start_servo_session("servop")
        generation = servo.update_pose_target(
            pose,
            velocity_scale=effective_scale,
        )
        status = servo.wait_for_target(generation, wait_timeout_s)
        if status.error is not None:
            raise ControlError(status.error)
        return status

    def _stop_servo_session(
        self,
        kind: str,
        *,
        reason: str,
        deceleration_scale: float,
        timeout_s: float,
    ) -> bool:
        with self._mode_lock:
            self._clear_finished_servo_sessions_locked()
            servo = (
                self._joint_servo if kind == "servoj" else self._cartesian_servo
            )
        if servo is None:
            return False
        servo.stop(
            reason,
            timeout_s=timeout_s,
            deceleration_scale=deceleration_scale,
        )
        with self._mode_lock:
            if kind == "servoj" and self._joint_servo is servo:
                self._joint_servo = None
            if kind == "servop" and self._cartesian_servo is servo:
                self._cartesian_servo = None
            if self._active_servo_kind == kind:
                self._active_servo_kind = None
                self._servo_context_revision = None
                self._motion_in_progress = False
        return True

    def _request_trajectory_stop(
        self,
        kind: str,
        *,
        deceleration_scale: float,
        timeout_s: float,
    ) -> bool:
        if (
            not math.isfinite(deceleration_scale)
            or not 0.0 < deceleration_scale <= 1.0
        ):
            raise ValueError("stop deceleration scale must be in (0, 1]")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("stop timeout must be positive and finite")
        with self._mode_lock:
            if self._motion_in_progress and self._active_servo_kind is not None:
                active_kind = (
                    "joint" if self._active_servo_kind == "servoj" else "cartesian"
                )
                if active_kind != kind:
                    raise BusyError(
                        f"{kind} stop cannot interrupt {self._active_servo_kind} session"
                    )
            if (
                self._motion_in_progress
                and self._active_servo_kind is None
                and self._trajectory_stop_kind not in (None, kind)
            ):
                raise BusyError(
                    f"{kind} stop cannot interrupt {self._trajectory_stop_kind} motion"
                )
            if (
                not self._motion_in_progress
                or self._active_servo_kind is not None
                or self._trajectory_stop_kind != kind
            ):
                return False
            self._trajectory_stop_scale = float(deceleration_scale)
            self._trajectory_stop_completed.clear()
            self._trajectory_stop_requested.set()
        if not self._trajectory_stop_completed.wait(timeout_s):
            self.emergency_damping(
                f"{kind} trajectory did not stop within {timeout_s:.1f}s"
            )
            raise ControlError(f"{kind} trajectory stop timed out")
        return True

    def _stop_online_servos(self, reason: str) -> None:
        with self._mode_lock:
            self._clear_finished_servo_sessions_locked()
            trajectory_kind = (
                self._trajectory_stop_kind
                if self._motion_in_progress and self._active_servo_kind is None
                else None
            )
            sessions = tuple(
                servo
                for servo in (self._joint_servo, self._cartesian_servo)
                if servo is not None
            )
        if trajectory_kind is not None:
            self._request_trajectory_stop(
                trajectory_kind,
                deceleration_scale=1.0,
                timeout_s=3.0,
            )
        for servo in sessions:
            servo.stop(reason, timeout_s=3.0)
        with self._mode_lock:
            self._joint_servo = None
            self._cartesian_servo = None
            if self._active_servo_kind is not None:
                self._active_servo_kind = None
                self._servo_context_revision = None
                self._motion_in_progress = False

    def stopj(
        self,
        deceleration_scale: float = 1.0,
        *,
        timeout_s: float = 3.0,
    ) -> ArmSnapshot:
        """Jerk-limited stop for the active ``servoj`` session."""
        stopped = self._stop_servo_session(
            "servoj",
            reason="stopj",
            deceleration_scale=deceleration_scale,
            timeout_s=timeout_s,
        )
        if not stopped:
            self._request_trajectory_stop(
                "joint",
                deceleration_scale=deceleration_scale,
                timeout_s=timeout_s,
            )
        return self.get_state()

    def stopl(
        self,
        deceleration_scale: float = 1.0,
        *,
        timeout_s: float = 3.0,
    ) -> ArmSnapshot:
        """Jerk-limited stop for the active ``servop`` session."""
        stopped = self._stop_servo_session(
            "servop",
            reason="stopl",
            deceleration_scale=deceleration_scale,
            timeout_s=timeout_s,
        )
        if not stopped:
            self._request_trajectory_stop(
                "cartesian",
                deceleration_scale=deceleration_scale,
                timeout_s=timeout_s,
            )
        return self.get_state()

    def _command_joint_servo(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        gripper_position: float,
    ) -> ArmSnapshot:
        """Validate and send one online servo command in ACTIVE mode."""
        position = np.asarray(position, dtype=np.float64).copy()
        velocity = np.asarray(velocity, dtype=np.float64).copy()
        with self._mode_lock:
            self._require_mode(ControllerMode.ACTIVE)
            robot_config = self._require_robot_config()
            if (
                position.shape != (robot_config.joint_dof,)
                or velocity.shape != position.shape
            ):
                raise SafetyError("online servo joint command shape is invalid")
            if (
                not np.all(np.isfinite(position))
                or not np.all(np.isfinite(velocity))
            ):
                raise SafetyError("online servo joint command contains NaN or infinity")
            if not math.isfinite(gripper_position):
                raise SafetyError(
                    "online servo gripper position contains NaN or infinity"
                )
            maximum_gripper_width = float(robot_config.gripper_width)
            if not 0.0 <= gripper_position <= maximum_gripper_width:
                raise SafetyError(
                    "online servo gripper position is outside its configured range"
                )
            lower = robot_config.joint_pos_min + self.safety.joint_limit_margin_rad
            upper = robot_config.joint_pos_max - self.safety.joint_limit_margin_rad
            if np.any(position < lower) or np.any(position > upper):
                raise SafetyError("online servo command approaches a joint limit")
            servo_velocity_limits = np.minimum(
                np.asarray(robot_config.joint_vel_max, dtype=np.float64),
                self.safety.servo_max_velocity_rad_s,
            )
            if np.any(np.abs(velocity) > servo_velocity_limits + 1e-6):
                raise SafetyError(
                    "online servo velocity exceeded its per-joint limit"
                )

            snapshot = self._read_snapshot()
            self._validate_snapshot(snapshot)
            self._check_gripper_feedback(
                snapshot.gripper_position,
                snapshot.gripper_velocity,
                snapshot.gripper_torque,
                0.0,
                robot_config.gripper_width,
            )
            self._check_motion_feedback(
                snapshot.joint_position,
                snapshot.joint_velocity,
                snapshot.joint_torque,
                snapshot.joint_position,
                position,
                envelope_min=robot_config.joint_pos_min,
                envelope_max=robot_config.joint_pos_max,
            )
            tracking_error = np.abs(position - snapshot.joint_position)
            if np.any(tracking_error > self.safety.servo_tracking_error_rad):
                joint = int(np.argmax(tracking_error))
                raise SafetyError(
                    f"joint {joint + 1} servo tracking error "
                    f"{math.degrees(tracking_error[joint]):.1f} deg exceeded the limit"
                )
            self._require_sdk().set_joint_cmd(
                self._make_joint_state(
                    position,
                    gripper_position,
                    0.0,
                    joint_velocity=velocity,
                )
            )
            return snapshot

    def minimum_duration(self, delta_rad: float) -> float:
        delta = np.array([float(delta_rad)], dtype=np.float64)
        return minimum_cosine_duration(
            delta, self.safety.max_commanded_velocity_rad_s
        )

    def move_gripper(
        self, target_width_m: float, duration_s: float
    ) -> GripperMotionResult:
        """Move the gripper to an absolute finger opening in metres."""
        self._clear_gripper_force_hold()
        self.activate_gripper()
        start = self.get_state()
        target_width_m = float(target_width_m)
        return self._run_gripper_motion(
            start=start,
            excursion_width_m=target_width_m,
            final_width_m=target_width_m,
            samples_factory=lambda timestamp: cosine_segment(
                np.array([start.gripper_position], dtype=np.float64),
                np.array([target_width_m], dtype=np.float64),
                timestamp,
                duration_s,
                self.safety.trajectory_rate_hz,
            ),
            commanded_duration_s=duration_s,
            segment_deltas_m=(target_width_m - start.gripper_position,),
            segment_durations_s=(duration_s,),
        )

    def open_close_gripper(
        self,
        open_width_m: float = 0.040,
        *,
        open_duration_s: float = 2.0,
        hold_s: float = 0.5,
        close_duration_s: float = 2.0,
        close_width_m: float = 0.0,
    ) -> GripperMotionResult:
        """Open the gripper, pause, and then close it slowly."""
        if not math.isfinite(hold_s) or not 0.0 <= hold_s <= 30.0:
            raise SafetyError("gripper hold duration must be in [0, 30] s")
        self._clear_gripper_force_hold()
        self.activate_gripper()
        start = self.get_state()
        open_width_m = float(open_width_m)
        close_width_m = float(close_width_m)

        return self._run_gripper_motion(
            start=start,
            excursion_width_m=open_width_m,
            final_width_m=close_width_m,
            samples_factory=lambda timestamp: via_point(
                np.array([start.gripper_position], dtype=np.float64),
                np.array([open_width_m], dtype=np.float64),
                np.array([close_width_m], dtype=np.float64),
                timestamp,
                open_duration_s,
                hold_s,
                close_duration_s,
                self.safety.trajectory_rate_hz,
            ),
            commanded_duration_s=open_duration_s + hold_s + close_duration_s,
            segment_deltas_m=(
                open_width_m - start.gripper_position,
                close_width_m - open_width_m,
            ),
            segment_durations_s=(open_duration_s, close_duration_s),
        )

    def minimum_gripper_duration(self, delta_m: float) -> float:
        delta = np.array([float(delta_m)], dtype=np.float64)
        return minimum_cosine_duration(
            delta, self.safety.max_commanded_gripper_velocity_m_s
        )

    def force_close_gripper(
        self,
        closing_torque_nm: float = 0.15,
        *,
        minimum_width_m: float = 0.0,
        timeout_s: float = 5.0,
        contact_hold_s: float = 0.5,
        position_hold_after: bool = True,
        torque_hold_after: bool = False,
    ) -> GripperForceResult:
        """Close under limited motor torque, bounded by travel and time.

        ``closing_torque_nm`` is the SDK's current-estimated motor torque, not
        a calibrated fingertip force in newtons.  Contact is inferred from a
        sustained low gripper velocity after the torque ramp.
        """
        self._validate_force_close(
            closing_torque_nm,
            minimum_width_m,
            timeout_s,
            contact_hold_s,
        )
        if torque_hold_after and not position_hold_after:
            raise SafetyError("torque hold requires position hold")
        self._ensure_active()
        start = self.get_state()
        if minimum_width_m > (
            start.gripper_position + self.safety.force_close_position_margin_m
        ):
            raise SafetyError(
                "minimum width is larger than the current gripper opening"
            )
        # Remove position stiffness before applying torque.  Keep the arm at
        # its measured joint pose and explicitly clear any previous torque.
        self._clear_gripper_force_hold()
        self._set_joint_cmd_in_mode(
            self._make_joint_state(
                start.joint_position, start.gripper_position, 0.0
            ),
            ControllerMode.ACTIVE,
        )
        if self._current_gripper_gain_scale > 0.0:
            self._ramp_gains(
                self._current_arm_gain_scale,
                self._current_arm_gain_scale,
                self._current_gripper_gain_scale,
                0.0,
                self.gripper_gain_ramp_s,
            )

        motor_torque = self._closing_motor_torque(
            closing_torque_nm,
            self._require_robot_config().gripper_open_readout,
        )
        started_at = time.monotonic()
        stall_since: float | None = None
        max_velocity = abs(start.gripper_velocity)
        max_torque = abs(start.gripper_torque)
        stop_reason = "timeout"
        final = start

        try:
            while True:
                now = time.monotonic()
                elapsed = now - started_at
                ramp_alpha = min(1.0, elapsed / self.safety.force_ramp_s)
                torque_scale = cosine_fraction(ramp_alpha)
                state = self.get_state()
                self._check_motion_feedback(
                    state.joint_position,
                    state.joint_velocity,
                    state.joint_torque,
                    start.joint_position,
                    start.joint_position,
                )
                self._check_gripper_feedback(
                    state.gripper_position,
                    state.gripper_velocity,
                    state.gripper_torque,
                    minimum_width_m,
                    start.gripper_position,
                )
                max_velocity = max(max_velocity, abs(state.gripper_velocity))
                max_torque = max(max_torque, abs(state.gripper_torque))
                self._set_joint_cmd_in_mode(
                    self._make_joint_state(
                        start.joint_position,
                        state.gripper_position,
                        0.0,
                        gripper_torque=motor_torque * torque_scale,
                    ),
                    ControllerMode.ACTIVE,
                )

                if state.gripper_position <= (
                    minimum_width_m + self.safety.force_close_position_margin_m
                ):
                    stop_reason = "minimum_width"
                    final = state
                    break

                if ramp_alpha >= 1.0:
                    if (
                        abs(state.gripper_velocity)
                        <= self.safety.force_close_stall_velocity_m_s
                    ):
                        if stall_since is None:
                            stall_since = now
                        elif now - stall_since >= self.safety.force_close_stall_time_s:
                            stop_reason = "contact"
                            final = state
                            break
                    else:
                        stall_since = None

                if elapsed >= timeout_s:
                    final = state
                    break
                time.sleep(self.safety.monitor_period_s)

            if stop_reason == "contact" and contact_hold_s > 0.0:
                hold_deadline = time.monotonic() + contact_hold_s
                while time.monotonic() < hold_deadline:
                    state = self.get_state()
                    self._check_motion_feedback(
                        state.joint_position,
                        state.joint_velocity,
                        state.joint_torque,
                        start.joint_position,
                        start.joint_position,
                    )
                    self._check_gripper_feedback(
                        state.gripper_position,
                        state.gripper_velocity,
                        state.gripper_torque,
                        minimum_width_m,
                        start.gripper_position,
                    )
                    self._set_joint_cmd_in_mode(
                        self._make_joint_state(
                            start.joint_position,
                            state.gripper_position,
                            0.0,
                            gripper_torque=motor_torque,
                        ),
                        ControllerMode.ACTIVE,
                    )
                    max_velocity = max(max_velocity, abs(state.gripper_velocity))
                    max_torque = max(max_torque, abs(state.gripper_torque))
                    final = state
                    time.sleep(self.safety.monitor_period_s)

            if torque_hold_after and stop_reason == "contact":
                self._gripper_force_hold_position_m = final.gripper_position
                self._gripper_force_hold_motor_torque_nm = motor_torque
                self._set_joint_cmd_in_mode(
                    self._make_joint_state(
                        start.joint_position,
                        self._gripper_force_hold_position_m,
                        0.0,
                        gripper_torque=motor_torque,
                    ),
                    ControllerMode.ACTIVE,
                )
                final = self.activate_gripper()
            else:
                ramp_down_start = time.monotonic()
                while True:
                    elapsed_down = time.monotonic() - ramp_down_start
                    if elapsed_down >= self.safety.force_ramp_s:
                        break
                    state = self.get_state()
                    self._check_motion_feedback(
                        state.joint_position,
                        state.joint_velocity,
                        state.joint_torque,
                        start.joint_position,
                        start.joint_position,
                    )
                    self._check_gripper_feedback(
                        state.gripper_position,
                        state.gripper_velocity,
                        state.gripper_torque,
                        minimum_width_m,
                        start.gripper_position,
                    )
                    scale = 1.0 - cosine_fraction(
                        elapsed_down / self.safety.force_ramp_s
                    )
                    self._set_joint_cmd_in_mode(
                        self._make_joint_state(
                            start.joint_position,
                            state.gripper_position,
                            0.0,
                            gripper_torque=motor_torque * scale,
                        ),
                        ControllerMode.ACTIVE,
                    )
                    max_velocity = max(max_velocity, abs(state.gripper_velocity))
                    max_torque = max(max_torque, abs(state.gripper_torque))
                    final = state
                    time.sleep(self.safety.monitor_period_s)

                zero_state = self.get_state()
                self._set_joint_cmd_in_mode(
                    self._make_joint_state(
                        start.joint_position, zero_state.gripper_position, 0.0
                    ),
                    ControllerMode.ACTIVE,
                )
                final = self.get_state()
                if position_hold_after:
                    final = self.activate_gripper()
        except BaseException as exc:
            self.emergency_damping(f"force close aborted: {exc}")
            raise

        return GripperForceResult(
            start_width_m=start.gripper_position,
            final_width_m=final.gripper_position,
            minimum_width_m=minimum_width_m,
            commanded_closing_torque_nm=closing_torque_nm,
            elapsed_s=time.monotonic() - started_at,
            stop_reason=stop_reason,
            max_abs_velocity_m_s=max_velocity,
            max_abs_torque_nm=max_torque,
            position_hold_enabled=position_hold_after,
        )

    def close(self, *, emergency: bool = False) -> None:
        if self._mode in (ControllerMode.CLOSED, ControllerMode.DISCONNECTED):
            self._lock.release()
            self._mode = ControllerMode.CLOSED
            return

        if self._mode not in (ControllerMode.CLOSED, ControllerMode.DISCONNECTED):
            self._stop_online_servos("controller close")
        sdk = self._sdk
        try:
            if sdk is not None:
                if emergency:
                    self.emergency_damping("emergency close")
                elif self._mode == ControllerMode.ACTIVE:
                    self.damping()
                else:
                    try:
                        sdk.set_to_damping()
                    except Exception:
                        pass
        finally:
            self._sdk = None
            self._robot_config = None
            self._controller_config = None
            self._cartesian_planner = None
            self._last_feedback_for_acceleration = None
            self._clear_cached_state()
            self._clear_gripper_force_hold()
            if sdk is not None:
                del sdk
            self._lock.release()
            self._mode = ControllerMode.CLOSED

    def __enter__(self) -> ArmController:
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close(emergency=exc_type is not None)

    def _ensure_active(self) -> None:
        if self._mode == ControllerMode.DAMPING:
            self.activate()
        self._require_active()

    def _scaled_derivative_limits(
        self,
        velocity_scale: float,
    ) -> tuple[float, float, float]:
        """Scale motion derivatives while keeping low-speed ramps practical."""
        return scaled_derivative_limits(
            self.safety.max_commanded_velocity_rad_s,
            self.safety.servo_max_acceleration_rad_s2,
            self.safety.servo_max_jerk_rad_s3,
            velocity_scale,
        )

    def _scaled_joint_derivative_limits(
        self,
        velocity_scale: float,
        *,
        servo: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return conservative per-joint Ruckig limits.

        SDK exposes different velocity caps per joint. Application caps still
        bound every joint, while acceleration and jerk remain explicit local
        limits because SDK does not publish those derivatives.
        """
        robot_config = self._require_robot_config()
        if servo:
            velocity, acceleration, jerk = scaled_derivative_limits(
                self.safety.servo_max_velocity_rad_s,
                self.safety.servo_max_acceleration_rad_s2,
                self.safety.servo_max_jerk_rad_s3,
                velocity_scale,
            )
        else:
            velocity, acceleration, jerk = scaled_derivative_limits(
                self.safety.max_commanded_velocity_rad_s,
                self.safety.servo_max_acceleration_rad_s2,
                self.safety.servo_max_jerk_rad_s3,
                velocity_scale,
            )
        joint_velocity = np.minimum(
            np.asarray(robot_config.joint_vel_max, dtype=np.float64),
            velocity,
        )
        size = int(robot_config.joint_dof)
        return (
            joint_velocity,
            np.full(size, acceleration, dtype=np.float64),
            np.full(size, jerk, dtype=np.float64),
        )

    def _plan_joint_curve(
        self,
        start: ArmSnapshot,
        target: np.ndarray,
        *,
        minimum_duration_s: float,
        velocity_scale: float = 1.0,
    ) -> RuckigJointTrajectory:
        """Build one synchronized joint S-curve for every ordinary move."""
        if not math.isfinite(velocity_scale) or not 0.0 < velocity_scale <= 1.0:
            raise SafetyError("joint velocity scale must be in (0, 1]")
        robot_config = self._require_robot_config()
        velocity_limit, acceleration_limit, jerk_limit = (
            self._scaled_joint_derivative_limits(1.0)
        )
        velocity_limit = velocity_limit * velocity_scale
        acceleration_limit = acceleration_limit * velocity_scale**2
        jerk_limit = jerk_limit * velocity_scale**3
        try:
            return plan_ruckig_joint_trajectory(
                start.joint_position,
                target,
                start.joint_velocity,
                current_acceleration=getattr(start, "joint_acceleration", None),
                min_position=robot_config.joint_pos_min
                + self.safety.joint_limit_margin_rad,
                max_position=robot_config.joint_pos_max
                - self.safety.joint_limit_margin_rad,
                max_velocity=velocity_limit,
                max_acceleration=acceleration_limit,
                max_jerk=jerk_limit,
                sample_rate_hz=self.safety.trajectory_rate_hz,
                minimum_duration_s=float(minimum_duration_s),
            )
        except (ValueError, RuntimeError) as exc:
            raise SafetyError(f"joint S-curve planning failed: {exc}") from exc

    def _trajectory_samples_with_final_hold(
        self,
        trajectory: RuckigJointTrajectory,
        start_timestamp: float,
    ) -> list[TrajectorySample]:
        samples = trajectory.samples(start_timestamp)
        samples.append(
            TrajectorySample(
                timestamp=(
                    start_timestamp
                    + trajectory.duration_s
                    + self.safety.trajectory_final_hold_s
                ),
                position=trajectory.positions[-1].copy(),
                velocity=np.zeros(trajectory.positions.shape[1], dtype=np.float64),
            )
        )
        return samples

    def _hold_measured_pose_during_active_motion(self) -> None:
        """Override a completed trajectory without waiting on this same worker."""
        snapshot = self.get_state()
        gripper_position, gripper_torque = self._gripper_hold_command(
            snapshot.gripper_position
        )
        self._set_joint_cmd_in_mode(
            self._make_joint_state(
                snapshot.joint_position,
                gripper_position,
                0.0,
                gripper_torque=gripper_torque,
            ),
            ControllerMode.ACTIVE,
        )

    def _run_motion(
        self,
        *,
        start: ArmSnapshot,
        excursion_target: np.ndarray,
        final_target: np.ndarray,
        samples_factory,
        commanded_duration_s: float,
        segment_durations: tuple[float, ...] | None = None,
        allow_large_motion: bool = False,
        max_duration_s: float = 30.0,
        require_cosine_duration: bool = True,
        trajectory_start_velocity: np.ndarray | None = None,
        stop_kind: str = "joint",
    ) -> MotionResult:
        self._require_active()
        excursion_target = np.asarray(excursion_target, dtype=np.float64)
        final_target = np.asarray(final_target, dtype=np.float64)
        delta = excursion_target - start.joint_position
        durations = segment_durations or (commanded_duration_s,)
        self._validate_motion(
            start.joint_position,
            excursion_target,
            delta,
            durations,
            allow_large_motion=allow_large_motion,
            max_duration_s=max_duration_s,
            require_cosine_duration=require_cosine_duration,
        )

        start_timestamp = (
            self._require_sdk().get_timestamp() + self.safety.preview_time_s
        )
        samples: list[TrajectorySample] = samples_factory(start_timestamp)
        if not samples:
            raise ControlError("trajectory generator returned no samples")
        self._validate_trajectory_samples(
            start.joint_position,
            samples,
            start_timestamp,
            start_velocity=(
                start.joint_velocity
                if trajectory_start_velocity is None
                else np.asarray(trajectory_start_velocity, dtype=np.float64)
            ),
            allow_large_motion=allow_large_motion,
        )
        sample_positions = np.stack([sample.position for sample in samples])
        trajectory_min = np.minimum(
            start.joint_position,
            np.min(sample_positions, axis=0),
        )
        trajectory_max = np.maximum(
            start.joint_position,
            np.max(sample_positions, axis=0),
        )
        gripper_position, gripper_torque = self._gripper_hold_command(
            start.gripper_position
        )
        waypoints = [
            self._make_joint_state(
                sample.position,
                gripper_position,
                sample.timestamp,
                joint_velocity=sample.velocity,
                gripper_torque=gripper_torque,
            )
            for sample in samples
        ]

        with self._mode_lock:
            if self._motion_in_progress:
                raise BusyError("another motion is already in progress")
            self._motion_in_progress = True
            self._trajectory_stop_kind = stop_kind
            self._trajectory_stop_scale = 1.0
            self._trajectory_stop_requested.clear()
            self._trajectory_stop_completed.clear()
        try:
            self._set_joint_traj_in_mode(waypoints, ControllerMode.ACTIVE)
            return self._monitor_motion(
                start=start,
                excursion_target=excursion_target,
                final_target=final_target,
                finish_timestamp=samples[-1].timestamp,
                commanded_duration_s=commanded_duration_s,
                trajectory_min=trajectory_min,
                trajectory_max=trajectory_max,
            )
        except SafetyError as exc:
            # A terminal settle miss means feedback is still available and the
            # arm can safely hold its measured pose. Other safety failures keep
            # their existing emergency-damping behavior.
            if not isinstance(exc, MotionSettleError):
                self.emergency_damping(f"motion aborted: {exc}")
                raise
            try:
                # Calling hold_current here would request trajectory stop and
                # wait for this very worker, causing a self-wait timeout.
                self._hold_measured_pose_during_active_motion()
            except BaseException as hold_error:
                self.emergency_damping(
                    f"motion settle recovery failed: {hold_error}"
                )
            raise
        except BaseException as exc:
            self.emergency_damping(f"motion aborted: {exc}")
            raise
        finally:
            with self._mode_lock:
                self._motion_in_progress = False
                self._trajectory_stop_kind = None
                self._trajectory_stop_requested.clear()
                self._trajectory_stop_completed.set()

    def _run_gripper_motion(
        self,
        *,
        start: ArmSnapshot,
        excursion_width_m: float,
        final_width_m: float,
        samples_factory,
        commanded_duration_s: float,
        segment_deltas_m: tuple[float, ...],
        segment_durations_s: tuple[float, ...],
    ) -> GripperMotionResult:
        self._require_active()
        self._validate_gripper_motion(
            start.gripper_position,
            (excursion_width_m, final_width_m),
            segment_deltas_m,
            segment_durations_s,
        )

        start_timestamp = (
            self._require_sdk().get_timestamp() + self.safety.preview_time_s
        )
        samples: list[TrajectorySample] = samples_factory(start_timestamp)
        if not samples:
            raise ControlError("trajectory generator returned no samples")
        waypoints = [
            self._make_joint_state(
                start.joint_position, float(sample.position[0]), sample.timestamp
            )
            for sample in samples
        ]

        try:
            self._set_joint_traj_in_mode(waypoints, ControllerMode.ACTIVE)
            return self._monitor_gripper_motion(
                start=start,
                excursion_width_m=excursion_width_m,
                final_width_m=final_width_m,
                finish_timestamp=samples[-1].timestamp,
                commanded_duration_s=commanded_duration_s,
            )
        except BaseException as exc:
            self.emergency_damping(f"gripper motion aborted: {exc}")
            raise

    def _monitor_gripper_motion(
        self,
        *,
        start: ArmSnapshot,
        excursion_width_m: float,
        final_width_m: float,
        finish_timestamp: float,
        commanded_duration_s: float,
    ) -> GripperMotionResult:
        sdk = self._require_sdk()
        max_gripper_velocity = abs(start.gripper_velocity)
        max_gripper_torque = abs(start.gripper_torque)
        min_gripper_position = start.gripper_position
        max_gripper_position = start.gripper_position
        last_state_timestamp = start.timestamp
        last_progress_time = time.monotonic()
        wall_deadline = (
            time.monotonic()
            + commanded_duration_s
            + self.safety.preview_time_s
            + self.safety.settle_time_s
            + 2.0
        )
        envelope_min = min(
            start.gripper_position, excursion_width_m, final_width_m
        )
        envelope_max = max(
            start.gripper_position, excursion_width_m, final_width_m
        )

        while sdk.get_timestamp() < finish_timestamp + self.safety.settle_time_s:
            self._require_active()
            state = sdk.get_joint_state()
            position = state.pos().copy()
            velocity = state.vel().copy()
            torque = state.torque().copy()
            gripper_position = float(state.gripper_pos)
            gripper_velocity = float(state.gripper_vel)
            gripper_torque = float(state.gripper_torque)
            now = time.monotonic()

            if state.timestamp > last_state_timestamp + 1e-6:
                last_state_timestamp = state.timestamp
                last_progress_time = now
            elif now - last_progress_time > self.safety.state_stale_timeout_s:
                raise SafetyError("joint feedback timestamp stopped advancing")
            if now > wall_deadline:
                raise SafetyError("gripper motion exceeded its wall-clock deadline")

            self._check_motion_feedback(
                position,
                velocity,
                torque,
                start.joint_position,
                start.joint_position,
            )
            self._check_gripper_feedback(
                gripper_position,
                gripper_velocity,
                gripper_torque,
                envelope_min,
                envelope_max,
            )
            self._remember_monitored_feedback(state)
            min_gripper_position = min(min_gripper_position, gripper_position)
            max_gripper_position = max(max_gripper_position, gripper_position)
            max_gripper_velocity = max(
                max_gripper_velocity, abs(gripper_velocity)
            )
            max_gripper_torque = max(max_gripper_torque, abs(gripper_torque))
            time.sleep(self.safety.monitor_period_s)

        self._require_active()
        final = self.get_state()
        tolerance = self.safety.gripper_position_tolerance_m
        if abs(final.gripper_position - final_width_m) > tolerance:
            raise SafetyError(
                f"gripper final error was "
                f"{1000.0 * (final.gripper_position - final_width_m):+.1f} mm"
            )
        if excursion_width_m > start.gripper_position + tolerance:
            if max_gripper_position < excursion_width_m - tolerance:
                raise SafetyError(
                    "gripper did not reach the requested excursion: "
                    f"requested {1000.0 * excursion_width_m:.1f} mm, "
                    f"measured maximum {1000.0 * max_gripper_position:.1f} mm"
                )
        elif excursion_width_m < start.gripper_position - tolerance:
            if min_gripper_position > excursion_width_m + tolerance:
                raise SafetyError(
                    "gripper did not reach the requested excursion: "
                    f"requested {1000.0 * excursion_width_m:.1f} mm, "
                    f"measured minimum {1000.0 * min_gripper_position:.1f} mm"
                )

        return GripperMotionResult(
            start_width_m=start.gripper_position,
            excursion_width_m=excursion_width_m,
            target_width_m=final_width_m,
            final_width_m=final.gripper_position,
            commanded_duration_s=commanded_duration_s,
            min_measured_width_m=min_gripper_position,
            max_measured_width_m=max_gripper_position,
            max_abs_velocity_m_s=max_gripper_velocity,
            max_abs_torque_nm=max_gripper_torque,
        )

    def _terminal_feedback_is_settled(
        self,
        samples: Sequence[tuple[float, np.ndarray, np.ndarray]],
        target: np.ndarray,
    ) -> bool:
        if len(samples) < 2:
            return False
        covered_time_s = float(samples[-1][0]) - float(samples[0][0])
        if covered_time_s + 1e-9 < self.safety.terminal_stability_window_s:
            return False
        positions = np.stack([sample[1] for sample in samples])
        target_position = np.asarray(target, dtype=np.float64)
        target_error = np.max(np.abs(positions - target_position), axis=0)
        position_span = np.ptp(positions, axis=0)
        return bool(
            np.all(
                target_error
                <= self.safety.terminal_position_tolerance_rad
            )
            and np.max(position_span)
            <= self.safety.terminal_position_stability_rad
        )

    def _brake_active_trajectory(
        self,
        snapshot: ArmSnapshot,
        deceleration_scale: float,
    ) -> tuple[ArmSnapshot, np.ndarray, np.ndarray]:
        """Replace a queued trajectory with fixed-period jerk-limited braking."""
        from .motion import JointRuckigPlanner

        robot_config = self._require_robot_config()
        velocity_limit, acceleration_limit, jerk_limit = (
            self._scaled_joint_derivative_limits(1.0, servo=True)
        )
        velocity_limit = np.maximum(
            velocity_limit,
            np.abs(snapshot.joint_velocity) + 1e-6,
        )
        planner = JointRuckigPlanner(
            snapshot.joint_position,
            snapshot.joint_velocity,
            initial_acceleration=snapshot.joint_acceleration,
            period_s=1.0 / self.safety.servo_rate_hz,
            max_velocity=velocity_limit,
            max_acceleration=acceleration_limit,
            max_jerk=jerk_limit,
            min_position=robot_config.joint_pos_min,
            max_position=robot_config.joint_pos_max,
        )
        planner.set_brake_scale(deceleration_scale)
        planner.brake()
        max_velocity = np.abs(snapshot.joint_velocity)
        max_torque = np.abs(snapshot.joint_torque)
        final = snapshot
        period_s = 1.0 / self.safety.servo_rate_hz
        deadline = time.monotonic()
        wall_deadline = time.monotonic() + 5.0 / deceleration_scale
        gripper_position, gripper_torque = self._gripper_hold_command(
            snapshot.gripper_position
        )
        while True:
            self._require_active()
            command = planner.step()
            self._set_joint_cmd_in_mode(
                self._make_joint_state(
                    command.position,
                    gripper_position,
                    0.0,
                    joint_velocity=command.velocity,
                    gripper_torque=gripper_torque,
                ),
                ControllerMode.ACTIVE,
            )
            deadline += period_s
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            state = self._require_sdk().get_joint_state()
            final = self._remember_monitored_feedback(state)
            self._check_motion_feedback(
                final.joint_position,
                final.joint_velocity,
                final.joint_torque,
                snapshot.joint_position,
                command.position,
                envelope_min=robot_config.joint_pos_min,
                envelope_max=robot_config.joint_pos_max,
            )
            max_velocity = np.maximum(max_velocity, np.abs(final.joint_velocity))
            max_torque = np.maximum(max_torque, np.abs(final.joint_torque))
            if command.finished:
                self._set_joint_cmd_in_mode(
                    self._make_joint_state(
                        command.position,
                        gripper_position,
                        0.0,
                        joint_velocity=np.zeros_like(command.velocity),
                        gripper_torque=gripper_torque,
                    ),
                    ControllerMode.ACTIVE,
                )
                break
            if time.monotonic() > wall_deadline:
                raise SafetyError("trajectory braking exceeded its deadline")
        return final, max_velocity, max_torque

    def _monitor_motion(
        self,
        *,
        start: ArmSnapshot,
        excursion_target: np.ndarray,
        final_target: np.ndarray,
        finish_timestamp: float,
        commanded_duration_s: float,
        trajectory_min: np.ndarray,
        trajectory_max: np.ndarray,
    ) -> MotionResult:
        sdk = self._require_sdk()
        max_velocity = np.zeros_like(start.joint_velocity)
        max_torque = np.zeros_like(start.joint_torque)
        last_state_timestamp = start.timestamp
        last_progress_time = time.monotonic()
        terminal_samples: deque[tuple[float, np.ndarray, np.ndarray]] = deque()
        settled_position: np.ndarray | None = None
        wall_deadline = (
            time.monotonic()
            + commanded_duration_s
            + self.safety.preview_time_s
            + self.safety.settle_time_s
            + 2.0
        )

        while sdk.get_timestamp() < finish_timestamp + self.safety.settle_time_s:
            self._require_active()
            state = sdk.get_joint_state()
            position = state.pos().copy()
            velocity = state.vel().copy()
            torque = state.torque().copy()
            now = time.monotonic()

            if state.timestamp > last_state_timestamp + 1e-6:
                last_state_timestamp = state.timestamp
                last_progress_time = now
            elif now - last_progress_time > self.safety.state_stale_timeout_s:
                raise SafetyError("joint feedback timestamp stopped advancing")
            if now > wall_deadline:
                raise SafetyError("motion exceeded its wall-clock deadline")

            self._check_motion_feedback(
                position,
                velocity,
                torque,
                start.joint_position,
                excursion_target,
                envelope_min=trajectory_min,
                envelope_max=trajectory_max,
            )
            snapshot = self._remember_monitored_feedback(state)
            max_velocity = np.maximum(max_velocity, np.abs(velocity))
            max_torque = np.maximum(max_torque, np.abs(torque))
            if self._trajectory_stop_requested.is_set():
                stopped, brake_velocity, brake_torque = (
                    self._brake_active_trajectory(
                        snapshot,
                        self._trajectory_stop_scale,
                    )
                )
                return MotionResult(
                    start_position=start.joint_position.copy(),
                    target_position=final_target.copy(),
                    final_position=stopped.joint_position.copy(),
                    commanded_duration_s=commanded_duration_s,
                    max_abs_velocity_rad_s=np.maximum(
                        max_velocity,
                        brake_velocity,
                    ),
                    max_abs_torque_nm=np.maximum(max_torque, brake_torque),
                )
            if float(state.timestamp) >= finish_timestamp:
                terminal_samples.append(
                    (
                        float(state.timestamp),
                        position.copy(),
                        velocity.copy(),
                    )
                )
                cutoff = (
                    float(state.timestamp)
                    - self.safety.terminal_stability_window_s
                )
                while (
                    len(terminal_samples) >= 2
                    and terminal_samples[1][0] <= cutoff
                ):
                    terminal_samples.popleft()
                if self._terminal_feedback_is_settled(
                    terminal_samples,
                    final_target,
                ):
                    settled_position = position.copy()
                    break
            time.sleep(self.safety.monitor_period_s)

        self._require_active()
        if settled_position is not None:
            return MotionResult(
                start_position=start.joint_position.copy(),
                target_position=final_target.copy(),
                final_position=settled_position,
                commanded_duration_s=commanded_duration_s,
                max_abs_velocity_rad_s=max_velocity,
                max_abs_torque_nm=max_torque,
            )
        final = self.get_state()
        terminal_error = np.abs(final.joint_position - final_target)
        if (
            float(np.max(terminal_error))
            > self.safety.terminal_position_tolerance_rad
        ):
            joint = int(np.argmax(terminal_error))
            raise MotionSettleError(
                f"joint {joint + 1} did not settle at the trajectory target "
                f"({math.degrees(terminal_error[joint]):.2f} deg error)"
            )
        terminal_positions = (
            np.stack([sample[1] for sample in terminal_samples])
            if terminal_samples
            else np.empty((0, final.joint_position.size), dtype=np.float64)
        )
        position_span = (
            np.ptp(terminal_positions, axis=0)
            if terminal_positions.size
            else np.full_like(final.joint_position, math.inf)
        )
        observed_target_error = (
            np.max(np.abs(terminal_positions - final_target), axis=0)
            if terminal_positions.size
            else np.full_like(final.joint_position, math.inf)
        )
        terminal_velocity = np.abs(final.joint_velocity)
        normalized_instability = np.maximum(
            position_span / self.safety.terminal_position_stability_rad,
            observed_target_error / self.safety.terminal_position_tolerance_rad,
        )
        joint = int(np.argmax(normalized_instability))
        raise MotionSettleError(
            f"joint {joint + 1} did not stabilize at the trajectory target "
            f"({math.degrees(observed_target_error[joint]):.2f} deg peak error, "
            f"{math.degrees(position_span[joint]):.2f} deg feedback drift, "
            f"{terminal_velocity[joint]:.3f} rad/s reported)"
        )

    def _validate_motion(
        self,
        start: np.ndarray,
        target: np.ndarray,
        delta: np.ndarray,
        durations: tuple[float, ...],
        *,
        allow_large_motion: bool = False,
        max_duration_s: float = 30.0,
        require_cosine_duration: bool = True,
    ) -> None:
        robot_config = self._require_robot_config()
        if target.shape != start.shape or not np.all(np.isfinite(target)):
            raise SafetyError("target must be a finite vector matching the arm DOF")
        if (
            not allow_large_motion
            and np.max(np.abs(delta))
            > self.safety.max_relative_motion_rad + 1e-12
        ):
            raise SafetyError(
                "relative target exceeds the configured per-command limit of "
                f"{math.degrees(self.safety.max_relative_motion_rad):.1f} deg"
            )

        lower = robot_config.joint_pos_min + self.safety.joint_limit_margin_rad
        upper = robot_config.joint_pos_max - self.safety.joint_limit_margin_rad
        if np.any(target < lower) or np.any(target > upper):
            joint = int(np.argmax((target < lower) | (target > upper)))
            raise SafetyError(f"joint {joint + 1} target is too close to a software limit")

        for duration in durations:
            minimum_duration_s = self.safety.minimum_motion_duration_s
            if (
                not math.isfinite(duration)
                or not minimum_duration_s <= duration <= max_duration_s
            ):
                raise SafetyError(
                    "each trajectory duration must be in "
                    f"[{minimum_duration_s:.2f}, {max_duration_s:.0f}] s"
                )
            if require_cosine_duration:
                required = minimum_cosine_duration(
                    delta, self.safety.max_commanded_velocity_rad_s
                )
                if duration + 1e-9 < required:
                    raise SafetyError(
                        f"duration {duration:.2f}s is too short; use at least "
                        f"{required:.2f}s for the configured velocity cap"
                    )

    def _validate_trajectory_samples(
        self,
        start: np.ndarray,
        samples: list[TrajectorySample],
        start_timestamp: float,
        *,
        start_velocity: np.ndarray | None = None,
        allow_large_motion: bool = False,
    ) -> None:
        """Validate every precomputed point before submitting any trajectory."""
        positions = np.asarray(
            [sample.position for sample in samples],
            dtype=np.float64,
        )
        timestamps = np.asarray(
            [sample.timestamp for sample in samples],
            dtype=np.float64,
        )
        if positions.ndim != 2 or positions.shape[1:] != start.shape:
            raise SafetyError("trajectory samples have an invalid joint shape")
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(timestamps)):
            raise SafetyError("trajectory samples contain NaN or infinity")
        specified_velocities = [
            sample.velocity for sample in samples if sample.velocity is not None
        ]
        robot_config = self._require_robot_config()
        joint_velocity_limits = np.minimum(
            np.asarray(robot_config.joint_vel_max, dtype=np.float64),
            self.safety.max_commanded_velocity_rad_s,
        )
        if specified_velocities:
            if len(specified_velocities) != len(samples):
                raise SafetyError("trajectory velocities must be set for every sample")
            velocities = np.asarray(specified_velocities, dtype=np.float64)
            if velocities.shape != positions.shape or not np.all(
                np.isfinite(velocities)
            ):
                raise SafetyError(
                    "trajectory velocities have an invalid shape or value"
                )
            velocity_ratio = np.max(
                np.abs(velocities) / joint_velocity_limits[None, :]
            )
            if velocity_ratio > 1.0 + 1e-6:
                joint = int(np.argmax(np.max(
                    np.abs(velocities) / joint_velocity_limits[None, :], axis=0
                )))
                raise SafetyError(
                    f"trajectory command velocity for joint {joint + 1} "
                    "exceeds its configured limit"
                )
        times = np.concatenate(([start_timestamp], timestamps))
        intervals = np.diff(times)
        if np.any(intervals <= 0.0):
            raise SafetyError("trajectory timestamps must be strictly increasing")

        lower = robot_config.joint_pos_min + self.safety.joint_limit_margin_rad
        upper = robot_config.joint_pos_max - self.safety.joint_limit_margin_rad
        outside = (positions < lower) | (positions > upper)
        if np.any(outside):
            invalid = outside.copy()
            if allow_large_motion:
                # A recovery move may start inside the application margin.
                # Permit only samples that never move farther outside than the
                # measured start, while the physical joint limits remain hard.
                for joint in range(start.size):
                    if start[joint] < lower[joint]:
                        invalid[:, joint] &= positions[:, joint] < start[joint] - 1e-9
                    elif start[joint] > upper[joint]:
                        invalid[:, joint] &= positions[:, joint] > start[joint] + 1e-9
                physical_outside = (
                    (positions < robot_config.joint_pos_min)
                    | (positions > robot_config.joint_pos_max)
                )
                invalid |= physical_outside
            if np.any(invalid):
                _, joint = np.argwhere(invalid)[0]
                raise SafetyError(
                    f"trajectory for joint {int(joint) + 1} approaches a software limit"
                )
        relative = positions - start
        if (
            not allow_large_motion
            and np.max(np.abs(relative))
            > self.safety.max_relative_motion_rad + 1e-12
        ):
            raise SafetyError("trajectory exceeds the configured per-command joint limit")

        with_start = np.vstack([start, positions])
        planner = self._cartesian_planner
        if planner is not None:
            try:
                check_tcp = getattr(planner, "check_tcp_collision", None)
                if callable(check_tcp):
                    tcp_collision = check_tcp(with_start)
                    if not tcp_collision.safe:
                        raise SafetyError(
                            "trajectory TCP collides with "
                            f"{tcp_collision.obstacle_name} near waypoint "
                            f"{tcp_collision.waypoint_index}"
                        )
                check_links = getattr(planner, "check_link_collision", None)
                if callable(check_links):
                    link_collision = check_links(with_start)
                    if not link_collision.safe:
                        raise SafetyError(
                            f"trajectory {link_collision.collision_type} collision "
                            f"between {link_collision.first_body} and "
                            f"{link_collision.second_body} near waypoint "
                            f"{link_collision.waypoint_index}"
                        )
            except CartesianPlanningError as exc:
                raise SafetyError(str(exc)) from exc
        waypoint_velocity = np.abs(np.diff(with_start, axis=0)) / intervals[:, None]
        waypoint_ratio = np.max(
            waypoint_velocity / joint_velocity_limits[None, :]
        )
        if waypoint_ratio > 1.0 + 1e-6:
            joint = int(np.argmax(np.max(
                waypoint_velocity / joint_velocity_limits[None, :], axis=0
            )))
            raise SafetyError(
                f"trajectory waypoint velocity for joint {joint + 1} exceeds "
                "its configured limit"
            )
        if specified_velocities:
            initial_velocity = (
                np.zeros_like(start)
                if start_velocity is None
                else np.asarray(start_velocity, dtype=np.float64)
            )
            if initial_velocity.shape != start.shape or not np.all(
                np.isfinite(initial_velocity)
            ):
                raise SafetyError("trajectory start velocity is invalid")
            try:
                dynamics = analyze_sampled_motion(
                    with_start,
                    times,
                    command_velocities=np.vstack(
                        [initial_velocity, velocities]
                    ),
                )
            except ValueError as exc:
                raise SafetyError(str(exc)) from exc
            acceleration_ratio = np.max(
                np.abs(dynamics.interval_accelerations)
                / self.safety.servo_max_acceleration_rad_s2
            )
            if acceleration_ratio > 1.0 + 1e-6:
                joint = int(np.argmax(np.max(
                    np.abs(dynamics.interval_accelerations), axis=0
                )))
                raise SafetyError(
                    f"trajectory acceleration for joint {joint + 1} exceeds "
                    "the configured limit"
                )
            jerk_ratio = (
                np.max(np.abs(dynamics.interval_jerks))
                / self.safety.servo_max_jerk_rad_s3
                if dynamics.interval_jerks.size
                else 0.0
            )
            if jerk_ratio > 1.0 + 1e-6:
                joint = int(np.argmax(np.max(
                    np.abs(dynamics.interval_jerks), axis=0
                )))
                raise SafetyError(
                    f"trajectory jerk for joint {joint + 1} exceeds the "
                    "configured limit"
                )

    def _validate_gripper_motion(
        self,
        start_width_m: float,
        targets_m: tuple[float, ...],
        deltas_m: tuple[float, ...],
        durations_s: tuple[float, ...],
    ) -> None:
        robot_config = self._require_robot_config()
        if len(deltas_m) != len(durations_s) or not deltas_m:
            raise SafetyError("gripper segments are inconsistent")
        values = (start_width_m, *targets_m, *deltas_m, *durations_s)
        if not all(math.isfinite(value) for value in values):
            raise SafetyError("gripper command must contain only finite values")
        for target in targets_m:
            if not 0.0 <= target <= robot_config.gripper_width:
                raise SafetyError(
                    "gripper target must be in [0, "
                    f"{1000.0 * robot_config.gripper_width:.1f}] mm"
                )
        minimum_duration_s = self.safety.minimum_motion_duration_s
        maximum_duration_s = self.EXTENDED_MOTION_MAX_DURATION_S
        for delta, duration in zip(deltas_m, durations_s):
            if not minimum_duration_s <= duration <= maximum_duration_s:
                raise SafetyError(
                    "each gripper trajectory duration must be in "
                    f"[{minimum_duration_s:g}, {maximum_duration_s:g}] s"
                )
            required = self.minimum_gripper_duration(delta)
            if duration + 1e-9 < required:
                raise SafetyError(
                    f"gripper duration {duration:.2f}s is too short; use at least "
                    f"{required:.2f}s for the configured velocity cap"
                )

    def _validate_force_close(
        self,
        closing_torque_nm: float,
        minimum_width_m: float,
        timeout_s: float,
        contact_hold_s: float,
    ) -> None:
        robot_config = self._require_robot_config()
        values = (
            closing_torque_nm,
            minimum_width_m,
            timeout_s,
            contact_hold_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise SafetyError("force-close parameters must be finite")
        torque_limit = min(
            self.safety.max_force_close_torque_nm,
            robot_config.gripper_torque_max
            * self.safety.gripper_torque_limit_ratio,
        )
        if not 0.02 <= closing_torque_nm <= torque_limit:
            raise SafetyError(
                f"closing torque must be in [0.02, {torque_limit:.2f}] N*m"
            )
        if not 0.0 <= minimum_width_m <= robot_config.gripper_width:
            raise SafetyError(
                "minimum gripper width must be in [0, "
                f"{1000.0 * robot_config.gripper_width:.1f}] mm"
            )
        if not 0.5 <= timeout_s <= 10.0:
            raise SafetyError("force-close timeout must be in [0.5, 10] s")
        if not 0.0 <= contact_hold_s <= 5.0:
            raise SafetyError("contact hold time must be in [0, 5] s")

    @staticmethod
    def _closing_motor_torque(
        closing_torque_nm: float, gripper_open_readout: float
    ) -> float:
        """Map positive closing magnitude to the physical motor direction."""
        return -math.copysign(float(closing_torque_nm), gripper_open_readout)

    def _check_motion_feedback(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        torque: np.ndarray,
        start: np.ndarray,
        excursion_target: np.ndarray,
        *,
        envelope_min: np.ndarray | None = None,
        envelope_max: np.ndarray | None = None,
    ) -> None:
        robot_config = self._require_robot_config()
        if not all(
            np.all(np.isfinite(values)) for values in (position, velocity, torque)
        ):
            raise SafetyError("feedback contains NaN or infinity")

        peak_velocity = float(np.max(np.abs(velocity)))
        if peak_velocity > self.safety.feedback_velocity_limit_rad_s:
            joint = int(np.argmax(np.abs(velocity))) + 1
            raise SafetyError(
                f"joint {joint} feedback velocity {peak_velocity:.3f} rad/s "
                "exceeded the limit"
            )

        ratios = np.abs(torque) / robot_config.joint_torque_max
        if np.any(ratios > self.safety.torque_limit_ratio):
            joint = int(np.argmax(ratios))
            raise SafetyError(
                f"joint {joint + 1} torque {torque[joint]:.3f} N*m "
                "approached its configured limit"
            )

        if np.any(position < robot_config.joint_pos_min) or np.any(
            position > robot_config.joint_pos_max
        ):
            raise SafetyError("measured joint position crossed a software limit")

        delta = excursion_target - start
        moving = np.abs(delta) > 1e-9
        lower = (
            np.minimum(start, excursion_target)
            if envelope_min is None
            else np.asarray(envelope_min, dtype=np.float64).copy()
        )
        upper = (
            np.maximum(start, excursion_target)
            if envelope_max is None
            else np.asarray(envelope_max, dtype=np.float64).copy()
        )
        if envelope_min is not None and envelope_max is not None:
            moving = np.abs(upper - lower) > 1e-9
        lower[moving] -= self.safety.target_overshoot_margin_rad
        upper[moving] += self.safety.target_overshoot_margin_rad
        lower[~moving] = start[~moving] - self.safety.non_target_drift_rad
        upper[~moving] = start[~moving] + self.safety.non_target_drift_rad
        if np.any(position < lower) or np.any(position > upper):
            joint = int(np.argmax((position < lower) | (position > upper)))
            raise SafetyError(f"joint {joint + 1} left the expected motion envelope")

    def _check_gripper_feedback(
        self,
        position: float,
        velocity: float,
        torque: float,
        envelope_min: float,
        envelope_max: float,
    ) -> None:
        if not all(math.isfinite(value) for value in (position, velocity, torque)):
            raise SafetyError("gripper feedback contains NaN or infinity")
        if abs(velocity) > self.safety.feedback_gripper_velocity_limit_m_s:
            raise SafetyError(
                f"gripper feedback velocity {velocity:.3f} m/s exceeded the limit"
            )

        robot_config = self._require_robot_config()
        torque_ratio = abs(torque) / robot_config.gripper_torque_max
        if torque_ratio > self.safety.gripper_torque_limit_ratio:
            raise SafetyError(
                f"gripper torque {torque:.3f} N*m approached its configured limit"
            )

        tolerance = self.safety.gripper_position_tolerance_m
        physical_min = -tolerance
        physical_max = robot_config.gripper_width + tolerance
        if not physical_min <= position <= physical_max:
            raise SafetyError("gripper feedback crossed its configured width limit")
        if not envelope_min - tolerance <= position <= envelope_max + tolerance:
            raise SafetyError("gripper left the expected motion envelope")

    def _ramp_gains(
        self,
        start_arm_scale: float,
        end_arm_scale: float,
        start_gripper_scale: float,
        end_gripper_scale: float,
        duration: float,
    ) -> None:
        with self._mode_lock:
            expected_mode = self._mode
        step_count = max(2, int(math.ceil(duration * self.GAIN_UPDATE_RATE_HZ)))
        start_time = time.monotonic()
        for step in range(1, step_count + 1):
            alpha = cosine_fraction(step / step_count)
            arm_scale = start_arm_scale + (
                end_arm_scale - start_arm_scale
            ) * alpha
            gripper_scale = start_gripper_scale + (
                end_gripper_scale - start_gripper_scale
            ) * alpha
            self._apply_gain_scales(
                arm_scale,
                gripper_scale,
                expected_mode=expected_mode,
            )
            deadline = start_time + duration * step / step_count
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

    def _apply_gain_scales(
        self,
        arm_scale: float,
        gripper_scale: float,
        *,
        expected_mode: ControllerMode,
    ) -> None:
        with self._mode_lock:
            self._require_mode(expected_mode)
            controller_config = self._require_controller_config()
            gain = arx5.Gain(
                controller_config.default_kp * arm_scale,
                controller_config.default_kd,
                controller_config.default_gripper_kp * gripper_scale,
                controller_config.default_gripper_kd,
            )
            self._require_sdk().set_gain(gain)
            self._current_arm_gain_scale = float(arm_scale)
            self._current_gripper_gain_scale = float(gripper_scale)

    def _set_joint_cmd_in_mode(
        self, command, expected_mode: ControllerMode
    ) -> None:
        with self._mode_lock:
            self._require_mode(expected_mode)
            self._require_sdk().set_joint_cmd(command)

    def _set_joint_traj_in_mode(
        self, waypoints, expected_mode: ControllerMode
    ) -> None:
        with self._mode_lock:
            self._require_mode(expected_mode)
            self._require_sdk().set_joint_traj(waypoints)

    def _require_mode(self, expected_mode: ControllerMode) -> None:
        if self._mode != expected_mode:
            raise ControlError(
                f"operation was interrupted; controller mode is {self._mode.value}"
            )

    def _read_snapshot(self) -> ArmSnapshot:
        sdk = self._require_sdk()
        state = sdk.get_joint_state()
        timestamp = float(state.timestamp)
        position = state.pos().copy()
        eef_pose_6d = self._tcp_pose_from_joint_position(position)
        velocity = state.vel().copy()
        acceleration = None
        previous = self._last_feedback_for_acceleration
        if previous is not None:
            previous_timestamp, previous_velocity = previous
            interval = timestamp - previous_timestamp
            if interval > 1e-6:
                estimate = (velocity - previous_velocity) / interval
                if np.all(np.isfinite(estimate)):
                    acceleration = np.clip(
                        estimate,
                        -self.safety.servo_max_acceleration_rad_s2,
                        self.safety.servo_max_acceleration_rad_s2,
                    )
        self._last_feedback_for_acceleration = (timestamp, velocity.copy())
        return ArmSnapshot(
            timestamp=timestamp,
            joint_position=position,
            joint_velocity=velocity,
            joint_torque=state.torque().copy(),
            gripper_position=float(state.gripper_pos),
            gripper_velocity=float(state.gripper_vel),
            gripper_torque=float(state.gripper_torque),
            eef_pose_6d=eef_pose_6d,
            joint_acceleration=acceleration,
        )

    def _remember_snapshot(self, snapshot: ArmSnapshot) -> None:
        with self._snapshot_lock:
            self._latest_snapshot = snapshot

    def _clear_cached_state(self) -> None:
        with self._snapshot_lock:
            self._latest_snapshot = None

    def _remember_monitored_feedback(self, state) -> ArmSnapshot:
        """Publish feedback already read by a motion monitor for UI consumers."""
        position = state.pos().copy()
        snapshot = ArmSnapshot(
            timestamp=float(state.timestamp),
            joint_position=position,
            joint_velocity=state.vel().copy(),
            joint_torque=state.torque().copy(),
            gripper_position=float(state.gripper_pos),
            gripper_velocity=float(state.gripper_vel),
            gripper_torque=float(state.gripper_torque),
            eef_pose_6d=self._tcp_pose_from_joint_position(position),
        )
        self._validate_snapshot(snapshot)
        self._remember_snapshot(snapshot)
        return snapshot

    def forward_kinematics(self, joint_position: np.ndarray) -> np.ndarray:
        """Return the checked base-to-flange pose for one joint endpoint."""
        position = np.asarray(joint_position, dtype=np.float64)
        if position.shape != (6,) or not np.all(np.isfinite(position)):
            raise ValueError("joint position must contain six finite values")
        return self._require_cartesian_planner().forward_kinematics_path(
            position[None, :]
        )[0].copy()

    def _tcp_pose_from_joint_position(self, position: np.ndarray) -> np.ndarray:
        """Resolve measured joints to TCP pose instead of using SDK command state."""
        return self.forward_kinematics(position)

    def _validate_snapshot(self, snapshot: ArmSnapshot) -> None:
        if not math.isfinite(snapshot.timestamp):
            raise SafetyError("invalid controller timestamp")
        arrays = (
            snapshot.joint_position,
            snapshot.joint_velocity,
            snapshot.joint_torque,
            snapshot.eef_pose_6d,
        )
        if not all(np.all(np.isfinite(values)) for values in arrays):
            raise SafetyError("state contains NaN or infinity")
        if snapshot.joint_acceleration is not None:
            if (
                snapshot.joint_acceleration.shape != snapshot.joint_position.shape
                or not np.all(np.isfinite(snapshot.joint_acceleration))
            ):
                raise SafetyError("state acceleration estimate is invalid")
        if not all(
            math.isfinite(value)
            for value in (
                snapshot.gripper_position,
                snapshot.gripper_velocity,
                snapshot.gripper_torque,
            )
        ):
            raise SafetyError("invalid gripper feedback")

    @staticmethod
    def _make_joint_state(
        position: np.ndarray,
        gripper_position: float,
        timestamp: float,
        *,
        joint_velocity: np.ndarray | None = None,
        gripper_velocity: float = 0.0,
        gripper_torque: float = 0.0,
    ) -> arx5.JointState:
        position = np.asarray(position, dtype=np.float64)
        zeros = np.zeros(position.size, dtype=np.float64)
        velocity = (
            zeros.copy()
            if joint_velocity is None
            else np.asarray(joint_velocity, dtype=np.float64).copy()
        )
        if velocity.shape != position.shape or not np.all(np.isfinite(velocity)):
            raise ValueError("joint velocity has an invalid shape or value")
        command = arx5.JointState(
            position.copy(),
            velocity,
            zeros.copy(),
            float(gripper_position),
            float(gripper_velocity),
            float(gripper_torque),
        )
        command.timestamp = float(timestamp)
        return command

    def _joint_index(self, joint: int) -> int:
        robot_config = self._require_robot_config()
        if not 1 <= joint <= robot_config.joint_dof:
            raise ValueError(f"joint must be in [1, {robot_config.joint_dof}]")
        return joint - 1

    def _require_active(self) -> None:
        if self._mode != ControllerMode.ACTIVE:
            raise ControlError(f"controller is not active (mode={self._mode.value})")

    def _require_sdk(self) -> arx5.Arx5JointController:
        if self._sdk is None:
            raise ControlError("controller is not connected")
        return self._sdk

    def _require_robot_config(self) -> arx5.RobotConfig:
        if self._robot_config is None:
            raise ControlError("robot config is not available before connect()")
        return self._robot_config

    def _require_controller_config(self) -> arx5.ControllerConfig:
        if self._controller_config is None:
            raise ControlError("controller config is not available before connect()")
        return self._controller_config

    def _require_cartesian_planner(self) -> CartesianPlanner:
        if self._cartesian_planner is None:
            raise ControlError("Cartesian planner is not available before connect()")
        return self._cartesian_planner
