"""Robot-independent input and target safety checks."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .capabilities import InputCapabilities
from .frames import quaternion_angular_distance, quaternion_slerp
from .models import (
    ControlMode,
    Pose,
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    Vector3,
)


class SafetyViolation(ValueError):
    """Canonical safety validation rejected an input or target."""


@dataclass(frozen=True, slots=True)
class InputSafetyLimits:
    maximum_position_jump_m: float = 0.25
    maximum_rotation_jump_rad: float = math.radians(90.0)

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.maximum_position_jump_m)
            or self.maximum_position_jump_m <= 0.0
        ):
            raise ValueError("maximum_position_jump_m must be positive and finite")
        if (
            not math.isfinite(self.maximum_rotation_jump_rad)
            or self.maximum_rotation_jump_rad <= 0.0
        ):
            raise ValueError("maximum_rotation_jump_rad must be positive and finite")


class InputSafetyValidator:
    def __init__(
        self,
        limits: InputSafetyLimits,
        capabilities: InputCapabilities,
    ) -> None:
        self._limits = limits
        self._capabilities = capabilities
        self._last: TeleopInput | None = None

    def reset(self) -> None:
        self._last = None

    def validate(self, input_state: TeleopInput) -> None:
        if (
            input_state.linear_velocity is not None
            and not self._capabilities.linear_velocity
        ):
            raise SafetyViolation("source supplied linear velocity without capability")
        if (
            input_state.angular_velocity is not None
            and not self._capabilities.angular_velocity
        ):
            raise SafetyViolation("source supplied angular velocity without capability")
        if input_state.gripper is not None and not self._capabilities.analog_gripper:
            raise SafetyViolation("source supplied gripper without capability")

        # ARKit can change its world solution while tracking is limited. Do
        # not compare that re-localized pose with the last normal frame; the
        # session layer will hold and require a fresh clutch edge on recovery.
        if input_state.tracking_quality is not TrackingQuality.NORMAL:
            self._last = None
            return

        previous = self._last
        if previous is not None and previous.session_id == input_state.session_id:
            sequence_delta = (input_state.sequence - previous.sequence) & 0xFFFFFFFF
            if sequence_delta == 0 or sequence_delta >= 1 << 31:
                raise SafetyViolation("input sequence is duplicate or out of order")
            if input_state.timestamp_ns <= previous.timestamp_ns:
                raise SafetyViolation("input timestamp is not monotonic")
            position_jump = (input_state.pose.position - previous.pose.position).norm()
            if position_jump > self._limits.maximum_position_jump_m:
                raise SafetyViolation("input position jump exceeds the limit")
            rotation_jump = quaternion_angular_distance(
                previous.pose.orientation,
                input_state.pose.orientation,
            )
            if rotation_jump > self._limits.maximum_rotation_jump_rad:
                raise SafetyViolation("input rotation jump exceeds the limit")
        self._last = input_state


@dataclass(frozen=True, slots=True)
class TargetSafetyLimits:
    maximum_target_step_m: float = 0.05
    maximum_rotation_step_rad: float = math.radians(20.0)
    maximum_linear_velocity_m_s: float = 0.5
    maximum_angular_velocity_rad_s: float = 1.5
    workspace_minimum_m: Vector3 | None = None
    workspace_maximum_m: Vector3 | None = None

    def __post_init__(self) -> None:
        values = (
            self.maximum_target_step_m,
            self.maximum_rotation_step_rad,
            self.maximum_linear_velocity_m_s,
            self.maximum_angular_velocity_rad_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("target safety limits must be positive and finite")
        if (self.workspace_minimum_m is None) != (self.workspace_maximum_m is None):
            raise ValueError(
                "workspace minimum and maximum must be configured together"
            )
        if (
            self.workspace_minimum_m is not None
            and self.workspace_maximum_m is not None
        ):
            minimum = self.workspace_minimum_m
            maximum = self.workspace_maximum_m
            if any(
                lower >= upper
                for lower, upper in zip(
                    minimum.as_tuple(), maximum.as_tuple(), strict=True
                )
            ):
                raise ValueError("workspace minimum must be below maximum")


class TargetSafetyGate:
    def __init__(self, limits: TargetSafetyLimits) -> None:
        self._limits = limits
        self._last_pose: Pose | None = None

    def reset(self, reference_pose: Pose) -> None:
        self._last_pose = reference_pose

    def validate(
        self,
        target: RobotTarget,
        robot_state: RobotState,
        dt: float,
        *,
        is_new_input: bool = True,
    ) -> RobotTarget:
        if not math.isfinite(dt) or dt <= 0.0:
            raise SafetyViolation("control dt must be positive and finite")
        if target.mode is ControlMode.CARTESIAN_POSITION:
            assert target.target_pose is not None
            previous = self._last_pose or robot_state.tcp_pose
            self._validate_workspace(target.target_pose.position)
            if not is_new_input and self._last_pose is not None:
                return replace(target, target_pose=previous)

            position_delta = target.target_pose.position - previous.position
            position_distance = position_delta.norm()
            position_fraction = self._limited_fraction(
                position_distance,
                step_limit=self._limits.maximum_target_step_m,
                velocity_limit=self._limits.maximum_linear_velocity_m_s,
                dt=dt,
            )
            orientation_distance = quaternion_angular_distance(
                previous.orientation,
                target.target_pose.orientation,
            )
            orientation_fraction = self._limited_fraction(
                orientation_distance,
                step_limit=self._limits.maximum_rotation_step_rad,
                velocity_limit=self._limits.maximum_angular_velocity_rad_s,
                dt=dt,
            )
            limited_pose = Pose(
                previous.position + position_delta * position_fraction,
                quaternion_slerp(
                    previous.orientation,
                    target.target_pose.orientation,
                    orientation_fraction,
                ),
            )
            self._last_pose = limited_pose
            return replace(target, target_pose=limited_pose)
        elif target.mode is ControlMode.CARTESIAN_VELOCITY:
            assert target.target_twist is not None
            if (
                target.target_twist.linear.norm()
                > self._limits.maximum_linear_velocity_m_s
            ):
                raise SafetyViolation("Cartesian target velocity exceeds the limit")
            if (
                target.target_twist.angular.norm()
                > self._limits.maximum_angular_velocity_rad_s
            ):
                raise SafetyViolation("Cartesian angular velocity exceeds the limit")
        return target

    @staticmethod
    def _limited_fraction(
        distance: float,
        *,
        step_limit: float,
        velocity_limit: float,
        dt: float,
    ) -> float:
        if distance <= 0.0:
            return 1.0
        return min(1.0, step_limit / distance, velocity_limit * dt / distance)

    def _validate_workspace(self, position: Vector3) -> None:
        minimum = self._limits.workspace_minimum_m
        maximum = self._limits.workspace_maximum_m
        if minimum is None or maximum is None:
            return
        if any(
            value < lower or value > upper
            for value, lower, upper in zip(
                position.as_tuple(),
                minimum.as_tuple(),
                maximum.as_tuple(),
                strict=True,
            )
        ):
            raise SafetyViolation("Cartesian target is outside the workspace")
