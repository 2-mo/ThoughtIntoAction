"""Immutable canonical teleoperation data models."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Self


def _require_finite(name: str, *values: float) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")


def _finite_tuple(name: str, values: tuple[float, ...]) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    _require_finite(name, *result)
    return result


def _optional_unit_interval(name: str, value: float | None) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError(f"{name} must be finite and in [0, 1]")
    return result


@dataclass(frozen=True, slots=True)
class Vector3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        values = (float(self.x), float(self.y), float(self.z))
        _require_finite("Vector3", *values)
        object.__setattr__(self, "x", values[0])
        object.__setattr__(self, "y", values[1])
        object.__setattr__(self, "z", values[2])

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __add__(self, other: Self) -> Self:
        return type(self)(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Self) -> Self:
        return type(self)(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scale: float) -> Self:
        _require_finite("Vector3 scale", scale)
        return type(self)(self.x * scale, self.y * scale, self.z * scale)

    def __neg__(self) -> Self:
        return type(self)(-self.x, -self.y, -self.z)

    def norm(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


@dataclass(frozen=True, slots=True)
class Quaternion:
    """Unit quaternion in the canonical x, y, z, w field order."""

    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        values = (float(self.x), float(self.y), float(self.z), float(self.w))
        _require_finite("Quaternion", *values)
        norm = math.sqrt(sum(value * value for value in values))
        if norm <= 1e-12:
            raise ValueError("Quaternion norm must be positive")
        object.__setattr__(self, "x", values[0] / norm)
        object.__setattr__(self, "y", values[1] / norm)
        object.__setattr__(self, "z", values[2] / norm)
        object.__setattr__(self, "w", values[3] / norm)

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.w)


@dataclass(frozen=True, slots=True)
class Pose:
    position: Vector3
    orientation: Quaternion


@dataclass(frozen=True, slots=True)
class Twist:
    linear: Vector3
    angular: Vector3


@dataclass(frozen=True, slots=True)
class Wrench:
    force: Vector3
    torque: Vector3


class TrackingQuality(Enum):
    UNAVAILABLE = "unavailable"
    LIMITED = "limited"
    NORMAL = "normal"


class ControlMode(Enum):
    CARTESIAN_POSITION = "cartesian_position"
    CARTESIAN_VELOCITY = "cartesian_velocity"
    JOINT_POSITION = "joint_position"
    HOLD = "hold"
    STOP = "stop"
    HOME = "home"


@dataclass(frozen=True, slots=True)
class TeleopInput:
    sequence: int
    session_id: int
    timestamp_ns: int
    pose: Pose
    linear_velocity: Vector3 | None
    angular_velocity: Vector3 | None
    clutch_active: bool
    recenter_requested: bool
    estop_requested: bool
    gripper: float | None
    tracking_quality: TrackingQuality
    source_name: str
    home_requested: bool = False
    joint_zero_requested: bool = False

    def __post_init__(self) -> None:
        for name, value in (
            ("sequence", self.sequence),
            ("session_id", self.session_id),
        ):
            if isinstance(value, bool) or not 0 <= value <= 0xFFFFFFFF:
                raise ValueError(f"{name} must be a UInt32 value")
        if isinstance(self.timestamp_ns, bool) or self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        source_name = self.source_name.strip()
        if not source_name:
            raise ValueError("source_name must not be empty")
        object.__setattr__(self, "source_name", source_name)
        object.__setattr__(
            self,
            "tracking_quality",
            TrackingQuality(self.tracking_quality),
        )
        object.__setattr__(
            self,
            "gripper",
            _optional_unit_interval("gripper", self.gripper),
        )


@dataclass(frozen=True, slots=True)
class RobotState:
    timestamp_ns: int
    joint_position: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    joint_torque: tuple[float, ...] | None
    tcp_pose: Pose
    tcp_twist: Twist | None
    gripper: float | None
    external_wrench: Wrench | None

    def __post_init__(self) -> None:
        if isinstance(self.timestamp_ns, bool) or self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        position = _finite_tuple("joint_position", self.joint_position)
        velocity = _finite_tuple("joint_velocity", self.joint_velocity)
        if not position:
            raise ValueError("joint_position must not be empty")
        if len(velocity) != len(position):
            raise ValueError("joint_velocity length must match joint_position")
        torque = self.joint_torque
        if torque is not None:
            torque = _finite_tuple("joint_torque", torque)
            if len(torque) != len(position):
                raise ValueError("joint_torque length must match joint_position")
        object.__setattr__(self, "joint_position", position)
        object.__setattr__(self, "joint_velocity", velocity)
        object.__setattr__(self, "joint_torque", torque)
        object.__setattr__(
            self,
            "gripper",
            _optional_unit_interval("gripper", self.gripper),
        )


@dataclass(frozen=True, slots=True)
class RobotTarget:
    mode: ControlMode
    target_pose: Pose | None
    target_twist: Twist | None
    target_joint_position: tuple[float, ...] | None
    gripper: float | None
    velocity_scale: float
    timestamp_ns: int

    def __post_init__(self) -> None:
        mode = ControlMode(self.mode)
        object.__setattr__(self, "mode", mode)
        if mode is ControlMode.CARTESIAN_POSITION and self.target_pose is None:
            raise ValueError("target_pose is required for Cartesian position mode")
        if mode is ControlMode.CARTESIAN_VELOCITY and self.target_twist is None:
            raise ValueError("target_twist is required for Cartesian velocity mode")
        if mode is ControlMode.JOINT_POSITION and self.target_joint_position is None:
            raise ValueError(
                "target_joint_position is required for joint position mode"
            )
        if mode in (ControlMode.HOLD, ControlMode.STOP, ControlMode.HOME) and any(
            value is not None
            for value in (
                self.target_pose,
                self.target_twist,
                self.target_joint_position,
            )
        ):
            raise ValueError(
                "HOLD, STOP, and HOME targets cannot contain an active target"
            )
        joints = self.target_joint_position
        if joints is not None:
            joints = _finite_tuple("target_joint_position", joints)
        velocity_scale = float(self.velocity_scale)
        if not math.isfinite(velocity_scale) or not 0.0 <= velocity_scale <= 1.0:
            raise ValueError("velocity_scale must be finite and in [0, 1]")
        if isinstance(self.timestamp_ns, bool) or self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        object.__setattr__(self, "target_joint_position", joints)
        object.__setattr__(self, "velocity_scale", velocity_scale)
        object.__setattr__(
            self,
            "gripper",
            _optional_unit_interval("gripper", self.gripper),
        )


@dataclass(frozen=True, slots=True)
class RobotCommand:
    mode: ControlMode
    target_pose: Pose | None
    target_twist: Twist | None
    target_joint_position: tuple[float, ...] | None
    gripper: float | None
    velocity_scale: float
    timestamp_ns: int
    reason: str | None = None

    def __post_init__(self) -> None:
        target = RobotTarget(
            mode=self.mode,
            target_pose=self.target_pose,
            target_twist=self.target_twist,
            target_joint_position=self.target_joint_position,
            gripper=self.gripper,
            velocity_scale=self.velocity_scale,
            timestamp_ns=self.timestamp_ns,
        )
        object.__setattr__(self, "mode", target.mode)
        object.__setattr__(self, "target_joint_position", target.target_joint_position)
        object.__setattr__(self, "gripper", target.gripper)
        object.__setattr__(self, "velocity_scale", target.velocity_scale)
        if self.reason is not None and not self.reason.strip():
            raise ValueError("reason must not be blank")


def hold_target(timestamp_ns: int, *, gripper: float | None = None) -> RobotTarget:
    return RobotTarget(
        mode=ControlMode.HOLD,
        target_pose=None,
        target_twist=None,
        target_joint_position=None,
        gripper=gripper,
        velocity_scale=0.0,
        timestamp_ns=timestamp_ns,
    )


def stop_target(timestamp_ns: int) -> RobotTarget:
    return RobotTarget(
        mode=ControlMode.STOP,
        target_pose=None,
        target_twist=None,
        target_joint_position=None,
        gripper=None,
        velocity_scale=0.0,
        timestamp_ns=timestamp_ns,
    )


def home_target(
    timestamp_ns: int, *, gripper: float | None = None
) -> RobotTarget:
    return RobotTarget(
        mode=ControlMode.HOME,
        target_pose=None,
        target_twist=None,
        target_joint_position=None,
        gripper=gripper,
        velocity_scale=0.0,
        timestamp_ns=timestamp_ns,
    )


def joint_zero_target(
    timestamp_ns: int,
    joint_count: int,
    *,
    gripper: float | None = None,
) -> RobotTarget:
    if isinstance(joint_count, bool) or joint_count <= 0:
        raise ValueError("joint_count must be positive")
    return RobotTarget(
        mode=ControlMode.JOINT_POSITION,
        target_pose=None,
        target_twist=None,
        target_joint_position=(0.0,) * joint_count,
        gripper=gripper,
        velocity_scale=0.25,
        timestamp_ns=timestamp_ns,
    )
