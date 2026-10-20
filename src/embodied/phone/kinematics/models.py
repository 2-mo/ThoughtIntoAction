"""Typed offline kinematics contracts and immutable model metadata."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from embodied.phone.core.models import Pose


def _finite_tuple(name: str, values: tuple[float, ...]) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must be a non-empty finite tuple")
    return result


@dataclass(frozen=True, slots=True)
class JointLimits:
    """Offline joint envelope from an explicitly cited model configuration."""

    lower_rad: tuple[float, ...]
    upper_rad: tuple[float, ...]
    velocity_max_rad_s: tuple[float, ...]

    def __post_init__(self) -> None:
        lower = _finite_tuple("lower_rad", self.lower_rad)
        upper = _finite_tuple("upper_rad", self.upper_rad)
        velocity = _finite_tuple("velocity_max_rad_s", self.velocity_max_rad_s)
        if not (len(lower) == len(upper) == len(velocity)):
            raise ValueError("joint limit vectors must have the same length")
        if any(
            minimum >= maximum for minimum, maximum in zip(lower, upper, strict=True)
        ):
            raise ValueError("each lower joint limit must be below its upper limit")
        if any(value <= 0.0 for value in velocity):
            raise ValueError("joint velocity limits must be positive")
        object.__setattr__(self, "lower_rad", lower)
        object.__setattr__(self, "upper_rad", upper)
        object.__setattr__(self, "velocity_max_rad_s", velocity)

    @property
    def joint_count(self) -> int:
        return len(self.lower_rad)

    def validate_position(self, values: tuple[float, ...]) -> tuple[float, ...]:
        position = _finite_tuple("joint_position", values)
        if len(position) != self.joint_count:
            raise ValueError("joint_position length does not match the robot model")
        if any(
            value < lower or value > upper
            for value, lower, upper in zip(
                position,
                self.lower_rad,
                self.upper_rad,
                strict=True,
            )
        ):
            raise ValueError("joint_position is outside offline model limits")
        return position


@dataclass(frozen=True, slots=True)
class RobotModel:
    """A robot description usable by an offline kinematics adapter only."""

    name: str
    urdf_path: Path
    base_frame: str
    end_effector_frame: str
    joint_names: tuple[str, ...]
    joint_limits: JointLimits
    source_repository: str
    source_commit: str
    source_license: str

    def __post_init__(self) -> None:
        name = self.name.strip()
        base_frame = self.base_frame.strip()
        end_effector_frame = self.end_effector_frame.strip()
        joint_names = tuple(name.strip() for name in self.joint_names)
        if not name or not base_frame or not end_effector_frame:
            raise ValueError("robot model names and frames must not be blank")
        if not joint_names or any(not joint_name for joint_name in joint_names):
            raise ValueError("robot model joint names must not be blank")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("robot model joint names must be unique")
        if len(joint_names) != self.joint_limits.joint_count:
            raise ValueError("joint names and joint limits must have the same length")
        if not self.source_repository.startswith("https://"):
            raise ValueError("source_repository must be an HTTPS URL")
        if not self.source_commit.strip() or not self.source_license.strip():
            raise ValueError("robot model source metadata must not be blank")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "urdf_path", Path(self.urdf_path))
        object.__setattr__(self, "base_frame", base_frame)
        object.__setattr__(self, "end_effector_frame", end_effector_frame)
        object.__setattr__(self, "joint_names", joint_names)


@dataclass(frozen=True, slots=True)
class IKSolverConfig:
    """Bounded parameters for an offline, seed-continuous IK check."""

    max_iterations: int = 100
    position_tolerance_m: float = 0.001
    orientation_tolerance_rad: float = math.radians(1.0)
    damping: float = 1e-4
    maximum_joint_step_rad: float = 0.15

    def __post_init__(self) -> None:
        if isinstance(self.max_iterations, bool) or self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        values = (
            self.position_tolerance_m,
            self.orientation_tolerance_rad,
            self.damping,
            self.maximum_joint_step_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("IK solver parameters must be positive and finite")


@dataclass(frozen=True, slots=True)
class IKResult:
    """One bounded offline IK attempt, including residuals for review."""

    reachable: bool
    joint_position: tuple[float, ...]
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    reason: str | None

    def __post_init__(self) -> None:
        position = _finite_tuple("joint_position", self.joint_position)
        if isinstance(self.iterations, bool) or self.iterations < 0:
            raise ValueError("iterations must be non-negative")
        errors = (self.position_error_m, self.orientation_error_rad)
        if not all(math.isfinite(value) and value >= 0.0 for value in errors):
            raise ValueError("IK residuals must be non-negative and finite")
        reason = None if self.reason is None else self.reason.strip()
        if reason == "":
            reason = None
        object.__setattr__(self, "joint_position", position)
        object.__setattr__(self, "reason", reason)


@dataclass(frozen=True, slots=True)
class JacobianDiagnostics:
    """Dimensionless singularity observability for a configured frame."""

    singular_values: tuple[float, ...]
    minimum_singular_value: float
    condition_number: float
    manipulability: float
    orientation_scale_m: float

    def __post_init__(self) -> None:
        singular_values = _finite_tuple("singular_values", self.singular_values)
        finite_values = (
            self.minimum_singular_value,
            self.manipulability,
            self.orientation_scale_m,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in finite_values):
            raise ValueError("Jacobian diagnostics must be non-negative and finite")
        if math.isnan(self.condition_number) or self.condition_number < 0.0:
            raise ValueError("condition_number must be non-negative")
        if self.orientation_scale_m <= 0.0:
            raise ValueError("orientation_scale_m must be positive")
        object.__setattr__(self, "singular_values", singular_values)


class KinematicsUnavailableError(RuntimeError):
    """Raised when an optional offline kinematics dependency is unavailable."""


class OfflineKinematics(Protocol):
    """Port used by offline feasibility tools, never by canonical core."""

    @property
    def model(self) -> RobotModel: ...

    @property
    def backend_name(self) -> str: ...

    def forward_pose(self, joint_position: tuple[float, ...]) -> Pose: ...

    def solve_ik(
        self,
        target_pose: Pose,
        seed_joint_position: tuple[float, ...],
        *,
        config: IKSolverConfig | None = None,
    ) -> IKResult: ...

    def jacobian_diagnostics(
        self,
        joint_position: tuple[float, ...],
        *,
        orientation_scale_m: float = 0.10,
    ) -> JacobianDiagnostics: ...
