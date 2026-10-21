"""Robot-local ARX5 safety limits and boundary validation.

The values accepted by this module are deliberately supplied by the
composition root.  The package does not invent physical limits and the
default ARX5 backend does not construct an envelope.  This keeps static model
evidence separate from limits that an operator has actually verified on one
arm.
"""

from __future__ import annotations

import math
import re
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any, cast

from embodied.phone.core.models import Pose, Twist, Vector3


class Arx5SafetyViolation(ValueError):
    """Raised when feedback or a target exceeds a supplied ARX5 envelope."""


_INTERFACE_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_.-]{0,31}$")


def validate_interface_name(value: str) -> str:
    """Normalize a CAN interface name without allowing path-like input."""

    interface = value.strip()
    if not interface or _INTERFACE_PATTERN.fullmatch(interface) is None:
        raise Arx5SafetyViolation(
            "ARX5 robot interface must be a simple CAN name (for example can0)"
        )
    return interface


def _finite_vector(
    name: str,
    values: Iterable[object],
    *,
    expected_length: int | None = None,
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{name} must be a numeric sequence")
    try:
        result = tuple(float(cast(Any, value)) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a numeric sequence") from exc
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    if expected_length is not None and len(result) != expected_length:
        raise ValueError(f"{name} must contain {expected_length} values")
    return result


def _optional_limit_vector(
    name: str,
    values: Iterable[object] | None,
    *,
    expected_length: int,
) -> tuple[float, ...] | None:
    if values is None:
        return None
    result = _finite_vector(name, values, expected_length=expected_length)
    if any(value <= 0.0 for value in result):
        raise ValueError(f"{name} must contain positive values")
    return result


def _validate_absolute_vector(
    name: str,
    values: Iterable[object],
    limits: tuple[float, ...],
) -> tuple[float, ...]:
    result = _finite_vector(name, values, expected_length=len(limits))
    for index, (value, limit) in enumerate(zip(result, limits, strict=True)):
        if abs(value) > limit + 1e-9:
            raise Arx5SafetyViolation(
                f"{name}[{index}] exceeds the configured ARX5 limit"
            )
    return result


@dataclass(frozen=True, slots=True)
class Arx5SafetyEnvelope:
    """One explicit robot-local envelope used at the backend boundary.

    ``max_acceleration_rad_s2`` and ``max_jerk_rad_s3`` are required because a
    trajectory generator cannot be considered bounded without them.  Torque,
    Cartesian, and singularity values may remain ``None`` while evidence is
    being collected; in that case the corresponding validator still checks
    finiteness but does not claim a physical bound.
    """

    position_lower_rad: tuple[float, ...]
    position_upper_rad: tuple[float, ...]
    max_velocity_rad_s: tuple[float, ...]
    max_acceleration_rad_s2: tuple[float, ...]
    max_jerk_rad_s3: tuple[float, ...]
    max_torque_nm: tuple[float, ...] | None = None
    workspace_minimum_m: Vector3 | None = None
    workspace_maximum_m: Vector3 | None = None
    tcp_linear_velocity_max_m_s: float | None = None
    tcp_angular_velocity_max_rad_s: float | None = None
    singularity_warn: float | None = None
    singularity_stop: float | None = None
    verified: bool = False
    evidence_source: str = "unverified"

    def __post_init__(self) -> None:
        lower = _finite_vector("position_lower_rad", self.position_lower_rad)
        upper = _finite_vector("position_upper_rad", self.position_upper_rad)
        velocity = _optional_limit_vector(
            "max_velocity_rad_s",
            self.max_velocity_rad_s,
            expected_length=len(lower),
        )
        acceleration = _optional_limit_vector(
            "max_acceleration_rad_s2",
            self.max_acceleration_rad_s2,
            expected_length=len(lower),
        )
        jerk = _optional_limit_vector(
            "max_jerk_rad_s3",
            self.max_jerk_rad_s3,
            expected_length=len(lower),
        )
        if len(upper) != len(lower):
            raise ValueError("position limit vectors must have the same length")
        if velocity is None or acceleration is None or jerk is None:
            raise ValueError("joint derivative limits are required")
        if any(
            minimum >= maximum for minimum, maximum in zip(lower, upper, strict=True)
        ):
            raise ValueError("each lower joint position limit must be below its upper")
        torque = _optional_limit_vector(
            "max_torque_nm",
            self.max_torque_nm,
            expected_length=len(lower),
        )
        if (self.workspace_minimum_m is None) != (self.workspace_maximum_m is None):
            raise ValueError(
                "workspace minimum and maximum must be configured together"
            )
        if (
            self.workspace_minimum_m is not None
            and self.workspace_maximum_m is not None
            and any(
                minimum >= maximum
                for minimum, maximum in zip(
                    self.workspace_minimum_m.as_tuple(),
                    self.workspace_maximum_m.as_tuple(),
                    strict=True,
                )
            )
        ):
            raise ValueError("workspace minimum must be below maximum")
        for name, value in (
            ("tcp_linear_velocity_max_m_s", self.tcp_linear_velocity_max_m_s),
            ("tcp_angular_velocity_max_rad_s", self.tcp_angular_velocity_max_rad_s),
        ):
            if value is not None and (not math.isfinite(value) or value <= 0.0):
                raise ValueError(f"{name} must be positive and finite")
        warn = self.singularity_warn
        stop = self.singularity_stop
        if (warn is None) != (stop is None):
            raise ValueError("singularity_warn and singularity_stop must be paired")
        if warn is not None and (not math.isfinite(warn) or warn <= 0.0):
            raise ValueError("singularity_warn must be positive and finite")
        if stop is not None and (not math.isfinite(stop) or stop <= 0.0):
            raise ValueError("singularity_stop must be positive and finite")
        if warn is not None and stop is not None and warn <= stop:
            raise ValueError("singularity_warn must exceed singularity_stop")
        if not isinstance(self.verified, bool):
            raise ValueError("verified must be boolean")
        source = self.evidence_source.strip()
        if not source:
            raise ValueError("evidence_source must not be blank")
        object.__setattr__(self, "position_lower_rad", lower)
        object.__setattr__(self, "position_upper_rad", upper)
        object.__setattr__(self, "max_velocity_rad_s", velocity)
        object.__setattr__(self, "max_acceleration_rad_s2", acceleration)
        object.__setattr__(self, "max_jerk_rad_s3", jerk)
        object.__setattr__(self, "max_torque_nm", torque)
        object.__setattr__(self, "evidence_source", source)

    @property
    def joint_count(self) -> int:
        return len(self.position_lower_rad)

    def validate_feedback(
        self,
        *,
        joint_position: Iterable[object],
        joint_velocity: Iterable[object],
        joint_torque: Iterable[object] | None,
    ) -> None:
        """Validate one finite measured state against the envelope."""

        self.validate_joint_position(joint_position, name="joint position")
        _validate_absolute_vector(
            "joint velocity", joint_velocity, self.max_velocity_rad_s
        )
        if joint_torque is not None:
            torque = _finite_vector(
                "joint torque", joint_torque, expected_length=self.joint_count
            )
            if self.max_torque_nm is not None:
                _validate_absolute_vector("joint torque", torque, self.max_torque_nm)

    def validate_joint_position(
        self,
        values: Iterable[object],
        *,
        name: str = "joint position",
    ) -> tuple[float, ...]:
        """Validate one joint-position vector independently of a trajectory."""

        position = _finite_vector(name, values, expected_length=self.joint_count)
        for index, (value, lower, upper) in enumerate(
            zip(
                position,
                self.position_lower_rad,
                self.position_upper_rad,
                strict=True,
            )
        ):
            if value < lower - 1e-9 or value > upper + 1e-9:
                raise Arx5SafetyViolation(
                    f"{name}[{index}] is outside the configured ARX5 limit"
                )
        return position

    def validate_joint_command(
        self,
        *,
        position: Iterable[object],
        velocity: Iterable[object],
        acceleration: Iterable[object],
        jerk: Iterable[object],
    ) -> None:
        """Validate a complete joint trajectory sample before SDK submission."""

        self.validate_joint_position(position, name="joint command position")
        _validate_absolute_vector(
            "joint command velocity", velocity, self.max_velocity_rad_s
        )
        _validate_absolute_vector(
            "joint command acceleration", acceleration, self.max_acceleration_rad_s2
        )
        _validate_absolute_vector("joint command jerk", jerk, self.max_jerk_rad_s3)

    def validate_pose(self, pose: Pose) -> None:
        """Validate the Cartesian workspace portion of one target/feedback pose."""

        if not isinstance(pose, Pose):
            raise Arx5SafetyViolation("ARX5 TCP pose must use the canonical Pose type")
        minimum = self.workspace_minimum_m
        maximum = self.workspace_maximum_m
        if minimum is None or maximum is None:
            return
        if any(
            value < lower - 1e-9 or value > upper + 1e-9
            for value, lower, upper in zip(
                pose.position.as_tuple(),
                minimum.as_tuple(),
                maximum.as_tuple(),
                strict=True,
            )
        ):
            raise Arx5SafetyViolation(
                "ARX5 TCP pose is outside the configured workspace"
            )

    def validate_twist(self, twist: Twist) -> None:
        """Validate optional Cartesian velocity limits."""

        if not isinstance(twist, Twist):
            raise Arx5SafetyViolation(
                "ARX5 TCP twist must use the canonical Twist type"
            )
        linear_limit = self.tcp_linear_velocity_max_m_s
        if linear_limit is not None and twist.linear.norm() > linear_limit + 1e-9:
            raise Arx5SafetyViolation("ARX5 TCP linear velocity exceeds its limit")
        angular_limit = self.tcp_angular_velocity_max_rad_s
        if angular_limit is not None and twist.angular.norm() > angular_limit + 1e-9:
            raise Arx5SafetyViolation("ARX5 TCP angular velocity exceeds its limit")

    def validate_singularity(self, minimum_singular_value: float) -> None:
        """Reject a Jacobian diagnostic below the configured stop threshold."""

        try:
            value = float(cast(Any, minimum_singular_value))
        except (TypeError, ValueError) as exc:
            raise Arx5SafetyViolation("minimum singular value is not numeric") from exc
        if not math.isfinite(value) or value < 0.0:
            raise Arx5SafetyViolation("minimum singular value must be finite")
        stop = self.singularity_stop
        if stop is not None and value < stop:
            raise Arx5SafetyViolation(
                "ARX5 target is below the configured singularity stop threshold"
            )

    def as_report(self) -> dict[str, object]:
        """Return JSON-ready evidence without implying physical verification."""

        return {
            "joint_count": self.joint_count,
            "position_lower_rad": self.position_lower_rad,
            "position_upper_rad": self.position_upper_rad,
            "max_velocity_rad_s": self.max_velocity_rad_s,
            "max_acceleration_rad_s2": self.max_acceleration_rad_s2,
            "max_jerk_rad_s3": self.max_jerk_rad_s3,
            "max_torque_nm": self.max_torque_nm,
            "workspace_minimum_m": (
                None
                if self.workspace_minimum_m is None
                else self.workspace_minimum_m.as_tuple()
            ),
            "workspace_maximum_m": (
                None
                if self.workspace_maximum_m is None
                else self.workspace_maximum_m.as_tuple()
            ),
            "tcp_linear_velocity_max_m_s": self.tcp_linear_velocity_max_m_s,
            "tcp_angular_velocity_max_rad_s": self.tcp_angular_velocity_max_rad_s,
            "singularity_warn": self.singularity_warn,
            "singularity_stop": self.singularity_stop,
            "verified": self.verified,
            "evidence_source": self.evidence_source,
        }


__all__ = [
    "Arx5SafetyEnvelope",
    "Arx5SafetyViolation",
    "validate_interface_name",
]
