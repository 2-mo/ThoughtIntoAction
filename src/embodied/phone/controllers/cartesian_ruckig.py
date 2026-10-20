"""Controller-independent Cartesian IK and online joint motion composition.

This module is intentionally a composition boundary, not a robot backend. It
accepts an offline kinematics port and explicit joint derivative limits, then
solves a new Cartesian target from the latest joint state and advances one
Ruckig sample per caller cycle. It has no transport, SDK, filesystem, or
thread lifecycle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from embodied.phone.core.models import Pose
from embodied.phone.kinematics.models import (
    IKResult,
    IKSolverConfig,
    JacobianDiagnostics,
    OfflineKinematics,
)

from embodied.phone.controllers.ruckig_servo import (
    JointServoState,
    JointServoStep,
    RuckigJointServo,
)


class CartesianServoError(RuntimeError):
    """Base error for a rejected Cartesian target or invalid servo step."""


class CartesianTargetRejected(CartesianServoError):
    """Raised when a target cannot be accepted without breaking continuity."""


@dataclass(frozen=True, slots=True)
class CartesianServoConfig:
    """Explicit limits for the Cartesian-to-joint composition.

    Derivative limits belong to ``JointServoLimits`` and must be supplied by
    the caller. These values only govern target continuity and optional
    singularity scaling; none are claimed to be physical robot limits.
    """

    ik: IKSolverConfig = field(default_factory=IKSolverConfig)
    maximum_joint_target_step_rad: float = 0.35
    orientation_scale_m: float = 0.10
    singularity_warn: float | None = None
    singularity_stop: float | None = None
    singularity_min_speed_scale: float = 0.25

    def __post_init__(self) -> None:
        values = (
            self.maximum_joint_target_step_rad,
            self.orientation_scale_m,
            self.singularity_min_speed_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values[:2]):
            raise ValueError("Cartesian servo limits must be positive and finite")
        if not 0.0 < self.singularity_min_speed_scale <= 1.0:
            raise ValueError("singularity_min_speed_scale must be in (0, 1]")
        if self.singularity_warn is not None and (
            not math.isfinite(self.singularity_warn) or self.singularity_warn <= 0.0
        ):
            raise ValueError("singularity_warn must be positive and finite")
        if self.singularity_stop is not None and (
            not math.isfinite(self.singularity_stop) or self.singularity_stop <= 0.0
        ):
            raise ValueError("singularity_stop must be positive and finite")
        if (self.singularity_warn is None) != (self.singularity_stop is None):
            raise ValueError(
                "singularity_warn and singularity_stop must be set together"
            )
        if (
            self.singularity_warn is not None
            and self.singularity_stop is not None
            and self.singularity_warn <= self.singularity_stop
        ):
            raise ValueError("singularity_warn must exceed singularity_stop")


@dataclass(frozen=True, slots=True)
class CartesianTargetDecision:
    """Diagnostics returned after accepting one latest Cartesian target."""

    target_pose: Pose
    joint_target: tuple[float, ...]
    ik: IKResult
    jacobian: JacobianDiagnostics
    derivative_scale: float


@dataclass(frozen=True, slots=True)
class CartesianServoStep:
    """One fixed-period joint step plus its model-space TCP pose."""

    joint: JointServoStep
    tcp_pose: Pose
    braking: bool


class CartesianRuckigServo:
    """Latest-target Cartesian servo backed by an injected kinematics port."""

    def __init__(
        self,
        *,
        kinematics: OfflineKinematics,
        joint_servo: RuckigJointServo,
        config: CartesianServoConfig | None = None,
    ) -> None:
        if joint_servo.limits.joint_count != kinematics.model.joint_limits.joint_count:
            raise ValueError("kinematics and joint servo dimensions do not match")
        self._kinematics = kinematics
        self._joint_servo = joint_servo
        self._config = config or CartesianServoConfig()
        self._target_pose = kinematics.forward_pose(joint_servo.state.position)
        self._last_decision: CartesianTargetDecision | None = None

    @property
    def config(self) -> CartesianServoConfig:
        return self._config

    @property
    def joint_state(self) -> JointServoState:
        return self._joint_servo.state

    @property
    def target_pose(self) -> Pose:
        return self._target_pose

    @property
    def last_decision(self) -> CartesianTargetDecision | None:
        return self._last_decision

    @property
    def braking(self) -> bool:
        return self._joint_servo.braking

    def set_target(self, target_pose: Pose) -> CartesianTargetDecision:
        """Solve and accept one latest Cartesian target.

        The current Ruckig state is the IK seed. A rejected target leaves the
        previous target and motion state untouched, allowing the caller to
        choose a hold or brake policy without executing a discontinuity.
        """

        current = self._joint_servo.state.position
        result = self._kinematics.solve_ik(
            target_pose,
            current,
            config=self._config.ik,
        )
        if not result.reachable:
            reason = result.reason or "offline IK did not reach target"
            raise CartesianTargetRejected(reason)
        joint_target = self._joint_servo.limits.validate_position(
            result.joint_position,
            name="IK joint target",
        )
        joint_step = math.sqrt(
            sum(
                (target - position) * (target - position)
                for target, position in zip(joint_target, current, strict=True)
            )
        )
        if joint_step > self._config.maximum_joint_target_step_rad:
            raise CartesianTargetRejected(
                "Cartesian target exceeds the configured joint branch step"
            )
        jacobian = self._kinematics.jacobian_diagnostics(
            joint_target,
            orientation_scale_m=self._config.orientation_scale_m,
        )
        if (
            self._config.singularity_stop is not None
            and jacobian.minimum_singular_value < self._config.singularity_stop
        ):
            raise CartesianTargetRejected(
                "Cartesian target is below the configured singularity threshold"
            )
        derivative_scale = self._derivative_scale(jacobian.minimum_singular_value)
        self._joint_servo.set_limit_scale(derivative_scale)
        self._joint_servo.set_target(joint_target)
        decision = CartesianTargetDecision(
            target_pose=target_pose,
            joint_target=joint_target,
            ik=result,
            jacobian=jacobian,
            derivative_scale=derivative_scale,
        )
        self._target_pose = target_pose
        self._last_decision = decision
        return decision

    def brake(self) -> None:
        """Request a smooth zero-velocity Ruckig stop from the current state."""

        self._joint_servo.brake()

    def step(self) -> CartesianServoStep:
        """Advance exactly one online motion sample and compute model TCP pose."""

        joint = self._joint_servo.step()
        return CartesianServoStep(
            joint=joint,
            tcp_pose=self._kinematics.forward_pose(joint.position),
            braking=self._joint_servo.braking,
        )

    def _derivative_scale(self, minimum_singular_value: float) -> float:
        warn = self._config.singularity_warn
        stop = self._config.singularity_stop
        if warn is None or stop is None or minimum_singular_value >= warn:
            return 1.0
        fraction = (minimum_singular_value - stop) / (warn - stop)
        return max(
            self._config.singularity_min_speed_scale,
            min(1.0, fraction),
        )


__all__ = [
    "CartesianRuckigServo",
    "CartesianServoConfig",
    "CartesianServoError",
    "CartesianServoStep",
    "CartesianTargetDecision",
    "CartesianTargetRejected",
]
