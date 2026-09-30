"""Optional online joint motion shaping backed by upstream Ruckig.

The class deliberately owns only a fixed-period joint trajectory state. It has
no robot SDK, transport, filesystem, or thread lifecycle and can therefore be
used by more than one robot backend after each backend supplies its verified
joint envelope.
"""

from __future__ import annotations

import importlib
import math
from dataclasses import dataclass
from typing import Any


class RuckigUnavailableError(RuntimeError):
    """Raised when the optional Ruckig package is not installed."""


class RuckigServoError(RuntimeError):
    """Raised when Ruckig cannot produce a valid bounded step."""


def _finite_vector(name: str, values: tuple[float, ...]) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must be a non-empty finite tuple")
    return result


@dataclass(frozen=True, slots=True)
class JointServoLimits:
    """Verified joint envelope passed to one Ruckig instance."""

    minimum_position: tuple[float, ...]
    maximum_position: tuple[float, ...]
    maximum_velocity: tuple[float, ...]
    maximum_acceleration: tuple[float, ...]
    maximum_jerk: tuple[float, ...]

    def __post_init__(self) -> None:
        minimum = _finite_vector("minimum_position", self.minimum_position)
        maximum = _finite_vector("maximum_position", self.maximum_position)
        velocity = _finite_vector("maximum_velocity", self.maximum_velocity)
        acceleration = _finite_vector("maximum_acceleration", self.maximum_acceleration)
        jerk = _finite_vector("maximum_jerk", self.maximum_jerk)
        lengths = {
            len(minimum),
            len(maximum),
            len(velocity),
            len(acceleration),
            len(jerk),
        }
        if len(lengths) != 1:
            raise ValueError("all joint limit vectors must have the same length")
        if any(lower >= upper for lower, upper in zip(minimum, maximum, strict=True)):
            raise ValueError("each minimum position must be below its maximum")
        if any(value <= 0.0 for value in (*velocity, *acceleration, *jerk)):
            raise ValueError("Ruckig derivative limits must be positive")
        object.__setattr__(self, "minimum_position", minimum)
        object.__setattr__(self, "maximum_position", maximum)
        object.__setattr__(self, "maximum_velocity", velocity)
        object.__setattr__(self, "maximum_acceleration", acceleration)
        object.__setattr__(self, "maximum_jerk", jerk)

    @property
    def joint_count(self) -> int:
        return len(self.minimum_position)

    def validate_position(
        self,
        values: tuple[float, ...],
        *,
        name: str,
    ) -> tuple[float, ...]:
        position = _finite_vector(name, values)
        if len(position) != self.joint_count:
            raise ValueError(f"{name} length does not match joint limits")
        if any(
            value < lower or value > upper
            for value, lower, upper in zip(
                position,
                self.minimum_position,
                self.maximum_position,
                strict=True,
            )
        ):
            raise ValueError(f"{name} is outside the configured position limits")
        return position

    def scaled(self, scale: float) -> JointServoLimits:
        if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
            raise ValueError("Ruckig limit scale must be in (0, 1]")
        return JointServoLimits(
            minimum_position=self.minimum_position,
            maximum_position=self.maximum_position,
            maximum_velocity=tuple(value * scale for value in self.maximum_velocity),
            maximum_acceleration=tuple(
                value * scale for value in self.maximum_acceleration
            ),
            maximum_jerk=tuple(value * scale for value in self.maximum_jerk),
        )


@dataclass(frozen=True, slots=True)
class JointServoState:
    """Current position, velocity, and acceleration of the online generator."""

    position: tuple[float, ...]
    velocity: tuple[float, ...]
    acceleration: tuple[float, ...]

    def __post_init__(self) -> None:
        position = _finite_vector("position", self.position)
        velocity = _finite_vector("velocity", self.velocity)
        acceleration = _finite_vector("acceleration", self.acceleration)
        if not len(position) == len(velocity) == len(acceleration):
            raise ValueError("joint servo state vectors must have the same length")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "velocity", velocity)
        object.__setattr__(self, "acceleration", acceleration)


@dataclass(frozen=True, slots=True)
class JointServoStep:
    """One output sample from the fixed-period Ruckig update."""

    position: tuple[float, ...]
    velocity: tuple[float, ...]
    acceleration: tuple[float, ...]
    jerk: tuple[float, ...]
    finished: bool

    def __post_init__(self) -> None:
        vectors = (
            _finite_vector("position", self.position),
            _finite_vector("velocity", self.velocity),
            _finite_vector("acceleration", self.acceleration),
            _finite_vector("jerk", self.jerk),
        )
        if len({len(value) for value in vectors}) != 1:
            raise ValueError("Ruckig output vectors must have the same length")
        for name, value in zip(
            ("position", "velocity", "acceleration", "jerk"), vectors, strict=True
        ):
            object.__setattr__(self, name, value)


class RuckigJointServo:
    """Stateful online Ruckig wrapper with latest-target semantics.

    Calling ``set_target`` replaces the current target. No queue is kept, and
    ``step`` advances Ruckig's own position/velocity/acceleration state exactly
    once per caller cycle.
    """

    def __init__(
        self,
        initial_state: JointServoState,
        limits: JointServoLimits,
        *,
        period_s: float,
        ruckig_module: Any | None = None,
    ) -> None:
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("period_s must be positive and finite")
        if len(initial_state.position) != limits.joint_count:
            raise ValueError("initial state does not match joint limits")
        limits.validate_position(initial_state.position, name="initial position")
        if any(
            abs(value) > limit + 1e-9
            for value, limit in zip(
                initial_state.velocity,
                limits.maximum_velocity,
                strict=True,
            )
        ):
            raise ValueError("initial velocity exceeds the configured limit")
        if any(
            abs(value) > limit + 1e-9
            for value, limit in zip(
                initial_state.acceleration,
                limits.maximum_acceleration,
                strict=True,
            )
        ):
            raise ValueError("initial acceleration exceeds the configured limit")
        self._ruckig = _load_ruckig() if ruckig_module is None else ruckig_module
        self._dof = limits.joint_count
        self._period_s = float(period_s)
        self._base_limits = limits
        self._otg = self._ruckig.Ruckig(self._dof, self._period_s)
        self._input = self._ruckig.InputParameter(self._dof)
        self._output = self._ruckig.OutputParameter(self._dof)
        self._input.current_position = list(initial_state.position)
        self._input.current_velocity = list(initial_state.velocity)
        self._input.current_acceleration = list(initial_state.acceleration)
        self._input.target_position = list(initial_state.position)
        self._input.target_velocity = [0.0] * self._dof
        self._input.target_acceleration = [0.0] * self._dof
        self._apply_limits(limits)
        self._input.control_interface = self._ruckig.ControlInterface.Position
        self._braking = False

    @property
    def state(self) -> JointServoState:
        return JointServoState(
            position=tuple(float(value) for value in self._input.current_position),
            velocity=tuple(float(value) for value in self._input.current_velocity),
            acceleration=tuple(
                float(value) for value in self._input.current_acceleration
            ),
        )

    @property
    def limits(self) -> JointServoLimits:
        return self._base_limits

    @property
    def braking(self) -> bool:
        return self._braking

    def set_target(self, position: tuple[float, ...]) -> None:
        target = self._base_limits.validate_position(position, name="target position")
        self._input.control_interface = self._ruckig.ControlInterface.Position
        self._input.target_position = list(target)
        self._input.target_velocity = [0.0] * self._dof
        self._input.target_acceleration = [0.0] * self._dof
        self._braking = False

    def set_velocity_target(self, velocity: tuple[float, ...]) -> None:
        target = _finite_vector("target velocity", velocity)
        if len(target) != self._dof:
            raise ValueError("target velocity length does not match joint limits")
        if any(
            abs(value) > limit + 1e-9
            for value, limit in zip(
                target,
                self._base_limits.maximum_velocity,
                strict=True,
            )
        ):
            raise ValueError("target velocity exceeds the configured limit")
        self._input.control_interface = self._ruckig.ControlInterface.Velocity
        self._input.target_velocity = list(target)
        self._input.target_acceleration = [0.0] * self._dof
        self._braking = False

    def set_limit_scale(self, scale: float) -> None:
        """Scale derivative limits for a singularity-aware backend decision."""
        self._apply_limits(self._base_limits.scaled(scale))

    def brake(self) -> None:
        self._input.control_interface = self._ruckig.ControlInterface.Velocity
        self._input.target_velocity = [0.0] * self._dof
        self._input.target_acceleration = [0.0] * self._dof
        self._braking = True

    def step(self) -> JointServoStep:
        result = self._otg.update(self._input, self._output)
        if result not in (self._ruckig.Result.Working, self._ruckig.Result.Finished):
            raise RuckigServoError(f"Ruckig update failed: {result}")
        step = JointServoStep(
            position=tuple(float(value) for value in self._output.new_position),
            velocity=tuple(float(value) for value in self._output.new_velocity),
            acceleration=tuple(float(value) for value in self._output.new_acceleration),
            jerk=tuple(float(value) for value in self._output.new_jerk),
            finished=result == self._ruckig.Result.Finished,
        )
        self._output.pass_to_input(self._input)
        self._assert_step_within_limits(step)
        return step

    def _apply_limits(self, limits: JointServoLimits) -> None:
        self._input.max_velocity = list(limits.maximum_velocity)
        self._input.max_acceleration = list(limits.maximum_acceleration)
        self._input.max_jerk = list(limits.maximum_jerk)
        self._input.min_position = list(limits.minimum_position)
        self._input.max_position = list(limits.maximum_position)

    def _assert_step_within_limits(self, step: JointServoStep) -> None:
        limits = self._base_limits
        if any(
            position < lower - 1e-9 or position > upper + 1e-9
            for position, lower, upper in zip(
                step.position,
                limits.minimum_position,
                limits.maximum_position,
                strict=True,
            )
        ):
            raise RuckigServoError("Ruckig produced a position outside its limits")
        if any(
            abs(velocity) > limit + 1e-7
            for velocity, limit in zip(
                step.velocity,
                limits.maximum_velocity,
                strict=True,
            )
        ):
            raise RuckigServoError("Ruckig produced a velocity outside its limits")
        if any(
            abs(acceleration) > limit + 1e-6
            for acceleration, limit in zip(
                step.acceleration,
                limits.maximum_acceleration,
                strict=True,
            )
        ):
            raise RuckigServoError("Ruckig produced an acceleration outside its limits")
        if any(
            abs(jerk) > limit + 1e-5
            for jerk, limit in zip(
                step.jerk,
                limits.maximum_jerk,
                strict=True,
            )
        ):
            raise RuckigServoError("Ruckig produced a jerk outside its limits")


def _load_ruckig() -> Any:
    try:
        return importlib.import_module("ruckig")
    except ImportError as exc:
        raise RuckigUnavailableError(
            "Ruckig is optional; run uv sync --extra arx5"
        ) from exc


__all__ = [
    "JointServoLimits",
    "JointServoState",
    "JointServoStep",
    "RuckigJointServo",
    "RuckigServoError",
    "RuckigUnavailableError",
]
