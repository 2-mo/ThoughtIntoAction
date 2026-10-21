"""Stateful jerk-limited online joint trajectory generation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import ruckig

from legacy.arx5_control.errors import SafetyError


@dataclass(frozen=True)
class JointServoCommand:
    """One fixed-period command produced by the online trajectory generator."""

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    finished: bool


class JointRuckigPlanner:
    """Small stateful wrapper around Ruckig's online joint planner."""

    def __init__(
        self,
        initial_position: np.ndarray,
        initial_velocity: np.ndarray,
        *,
        initial_acceleration: np.ndarray | None = None,
        period_s: float,
        max_velocity: float | np.ndarray,
        max_acceleration: float | np.ndarray,
        max_jerk: float | np.ndarray,
        min_position: np.ndarray,
        max_position: np.ndarray,
    ) -> None:
        position = self._vector(initial_position, "initial_position")
        velocity = self._vector(initial_velocity, "initial_velocity", position.size)
        acceleration = (
            np.zeros(position.size, dtype=np.float64)
            if initial_acceleration is None
            else self._vector(
                initial_acceleration, "initial_acceleration", position.size
            )
        )
        minimum = self._vector(min_position, "min_position", position.size)
        maximum = self._vector(max_position, "max_position", position.size)
        scalars = (period_s,)
        if not all(math.isfinite(value) and value > 0.0 for value in scalars):
            raise ValueError("servo period must be positive and finite")
        velocity_limit = self._limit_vector(max_velocity, "max_velocity", position.size)
        acceleration_limit = self._limit_vector(
            max_acceleration, "max_acceleration", position.size
        )
        jerk_limit = self._limit_vector(max_jerk, "max_jerk", position.size)
        acceleration = np.clip(
            acceleration, -acceleration_limit, acceleration_limit
        )
        if np.any(minimum >= maximum):
            raise ValueError("servo position limits are invalid")
        if np.any(position < minimum) or np.any(position > maximum):
            raise SafetyError("current joints are outside the servo position limits")
        if np.any(np.abs(velocity) > velocity_limit + 1e-9):
            raise SafetyError("current joint velocity is too high to start servoing")

        self._dof = position.size
        self._otg = ruckig.Ruckig(self._dof, float(period_s))
        self._input = ruckig.InputParameter(self._dof)
        self._output = ruckig.OutputParameter(self._dof)
        self._input.current_position = position.tolist()
        self._input.current_velocity = velocity.tolist()
        self._input.current_acceleration = acceleration.tolist()
        self._input.target_position = position.tolist()
        self._input.target_velocity = [0.0] * self._dof
        self._input.target_acceleration = [0.0] * self._dof
        self._input.max_velocity = velocity_limit.tolist()
        self._input.max_acceleration = acceleration_limit.tolist()
        self._input.max_jerk = jerk_limit.tolist()
        self._base_max_velocity = velocity_limit
        self._base_max_acceleration = acceleration_limit
        self._base_max_jerk = jerk_limit
        self._input.min_position = minimum.tolist()
        self._input.max_position = maximum.tolist()
        self._input.control_interface = ruckig.ControlInterface.Position
        self._braking = False

    @staticmethod
    def _vector(value: np.ndarray, name: str, size: int | None = None) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64).copy()
        if vector.ndim != 1 or (size is not None and vector.size != size):
            raise ValueError(f"{name} has an invalid shape")
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} must contain finite values")
        return vector

    @staticmethod
    def _limit_vector(
        value: float | np.ndarray,
        name: str,
        size: int,
    ) -> np.ndarray:
        limits = np.asarray(value, dtype=np.float64)
        if limits.ndim == 0:
            limits = np.full(size, float(limits), dtype=np.float64)
        elif limits.ndim == 1 and limits.size == size:
            limits = limits.copy()
        else:
            raise ValueError(f"{name} must be a scalar or a vector of length {size}")
        if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
            raise ValueError(f"{name} must contain positive finite values")
        return limits

    @property
    def position(self) -> np.ndarray:
        return np.asarray(self._input.current_position, dtype=np.float64).copy()

    @property
    def velocity(self) -> np.ndarray:
        return np.asarray(self._input.current_velocity, dtype=np.float64).copy()

    @property
    def acceleration(self) -> np.ndarray:
        return np.asarray(self._input.current_acceleration, dtype=np.float64).copy()

    @property
    def braking(self) -> bool:
        return self._braking

    def set_limit_scale(self, scale: float) -> None:
        """Scale online derivative limits for a singularity-safe target."""
        if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
            raise ValueError("online derivative limit scale must be in (0, 1]")
        self._input.max_velocity = (self._base_max_velocity * scale).tolist()
        self._input.max_acceleration = (
            self._base_max_acceleration * scale
        ).tolist()
        self._input.max_jerk = (self._base_max_jerk * scale).tolist()

    def set_brake_scale(self, scale: float) -> None:
        """Scale braking acceleration and jerk without lowering velocity bounds."""
        if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
            raise ValueError("online brake scale must be in (0, 1]")
        self._input.max_velocity = self._base_max_velocity.tolist()
        self._input.max_acceleration = (
            self._base_max_acceleration * scale
        ).tolist()
        self._input.max_jerk = (self._base_max_jerk * scale).tolist()

    def set_target(self, target_position: np.ndarray) -> None:
        target = self._vector(target_position, "target_position", self._dof)
        minimum = np.asarray(self._input.min_position, dtype=np.float64)
        maximum = np.asarray(self._input.max_position, dtype=np.float64)
        if np.any(target < minimum) or np.any(target > maximum):
            raise SafetyError("servo target approaches a software joint limit")
        self._input.control_interface = ruckig.ControlInterface.Position
        self._input.target_position = target.tolist()
        self._input.target_velocity = [0.0] * self._dof
        self._input.target_acceleration = [0.0] * self._dof
        self._braking = False

    def set_velocity_target(self, target_velocity: np.ndarray) -> None:
        target = self._vector(target_velocity, "target_velocity", self._dof)
        maximum = np.asarray(self._input.max_velocity, dtype=np.float64)
        if np.any(np.abs(target) > maximum + 1e-9):
            raise SafetyError("servo velocity target exceeds its configured limit")
        self._input.control_interface = ruckig.ControlInterface.Velocity
        self._input.target_velocity = target.tolist()
        self._input.target_acceleration = [0.0] * self._dof
        self._braking = False

    def brake(self) -> None:
        """Switch to velocity control so stopping never requires reversing."""
        self._input.control_interface = ruckig.ControlInterface.Velocity
        self._input.target_velocity = [0.0] * self._dof
        self._input.target_acceleration = [0.0] * self._dof
        self._braking = True

    def step(self) -> JointServoCommand:
        result = self._otg.update(self._input, self._output)
        if result not in (ruckig.Result.Working, ruckig.Result.Finished):
            raise SafetyError(f"Ruckig servo planning failed: {result}")
        command = JointServoCommand(
            position=np.asarray(self._output.new_position, dtype=np.float64).copy(),
            velocity=np.asarray(self._output.new_velocity, dtype=np.float64).copy(),
            acceleration=np.asarray(
                self._output.new_acceleration, dtype=np.float64
            ).copy(),
            jerk=np.asarray(self._output.new_jerk, dtype=np.float64).copy(),
            finished=result == ruckig.Result.Finished,
        )
        arrays = (
            command.position,
            command.velocity,
            command.acceleration,
            command.jerk,
        )
        if not all(np.all(np.isfinite(values)) for values in arrays):
            raise SafetyError("Ruckig produced a non-finite servo command")
        self._output.pass_to_input(self._input)
        return command


__all__ = ["JointRuckigPlanner", "JointServoCommand"]
