"""In-memory robot with a backend-local safety envelope."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from types import TracebackType
from typing import Self

from embodied.phone.core.capabilities import RobotCapabilities
from embodied.phone.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    RobotCommand,
    RobotState,
    Vector3,
)


class RobotConnectionError(RuntimeError):
    pass


class RobotSafetyError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class MockRobotConfig:
    joint_count: int = 6
    control_rate_hz: float = 200.0
    maximum_cartesian_step_m: float = 0.10
    initial_pose: Pose = field(
        default_factory=lambda: Pose(
            Vector3(0.4, 0.0, 0.3),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        )
    )
    workspace_minimum_m: Vector3 = field(
        default_factory=lambda: Vector3(0.0, -0.5, 0.0)
    )
    workspace_maximum_m: Vector3 = field(default_factory=lambda: Vector3(0.8, 0.5, 0.8))

    def __post_init__(self) -> None:
        if isinstance(self.joint_count, bool) or self.joint_count <= 0:
            raise ValueError("joint_count must be positive")
        if not math.isfinite(self.control_rate_hz) or self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive and finite")
        if (
            not math.isfinite(self.maximum_cartesian_step_m)
            or self.maximum_cartesian_step_m <= 0.0
        ):
            raise ValueError("maximum_cartesian_step_m must be positive and finite")


class MockRobotBackend:
    def __init__(self, config: MockRobotConfig | None = None) -> None:
        self._config = config or MockRobotConfig()
        self._capabilities = RobotCapabilities(
            cartesian_position=True,
            cartesian_velocity=False,
            joint_position=False,
            torque_control=False,
            gripper=True,
            external_wrench=False,
            control_rate_hz=self._config.control_rate_hz,
            joint_count=self._config.joint_count,
        )
        zero_joints = (0.0,) * self._config.joint_count
        self._state = RobotState(
            timestamp_ns=0,
            joint_position=zero_joints,
            joint_velocity=zero_joints,
            joint_torque=zero_joints,
            tcp_pose=self._config.initial_pose,
            tcp_twist=None,
            gripper=0.5,
            external_wrench=None,
        )
        self._lock = threading.RLock()
        self._connected = False
        self._safe_stopped = False
        self._feedback_frozen = False
        self._commands: deque[RobotCommand] = deque(maxlen=256)
        self._hold_count = 0
        self._stop_count = 0
        self._safe_stop_count = 0

    @property
    def capabilities(self) -> RobotCapabilities:
        return self._capabilities

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def safe_stopped(self) -> bool:
        with self._lock:
            return self._safe_stopped

    @property
    def hold_count(self) -> int:
        with self._lock:
            return self._hold_count

    @property
    def stop_count(self) -> int:
        with self._lock:
            return self._stop_count

    @property
    def safe_stop_count(self) -> int:
        with self._lock:
            return self._safe_stop_count

    @property
    def command_history(self) -> tuple[RobotCommand, ...]:
        with self._lock:
            return tuple(self._commands)

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._connected = True
            self._safe_stopped = False
            self._state = replace(self._state, timestamp_ns=time.monotonic_ns())

    def state(self) -> RobotState:
        with self._lock:
            self._require_connected()
            return self._state

    def send(self, command: RobotCommand) -> RobotState:
        with self._lock:
            self._require_connected()
            if self._safe_stopped:
                raise RobotSafetyError("robot is safe-stopped")
            try:
                if command.mode is ControlMode.CARTESIAN_POSITION:
                    assert command.target_pose is not None
                    self._validate_pose(command.target_pose)
                    self._state = replace(
                        self._state,
                        timestamp_ns=self._next_timestamp(),
                        tcp_pose=command.target_pose,
                        joint_velocity=(0.0,) * self._config.joint_count,
                        gripper=(
                            self._state.gripper
                            if command.gripper is None
                            else command.gripper
                        ),
                    )
                elif command.mode is ControlMode.HOLD:
                    self._hold_count += 1
                    self._state = self._stationary_state()
                elif command.mode is ControlMode.STOP:
                    self._stop_count += 1
                    self._state = self._stationary_state()
                else:
                    raise RobotSafetyError(
                        f"mock backend does not support {command.mode.value}"
                    )
            except RobotSafetyError:
                self._safe_stop_locked()
                raise
            self._commands.append(command)
            return self._state

    def hold(self, reason: str) -> None:
        if not reason.strip():
            raise ValueError("hold reason must not be blank")
        with self._lock:
            self._require_connected()
            if self._safe_stopped:
                raise RobotSafetyError("robot is safe-stopped")
            self._hold_count += 1
            self._state = self._stationary_state()

    def safe_stop(self, reason: str) -> None:
        if not reason.strip():
            raise ValueError("safe-stop reason must not be blank")
        with self._lock:
            self._require_connected()
            self._safe_stop_locked()

    def rearm_after_safe_stop(self) -> None:
        """Explicit Mock-only recovery that remains stationary."""
        with self._lock:
            self._require_connected()
            if not self._safe_stopped:
                return
            self._safe_stopped = False
            self._state = self._stationary_state()

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False

    def set_feedback_frozen(self, frozen: bool) -> None:
        with self._lock:
            self._feedback_frozen = bool(frozen)

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc_value, traceback
        self.disconnect()

    def _require_connected(self) -> None:
        if not self._connected:
            raise RobotConnectionError("robot is not connected")

    def _next_timestamp(self) -> int:
        if self._feedback_frozen:
            return self._state.timestamp_ns
        return max(time.monotonic_ns(), self._state.timestamp_ns + 1)

    def _stationary_state(self) -> RobotState:
        return replace(
            self._state,
            timestamp_ns=self._next_timestamp(),
            joint_velocity=(0.0,) * self._config.joint_count,
            tcp_twist=None,
        )

    def _safe_stop_locked(self) -> None:
        if self._safe_stopped:
            return
        self._safe_stopped = True
        self._safe_stop_count += 1
        self._state = self._stationary_state()

    def _validate_pose(self, target: Pose) -> None:
        step = (target.position - self._state.tcp_pose.position).norm()
        if step > self._config.maximum_cartesian_step_m:
            raise RobotSafetyError("Cartesian command step exceeds backend limit")
        if any(
            value < lower or value > upper
            for value, lower, upper in zip(
                target.position.as_tuple(),
                self._config.workspace_minimum_m.as_tuple(),
                self._config.workspace_maximum_m.as_tuple(),
                strict=True,
            )
        ):
            raise RobotSafetyError("Cartesian command is outside backend workspace")
