"""Dependency-inverted ports consumed by the canonical runtime."""

from __future__ import annotations

from typing import Protocol

from robot_teleop.core.capabilities import InputCapabilities, RobotCapabilities
from robot_teleop.core.models import RobotCommand, RobotState, RobotTarget, TeleopInput


class Transport(Protocol):
    def start(self) -> None: ...

    def receive(self) -> bytes | None: ...

    def send(self, payload: bytes) -> None: ...

    def stop(self) -> None: ...


class TeleopInputSource(Protocol):
    @property
    def capabilities(self) -> InputCapabilities: ...

    def start(self) -> None: ...

    def latest(self) -> TeleopInput | None: ...

    def stop(self) -> None: ...


class ControlPolicy(Protocol):
    def reset(self, input_state: TeleopInput, robot_state: RobotState) -> None: ...

    def update(
        self,
        input_state: TeleopInput,
        robot_state: RobotState,
        dt: float,
    ) -> RobotTarget: ...


class CompliancePolicy(Protocol):
    def modify(
        self,
        target: RobotTarget,
        state: RobotState,
        dt: float,
    ) -> RobotTarget: ...


class TargetFilter(Protocol):
    def reset(self, state: RobotState) -> None: ...

    def apply(
        self,
        target: RobotTarget,
        state: RobotState,
        dt: float,
    ) -> RobotTarget: ...


class Controller(Protocol):
    def reset(self, state: RobotState) -> None: ...

    def update(
        self,
        target: RobotTarget,
        state: RobotState,
        dt: float,
    ) -> RobotCommand: ...


class RobotBackend(Protocol):
    @property
    def capabilities(self) -> RobotCapabilities: ...

    def connect(self) -> None: ...

    def state(self) -> RobotState: ...

    def send(self, command: RobotCommand) -> RobotState: ...

    def hold(self, reason: str) -> None: ...

    def safe_stop(self, reason: str) -> None: ...

    def disconnect(self) -> None: ...
