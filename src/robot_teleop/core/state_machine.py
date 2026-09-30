"""Latched teleoperation safety state machine."""

from dataclasses import dataclass
from enum import Enum


class TeleopState(Enum):
    DISCONNECTED = "disconnected"
    READY = "ready"
    ACTIVE = "active"
    HOLD = "hold"
    DEGRADED = "degraded"
    ESTOP = "estop"
    FAULT = "fault"


class TeleopEvent(Enum):
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    ACTIVATE = "activate"
    RELEASE = "release"
    TRACKING_NORMAL = "tracking_normal"
    TRACKING_LIMITED = "tracking_limited"
    TRACKING_UNAVAILABLE = "tracking_unavailable"
    ESTOP = "estop"
    FAULT = "fault"
    RESET = "reset"


@dataclass(slots=True)
class TeleopStateMachine:
    _state: TeleopState = TeleopState.DISCONNECTED
    _connected: bool = False

    @property
    def state(self) -> TeleopState:
        return self._state

    @property
    def latched(self) -> bool:
        return self._state in (TeleopState.ESTOP, TeleopState.FAULT)

    def transition(self, event: TeleopEvent) -> TeleopState:
        event = TeleopEvent(event)
        if event is TeleopEvent.CONNECT:
            self._connected = True
            if not self.latched and self._state is TeleopState.DISCONNECTED:
                self._state = TeleopState.READY
            return self._state
        if event is TeleopEvent.DISCONNECT:
            self._connected = False
            if not self.latched:
                self._state = TeleopState.DISCONNECTED
            return self._state
        if event is TeleopEvent.ESTOP:
            self._state = TeleopState.ESTOP
            return self._state
        if event is TeleopEvent.FAULT:
            if self._state is not TeleopState.ESTOP:
                self._state = TeleopState.FAULT
            return self._state
        if event is TeleopEvent.RESET:
            if self.latched:
                self._state = (
                    TeleopState.READY if self._connected else TeleopState.DISCONNECTED
                )
            return self._state
        if self.latched:
            return self._state
        if not self._connected:
            self._state = TeleopState.DISCONNECTED
            return self._state
        if event is TeleopEvent.ACTIVATE:
            if self._state in (TeleopState.READY, TeleopState.HOLD):
                self._state = TeleopState.ACTIVE
        elif event is TeleopEvent.RELEASE:
            self._state = TeleopState.HOLD
        elif event is TeleopEvent.TRACKING_LIMITED:
            self._state = TeleopState.DEGRADED
        elif event is TeleopEvent.TRACKING_UNAVAILABLE or (
            event is TeleopEvent.TRACKING_NORMAL and self._state is TeleopState.DEGRADED
        ):
            self._state = TeleopState.HOLD
        return self._state
