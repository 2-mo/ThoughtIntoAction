"""Deterministic latest-only input source for tests and dry runs."""

import threading

from robot_teleop.core.capabilities import InputCapabilities
from robot_teleop.core.models import TeleopInput
from robot_teleop.core.safety import InputSafetyLimits, InputSafetyValidator


class InputSourceStateError(RuntimeError):
    pass


class MockInputSource:
    def __init__(
        self,
        capabilities: InputCapabilities | None = None,
        *,
        safety_limits: InputSafetyLimits | None = None,
    ) -> None:
        self._capabilities = capabilities or InputCapabilities(
            pose_6dof=True,
            linear_velocity=False,
            angular_velocity=False,
            analog_gripper=True,
            haptics=False,
            dual_hand=False,
        )
        self._validator = InputSafetyValidator(
            safety_limits or InputSafetyLimits(),
            self._capabilities,
        )
        self._lock = threading.Lock()
        self._running = False
        self._latest: TeleopInput | None = None

    @property
    def capabilities(self) -> InputCapabilities:
        return self._capabilities

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._validator.reset()
            self._latest = None
            self._running = True

    def push(self, input_state: TeleopInput) -> None:
        if not isinstance(input_state, TeleopInput):
            raise TypeError("MockInputSource accepts only TeleopInput")
        with self._lock:
            if not self._running:
                raise InputSourceStateError("input source is not running")
            self._validator.validate(input_state)
            self._latest = input_state

    def latest(self) -> TeleopInput | None:
        with self._lock:
            return self._latest if self._running else None

    def stop(self) -> None:
        with self._lock:
            self._running = False
            self._latest = None
