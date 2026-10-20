"""Explicit lifecycle owner for the Phase 0 runtime."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from types import TracebackType
from typing import Self

from embodied.phone.core.models import TeleopInput
from embodied.phone.core.ports import RobotBackend
from embodied.phone.core.state_machine import TeleopState
from embodied.phone.protocol.packet import EStopResetRequestPacket

from embodied.phone.runtime.debug_dashboard import DebugDashboardServer, DebugTelemetry
from embodied.phone.runtime.estop_reset import EStopResetGate
from embodied.phone.runtime.latest_value import LatestValue
from embodied.phone.runtime.metrics import MetricsWorker
from embodied.phone.runtime.receiver import InputReceiver
from embodied.phone.runtime.servo_loop import ServoLoop


class TeleopApplication:
    def __init__(
        self,
        *,
        receiver: InputReceiver,
        servo: ServoLoop,
        metrics_worker: MetricsWorker,
        backend: RobotBackend,
        estop_reset_gate: EStopResetGate | None = None,
        reset_acknowledger: Callable[[EStopResetRequestPacket], None] | None = None,
        reset_backend: Callable[[], None] | None = None,
        debug_telemetry: DebugTelemetry | None = None,
        debug_server: DebugDashboardServer | None = None,
    ) -> None:
        self._receiver = receiver
        self._servo = servo
        self._metrics_worker = metrics_worker
        self._backend = backend
        self._estop_reset_gate = estop_reset_gate
        self._reset_acknowledger = reset_acknowledger
        self._reset_backend = reset_backend
        self._debug_telemetry = debug_telemetry
        self._debug_server = debug_server
        self._lock = threading.Lock()
        self._running = False

    @property
    def latest_input(self) -> LatestValue[TeleopInput]:
        return self._receiver.latest_value

    @property
    def servo_error(self) -> str | None:
        return self._servo.error

    @property
    def receiver_error(self) -> str | None:
        return self._receiver.error

    @property
    def state(self) -> TeleopState:
        return self._servo.state

    @property
    def estop_reset_ready(self) -> bool:
        gate = self._estop_reset_gate
        return gate is not None and gate.ready_request is not None

    @property
    def worker_count(self) -> int:
        return sum(
            (
                self._receiver.alive,
                self._servo.alive,
                self._metrics_worker.alive,
            )
        )

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._backend.connect()
            if self._debug_server is not None:
                self._debug_server.start()
            try:
                self._receiver.start()
                self._metrics_worker.start()
                self._servo.start()
            except Exception:
                self._receiver.stop()
                self._metrics_worker.stop()
                self._backend.disconnect()
                if self._debug_server is not None:
                    self._debug_server.stop()
                raise
            self._running = True

    def stop(self) -> None:
        with self._lock:
            if not self._running:
                self._backend.disconnect()
                return
            errors: list[Exception] = []
            for stop in (
                self._servo.stop,
                self._receiver.stop,
                self._metrics_worker.stop,
            ):
                try:
                    stop()
                except Exception as exc:
                    errors.append(exc)
            self._backend.disconnect()
            if self._debug_server is not None:
                self._debug_server.stop()
            self._running = False
            if errors:
                raise RuntimeError(
                    "; ".join(str(error) for error in errors)
                ) from errors[0]

    def wait(self, duration_s: float) -> None:
        if not math.isfinite(duration_s) or duration_s < 0.0:
            raise ValueError("duration_s must be non-negative and finite")
        threading.Event().wait(duration_s)

    def reset_estop(self) -> bool:
        """Complete a ready E-stop reset only on an explicit local call."""
        with self._lock:
            gate = self._estop_reset_gate
            acknowledger = self._reset_acknowledger
            if gate is None or acknowledger is None:
                return False
            completed = gate.completed_request
            if completed is not None:
                acknowledger(completed)
                return True
            if not self._running:
                return False
            request = gate.ready_request
            if request is None:
                return False
            # The phone can issue Recover while the backend is still healthy.
            # Treat that as a harmless acknowledgement so a stale local latch
            # never strands an otherwise live control session.
            if self._servo.state not in (TeleopState.ESTOP, TeleopState.FAULT):
                gate.complete(request)
                acknowledger(request)
                return True
            if self._servo.alive:
                return False
            reset_backend = self._reset_backend
            if reset_backend is None:
                return False
            reset_backend()
            if self._servo.state is TeleopState.FAULT:
                self._servo.reset_after_fault()
            else:
                self._servo.reset_after_estop()
            gate.complete(request)
            acknowledger(request)
            return True

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc_value, traceback
        self.stop()
