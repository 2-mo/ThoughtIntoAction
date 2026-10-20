"""Minimal input receiver worker publishing only the newest valid sample."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from dataclasses import replace

from embodied.phone.core.models import TeleopInput
from embodied.phone.core.ports import TeleopInputSource
from embodied.phone.core.safety import (
    InputSafetyLimits,
    InputSafetyValidator,
    SafetyViolation,
)

from embodied.phone.runtime.latest_value import LatestValue
from embodied.phone.runtime.metrics import RuntimeMetrics

type InputAcceptedCallback = Callable[[TeleopInput], None]


class InputReceiver:
    def __init__(
        self,
        source: TeleopInputSource,
        latest: LatestValue[TeleopInput],
        metrics: RuntimeMetrics,
        *,
        poll_interval_s: float = 0.001,
        safety_limits: InputSafetyLimits | None = None,
        on_input_accepted: InputAcceptedCallback | None = None,
    ) -> None:
        if not math.isfinite(poll_interval_s) or poll_interval_s < 0.0:
            raise ValueError("poll_interval_s must be non-negative and finite")
        self._source = source
        self._latest = latest
        self._metrics = metrics
        self._poll_interval_s = poll_interval_s
        self._validator = InputSafetyValidator(
            safety_limits or InputSafetyLimits(),
            source.capabilities,
        )
        self._on_input_accepted = on_input_accepted
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: str | None = None
        self._last_accepted_stored_at_ns: int | None = None

    @property
    def latest_value(self) -> LatestValue[TeleopInput]:
        return self._latest

    @property
    def alive(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def error(self) -> str | None:
        return self._error

    def start(self) -> None:
        if self.alive:
            return
        self._error = None
        self._validator.reset()
        self._latest.clear()
        self._last_accepted_stored_at_ns = None
        self._stop_event.clear()
        self._source.start()
        self._thread = threading.Thread(
            target=self._run,
            name="robot-teleop-receiver",
            daemon=False,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        errors: list[Exception] = []
        try:
            self._source.stop()
        except Exception as exc:
            errors.append(exc)
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout_s)
            if thread.is_alive():
                errors.append(RuntimeError("input receiver did not stop"))
        if thread is None or not thread.is_alive():
            self._thread = None
        if errors:
            raise RuntimeError("; ".join(str(error) for error in errors)) from errors[0]

    def _run(self) -> None:
        last_observed: tuple[int, int] | None = None
        while not self._stop_event.is_set():
            try:
                input_state = self._source.latest()
            except Exception as exc:
                self._error = f"input source failed: {exc}"
                self._metrics.record_fault(self._error)
                return
            if input_state is not None:
                observed = (input_state.session_id, input_state.sequence)
                if observed != last_observed:
                    last_observed = observed
                    try:
                        self._validator.validate(input_state)
                    except SafetyViolation as exc:
                        self._metrics.record_input_rejected(str(exc))
                        is_pose_jump = (
                            "input position jump" in str(exc)
                            or "input rotation jump" in str(exc)
                        )
                        if is_pose_jump:
                            # Keep the stream alive and park the policy at the
                            # next servo tick. The next packet establishes a
                            # fresh input baseline instead of forcing a process
                            # restart for one bad ARKit frame.
                            self._validator.reset()
                            parked = replace(
                                input_state,
                                clutch_active=False,
                                recenter_requested=False,
                                home_requested=False,
                            )
                            snapshot = self._latest.store(parked)
                            self._last_accepted_stored_at_ns = snapshot.stored_at_ns
                            callback = self._on_input_accepted
                            if callback is not None:
                                try:
                                    callback(parked)
                                except Exception as callback_error:
                                    self._error = (
                                        "input acceptance callback failed: "
                                        f"{callback_error}"
                                    )
                    else:
                        snapshot = self._latest.store(input_state)
                        if self._last_accepted_stored_at_ns is not None:
                            self._metrics.record_input_interval(
                                (
                                    snapshot.stored_at_ns
                                    - self._last_accepted_stored_at_ns
                                )
                                / 1_000_000.0
                            )
                        self._last_accepted_stored_at_ns = snapshot.stored_at_ns
                        self._metrics.record_input_accepted()
                        callback = self._on_input_accepted
                        if callback is not None:
                            try:
                                callback(input_state)
                            except Exception as exc:
                                self._error = f"input acceptance callback failed: {exc}"
                                self._metrics.record_fault(self._error)
                                return
            if self._poll_interval_s > 0.0:
                self._stop_event.wait(self._poll_interval_s)
