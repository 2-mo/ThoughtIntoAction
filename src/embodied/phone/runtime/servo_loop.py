"""Fixed-rate latest-only dry-run servo worker."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from embodied.phone.core.models import TeleopInput, TrackingQuality, stop_target
from embodied.phone.core.pipeline import TeleopPipeline
from embodied.phone.core.ports import Controller, RobotBackend
from embodied.phone.core.state_machine import TeleopState

from embodied.phone.runtime.debug_dashboard import DebugTelemetry
from embodied.phone.runtime.latest_value import LatestValue
from embodied.phone.runtime.metrics import CycleTiming, RuntimeMetrics
from embodied.phone.runtime.watchdog import Watchdog, WatchdogStage


@dataclass(frozen=True, slots=True)
class ServoLoopConfig:
    rate_hz: float = 200.0
    state_stale_timeout_ms: float = 100.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("servo rate must be positive and finite")
        if (
            not math.isfinite(self.state_stale_timeout_ms)
            or self.state_stale_timeout_ms <= 0.0
        ):
            raise ValueError("state stale timeout must be positive and finite")


class ServoLoop:
    def __init__(
        self,
        *,
        latest: LatestValue[TeleopInput],
        pipeline: TeleopPipeline,
        controller: Controller,
        backend: RobotBackend,
        watchdog: Watchdog,
        metrics: RuntimeMetrics,
        config: ServoLoopConfig,
        telemetry: DebugTelemetry | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self._latest = latest
        self._pipeline = pipeline
        self._controller = controller
        self._backend = backend
        self._watchdog = watchdog
        self._metrics = metrics
        self._config = config
        self._telemetry = telemetry
        self._clock = clock
        self._period_ns = max(1, round(1_000_000_000.0 / config.rate_hz))
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: str | None = None

    @property
    def alive(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def error(self) -> str | None:
        return self._error

    @property
    def state(self) -> TeleopState:
        return self._pipeline.state

    def start(self) -> None:
        if self.alive:
            return
        if self._pipeline.state in (TeleopState.ESTOP, TeleopState.FAULT):
            raise RuntimeError("latched pipeline must be reset before servo restart")
        initial_state = self._backend.state()
        self._controller.reset(initial_state)
        now_ns = self._clock()
        self._watchdog.reset(now_ns)
        self._error = None
        if self._telemetry is not None:
            self._telemetry.record_errors(servo=None, receiver=None)
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(initial_state.timestamp_ns, now_ns),
            name="robot-teleop-servo",
            daemon=False,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout_s)
            if thread.is_alive():
                raise RuntimeError("servo loop did not stop")
        self._pipeline.disconnect()
        self._thread = None

    def reset_after_estop(self) -> None:
        """Restart after an explicit caller re-arms its backend boundary."""
        self._reset_after_latch(TeleopState.ESTOP)

    def reset_after_fault(self) -> None:
        """Restart after the backend has been reconnected from a latched fault."""
        self._reset_after_latch(TeleopState.FAULT)

    def _reset_after_latch(self, expected: TeleopState) -> None:
        if self.alive:
            raise RuntimeError("servo loop must stop before an E-stop reset")
        if self._pipeline.state is not expected:
            raise RuntimeError(f"servo loop is not latched in {expected.value}")
        self._pipeline.reset_latches()
        self.start()

    def _run(self, initial_feedback_ns: int, started_at_ns: int) -> None:
        deadline_ns = started_at_ns
        previous_start_ns: int | None = None
        last_feedback_ns = initial_feedback_ns
        last_feedback_progress_ns = started_at_ns
        last_input_generation: int | None = None
        last_input_stored_at_ns: int | None = None
        watchdog_armed = False
        previous_watchdog_stage = WatchdogStage.FRESH
        safe_stop_invoked = False
        try:
            while not self._stop_event.is_set():
                before_wait_ns = self._clock()
                if before_wait_ns < deadline_ns and self._stop_event.wait(
                    (deadline_ns - before_wait_ns) / 1_000_000_000.0
                ):
                    break
                actual_start_ns = self._clock()
                dt = (
                    self._period_ns / 1_000_000_000.0
                    if previous_start_ns is None
                    else max(
                        1e-9,
                        (actual_start_ns - previous_start_ns) / 1_000_000_000.0,
                    )
                )
                previous_start_ns = actual_start_ns
                deadline_miss = actual_start_ns > deadline_ns + self._period_ns

                snapshot = self._latest.load()
                robot_state = self._backend.state()
                if robot_state.timestamp_ns > last_feedback_ns:
                    last_feedback_ns = robot_state.timestamp_ns
                    last_feedback_progress_ns = actual_start_ns
                elif (
                    actual_start_ns - last_feedback_progress_ns
                    > self._config.state_stale_timeout_ms * 1_000_000.0
                ):
                    raise RuntimeError("robot feedback timestamp stopped advancing")

                # A session is not armed until one valid input has arrived.  Before
                # that point there is no command stream to time out, so remain
                # disconnected while continuously commanding a stationary hold.
                if snapshot is None:
                    target = self._pipeline.hold(actual_start_ns)
                else:
                    input_dt: float | None = None
                    is_new_input = snapshot.generation != last_input_generation
                    if is_new_input:
                        if last_input_stored_at_ns is not None:
                            input_dt = max(
                                1e-9,
                                (snapshot.stored_at_ns - last_input_stored_at_ns)
                                / 1_000_000_000.0,
                            )
                        last_input_generation = snapshot.generation
                        last_input_stored_at_ns = snapshot.stored_at_ns

                    if not watchdog_armed:
                        watchdog_armed = (
                            snapshot.value.estop_requested
                            or snapshot.value.joint_zero_requested
                            or (
                                snapshot.value.clutch_active
                                and snapshot.value.tracking_quality
                                is TrackingQuality.NORMAL
                            )
                        )

                    if not watchdog_armed:
                        target = self._pipeline.process(
                            snapshot.value,
                            robot_state,
                            dt=dt,
                            now_ns=actual_start_ns,
                            input_dt=input_dt,
                            is_new_input=is_new_input,
                        )
                    else:
                        decision = self._watchdog.evaluate(
                            snapshot.stored_at_ns,
                            actual_start_ns,
                        )
                        self._metrics.record_packet_age(
                            decision.age_ms,
                            decision.stage.value,
                        )
                        if (
                            decision.stage is not WatchdogStage.FRESH
                            and previous_watchdog_stage is WatchdogStage.FRESH
                        ):
                            self._pipeline.suspend()
                        previous_watchdog_stage = decision.stage
                        if decision.stage is WatchdogStage.SAFE_STOP:
                            self._pipeline.fault()
                            self._backend.safe_stop("input watchdog safe-stop timeout")
                            safe_stop_invoked = True
                            actual_end_ns = self._clock()
                            self._metrics.record_cycle(
                                CycleTiming(
                                    deadline_ns,
                                    actual_start_ns,
                                    actual_end_ns,
                                    deadline_miss,
                                    False,
                                )
                            )
                            break
                        if decision.stage is WatchdogStage.BRAKE:
                            target = stop_target(actual_start_ns)
                        elif decision.stage is WatchdogStage.HOLD:
                            target = self._pipeline.hold(actual_start_ns)
                        else:
                            target = self._pipeline.process(
                                snapshot.value,
                                robot_state,
                                dt=dt,
                                now_ns=actual_start_ns,
                                input_dt=input_dt,
                                is_new_input=is_new_input,
                            )

                if self._pipeline.state is TeleopState.ESTOP:
                    self._backend.safe_stop("teleoperation E-stop requested")
                    safe_stop_invoked = True
                    actual_end_ns = self._clock()
                    self._metrics.record_cycle(
                        CycleTiming(
                            deadline_ns,
                            actual_start_ns,
                            actual_end_ns,
                            deadline_miss,
                            False,
                        )
                    )
                    break

                command = self._controller.update(target, robot_state, dt)
                command_state = self._backend.send(command)
                command_end_ns = self._clock()
                if command_state.timestamp_ns > last_feedback_ns:
                    last_feedback_ns = command_state.timestamp_ns
                    last_feedback_progress_ns = command_end_ns
                if self._telemetry is not None:
                    self._telemetry.record_cycle(command_state, target, command)
                actual_end_ns = command_end_ns
                next_deadline_ns = deadline_ns + self._period_ns
                deadline_rebased = (
                    actual_end_ns - next_deadline_ns > 2 * self._period_ns
                )
                self._metrics.record_cycle(
                    CycleTiming(
                        deadline_ns,
                        actual_start_ns,
                        actual_end_ns,
                        deadline_miss,
                        deadline_rebased,
                    )
                )
                deadline_ns = (
                    actual_end_ns + self._period_ns
                    if deadline_rebased
                    else next_deadline_ns
                )
        except Exception as exc:
            self._error = str(exc)
            if self._telemetry is not None:
                self._telemetry.record_errors(servo=self._error, receiver=None)
            self._pipeline.fault()
            self._metrics.record_fault(self._error)
            try:
                self._backend.safe_stop(f"servo loop fault: {exc}")
                safe_stop_invoked = True
            except Exception as stop_exc:
                self._metrics.record_fault(f"backend safe stop failed: {stop_exc}")
        finally:
            if not safe_stop_invoked:
                try:
                    self._backend.hold("servo loop stopped")
                except Exception as hold_exc:
                    self._metrics.record_fault(f"backend hold failed: {hold_exc}")
