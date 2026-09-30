"""Thread-safe correlation for explicitly completing a latched E-stop reset."""

from __future__ import annotations

import threading

from robot_teleop.core.models import TeleopInput, TrackingQuality
from robot_teleop.protocol.packet import EStopResetRequestPacket


class EStopResetGate:
    """Require a request plus a later neutral input before a local reset."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest_input: TeleopInput | None = None
        self._pending_request: EStopResetRequestPacket | None = None
        self._ready_request: EStopResetRequestPacket | None = None
        self._completed_request: EStopResetRequestPacket | None = None

    @property
    def pending_request(self) -> EStopResetRequestPacket | None:
        with self._lock:
            return self._pending_request

    @property
    def ready_request(self) -> EStopResetRequestPacket | None:
        with self._lock:
            return self._ready_request

    @property
    def completed_request(self) -> EStopResetRequestPacket | None:
        with self._lock:
            return self._completed_request

    def record_request(self, request: EStopResetRequestPacket) -> bool:
        """Record a request and report whether its reset already completed."""
        with self._lock:
            if self._completed_request == request:
                return True
            self._pending_request = request
            self._ready_request = (
                request if self._is_safe_release(self._latest_input, request) else None
            )
            return False

    def observe_input(self, input_state: TeleopInput) -> None:
        """Record validated input and expire a request when its session changes."""
        with self._lock:
            self._latest_input = input_state
            request = self._pending_request
            if request is None:
                return
            if input_state.session_id != request.session_id:
                self._pending_request = None
                self._ready_request = None
                return
            if self._is_safe_release(input_state, request):
                self._ready_request = request

    def complete(self, request: EStopResetRequestPacket) -> None:
        """Mark a locally completed reset so its acknowledgement can be retried."""
        with self._lock:
            if self._ready_request != request:
                raise ValueError("E-stop reset request is not ready for completion")
            self._pending_request = None
            self._ready_request = None
            self._completed_request = request

    @staticmethod
    def _is_safe_release(
        input_state: TeleopInput | None,
        request: EStopResetRequestPacket,
    ) -> bool:
        return (
            input_state is not None
            and input_state.session_id == request.session_id
            and input_state.timestamp_ns > request.client_time_ns
            and input_state.tracking_quality is TrackingQuality.NORMAL
            and not input_state.clutch_active
            and not input_state.recenter_requested
            and not input_state.estop_requested
        )
