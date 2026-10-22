"""Bounded control-state observations for canonical teleoperation inputs."""

from __future__ import annotations

import math
from collections import Counter

from embodied.devices.phone.core.models import TeleopInput, TrackingQuality


class ControlObservations:
    """Accumulate control diagnostics without retaining input history."""

    def __init__(self) -> None:
        self._current_session_id: int | None = None
        self._sessions_observed = 0

        self._previous_clutch = False
        self._clutch_active_packets = 0
        self._clutch_activation_edges = 0
        self._clutch_active_session_starts = 0
        self._clutch_release_edges = 0
        self._last_clutch: bool | None = None

        self._previous_recenter = False
        self._recenter_requested_packets = 0
        self._recenter_activation_edges = 0
        self._recenter_requested_session_starts = 0
        self._last_recenter: bool | None = None

        self._previous_estop = False
        self._estop_requested_packets = 0
        self._estop_activation_edges = 0
        self._estop_requested_session_starts = 0
        self._estop_release_edges = 0
        self._last_estop: bool | None = None

        self._previous_gripper: float | None = None
        self._gripper_samples = 0
        self._gripper_changes = 0
        self._gripper_minimum: float | None = None
        self._gripper_maximum: float | None = None
        self._last_gripper: float | None = None

        self._tracking_counts: Counter[TrackingQuality] = Counter()

    def observe(self, input_state: TeleopInput) -> None:
        if input_state.session_id != self._current_session_id:
            self._start_session(input_state)

        clutch = input_state.clutch_active
        if clutch:
            self._clutch_active_packets += 1
        if clutch and not self._previous_clutch:
            self._clutch_activation_edges += 1
        elif self._previous_clutch and not clutch:
            self._clutch_release_edges += 1
        self._previous_clutch = clutch
        self._last_clutch = clutch

        recenter = input_state.recenter_requested
        if recenter:
            self._recenter_requested_packets += 1
        if recenter and not self._previous_recenter:
            self._recenter_activation_edges += 1
        self._previous_recenter = recenter
        self._last_recenter = recenter

        estop = input_state.estop_requested
        if estop:
            self._estop_requested_packets += 1
        if estop and not self._previous_estop:
            self._estop_activation_edges += 1
        elif self._previous_estop and not estop:
            self._estop_release_edges += 1
        self._previous_estop = estop
        self._last_estop = estop

        self._observe_gripper(input_state.gripper)
        self._tracking_counts[input_state.tracking_quality] += 1

    def report(self) -> dict[str, object]:
        return {
            "sessions_observed": self._sessions_observed,
            "clutch": {
                "active_packets": self._clutch_active_packets,
                "activation_edges": self._clutch_activation_edges,
                "active_session_starts": self._clutch_active_session_starts,
                "release_edges": self._clutch_release_edges,
                "last_active": self._last_clutch,
            },
            "recenter": {
                "requested_packets": self._recenter_requested_packets,
                "activation_edges": self._recenter_activation_edges,
                "requested_session_starts": (self._recenter_requested_session_starts),
                "last_requested": self._last_recenter,
            },
            "estop": {
                "requested_packets": self._estop_requested_packets,
                "activation_edges": self._estop_activation_edges,
                "requested_session_starts": self._estop_requested_session_starts,
                "release_edges": self._estop_release_edges,
                "ever_requested": self._estop_requested_packets > 0,
                "last_requested": self._last_estop,
            },
            "gripper": {
                "samples": self._gripper_samples,
                "changes": self._gripper_changes,
                "minimum": self._gripper_minimum,
                "maximum": self._gripper_maximum,
                "last": self._last_gripper,
            },
            "tracking": {
                quality.value: self._tracking_counts[quality]
                for quality in TrackingQuality
            },
        }

    def _start_session(self, input_state: TeleopInput) -> None:
        self._current_session_id = input_state.session_id
        self._sessions_observed += 1
        self._previous_clutch = input_state.clutch_active
        self._previous_recenter = input_state.recenter_requested
        self._previous_estop = input_state.estop_requested
        self._previous_gripper = None
        if input_state.clutch_active:
            self._clutch_active_session_starts += 1
        if input_state.recenter_requested:
            self._recenter_requested_session_starts += 1
        if input_state.estop_requested:
            self._estop_requested_session_starts += 1

    def _observe_gripper(self, gripper: float | None) -> None:
        if gripper is None:
            return
        self._gripper_samples += 1
        if self._gripper_minimum is None or gripper < self._gripper_minimum:
            self._gripper_minimum = gripper
        if self._gripper_maximum is None or gripper > self._gripper_maximum:
            self._gripper_maximum = gripper
        if self._previous_gripper is not None and not math.isclose(
            gripper,
            self._previous_gripper,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            self._gripper_changes += 1
        self._previous_gripper = gripper
        self._last_gripper = gripper
