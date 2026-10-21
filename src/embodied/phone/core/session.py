"""Single owner for clutch, recenter, session, and tracking continuity."""

from dataclasses import dataclass
from enum import Enum

from embodied.phone.core.models import TeleopInput, TrackingQuality


class SessionAction(Enum):
    IDLE = "idle"
    ACTIVATE = "activate"
    ACTIVE = "active"
    RELEASE = "release"
    RECENTER = "recenter"
    DEGRADED = "degraded"
    TRACKING_LOST = "tracking_lost"
    SESSION_CHANGED = "session_changed"


@dataclass(frozen=True, slots=True)
class SessionDecision:
    action: SessionAction
    session_changed: bool = False
    requires_release: bool = False


class TeleopSession:
    def __init__(self) -> None:
        self._session_id: int | None = None
        self._active = False
        self._requires_release = False
        self._tracking_was_normal: bool | None = None
        self._recenter_was_requested = False

    @property
    def session_id(self) -> int | None:
        return self._session_id

    @property
    def active(self) -> bool:
        return self._active

    def reset(self, *, require_release: bool = False) -> None:
        self._session_id = None
        self._active = False
        self._requires_release = require_release
        self._tracking_was_normal = None
        self._recenter_was_requested = False

    def observe(self, input_state: TeleopInput) -> SessionDecision:
        initial_session = self._session_id is None
        session_changed = (
            not initial_session and input_state.session_id != self._session_id
        )
        if initial_session or session_changed:
            self._session_id = input_state.session_id
            self._active = False
            self._tracking_was_normal = None
            self._recenter_was_requested = False
            if session_changed and input_state.clutch_active:
                self._requires_release = True

        if input_state.tracking_quality is not TrackingQuality.NORMAL:
            if self._active or input_state.clutch_active:
                self._requires_release = input_state.clutch_active
            self._active = False
            self._tracking_was_normal = False
            action = (
                SessionAction.DEGRADED
                if input_state.tracking_quality is TrackingQuality.LIMITED
                else SessionAction.TRACKING_LOST
            )
            return SessionDecision(
                action,
                session_changed=session_changed,
                requires_release=self._requires_release,
            )

        if self._tracking_was_normal is False and input_state.clutch_active:
            self._requires_release = True
        self._tracking_was_normal = True

        if session_changed:
            return SessionDecision(
                SessionAction.SESSION_CHANGED,
                session_changed=True,
                requires_release=self._requires_release,
            )

        if self._requires_release:
            if not input_state.clutch_active:
                self._requires_release = False
            return SessionDecision(
                SessionAction.IDLE,
                requires_release=self._requires_release,
            )

        recenter_edge = (
            input_state.recenter_requested and not self._recenter_was_requested
        )
        self._recenter_was_requested = input_state.recenter_requested

        if input_state.clutch_active:
            if not self._active:
                self._active = True
                return SessionDecision(SessionAction.ACTIVATE)
            if recenter_edge:
                return SessionDecision(SessionAction.RECENTER)
            return SessionDecision(SessionAction.ACTIVE)

        was_active = self._active
        self._active = False
        return SessionDecision(
            SessionAction.RELEASE if was_active else SessionAction.IDLE
        )
