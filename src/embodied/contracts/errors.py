"""Shared control and motion-planning exceptions."""


class ControlError(RuntimeError):
    """Base class for local controller errors."""


class SafetyError(ControlError):
    """Raised when an configured control or motion limit is violated."""


class MotionSettleError(SafetyError):
    """A completed trajectory could not prove its terminal hold condition."""


class BusyError(ControlError):
    """Raised when another process owns the CAN interface."""
