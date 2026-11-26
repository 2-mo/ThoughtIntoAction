"""Standard-library, read-only client of the control telemetry boundary."""

from embodied.contracts.runtime.client import RuntimeReader

HOME_PHASE = 5

__all__ = ["RuntimeReader", "HOME_PHASE"]
