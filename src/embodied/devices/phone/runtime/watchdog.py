"""Host-receipt-age watchdog with hold, brake, and safe-stop stages."""

import math
from dataclasses import dataclass
from enum import Enum


class WatchdogStage(Enum):
    FRESH = "fresh"
    HOLD = "hold"
    BRAKE = "brake"
    SAFE_STOP = "safe_stop"


@dataclass(frozen=True, slots=True)
class WatchdogConfig:
    hold_packet_age_ms: float = 50.0
    brake_packet_age_ms: float = 100.0
    stop_packet_age_ms: float = 300.0

    def __post_init__(self) -> None:
        values = (
            self.hold_packet_age_ms,
            self.brake_packet_age_ms,
            self.stop_packet_age_ms,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("watchdog thresholds must be positive and finite")
        if not (
            self.hold_packet_age_ms < self.brake_packet_age_ms < self.stop_packet_age_ms
        ):
            raise ValueError("watchdog thresholds must satisfy hold < brake < stop")


@dataclass(frozen=True, slots=True)
class WatchdogDecision:
    stage: WatchdogStage
    age_ms: float


class Watchdog:
    def __init__(self, config: WatchdogConfig) -> None:
        self._config = config
        self._started_at_ns: int | None = None

    def reset(self, now_ns: int) -> None:
        if isinstance(now_ns, bool) or now_ns < 0:
            raise ValueError("watchdog time must be non-negative")
        self._started_at_ns = now_ns

    def evaluate(
        self,
        received_at_ns: int | None,
        now_ns: int,
    ) -> WatchdogDecision:
        if self._started_at_ns is None:
            raise RuntimeError("watchdog must be reset before evaluation")
        reference_ns = self._started_at_ns if received_at_ns is None else received_at_ns
        age_ms = max(0.0, (now_ns - reference_ns) / 1_000_000.0)
        if age_ms > self._config.stop_packet_age_ms:
            stage = WatchdogStage.SAFE_STOP
        elif age_ms > self._config.brake_packet_age_ms:
            stage = WatchdogStage.BRAKE
        elif age_ms > self._config.hold_packet_age_ms:
            stage = WatchdogStage.HOLD
        else:
            stage = WatchdogStage.FRESH
        return WatchdogDecision(stage, age_ms)
