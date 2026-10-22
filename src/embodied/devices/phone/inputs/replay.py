"""Timestamp-driven replay input source with no private worker thread."""

import math
import threading
import time
from collections.abc import Callable, Sequence

from embodied.devices.phone.core.capabilities import InputCapabilities
from embodied.devices.phone.core.models import TeleopInput
from embodied.devices.phone.core.safety import InputSafetyLimits, InputSafetyValidator


class ReplayInputSource:
    def __init__(
        self,
        samples: Sequence[TeleopInput],
        capabilities: InputCapabilities,
        *,
        playback_rate: float = 1.0,
        clock: Callable[[], int] = time.monotonic_ns,
        safety_limits: InputSafetyLimits | None = None,
    ) -> None:
        if not math.isfinite(playback_rate) or playback_rate <= 0.0:
            raise ValueError("playback_rate must be positive and finite")
        validator = InputSafetyValidator(
            safety_limits or InputSafetyLimits(),
            capabilities,
        )
        copied = tuple(samples)
        for sample in copied:
            validator.validate(sample)
        self._samples = copied
        self._capabilities = capabilities
        self._playback_rate = playback_rate
        self._clock = clock
        self._lock = threading.Lock()
        self._running = False
        self._started_at_ns = 0
        self._index = 0

    @property
    def capabilities(self) -> InputCapabilities:
        return self._capabilities

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._started_at_ns = self._clock()
            self._index = 0
            self._running = True

    def latest(self) -> TeleopInput | None:
        with self._lock:
            if not self._running or not self._samples:
                return None
            source_start_ns = self._samples[0].timestamp_ns
            elapsed_ns = int(
                (self._clock() - self._started_at_ns) * self._playback_rate
            )
            while self._index + 1 < len(self._samples):
                next_sample = self._samples[self._index + 1]
                if next_sample.timestamp_ns - source_start_ns > elapsed_ns:
                    break
                self._index += 1
            return self._samples[self._index]

    def stop(self) -> None:
        with self._lock:
            self._running = False
