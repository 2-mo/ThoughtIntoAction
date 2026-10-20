"""Bounded, monotonic loop-rate measurement; never infers motor servo rate."""
from collections import deque
import time


class LoopRate:
    def __init__(self, configured_hz):
        self.configured_hz = configured_hz
        self.samples = deque(maxlen=4096)

    def tick(self, now=None):
        now = time.monotonic_ns() if now is None else now
        self.samples.append(now)
        while len(self.samples) > 2 and now - self.samples[0] > 1_000_000_000:
            self.samples.popleft()

    def snapshot(self, now=None):
        now = time.monotonic_ns() if now is None else now
        samples = tuple(self.samples)
        rate = None
        if len(samples) > 1 and 0 <= now - samples[-1] < 500_000_000:
            span = samples[-1] - samples[0]
            if span >= 200_000_000:
                rate = (len(samples) - 1) * 1e9 / span
        return {"configured_hz": self.configured_hz, "observed_hz": rate,
                "sample_monotonic_ns": samples[-1] if samples else None}
