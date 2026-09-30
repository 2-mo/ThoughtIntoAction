"""NTP-style clock offset estimation for unrelated monotonic clocks."""

from __future__ import annotations

import statistics
from collections import deque
from dataclasses import dataclass
from threading import Lock

UINT64_MAX = (1 << 64) - 1


class ClockSampleError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class ClockSample:
    client_send_time_ns: int
    server_receive_time_ns: int
    server_send_time_ns: int
    client_receive_time_ns: int
    rtt_ns: int
    server_minus_client_ns: float


def _timestamp(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not 0 <= value <= UINT64_MAX:
        raise ValueError(f"{name} must be a UInt64 value")
    return value


def clock_sample(
    client_send_time_ns: int,
    server_receive_time_ns: int,
    server_send_time_ns: int,
    client_receive_time_ns: int,
) -> ClockSample:
    t0 = _timestamp("client_send_time_ns", client_send_time_ns)
    t1 = _timestamp("server_receive_time_ns", server_receive_time_ns)
    t2 = _timestamp("server_send_time_ns", server_send_time_ns)
    t3 = _timestamp("client_receive_time_ns", client_receive_time_ns)
    if t3 < t0:
        raise ClockSampleError("client receive time precedes send time")
    if t2 < t1:
        raise ClockSampleError("server send time precedes receive time")
    rtt_ns = (t3 - t0) - (t2 - t1)
    if rtt_ns < 0:
        raise ClockSampleError("server processing exceeds observed round trip")
    offset_ns = ((t1 - t0) + (t2 - t3)) / 2.0
    return ClockSample(t0, t1, t2, t3, rtt_ns, offset_ns)


class ClockSyncEstimator:
    def __init__(
        self,
        *,
        max_rtt_ns: int = 100_000_000,
        sample_capacity: int = 32,
        low_rtt_sample_count: int = 8,
    ) -> None:
        for name, value in (
            ("max_rtt_ns", max_rtt_ns),
            ("sample_capacity", sample_capacity),
            ("low_rtt_sample_count", low_rtt_sample_count),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if low_rtt_sample_count > sample_capacity:
            raise ValueError("low_rtt_sample_count cannot exceed sample_capacity")
        self._max_rtt_ns = max_rtt_ns
        self._low_rtt_sample_count = low_rtt_sample_count
        self._samples: deque[ClockSample] = deque(maxlen=sample_capacity)
        self._rejected_samples = 0
        self._lock = Lock()

    def reset(self) -> None:
        with self._lock:
            self._samples.clear()
            self._rejected_samples = 0

    def add_exchange(
        self,
        client_send_time_ns: int,
        server_receive_time_ns: int,
        server_send_time_ns: int,
        client_receive_time_ns: int,
    ) -> bool:
        try:
            sample = clock_sample(
                client_send_time_ns,
                server_receive_time_ns,
                server_send_time_ns,
                client_receive_time_ns,
            )
        except (ClockSampleError, ValueError):
            with self._lock:
                self._rejected_samples += 1
            return False
        with self._lock:
            if sample.rtt_ns > self._max_rtt_ns:
                self._rejected_samples += 1
                return False
            self._samples.append(sample)
        return True

    def _selected_samples(self) -> list[ClockSample]:
        return sorted(self._samples, key=lambda sample: sample.rtt_ns)[
            : self._low_rtt_sample_count
        ]

    @property
    def sample_count(self) -> int:
        with self._lock:
            return len(self._samples)

    @property
    def rejected_samples(self) -> int:
        with self._lock:
            return self._rejected_samples

    @property
    def offset_ns(self) -> float | None:
        with self._lock:
            selected = self._selected_samples()
            if not selected:
                return None
            return float(
                statistics.median(sample.server_minus_client_ns for sample in selected)
            )

    @property
    def minimum_rtt_ns(self) -> int | None:
        with self._lock:
            if not self._samples:
                return None
            return min(sample.rtt_ns for sample in self._samples)

    def to_server_time_ns(self, client_time_ns: int) -> int:
        client_time = _timestamp("client_time_ns", client_time_ns)
        offset = self.offset_ns
        if offset is None:
            raise RuntimeError("clock offset is unavailable")
        return round(client_time + offset)
