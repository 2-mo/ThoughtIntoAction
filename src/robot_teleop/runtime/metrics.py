"""Bounded in-memory control and input metrics."""

from __future__ import annotations

import math
import statistics
import threading
from collections import Counter, deque
from collections.abc import Iterable
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class CycleTiming:
    expected_deadline_ns: int
    actual_start_ns: int
    actual_end_ns: int
    deadline_miss: bool
    deadline_rebased: bool

    def __post_init__(self) -> None:
        if (
            min(
                self.expected_deadline_ns,
                self.actual_start_ns,
                self.actual_end_ns,
            )
            < 0
        ):
            raise ValueError("cycle timestamps must be non-negative")
        if self.actual_end_ns < self.actual_start_ns:
            raise ValueError("cycle end must not precede cycle start")

    @property
    def compute_ns(self) -> int:
        return self.actual_end_ns - self.actual_start_ns

    @property
    def jitter_ns(self) -> int:
        return self.actual_start_ns - self.expected_deadline_ns


def _percentile(sorted_values: list[float], percentile: float) -> float:
    if not sorted_values:
        return 0.0
    index = max(0, math.ceil(percentile * len(sorted_values)) - 1)
    return sorted_values[index]


def _distribution(values: list[float]) -> dict[str, float]:
    if not values:
        return {
            "mean": 0.0,
            "median": 0.0,
            "p90": 0.0,
            "p95": 0.0,
            "p99": 0.0,
            "max": 0.0,
        }
    ordered = sorted(values)
    return {
        "mean": statistics.fmean(ordered),
        "median": statistics.median(ordered),
        "p90": _percentile(ordered, 0.90),
        "p95": _percentile(ordered, 0.95),
        "p99": _percentile(ordered, 0.99),
        "max": ordered[-1],
    }


def summarize_samples(values: Iterable[float]) -> dict[str, float]:
    """Use the runtime percentile convention in offline reports and tools."""

    samples = [float(value) for value in values]
    if not all(math.isfinite(value) for value in samples):
        raise ValueError("metric samples must be finite")
    return _distribution(samples)


class RuntimeMetrics:
    def __init__(self, *, sample_capacity: int = 4096) -> None:
        if isinstance(sample_capacity, bool) or sample_capacity <= 0:
            raise ValueError("sample_capacity must be positive")
        self._lock = threading.Lock()
        self._cycles: deque[CycleTiming] = deque(maxlen=sample_capacity)
        self._cycle_count = 0
        self._deadline_misses = 0
        self._deadline_rebases = 0
        self._consecutive_deadline_misses = 0
        self._maximum_consecutive_deadline_misses = 0
        self._packet_age_ms: deque[float] = deque(maxlen=sample_capacity)
        self._input_interval_ms: deque[float] = deque(maxlen=sample_capacity)
        self._input_interval_count = 0
        self._watchdog_stages: Counter[str] = Counter()
        self._accepted_inputs = 0
        self._rejected_inputs = 0
        self._input_rejections: Counter[str] = Counter()
        self._faults: Counter[str] = Counter()
        self._protocol_events: Counter[str] = Counter()
        self._protocol_timings_ms: dict[str, deque[float]] = {}
        self._sample_capacity = sample_capacity

    def record_cycle(self, timing: CycleTiming) -> None:
        with self._lock:
            self._cycles.append(timing)
            self._cycle_count += 1
            self._deadline_misses += int(timing.deadline_miss)
            self._deadline_rebases += int(timing.deadline_rebased)
            if timing.deadline_miss:
                self._consecutive_deadline_misses += 1
                self._maximum_consecutive_deadline_misses = max(
                    self._maximum_consecutive_deadline_misses,
                    self._consecutive_deadline_misses,
                )
            else:
                self._consecutive_deadline_misses = 0

    def record_packet_age(self, age_ms: float, watchdog_stage: str) -> None:
        if not math.isfinite(age_ms) or age_ms < 0.0:
            raise ValueError("packet age must be non-negative and finite")
        stage = watchdog_stage.strip()
        if not stage:
            raise ValueError("watchdog stage must not be blank")
        with self._lock:
            self._packet_age_ms.append(age_ms)
            self._watchdog_stages[stage] += 1

    def record_input_interval(self, interval_ms: float) -> None:
        if not math.isfinite(interval_ms) or interval_ms < 0.0:
            raise ValueError("input interval must be non-negative and finite")
        with self._lock:
            self._input_interval_ms.append(interval_ms)
            self._input_interval_count += 1

    def record_input_accepted(self) -> None:
        with self._lock:
            self._accepted_inputs += 1

    def record_input_rejected(self, reason: str) -> None:
        key = reason.strip() or "unknown"
        with self._lock:
            self._rejected_inputs += 1
            self._input_rejections[key] += 1

    def record_fault(self, reason: str) -> None:
        key = reason.strip() or "unknown"
        with self._lock:
            self._faults[key] += 1

    def record_protocol_event(self, event: str, count: int = 1) -> None:
        key = event.strip()
        if not key:
            raise ValueError("protocol event must not be blank")
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("protocol event count must be a positive integer")
        with self._lock:
            self._protocol_events[key] += count

    def record_protocol_timing(self, name: str, value_ms: float) -> None:
        key = name.strip()
        if not key:
            raise ValueError("protocol timing name must not be blank")
        if not math.isfinite(value_ms) or value_ms < 0.0:
            raise ValueError("protocol timing must be non-negative and finite")
        with self._lock:
            samples = self._protocol_timings_ms.get(key)
            if samples is None:
                samples = deque(maxlen=self._sample_capacity)
                self._protocol_timings_ms[key] = samples
            samples.append(value_ms)

    def report(self) -> dict[str, object]:
        with self._lock:
            cycles = list(self._cycles)
            cycle_count = self._cycle_count
            deadline_misses = self._deadline_misses
            deadline_rebases = self._deadline_rebases
            consecutive_deadline_misses = self._consecutive_deadline_misses
            maximum_consecutive_deadline_misses = (
                self._maximum_consecutive_deadline_misses
            )
            packet_age_ms = list(self._packet_age_ms)
            input_interval_ms = list(self._input_interval_ms)
            input_interval_count = self._input_interval_count
            watchdog_stages = dict(self._watchdog_stages)
            accepted_inputs = self._accepted_inputs
            rejected_inputs = self._rejected_inputs
            input_rejections = dict(self._input_rejections)
            faults = dict(self._faults)
            protocol_events = dict(self._protocol_events)
            protocol_timings_ms = {
                name: list(samples)
                for name, samples in self._protocol_timings_ms.items()
            }
            sample_capacity = self._sample_capacity
        compute_ms = [cycle.compute_ns / 1_000_000.0 for cycle in cycles]
        jitter_ms = [cycle.jitter_ns / 1_000_000.0 for cycle in cycles]
        frequency_hz = 0.0
        if len(cycles) > 1:
            duration_ns = cycles[-1].actual_start_ns - cycles[0].actual_start_ns
            if duration_ns > 0:
                frequency_hz = (len(cycles) - 1) * 1_000_000_000.0 / duration_ns
        packet_age_count = sum(watchdog_stages.values())
        return {
            "sample_window": {
                "capacity": sample_capacity,
                "cycle_timing": {
                    "sample_count": len(cycles),
                    "scope": (
                        "all_session"
                        if cycle_count <= sample_capacity
                        else "recent_window"
                    ),
                },
                "packet_age": {
                    "sample_count": len(packet_age_ms),
                    "scope": (
                        "all_observed"
                        if packet_age_count <= sample_capacity
                        else "recent_window"
                    ),
                },
                "input_interval": {
                    "sample_count": len(input_interval_ms),
                    "scope": (
                        "all_observed"
                        if input_interval_count <= sample_capacity
                        else "recent_window"
                    ),
                },
            },
            "cycle_count": cycle_count,
            "deadline_misses": deadline_misses,
            "deadline_rebases": deadline_rebases,
            "consecutive_deadline_misses": consecutive_deadline_misses,
            "maximum_consecutive_deadline_misses": (
                maximum_consecutive_deadline_misses
            ),
            "control_loop_frequency_hz": frequency_hz,
            "compute_ms": _distribution(compute_ms),
            "jitter_ms": _distribution(jitter_ms),
            "packet_age_ms": _distribution(packet_age_ms),
            "input_interval_ms": _distribution(input_interval_ms),
            "watchdog_stages": watchdog_stages,
            "accepted_inputs": accepted_inputs,
            "rejected_inputs": rejected_inputs,
            "input_rejections": input_rejections,
            "faults": faults,
            "protocol_events": protocol_events,
            "protocol_timings_ms": {
                name: _distribution(samples)
                for name, samples in protocol_timings_ms.items()
            },
        }


class MetricsWorker:
    def __init__(self, metrics: RuntimeMetrics, *, interval_s: float = 1.0) -> None:
        if not math.isfinite(interval_s) or interval_s <= 0.0:
            raise ValueError("metrics interval must be positive and finite")
        self._metrics = metrics
        self._interval_s = interval_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest_report: dict[str, object] = metrics.report()

    @property
    def alive(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def latest_report(self) -> dict[str, object]:
        with self._lock:
            return dict(self._latest_report)

    def start(self) -> None:
        if self.alive:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="robot-teleop-metrics",
            daemon=False,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout_s)
            if thread.is_alive():
                raise RuntimeError("metrics worker did not stop")
        with self._lock:
            self._latest_report = self._metrics.report()
        self._thread = None

    def _run(self) -> None:
        while not self._stop_event.wait(self._interval_s):
            report = self._metrics.report()
            with self._lock:
                self._latest_report = report
