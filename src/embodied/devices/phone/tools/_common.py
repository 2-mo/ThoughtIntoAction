"""Small shared helpers for hardware-free network tools."""

from __future__ import annotations

import json
import math
import threading
from collections import Counter
from collections.abc import Mapping

from embodied.devices.phone.runtime.metrics import summarize_samples


class EventCounts:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._counts: Counter[str] = Counter()

    def __call__(self, event: str, count: int) -> None:
        with self._lock:
            self._counts[event] += count

    def snapshot(self) -> dict[str, int]:
        with self._lock:
            return dict(self._counts)


def validate_positive(name: str, value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return value


def validate_nonnegative(name: str, value: float) -> float:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be non-negative and finite")
    return value


def validate_probability(name: str, value: float) -> float:
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be finite and in [0, 1]")
    return value


def validate_port(port: int, *, allow_zero: bool = False) -> int:
    minimum = 0 if allow_zero else 1
    if isinstance(port, bool) or not minimum <= port <= 65_535:
        raise ValueError(f"port must be in [{minimum}, 65535]")
    return port


def distribution_with_count(samples: list[float]) -> dict[str, float | int]:
    return {"count": len(samples), **summarize_samples(samples)}


def print_report(report: Mapping[str, object]) -> None:
    print(json.dumps(report, sort_keys=True))
