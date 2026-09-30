"""Constant-memory, overwrite-only cross-thread handoff."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class LatestSnapshot[T]:
    value: T
    generation: int
    stored_at_ns: int


class LatestValue[T]:
    def __init__(self, *, clock: Callable[[], int] = time.monotonic_ns) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._generation = 0
        self._snapshot: LatestSnapshot[T] | None = None

    def store(
        self,
        value: T,
        *,
        stored_at_ns: int | None = None,
    ) -> LatestSnapshot[T]:
        timestamp = self._clock() if stored_at_ns is None else stored_at_ns
        if isinstance(timestamp, bool) or timestamp < 0:
            raise ValueError("stored_at_ns must be non-negative")
        with self._lock:
            self._generation += 1
            self._snapshot = LatestSnapshot(value, self._generation, timestamp)
            return self._snapshot

    def load(self) -> LatestSnapshot[T] | None:
        with self._lock:
            return self._snapshot

    def clear(self) -> None:
        with self._lock:
            self._snapshot = None
