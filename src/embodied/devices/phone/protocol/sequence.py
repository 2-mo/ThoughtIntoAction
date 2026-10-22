"""Unsigned serial-number and session ordering for a live Pose stream."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import StrEnum

UINT32_MODULUS = 1 << 32
UINT32_MAX = UINT32_MODULUS - 1
UINT32_HALF_RANGE = 1 << 31
UINT64_MAX = (1 << 64) - 1


class SequenceErrorCode(StrEnum):
    DUPLICATE = "duplicate"
    OUT_OF_ORDER = "out_of_order"
    RETIRED_SESSION = "retired_session"
    TIMESTAMP = "timestamp"


class SequenceError(ValueError):
    def __init__(self, code: SequenceErrorCode, detail: str) -> None:
        self.code = code
        self.detail = detail
        super().__init__(f"{code.value}: {detail}")


@dataclass(frozen=True, slots=True)
class SequenceObservation:
    session_changed: bool
    lost_packets: int


def _wire_integer(
    name: str,
    value: int,
    maximum: int,
    *,
    nonzero: bool = False,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if value < int(nonzero) or value > maximum:
        raise ValueError(f"{name} is outside its wire range")
    return value


class SequenceTracker:
    def __init__(self, *, retired_session_capacity: int = 16) -> None:
        if isinstance(retired_session_capacity, bool) or retired_session_capacity <= 0:
            raise ValueError("retired_session_capacity must be positive")
        self._retired_capacity = retired_session_capacity
        self._retired_order: deque[int] = deque()
        self._retired: set[int] = set()
        self._session_id: int | None = None
        self._sequence: int | None = None
        self._timestamp_ns: int | None = None

    def reset(self) -> None:
        self._retired_order.clear()
        self._retired.clear()
        self._session_id = None
        self._sequence = None
        self._timestamp_ns = None

    def _retire(self, session_id: int) -> None:
        if len(self._retired_order) == self._retired_capacity:
            expired = self._retired_order.popleft()
            self._retired.remove(expired)
        self._retired_order.append(session_id)
        self._retired.add(session_id)

    def observe(
        self,
        *,
        session_id: int,
        sequence: int,
        timestamp_ns: int,
    ) -> SequenceObservation:
        session_id = _wire_integer(
            "session_id",
            session_id,
            UINT32_MAX,
            nonzero=True,
        )
        sequence = _wire_integer("sequence", sequence, UINT32_MAX)
        timestamp_ns = _wire_integer("timestamp_ns", timestamp_ns, UINT64_MAX)

        if self._session_id is None:
            self._session_id = session_id
            self._sequence = sequence
            self._timestamp_ns = timestamp_ns
            return SequenceObservation(session_changed=False, lost_packets=0)

        if session_id != self._session_id:
            if session_id in self._retired:
                raise SequenceError(
                    SequenceErrorCode.RETIRED_SESSION,
                    "datagram belongs to a retired session",
                )
            self._retire(self._session_id)
            self._session_id = session_id
            self._sequence = sequence
            self._timestamp_ns = timestamp_ns
            return SequenceObservation(session_changed=True, lost_packets=0)

        assert self._sequence is not None
        assert self._timestamp_ns is not None
        delta = (sequence - self._sequence) & UINT32_MAX
        if delta == 0:
            raise SequenceError(SequenceErrorCode.DUPLICATE, "duplicate sequence")
        if delta >= UINT32_HALF_RANGE:
            raise SequenceError(
                SequenceErrorCode.OUT_OF_ORDER,
                "sequence is old or ambiguous",
            )
        if timestamp_ns <= self._timestamp_ns:
            raise SequenceError(
                SequenceErrorCode.TIMESTAMP,
                "source timestamp did not increase",
            )
        self._sequence = sequence
        self._timestamp_ns = timestamp_ns
        return SequenceObservation(session_changed=False, lost_packets=delta - 1)
