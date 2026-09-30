"""Worker-local command candidates; devices never own arbitration or the backend."""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Candidate:
    epoch: int
    command: Any = None
    expires_at: float = 0.
    sequence: int = -1
    phase: str = 'hold'
    fault: str | None = None

