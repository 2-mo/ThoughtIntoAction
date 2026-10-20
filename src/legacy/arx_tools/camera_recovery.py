"""Small camera recovery decisions shared by runtime code and tests."""

from __future__ import annotations


def requires_clean_process_restart(error: BaseException | str) -> bool:
    """Return whether the camera error needs kernel-owned handles closed."""
    message = str(error).casefold()
    return "device or resource busy" in message or "errno 16" in message
