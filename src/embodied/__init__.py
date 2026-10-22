"""Embodied-AI platform: everything outside the arx kernel (control/)."""
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .applications.operations.robot import Robot

__all__ = ["Robot"]


def __getattr__(name):
    if name == "Robot":
        from .applications.operations.robot import Robot
        globals()[name] = Robot
        return Robot
    raise AttributeError(name)
