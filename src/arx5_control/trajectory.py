"""Compatibility imports for the trajectory profile module.

New code should import from :mod:`arx5_control.motion.profiles`.
"""

from .motion.profiles import (
    TrajectorySample,
    cosine_fraction,
    cosine_segment,
    minimum_cosine_duration,
    minimum_smoothstep9_duration,
    round_trip,
    smoothstep9_fraction,
    via_point,
)

__all__ = [
    "TrajectorySample",
    "cosine_fraction",
    "cosine_segment",
    "minimum_cosine_duration",
    "minimum_smoothstep9_duration",
    "round_trip",
    "smoothstep9_fraction",
    "via_point",
]
