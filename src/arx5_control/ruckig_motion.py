"""Compatibility imports for the offline Ruckig planner.

New code should import from :mod:`arx5_control.motion.offline`.
"""

from .motion.offline import (
    MAX_SAMPLED_WAYPOINTS,
    RuckigJointTrajectory,
    plan_ruckig_joint_trajectory,
)

__all__ = [
    "MAX_SAMPLED_WAYPOINTS",
    "RuckigJointTrajectory",
    "plan_ruckig_joint_trajectory",
]
