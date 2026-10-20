"""Trajectory profiles, derivative limits, and Ruckig planners.

This package is hardware-independent. SDK submission and feedback monitoring
remain in :mod:`arx5_control.controller` and :mod:`arx5_control.servo`.
"""

from arx5_control.motion.limits import (
    DEFAULT_DERIVATIVE_SCALE_FLOOR,
    MAX_CARTESIAN_WAYPOINTS,
    MIN_CARTESIAN_WAYPOINTS,
    SampledMotion,
    analyze_sampled_motion,
    derivative_scales,
    scaled_derivative_limits,
)
from arx5_control.motion.offline import (
    MAX_SAMPLED_WAYPOINTS,
    RuckigJointTrajectory,
    plan_blended_joint_trajectory,
    plan_ruckig_joint_trajectory,
)
from arx5_control.motion.online import JointRuckigPlanner, JointServoCommand
from arx5_control.motion.cartesian_time import (
    MAX_PROGRESS_SAMPLES,
    ScalarProgressTrajectory,
    plan_scalar_progress,
)
from arx5_control.motion.collision import (
    AabbObstacle,
    CollisionCheck,
    PrimitiveObstacle,
    SphereObstacle,
    check_tcp_path,
)
from arx5_control.motion.jacobian import (
    JacobianDiagnostics,
    analyze_jacobian,
    numerical_jacobian,
    singularity_speed_scale,
)
from arx5_control.motion.geometry import (
    CartesianGeometryPath,
    CartesianWaypoint,
    build_cartesian_path,
    build_circular_path,
    concatenate_cartesian_paths,
    quaternion_distance_rad,
    slerp_quaternion_xyzw,
)
from arx5_control.motion.contracts import (
    CartesianPathMode,
    CartesianPathSegment,
    MotionContext,
    MotionPlan,
    MotionType,
    Payload,
    Pose,
    ReferenceFrame,
    ToolCenterPoint,
    quaternion_multiply_xyzw,
    rotate_vector_xyzw,
)
from arx5_control.motion.link_collision import (
    LinkCapsule,
    LinkCollisionCheck,
    PinocchioCollisionModel,
    default_x5_capsules,
)
from arx5_control.motion.pinocchio_backend import PinocchioKinematics, PinocchioUnavailableError
from arx5_control.motion.profiles import (
    TrajectorySample,
    cosine_fraction,
    cosine_segment,
    minimum_cosine_duration,
    minimum_smoothstep9_duration,
    round_trip,
    smoothstep9_fraction,
    via_point,
)
from arx5_control.motion.interpolation import (
    MAX_RESAMPLED_SAMPLES,
    resample_cubic_hermite,
    resample_quintic_hermite,
)

__all__ = [
    "DEFAULT_DERIVATIVE_SCALE_FLOOR",
    "MAX_CARTESIAN_WAYPOINTS",
    "MAX_SAMPLED_WAYPOINTS",
    "MIN_CARTESIAN_WAYPOINTS",
    "RuckigJointTrajectory",
    "SampledMotion",
    "TrajectorySample",
    "JointRuckigPlanner",
    "JointServoCommand",
    "MAX_PROGRESS_SAMPLES",
    "MAX_RESAMPLED_SAMPLES",
    "ScalarProgressTrajectory",
    "JacobianDiagnostics",
    "CartesianGeometryPath",
    "CartesianWaypoint",
    "CartesianPathMode",
    "CartesianPathSegment",
    "MotionContext",
    "MotionPlan",
    "MotionType",
    "Payload",
    "Pose",
    "ReferenceFrame",
    "ToolCenterPoint",
    "LinkCapsule",
    "LinkCollisionCheck",
    "PinocchioCollisionModel",
    "PinocchioKinematics",
    "PinocchioUnavailableError",
    "analyze_jacobian",
    "analyze_sampled_motion",
    "cosine_fraction",
    "cosine_segment",
    "derivative_scales",
    "minimum_cosine_duration",
    "minimum_smoothstep9_duration",
    "plan_blended_joint_trajectory",
    "plan_ruckig_joint_trajectory",
    "round_trip",
    "resample_cubic_hermite",
    "resample_quintic_hermite",
    "scaled_derivative_limits",
    "numerical_jacobian",
    "singularity_speed_scale",
    "plan_scalar_progress",
    "AabbObstacle",
    "CollisionCheck",
    "PrimitiveObstacle",
    "SphereObstacle",
    "check_tcp_path",
    "build_cartesian_path",
    "build_circular_path",
    "concatenate_cartesian_paths",
    "default_x5_capsules",
    "quaternion_distance_rad",
    "slerp_quaternion_xyzw",
    "quaternion_multiply_xyzw",
    "rotate_vector_xyzw",
    "smoothstep9_fraction",
    "via_point",
]
