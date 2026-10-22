"""Hardware-independent motion algorithms, loaded only when selected.

Public imports remain compatible with the original motion package. Optional
kinematics/collision dependencies are not imported by the control planner.
"""

from importlib import import_module

_MODULE_EXPORTS = {
    'embodied.motion.limits': ('DEFAULT_DERIVATIVE_SCALE_FLOOR', 'MAX_CARTESIAN_WAYPOINTS', 'MIN_CARTESIAN_WAYPOINTS', 'SampledMotion', 'analyze_sampled_motion', 'derivative_scales', 'scaled_derivative_limits'),
    'embodied.motion.offline': ('MAX_SAMPLED_WAYPOINTS', 'RuckigJointTrajectory', 'plan_blended_joint_trajectory', 'plan_ruckig_joint_trajectory'),
    'embodied.motion.online': ('JointRuckigPlanner', 'JointServoCommand'),
    'embodied.motion.cartesian_time': ('MAX_PROGRESS_SAMPLES', 'ScalarProgressTrajectory', 'plan_scalar_progress'),
    'embodied.motion.collision': ('AabbObstacle', 'CollisionCheck', 'PrimitiveObstacle', 'SphereObstacle', 'check_tcp_path'),
    'embodied.motion.jacobian': ('JacobianDiagnostics', 'analyze_jacobian', 'numerical_jacobian', 'singularity_speed_scale'),
    'embodied.motion.geometry': ('CartesianGeometryPath', 'CartesianWaypoint', 'build_cartesian_path', 'build_circular_path', 'concatenate_cartesian_paths', 'quaternion_distance_rad', 'slerp_quaternion_xyzw'),
    'embodied.motion.contracts': ('CartesianPathMode', 'CartesianPathSegment', 'MotionContext', 'MotionPlan', 'MotionType', 'Payload', 'Pose', 'ReferenceFrame', 'ToolCenterPoint', 'quaternion_multiply_xyzw', 'rotate_vector_xyzw'),
    'embodied.motion.link_collision': ('LinkCapsule', 'LinkCollisionCheck', 'PinocchioCollisionModel', 'default_x5_capsules'),
    'embodied.motion.pinocchio_backend': ('PinocchioKinematics', 'PinocchioUnavailableError'),
    'embodied.motion.profiles': ('TrajectorySample', 'cosine_fraction', 'cosine_segment', 'minimum_cosine_duration', 'minimum_smoothstep9_duration', 'round_trip', 'smoothstep9_fraction', 'via_point'),
    'embodied.motion.interpolation': ('MAX_RESAMPLED_SAMPLES', 'resample_cubic_hermite', 'resample_quintic_hermite'),
}
_EXPORT_MODULE = {name: module for module, names in _MODULE_EXPORTS.items() for name in names}


def __getattr__(name):
    module = _EXPORT_MODULE.get(name)
    if module is None:
        raise AttributeError(name)
    value = getattr(import_module(module), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))


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
