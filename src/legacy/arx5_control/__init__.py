"""Migrated ARX algorithms and optional device operations, loaded on demand."""
from importlib import import_module

_EXPORTS = {'ArmController': '.controller', 'ArmLimits': '.controller', 'ArmSnapshot': '.controller', 'BusyError': '.controller', 'CartesianMotionResult': '.controller', 'CartesianWaypointTrajectoryPreview': '.controller', 'ControlError': '.controller', 'ControllerMode': '.controller', 'GripperForceResult': '.controller', 'GripperMotionResult': '.controller', 'JointTrajectoryPreview': '.controller', 'MotionSettleError': '.controller', 'MotionResult': '.controller', 'SafetyConfig': '.controller', 'SafetyError': '.controller', 'CartesianPlan': '.cartesian', 'AabbObstacle': 'embodied.motion', 'CartesianGeometryPath': 'embodied.motion', 'CartesianPathMode': 'embodied.motion', 'CartesianPathSegment': 'embodied.motion', 'CartesianWaypoint': 'embodied.motion', 'CollisionCheck': 'embodied.motion', 'LinkCapsule': 'embodied.motion', 'LinkCollisionCheck': 'embodied.motion', 'MotionContext': 'embodied.motion', 'MotionPlan': 'embodied.motion', 'MotionType': 'embodied.motion', 'Payload': 'embodied.motion', 'PinocchioKinematics': 'embodied.motion', 'SphereObstacle': 'embodied.motion', 'Pose': 'embodied.motion', 'ReferenceFrame': 'embodied.motion', 'ToolCenterPoint': 'embodied.motion', 'build_cartesian_path': 'embodied.motion', 'build_circular_path': 'embodied.motion', 'concatenate_cartesian_paths': 'embodied.motion', 'check_tcp_path': 'embodied.motion', 'default_x5_capsules': 'embodied.motion', 'CartesianServo': '.servo', 'CartesianServoStatus': '.servo', 'GripperServo': '.servo', 'GripperServoStatus': '.servo', 'JointServo': '.servo', 'JointServoStatus': '.servo', 'JointRuckigPlanner': '.servo', 'JointServoCommand': '.servo', 'GripperState': '.teleoperation', 'TeleoperationConfig': '.teleoperation', 'TeleoperationProfile': '.teleoperation', 'TeleoperationTarget': '.teleoperation', 'WorkspaceBounds': '.teleoperation', 'load_teleoperation_config': '.teleoperation', 'TeleoperationServo': '.teleoperation_servo', 'TeleoperationServoStatus': '.teleoperation_servo'}
__all__ = list(_EXPORTS)

def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    value = getattr(import_module(_EXPORTS[name], __name__), name)
    globals()[name] = value
    return value
