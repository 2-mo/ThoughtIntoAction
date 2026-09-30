"""Migrated ARX algorithms and optional device operations, loaded on demand."""
from importlib import import_module

_EXPORTS = {'ArmController': '.controller', 'ArmLimits': '.controller', 'ArmSnapshot': '.controller', 'BusyError': '.controller', 'CartesianMotionResult': '.controller', 'CartesianWaypointTrajectoryPreview': '.controller', 'ControlError': '.controller', 'ControllerMode': '.controller', 'GripperForceResult': '.controller', 'GripperMotionResult': '.controller', 'JointTrajectoryPreview': '.controller', 'MotionSettleError': '.controller', 'MotionResult': '.controller', 'SafetyConfig': '.controller', 'SafetyError': '.controller', 'CartesianPlan': '.cartesian', 'AabbObstacle': '.motion', 'CartesianGeometryPath': '.motion', 'CartesianPathMode': '.motion', 'CartesianPathSegment': '.motion', 'CartesianWaypoint': '.motion', 'CollisionCheck': '.motion', 'LinkCapsule': '.motion', 'LinkCollisionCheck': '.motion', 'MotionContext': '.motion', 'MotionPlan': '.motion', 'MotionType': '.motion', 'Payload': '.motion', 'PinocchioKinematics': '.motion', 'SphereObstacle': '.motion', 'Pose': '.motion', 'ReferenceFrame': '.motion', 'ToolCenterPoint': '.motion', 'build_cartesian_path': '.motion', 'build_circular_path': '.motion', 'concatenate_cartesian_paths': '.motion', 'check_tcp_path': '.motion', 'default_x5_capsules': '.motion', 'CartesianServo': '.servo', 'CartesianServoStatus': '.servo', 'GripperServo': '.servo', 'GripperServoStatus': '.servo', 'JointServo': '.servo', 'JointServoStatus': '.servo', 'JointRuckigPlanner': '.servo', 'JointServoCommand': '.servo', 'GripperState': '.teleoperation', 'TeleoperationConfig': '.teleoperation', 'TeleoperationProfile': '.teleoperation', 'TeleoperationTarget': '.teleoperation', 'WorkspaceBounds': '.teleoperation', 'load_teleoperation_config': '.teleoperation', 'TeleoperationServo': '.teleoperation_servo', 'TeleoperationServoStatus': '.teleoperation_servo'}
__all__ = list(_EXPORTS)

def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    value = getattr(import_module(_EXPORTS[name], __name__), name)
    globals()[name] = value
    return value
