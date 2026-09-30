"""Lazy factories for migrated algorithm and workflow components.

Module components expose their existing typed public API. They do not connect
devices or start an execution loop when selected. Stateful implementations retain
their own constructor arguments instead of hiding model/limits dependencies.
"""
from importlib import import_module


_MODULES = {
    'motion_math': 'arx5_control.motion',
    'offline_ruckig': 'arx5_control.motion.offline',
    'cartesian_path': 'arx5_control.motion.geometry',
    'tcp_collision': 'arx5_control.motion.collision',
    'link_collision': 'arx5_control.motion.link_collision',
    'jacobian': 'arx5_control.motion.jacobian',
    'eye_in_hand': 'hand_eye_calibration.eye_in_hand.adapter',
    'eye_to_hand': 'hand_eye_calibration.eye_to_hand.adapter',
    'calibration_quality': 'hand_eye_calibration.core.validation',
    'payload': 'arx5_control.payload_profiles',
    'gripper_calibration': 'arx5_control.gripper_calibration',
    'rebot_kinematics': 'reBotArm_control_py.kinematics',
    'rebot_dynamics': 'reBotArm_control_py.dynamics',
    'se3_path': 'reBotArm_control_py.trajectory.sampler',
    'clik': 'reBotArm_control_py.trajectory.clik_tracker',
    'cube_stack': 'arx5_backend.features.cube.stack',
    'cube_pick_place': 'experiments.cube_pick_rotate_place.planner',
    'groot_export': 'isaac_groot_data_collection.exporter',
    'groot_validation': 'isaac_groot_data_collection.validator',
    'groot_collection': 'isaac_groot_data_collection.raw_episode',
    'groot_policy': 'gr_zeroshot.policy',
    'groot_actions': 'gr_zeroshot.runtime',
    'groot_checkpoint': 'gr_zeroshot.checkpoint',
}


def __getattr__(name):
    if name not in _MODULES:
        raise AttributeError(name)
    def factory():
        return import_module(_MODULES[name])
    factory.__name__ = name
    return factory


def rebot_dm(**options):
    from reBotArm_control_py.actuator.rebotarm import RebotArm
    return RebotArm(hw_yaml='rebotarm_dm.yaml', **options)


def rebot_rs(**options):
    from reBotArm_control_py.actuator.rebotarm import RebotArm
    return RebotArm(hw_yaml='rebotarm_rs.yaml', **options)
