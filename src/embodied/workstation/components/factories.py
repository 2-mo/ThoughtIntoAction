"""Lazy factories for migrated algorithm and workflow components.

Module components expose their existing typed public API. They do not connect
devices or start an execution loop when selected. Stateful implementations retain
their own constructor arguments instead of hiding model/limits dependencies.
"""
from importlib import import_module


_MODULES = {
    'motion_math': 'embodied.motion',
    'offline_ruckig': 'embodied.motion.offline',
    'cartesian_path': 'embodied.motion.geometry',
    'tcp_collision': 'embodied.motion.collision',
    'link_collision': 'embodied.motion.link_collision',
    'jacobian': 'embodied.motion.jacobian',
    'eye_in_hand': 'embodied.calibration.hand_eye.eye_in_hand.adapter',
    'eye_to_hand': 'embodied.calibration.hand_eye.eye_to_hand.adapter',
    'calibration_quality': 'embodied.calibration.hand_eye.core.validation',
    'payload': 'legacy.arx5_control.payload_profiles',
    'gripper_calibration': 'legacy.arx5_control.gripper_calibration',
    'rebot_kinematics': 'embodied.robots.rebot.kinematics',
    'rebot_dynamics': 'embodied.robots.rebot.dynamics',
    'se3_path': 'embodied.robots.rebot.trajectory.sampler',
    'clik': 'embodied.robots.rebot.trajectory.clik_tracker',
    'cube_stack': 'embodied.console.features.cube.stack',
    'cube_pick_place': 'embodied.tasks.cube.planner',
    'groot_export': 'embodied.data.groot.exporter',
    'groot_validation': 'embodied.data.groot.validator',
    'groot_collection': 'embodied.data.groot.raw_episode',
    'groot_policy': 'embodied.policy.groot.policy',
    'groot_actions': 'embodied.policy.groot.runtime',
    'groot_checkpoint': 'embodied.policy.groot.checkpoint',
}


def __getattr__(name):
    if name not in _MODULES:
        raise AttributeError(name)
    def factory():
        return import_module(_MODULES[name])
    factory.__name__ = name
    return factory


def rebot_dm(**options):
    from embodied.robots.rebot.actuator.rebotarm import RebotArm
    return RebotArm(hw_yaml='rebotarm_dm.yaml', **options)


def rebot_rs(**options):
    from embodied.robots.rebot.actuator.rebotarm import RebotArm
    return RebotArm(hw_yaml='rebotarm_rs.yaml', **options)
