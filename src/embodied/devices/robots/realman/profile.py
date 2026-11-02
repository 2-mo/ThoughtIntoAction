"""Explicit RM65 simulation deployment; it cannot select a hardware backend."""
from dataclasses import dataclass, fields
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Profile:
    model_path: str
    name: str
    control_hz: int
    park_q: tuple
    joint_min: tuple
    joint_max: tuple
    joint_velocity_max: tuple
    torque_max: tuple
    kp: tuple
    kd: tuple
    joint_step_limit: float
    gripper_max_width: float
    gripper_velocity_max_m_s: float
    gripper_kp: float
    gripper_kd: float
    gripper_force_limit_n: float
    tool_offset_m: float
    has_gripper: bool = True
    simulation_plugin: str = 'realman.sim'
    hardware_plugin: str = ''
    model_plugin: str = 'realman'
    viewer_plugin: str = 'realman'

    @property
    def joint_count(self):
        return 6

    @property
    def model_id(self):
        path = Path(self.model_path)
        manifest_path = path.with_suffix('.model.json')
        manifest = json.loads(manifest_path.read_text())
        root = path.parent.parent
        for name, digest in manifest['sha256'].items():
            if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
                raise ValueError(f'RM65 model artifact changed: {name}; rebuild scripts/realman/build_model.py')
        if manifest['tool_offset_m'] != self.tool_offset_m or manifest['simplified_gripper_width_m'] != self.gripper_max_width:
            raise ValueError('RM65 tool/gripper configuration changed; rebuild scripts/realman/build_model.py')
        return hashlib.sha256(manifest_path.read_bytes()).hexdigest()


def load_profile(values, directory):
    values = dict(values)
    if set(values) - {field.name for field in fields(Profile)}:
        raise ValueError('Unknown RM65 simulation profile fields')
    path = Path(values['model_path'])
    values['model_path'] = str((path if path.is_absolute() else Path(directory) / path).resolve())
    for key in ('park_q', 'joint_min', 'joint_max', 'joint_velocity_max', 'torque_max', 'kp', 'kd'):
        values[key] = tuple(values[key])
        if len(values[key]) != 6:
            raise ValueError(f'{key} requires six arm values')
    config = Profile(**values)
    if (not config.has_gripper or config.hardware_plugin or config.simulation_plugin != 'realman.sim'
            or config.model_plugin != 'realman' or config.viewer_plugin != 'realman'):
        raise ValueError('RM65 profile is simulation only with a simplified gripper')
    if not 0 < config.tool_offset_m < 1:
        raise ValueError('Invalid RM65 simulation TCP offset')
    if any(not math.isfinite(v) or v <= 0 for v in (config.gripper_kp, config.gripper_kd, config.gripper_force_limit_n)):
        raise ValueError("Gripper gains and force limit must be finite and positive")
    config.model_id
    return config
