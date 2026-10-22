"""A small, explicit non-ARX profile used to exercise platform contracts."""
from dataclasses import dataclass, fields
from pathlib import Path
import hashlib


@dataclass(frozen=True)
class Profile:
    model_path: str
    name: str = 'Reference7 (simulation only)'
    control_hz: int = 100
    park_q: tuple = (0., -.3, .2, .5, -.2, .3, 0.)
    joint_min: tuple = (-2.8,) * 7
    joint_max: tuple = (2.8,) * 7
    joint_velocity_max: tuple = (1.5,) * 7
    torque_max: tuple = (30.,) * 7
    kp: tuple = (120.,) * 7
    kd: tuple = (8.,) * 7
    joint_step_limit: float = .05
    has_gripper: bool = False
    gripper_max_width: float = 0.
    gripper_velocity_max_m_s: float = 0.
    simulation_plugin: str = 'reference7.sim'
    hardware_plugin: str = ''
    model_plugin: str = 'reference7'
    viewer_plugin: str = 'reference7'

    @property
    def joint_count(self):
        return len(self.park_q)

    @property
    def model_id(self):
        path = Path(self.model_path)
        return hashlib.sha256(path.read_bytes() + path.with_suffix('.urdf').read_bytes()).hexdigest()


def load_profile(values, directory):
    values = dict(values)
    if set(values) - {field.name for field in fields(Profile)}:
        raise ValueError('Unknown Reference7 profile fields')
    path = Path(values['model_path']).expanduser()
    values['model_path'] = str((path if path.is_absolute() else Path(directory) / path).resolve())
    for key in ('park_q', 'joint_min', 'joint_max', 'joint_velocity_max', 'torque_max', 'kp', 'kd'):
        if key in values:
            values[key] = tuple(values[key])
    config = Profile(**values)
    if config.joint_count != 7 or config.has_gripper is not False:
        raise ValueError('Reference7 has exactly seven revolute joints and no gripper')
    return config
