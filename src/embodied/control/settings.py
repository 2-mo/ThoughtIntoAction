"""Independent robot and VR configuration, resolved from their own files."""
from dataclasses import dataclass
from functools import cached_property
import json
import math
from pathlib import Path
from embodied.contracts.registry import Registry
from embodied.contracts.interfaces import RobotProfile
from embodied.contracts.paths import PROJECT_ROOT as ROOT, ROBOT_CONFIG




@dataclass(frozen=True)
class Settings:
    robot: RobotProfile
    robot_config_path: str
    vr_config_path: str

    @cached_property
    def vr(self):
        return Registry().create('input_profile', 'quest', runtime='control', path=self.vr_config_path)

    @property
    def sdk_library(self):
        return self.vr.sdk_library


def load_settings(path: str | Path | None = None, vr_path: str | Path | None = None) -> Settings:
    config_path = (Path(path).expanduser() if path else ROBOT_CONFIG).resolve()
    vr_config_path = (Path(vr_path).expanduser() if vr_path else ROOT / 'configs/inputs/quest.json').resolve()
    values = json.loads(config_path.read_text())
    if not isinstance(values, dict) or not isinstance(values.get('profile'), str):
        raise ValueError('Robot configuration must select a registered profile')
    selected = values.pop('profile')
    robot = Registry().create('robot_profile', selected, runtime='control', values=values, directory=config_path.parent)
    count = len(robot.park_q)
    if count < 1 or not math.isfinite(robot.control_hz) or robot.control_hz <= 0:
        raise ValueError('Robot profile requires joints and a positive control rate')
    for key in ('park_q', 'joint_min', 'joint_max', 'joint_velocity_max', 'kp', 'kd', 'torque_max'):
        vector = getattr(robot, key)
        if len(vector) != count or not all(math.isfinite(x) for x in vector):
            raise ValueError(f'{key} must have {count} finite values')
    if any(low >= high for low, high in zip(robot.joint_min, robot.joint_max)):
        raise ValueError('joint_min must be below joint_max')
    if any(not low <= q <= high for low, q, high in zip(robot.joint_min, robot.park_q, robot.joint_max)):
        raise ValueError('park_q is outside joint limits')
    if any(x <= 0 for x in robot.joint_velocity_max + robot.torque_max) or any(x < 0 for x in robot.kp + robot.kd):
        raise ValueError('Profile capabilities must be positive and gains nonnegative')
    for key in ('joint_step_limit', 'gripper_max_width', 'gripper_velocity_max_m_s'):
        if not math.isfinite(getattr(robot, key)) or getattr(robot, key) <= 0:
            raise ValueError(f'{key} must be finite and positive')
    return Settings(robot, str(config_path), str(vr_config_path))
