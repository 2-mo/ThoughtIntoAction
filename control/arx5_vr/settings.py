"""Independent robot and VR configuration, resolved from their own files."""
from dataclasses import dataclass, fields
import json
import math
import os
from pathlib import Path
from .types import RobotConfig

ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class VRConfig:
    sdk_library: str = "/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so"
    base_yaw_deg: float = 0.0
    grip_threshold: float = 0.7
    position_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass(frozen=True)
class Settings:
    robot: RobotConfig
    vr: VRConfig
    robot_config_path: str
    vr_config_path: str

    @property
    def sdk_library(self):
        return self.vr.sdk_library


def _read(path, schema):
    values = json.loads(path.read_text())
    if not isinstance(values, dict):
        raise ValueError(f'Configuration must be an object: {path}')
    unknown = set(values) - {field.name for field in fields(schema)}
    if unknown:
        raise ValueError(f'Unknown configuration keys in {path.name}: {sorted(unknown)}')
    return values


def _path(value, base):
    path = Path(value).expanduser()
    return str((path if path.is_absolute() else base / path).resolve())


def load_settings(path: str | Path | None = None, vr_path: str | Path | None = None) -> Settings:
    config_path = (Path(path).expanduser() if path else ROOT / 'config/robot.json').resolve()
    vr_config_path = (Path(vr_path).expanduser() if vr_path else ROOT / 'config/vr.json').resolve()
    values = _read(config_path, RobotConfig)
    for key in ('park_q', 'joint_min', 'joint_max', 'kp', 'kd', 'torque_max'):
        if key in values:
            values[key] = tuple(values[key])
    for key in ('model_path', 'tool_path'):
        if values.get(key):
            values[key] = _path(values[key], config_path.parent)
    robot = RobotConfig(**values)
    if robot.control_hz != 250:
        raise ValueError('control_hz must be 250 to match the native control pipeline')
    if not math.isfinite(robot.joint_step_limit) or robot.joint_step_limit <= 0:
        raise ValueError('joint_step_limit must be finite and positive')
    if not math.isfinite(robot.gripper_max_width) or not 0 < robot.gripper_max_width <= 0.1:
        raise ValueError('gripper_max_width must be positive and <= 0.1 metres')
    for key in ('park_q', 'joint_min', 'joint_max', 'kp', 'kd', 'torque_max'):
        vector = getattr(robot, key)
        if len(vector) != 6 or not all(math.isfinite(x) for x in vector):
            raise ValueError(f'{key} must have six finite values')
    if any(low >= high for low, high in zip(robot.joint_min, robot.joint_max)):
        raise ValueError('joint_min must be below joint_max')
    if any(not low <= q <= high for low, q, high in zip(robot.joint_min, robot.park_q, robot.joint_max)):
        raise ValueError('park_q is outside joint limits')
    if any(x < 0 for x in robot.kp + robot.kd) or any(x <= 0 for x in robot.torque_max):
        raise ValueError('gains must be nonnegative and torque limits positive')
    if not math.isfinite(robot.gripper_open_readout) or robot.gripper_open_readout == 0:
        raise ValueError('gripper_open_readout must be finite and nonzero')
    vr_values = _read(vr_config_path, VRConfig)
    library = os.environ.get('XROBOT_SDK_LIBRARY') or vr_values.get('sdk_library', VRConfig.sdk_library)
    vr_values['sdk_library'] = _path(library, vr_config_path.parent)
    if 'position_scale' in vr_values:
        vr_values['position_scale'] = tuple(vr_values['position_scale'])
    vr = VRConfig(**vr_values)
    if not math.isfinite(vr.base_yaw_deg):
        raise ValueError('base_yaw_deg must be finite')
    if not math.isfinite(vr.grip_threshold) or not 0 < vr.grip_threshold <= 1:
        raise ValueError('grip_threshold must be in (0, 1]')
    if len(vr.position_scale) != 3 or not all(math.isfinite(x) and x > 0 for x in vr.position_scale):
        raise ValueError('position_scale must contain three finite positive values')
    return Settings(robot, vr, str(config_path), str(vr_config_path))
