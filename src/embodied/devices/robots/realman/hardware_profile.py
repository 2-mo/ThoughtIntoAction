"""RM65 hardware deployment for the existing control worker."""
from dataclasses import dataclass, fields
import hashlib
import json
import math
from pathlib import Path

from .backend import load_config


@dataclass(frozen=True)
class Profile:
    name: str
    connection_path: str
    model_path: str
    control_hz: int = 25
    park_q: tuple = (0., 0., math.pi/2, 0., math.pi/2, 0.)
    joint_min: tuple = (-3.106, -2.2689, -2.356, -3.106, -2.234, -6.28)
    joint_max: tuple = (3.106, 2.2689, 2.356, 3.106, 2.234, 6.28)
    joint_velocity_max: tuple = tuple(math.radians(v) for v in (90, 90, 112.5, 112.5, 112.5, 112.5))
    torque_max: tuple = (60., 60., 30., 10., 10., 10.)
    kp: tuple = (0.,) * 6
    kd: tuple = (0.,) * 6
    joint_step_limit: float = math.radians(4.5)
    gripper_max_width: float = .07
    gripper_velocity_max_m_s: float = .035
    has_gripper: bool = True
    tool_offset_m: float = .14
    teleop_linear_speed_m_s: float = .25
    teleop_angular_speed_rad_s: float = .30
    simulation_plugin: str = ''
    hardware_plugin: str = 'realman.real'
    model_plugin: str = 'realman.hardware'
    viewer_plugin: str = 'realman'
    startup_behavior: str = '只连接并读取状态；不使能、不回零、不自动张开夹爪'
    shutdown_behavior: str = '请求控制器停止并关闭连接；不进入ARX阻尼、不回零'

    @property
    def joint_count(self):
        return 6

    @property
    def model_id(self):
        manifest = Path(self.model_path).with_suffix('.model.json')
        return hashlib.sha256(manifest.read_bytes() + json.dumps({
            'tcp_offset_m': self.tool_offset_m, 'joint_min': self.joint_min,
            'joint_max': self.joint_max, 'connection': load_config(self.connection_path)
        }, sort_keys=True).encode()).hexdigest()


def load_profile(values, directory):
    values = dict(values)
    if set(values) - {f.name for f in fields(Profile)}:
        raise ValueError('Unknown RM65 hardware profile fields')
    for key in ('connection_path', 'model_path'):
        path = Path(values[key])
        values[key] = str((path if path.is_absolute() else Path(directory)/path).resolve())
    for key in ('park_q', 'joint_min', 'joint_max', 'joint_velocity_max', 'torque_max', 'kp', 'kd'):
        if key in values:
            values[key] = tuple(values[key])
    config = Profile(**values)
    connection = load_config(config.connection_path)
    if (config.control_hz != connection['control_hz'] or config.hardware_plugin != 'realman.real'
            or config.model_plugin != 'realman.hardware' or config.gripper_max_width != .07
            or not config.has_gripper):
        raise ValueError('RM65 hardware requires its driver, model and EG2-4C2 nominal 70 mm contract')
    if connection['expected_tool_pose'] != [0, 0, round(config.tool_offset_m*1e6), 0, 0, 0]:
        raise ValueError('Hardware TCP offset disagrees with connection settings')
    return config
