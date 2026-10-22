"""ARX X5 model configuration. No SDK is imported while reading a profile."""
from dataclasses import dataclass, fields
from pathlib import Path
import hashlib
import math
import numpy as np

@dataclass(frozen=True)
class RobotConfig:
    model_path: str = ""
    tool_path: str = ""
    control_hz: int = 250
    gripper_max_width: float = 0.082
    park_q: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_min: tuple[float, ...] = (-3.14, -0.05, -0.1, -1.6, -1.57, -2.0)
    joint_max: tuple[float, ...] = (2.618, 3.50, 3.20, 1.55, 1.57, 2.0)
    # Nominal capability for simulation/offline planning. Real managed motion
    # obtains its capability from the active SDK runtime configuration instead.
    joint_velocity_max: tuple[float, ...] = (5.0, 5.0, 5.5, 5.5, 5.0, 5.0)
    # ARX SDK joint_controller_6 defaults; preserve each axis's position and
    # damping gains instead of weakening every joint to a uniform kp=20.
    kp: tuple[float, ...] = (80.0, 70.0, 70.0, 30.0, 30.0, 20.0)
    kd: tuple[float, ...] = (2.0, 2.0, 2.0, 1.0, 1.0, 0.7)
    torque_max: tuple[float, ...] = (30.0, 40.0, 30.0, 15.0, 10.0, 10.0)
    joint_step_limit: float = float(np.pi / 36)
    can_interface: str = "can0"
    gripper_open_readout: float = -3.4
    variant: str = "2025"

    name: str = "ARX X5"
    simulation_plugin: str = "arx.sim"
    hardware_plugin: str = "arx.real"
    model_plugin: str = "arx5"
    viewer_plugin: str = "arx5"
    gripper_velocity_max_m_s: float = 0.04
    gripper_position_effort_limit_nm: float = 0.15

    @property
    def model_id(self):
        from embodied.contracts.paths import PROJECT_ROOT
        model = Path(self.model_path) if self.model_path else PROJECT_ROOT / 'assets/robots/x5/runtime/x5_2025.xml'
        return hashlib.sha256(model.with_suffix('.model.json').read_bytes()).hexdigest()

    @property
    def joint_count(self):
        return len(self.park_q)


def load_profile(values, directory):
    values = dict(values)
    unknown = set(values) - {field.name for field in fields(RobotConfig)}
    if unknown:
        raise ValueError(f"Unknown ARX profile keys: {sorted(unknown)}")
    for key in ("park_q", "joint_min", "joint_max", "joint_velocity_max", "kp", "kd", "torque_max"):
        if key in values:
            values[key] = tuple(values[key])
    for key in ("model_path", "tool_path"):
        if values.get(key):
            value = Path(values[key]).expanduser()
            values[key] = str((value if value.is_absolute() else Path(directory) / value).resolve())
    profile = RobotConfig(**values)
    if profile.control_hz != 250:
        raise ValueError("ARX X5 requires a 250 Hz control cycle")
    if profile.joint_count != 6:
        raise ValueError("ARX X5 requires six arm joints")
    if not math.isfinite(profile.gripper_open_readout) or profile.gripper_open_readout == 0:
        raise ValueError("ARX gripper_open_readout must be finite and nonzero")
    if not math.isfinite(profile.gripper_position_effort_limit_nm) or profile.gripper_position_effort_limit_nm <= 0:
        raise ValueError("ARX gripper_position_effort_limit_nm must be finite and positive")
    return profile
