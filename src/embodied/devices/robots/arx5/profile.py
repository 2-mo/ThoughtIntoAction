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
    gripper_velocity_max_m_s: float = 0.3
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

DUAL_ARM_IDS = ("left", "right")


@dataclass(frozen=True)
class DualArmProfile:
    """Two X5 arms in one kernel; each arm keeps its own CAN interface and limits.

    Concatenated vectors (q, limits, gains) are ordered left then right so the
    existing single-vector transports keep working with joint_count = 12.
    """

    left: RobotConfig
    right: RobotConfig

    @property
    def arms(self):
        return (self.left, self.right)

    @property
    def joint_count(self):
        return self.left.joint_count + self.right.joint_count

    @property
    def can_interfaces(self):
        return tuple(arm.can_interface for arm in self.arms)

    @property
    def can_interface(self):
        """Primary interface; per-arm code must use can_interfaces."""
        return self.left.can_interface

    @property
    def name(self):
        return f"{self.left.name} ×2"

    @property
    def variant(self):
        return self.left.variant

    @property
    def control_hz(self):
        return self.left.control_hz

    @property
    def model_path(self):
        return self.left.model_path

    @property
    def tool_path(self):
        return self.left.tool_path

    @property
    def model_id(self):
        return self.left.model_id

    @property
    def simulation_plugin(self):
        return self.left.simulation_plugin

    @property
    def hardware_plugin(self):
        return self.left.hardware_plugin

    @property
    def model_plugin(self):
        return "arx5-dual"

    @property
    def viewer_plugin(self):
        return self.left.viewer_plugin

    @property
    def joint_step_limit(self):
        return self.left.joint_step_limit

    @property
    def gripper_max_width(self):
        return self.left.gripper_max_width

    @property
    def gripper_open_readout(self):
        return self.left.gripper_open_readout

    @property
    def gripper_velocity_max_m_s(self):
        return self.left.gripper_velocity_max_m_s

    @property
    def gripper_position_effort_limit_nm(self):
        return self.left.gripper_position_effort_limit_nm

    @property
    def park_q(self):
        return tuple(self.left.park_q) + tuple(self.right.park_q)

    @property
    def joint_min(self):
        return tuple(self.left.joint_min) + tuple(self.right.joint_min)

    @property
    def joint_max(self):
        return tuple(self.left.joint_max) + tuple(self.right.joint_max)

    @property
    def joint_velocity_max(self):
        return tuple(self.left.joint_velocity_max) + tuple(self.right.joint_velocity_max)

    @property
    def kp(self):
        return tuple(self.left.kp) + tuple(self.right.kp)

    @property
    def kd(self):
        return tuple(self.left.kd) + tuple(self.right.kd)

    @property
    def torque_max(self):
        return tuple(self.left.torque_max) + tuple(self.right.torque_max)

    def describe_arms(self):
        """Per-arm metadata for clients that address one arm at a time."""
        return [
            {
                "id": f"arm{index + 1}",
                "side": arm_id,
                "label": f"机械臂 {index + 1}",
                "can_interface": profile.can_interface,
                "joint_count": profile.joint_count,
                "joint_min": list(profile.joint_min),
                "joint_max": list(profile.joint_max),
                "park_q": list(profile.park_q),
                "gripper_max_width": profile.gripper_max_width,
            }
            for index, (arm_id, profile) in enumerate(zip(DUAL_ARM_IDS, self.arms))
        ]


def load_dual_profile(values, directory):
    """Parse the dual-arm profile: two arm blocks, each a complete RobotConfig."""
    values = dict(values)
    values.pop("profile", None)
    unknown = set(values) - {"version", "arms"}
    if unknown:
        raise ValueError(f"Unknown dual profile keys: {sorted(unknown)}")
    if values.get("version") != 1:
        raise ValueError("Unsupported dual profile version")
    blocks = values.get("arms")
    if not isinstance(blocks, list) or len(blocks) != 2:
        raise ValueError("A dual profile requires exactly two arms")
    parsed = {}
    for block in blocks:
        if not isinstance(block, dict):
            raise ValueError("Each dual arm entry must be a table")
        block = dict(block)
        arm_id = block.pop("id", None)
        if arm_id not in DUAL_ARM_IDS:
            raise ValueError(f"Dual arm id must be one of {DUAL_ARM_IDS}")
        if arm_id in parsed:
            raise ValueError(f"Duplicate dual arm id: {arm_id}")
        parsed[arm_id] = load_profile(block, directory)
    interfaces = {profile.can_interface for profile in parsed.values()}
    if len(interfaces) != 2:
        raise ValueError("Each dual arm needs a distinct CAN interface")
    return DualArmProfile(left=parsed["left"], right=parsed["right"])

