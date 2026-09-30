"""Data contracts shared by VR input, ARX control and simulation."""

from dataclasses import dataclass
from enum import Enum
import numpy as np


class Mode(str, Enum):
    DISABLED = "disabled"
    FREE_DRIVE = "free-drive"
    COMMAND_FOLLOWING = "command-following"


@dataclass(frozen=True)
class XRFrame:
    sequence: int
    sdk_timestamp_ns: int
    received_at: float
    pose: np.ndarray | None
    trigger: float = 0.0
    grip: float = 0.0
    primary: bool = False
    secondary: bool = False
    connected: bool = True
    valid: bool = True
    error: str = ""


@dataclass
class RobotState:
    q: np.ndarray
    dq: np.ndarray
    pose: np.ndarray
    gripper_width: float
    received_at: float
    mode: Mode = Mode.DISABLED
    fault: str = ""
    gripper_mode: Mode | None = None


@dataclass
class ControlCommand:
    mode: Mode
    pose_target: np.ndarray | None = None
    gripper_target: float | None = None
    hold: bool = False
    engaged: bool = False
    reason: str = ""
    gripper_mode: Mode | None = None
    joint_target: np.ndarray | None = None
    joint_step_limit: float | None = None
    reset_reference: bool = False


@dataclass(frozen=True)
class RobotConfig:
    model_path: str = ""
    tool_path: str = ""
    control_hz: int = 250
    gripper_max_width: float = 0.082
    park_q: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_min: tuple[float, ...] = (-3.14, -0.05, -0.1, -1.6, -1.57, -2.0)
    joint_max: tuple[float, ...] = (2.618, 3.50, 3.20, 1.55, 1.57, 2.0)
    kp: tuple[float, ...] = (20.0, 20.0, 20.0, 20.0, 20.0, 20.0)
    kd: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    torque_max: tuple[float, ...] = (30.0, 40.0, 30.0, 15.0, 10.0, 10.0)
    joint_step_limit: float = float(np.pi / 36)
    can_interface: str = "can0"
    gripper_open_readout: float = -3.4
    variant: str = "2025"
