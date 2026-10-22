"""Small structural interfaces; implementations own their SDK and lifecycle."""
from typing import Protocol, Any, Callable, Mapping
from .control import ControlCommand, RobotState


class RobotProfile(Protocol):
    """Resolved deployment facts; device loaders validate model-specific fields."""
    name: str
    control_hz: float
    park_q: tuple[float, ...]
    joint_min: tuple[float, ...]
    joint_max: tuple[float, ...]
    joint_velocity_max: tuple[float, ...]
    torque_max: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    joint_step_limit: float
    gripper_max_width: float
    gripper_velocity_max_m_s: float
    simulation_plugin: str
    hardware_plugin: str
    model_plugin: str
    viewer_plugin: str

    @property
    def model_id(self) -> str: ...


class RobotBackend(Protocol):
    """One process owns feedback, commands and the device connection."""
    def read_state(self, now: float) -> RobotState: ...
    def step(self, command: ControlCommand, now: float, **options) -> None: ...
    def telemetry(self) -> dict: ...
    def close(self) -> None: ...


class RobotDriver(Protocol):
    """Preparation and SDK-specific lifecycle stay outside the common worker."""
    def preflight(self, config: RobotProfile) -> dict: ...
    def resources(self, config: RobotProfile) -> tuple[str, ...]: ...
    def open(self, config: RobotProfile, **options) -> RobotBackend: ...
    def capabilities(self, config: RobotProfile, parameters: dict | None) -> dict: ...
    def snapshot(self, config: RobotProfile) -> dict: ...
    def shutdown(self, config: RobotProfile, backend: RobotBackend, *, initial,
                 publish: Callable, step: Callable, check: Callable) -> dict: ...


class PoseSource(Protocol):
    def start(self) -> None: ...
    def latest(self) -> Any: ...
    def stop(self) -> None: ...


class ControlSource(Protocol):
    """Own its input lifecycle and explicit lease; bounded shutdown in finally."""
    def run(self, duration: float = 0) -> None: ...


class NativeDevice(Protocol):
    """Supervisor factory for a device process publishing the native input ABI."""
    @staticmethod
    def validate(settings) -> None: ...
    @staticmethod
    def commands(configs, prefix) -> dict[str, list[str]]: ...


class NativeRouting(Protocol):
    """Supervisor factory for coordinator/router processes; retains native policy."""
    @staticmethod
    def commands(configs, prefix) -> dict[str, list[str]]: ...
    @classmethod
    def start(cls, children, configs, prefix, stopped) -> None: ...


class CommandIngress(Protocol):
    """Worker-local nonblocking candidate producer; no process or lease ownership."""
    status: dict
    def poll(self, state, now, epoch, selected) -> Any: ...
    def close(self) -> None: ...


class TargetController(Protocol):
    """Existing arx backend injection point, shared by hardware and simulation."""
    mode: Any
    gripper_mode: Any
    fault: str | None
    dq_target: Any
    gripper_target: float
    def update(self, command, q, now) -> Any: ...


class Camera(Protocol):
    metadata: dict
    def __enter__(self) -> 'Camera': ...
    def read(self) -> dict: ...
    def __exit__(self, *error) -> None: ...


class EpisodeExporter(Protocol):
    def prepare(self, path): ...
    def save_sidecar(self, path, rows) -> None: ...


class MotionPlanner(Protocol):
    def plan(self, request: dict, snapshot: dict, now: float) -> dict: ...
