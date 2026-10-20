"""Hold the current ARX5 joint position with explicit command-write gates."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

from embodied.contracts.device_lease import DeviceLease
from typing import Any, Protocol, cast

from embodied.phone.kinematics.config import load_model_config
from embodied.phone.robots.arx5 import Arx5BackendError

ROOT = Path(__file__).resolve().parents[4]


class HoldCurrentError(RuntimeError):
    """Raised when the hold-current validation cannot complete."""


class JointStateLike(Protocol):
    timestamp: float
    gripper_pos: float
    gripper_vel: float
    gripper_torque: float

    def pos(self) -> Sequence[float]: ...

    def vel(self) -> Sequence[float]: ...

    def torque(self) -> Sequence[float]: ...


class HoldCurrentController(Protocol):
    def get_joint_state(self) -> JointStateLike: ...

    def get_timestamp(self) -> float: ...

    def set_joint_cmd(self, command: object) -> None: ...

    def set_to_damping(self) -> None: ...


@dataclass(frozen=True, slots=True)
class HoldCurrentSample:
    index: int
    host_timestamp_ns: int
    sdk_timestamp_s: float
    joint_position_rad: tuple[float, ...]
    joint_velocity_rad_s: tuple[float, ...]
    abs_drift_rad: tuple[float, ...]

    @property
    def max_abs_drift_rad(self) -> float:
        return max(self.abs_drift_rad, default=0.0)

    def as_report(self) -> dict[str, object]:
        return {
            "index": self.index,
            "host_timestamp_ns": self.host_timestamp_ns,
            "sdk_timestamp_s": self.sdk_timestamp_s,
            "joint_position_rad": self.joint_position_rad,
            "joint_velocity_rad_s": self.joint_velocity_rad_s,
            "abs_drift_rad": self.abs_drift_rad,
            "max_abs_drift_rad": self.max_abs_drift_rad,
        }


SdkLoader = Callable[[], object]
ControllerFactory = Callable[[object, str, str, int], HoldCurrentController]
Sleeper = Callable[[float], None]
Clock = Callable[[], int]


def run_hold_current(
    *,
    model_config_path: Path,
    model: str = "X5",
    robot_interface: str | None = None,
    enable_hardware: bool = False,
    confirm_operator_present: bool = False,
    allow_command_writes: bool = False,
    duration_s: float = 1.0,
    command_rate_hz: float = 50.0,
    max_joint_drift_rad: float = 0.05,
    sdk_loader: SdkLoader | None = None,
    controller_factory: ControllerFactory | None = None,
    sleeper: Sleeper = time.sleep,
    clock: Clock = time.monotonic_ns,
) -> dict[str, object]:
    """Hold the first measured joint position for a short explicit validation.

    This tool does not consume iPhone input, generate Cartesian targets, run IK,
    or run the production Servo loop. It only validates that the SDK accepts a
    current-position joint command and that measured feedback stays close to the
    captured position during a short hold window.
    """

    _positive_finite("duration_s", duration_s)
    _positive_finite("command_rate_hz", command_rate_hz)
    _positive_finite("max_joint_drift_rad", max_joint_drift_rad)
    model_config = load_model_config(model_config_path)
    joint_count = model_config.model.joint_limits.joint_count
    base_report: dict[str, object] = {
        "event": "phase5_hold_current",
        "model": model,
        "model_config": str(model_config_path),
        "offline_model": {
            "name": model_config.model.name,
            "joint_count": joint_count,
            "base_frame": model_config.model.base_frame,
            "end_effector_frame": model_config.model.end_effector_frame,
            "joint_names": model_config.model.joint_names,
        },
        "hardware_enabled": enable_hardware,
        "operator_confirmed": confirm_operator_present,
        "allow_command_writes": allow_command_writes,
        "command_writes_enabled": False,
        "robot_interface": robot_interface,
        "controller_constructed": False,
        "can_interface_requested": False,
        "duration_s": duration_s,
        "command_rate_hz": command_rate_hz,
        "max_joint_drift_rad": max_joint_drift_rad,
        "sdk_background_send_recv": True,
        "sdk_interpolation_method": "cubic",
    }
    if not enable_hardware:
        return {
            **base_report,
            "dry_run": True,
            "hold_passed": False,
            "blocking_reasons": ("hardware gate not enabled",),
            "next_required_flags": (
                "--enable-hardware",
                "--confirm-operator-present",
                "--allow-command-writes",
                "--robot-interface",
            ),
        }
    if not confirm_operator_present:
        raise ValueError(
            "--confirm-operator-present is required when hardware is enabled"
        )
    if not allow_command_writes:
        raise ValueError(
            "--allow-command-writes is required for hold-current validation"
        )
    interface = "" if robot_interface is None else robot_interface.strip()
    if not interface:
        raise ValueError("--robot-interface is required when hardware is enabled")

    loader = _load_sdk if sdk_loader is None else sdk_loader
    factory = (
        _default_controller_factory
        if controller_factory is None
        else controller_factory
    )
    controller: HoldCurrentController | None = None
    # Real SDK only: share the arm service / kernel CAN lease; injected test controllers skip it.
    lease = None
    samples: list[HoldCurrentSample] = []
    command_writes = 0
    read_error: str | None = None
    command_error: str | None = None
    damping_error: str | None = None
    hold_position: tuple[float, ...] | None = None
    try:
        if controller_factory is None:
            lease = DeviceLease(interface)
        sdk = loader()
        controller = factory(sdk, model, interface, joint_count)
        initial = controller.get_joint_state()
        hold_position = _finite_sequence("joint position", initial.pos(), joint_count)
        gripper_position = _finite_scalar("gripper position", initial.gripper_pos)
        period_s = 1.0 / command_rate_hz
        command_count = max(1, math.ceil(duration_s * command_rate_hz))
        for index in range(command_count):
            feedback = controller.get_joint_state()
            samples.append(
                _sample_report(
                    index,
                    feedback,
                    hold_position,
                    joint_count=joint_count,
                    clock=clock,
                )
            )
            try:
                command = _make_joint_command(
                    sdk,
                    hold_position,
                    gripper_position=gripper_position,
                    timestamp_s=_command_timestamp(controller, feedback),
                )
                controller.set_joint_cmd(command)
                command_writes += 1
            except (AttributeError, TypeError, ValueError, RuntimeError) as exc:
                command_error = str(exc)
                break
            if index < command_count - 1:
                sleeper(period_s)
        if command_error is None:
            feedback = controller.get_joint_state()
            samples.append(
                _sample_report(
                    len(samples),
                    feedback,
                    hold_position,
                    joint_count=joint_count,
                    clock=clock,
                )
            )
    except (
        Arx5BackendError,
        AttributeError,
        TypeError,
        ValueError,
        RuntimeError,
    ) as exc:
        read_error = str(exc)
    finally:
        if controller is not None:
            try:
                controller.set_to_damping()
            except (AttributeError, TypeError, RuntimeError) as exc:
                damping_error = str(exc)
        if lease is not None:
            lease.close()

    max_drift = max((sample.max_abs_drift_rad for sample in samples), default=math.inf)
    final_drift = samples[-1].max_abs_drift_rad if samples else math.inf
    timestamp_progress = _timestamp_progress(samples)
    blocking_reasons: list[str] = []
    if read_error is not None:
        blocking_reasons.append(read_error)
    if command_error is not None:
        blocking_reasons.append(command_error)
    if damping_error is not None:
        blocking_reasons.append(f"damping exit failed: {damping_error}")
    if not timestamp_progress:
        blocking_reasons.append("SDK feedback timestamp did not advance")
    if max_drift > max_joint_drift_rad:
        blocking_reasons.append("joint drift exceeded the configured threshold")

    return {
        **base_report,
        "dry_run": False,
        "controller_constructed": controller is not None,
        "can_interface_requested": True,
        "command_writes_enabled": True,
        "command_writes": command_writes,
        "hold_passed": not blocking_reasons,
        "blocking_reasons": tuple(blocking_reasons),
        "read_error": read_error,
        "command_error": command_error,
        "damping_error": damping_error,
        "hold_joint_position_rad": hold_position,
        "sample_count": len(samples),
        "timestamp_progress": timestamp_progress,
        "maximum_abs_joint_drift_rad": max_drift,
        "final_abs_joint_drift_rad": final_drift,
        "samples": [sample.as_report() for sample in samples],
    }


def _load_sdk() -> object:
    try:
        return importlib.import_module("arx5_interface")
    except ImportError as exc:
        raise Arx5BackendError(
            "arx5-interface is optional; run uv sync --extra arx5 on Ubuntu x86_64"
        ) from exc


def _default_controller_factory(
    sdk: object,
    model: str,
    interface: str,
    joint_count: int,
) -> HoldCurrentController:
    try:
        module = cast(Any, sdk)
        robot_config = module.RobotConfigFactory.get_instance().get_config(model)
        controller_config = module.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", joint_count
        )
        controller_config.background_send_recv = True
        controller_config.interpolation_method = "cubic"
        return cast(
            HoldCurrentController,
            module.Arx5JointController(robot_config, controller_config, interface),
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise HoldCurrentError(
            f"could not construct ARX5 SDK controller: {exc}"
        ) from exc


def _sample_report(
    index: int,
    feedback: JointStateLike,
    hold_position: tuple[float, ...],
    *,
    joint_count: int,
    clock: Clock,
) -> HoldCurrentSample:
    position = _finite_sequence("joint position", feedback.pos(), joint_count)
    velocity = _finite_sequence("joint velocity", feedback.vel(), joint_count)
    timestamp_s = _finite_scalar("SDK timestamp", feedback.timestamp)
    return HoldCurrentSample(
        index=index,
        host_timestamp_ns=clock(),
        sdk_timestamp_s=timestamp_s,
        joint_position_rad=position,
        joint_velocity_rad_s=velocity,
        abs_drift_rad=tuple(
            abs(current - target)
            for current, target in zip(position, hold_position, strict=True)
        ),
    )


def _make_joint_command(
    sdk: object,
    position: tuple[float, ...],
    *,
    gripper_position: float,
    timestamp_s: float,
) -> object:
    module = cast(Any, sdk)
    zeros = (0.0,) * len(position)
    command = module.JointState(
        _sdk_vector(position),
        _sdk_vector(zeros),
        _sdk_vector(zeros),
        float(gripper_position),
        0.0,
        0.0,
    )
    command.timestamp = float(timestamp_s)
    return cast(object, command)


def _sdk_vector(values: tuple[float, ...]) -> object:
    try:
        numpy = importlib.import_module("numpy")
    except ImportError:
        return list(values)
    return cast(Any, numpy).asarray(values, dtype=cast(Any, numpy).float64)


def _command_timestamp(
    controller: HoldCurrentController,
    feedback: JointStateLike,
) -> float:
    try:
        return _finite_scalar("controller timestamp", controller.get_timestamp())
    except (AttributeError, TypeError, ValueError, RuntimeError):
        return _finite_scalar("SDK timestamp", feedback.timestamp)


def _finite_sequence(
    name: str,
    value: Iterable[object],
    expected_length: int,
) -> tuple[float, ...]:
    try:
        result = tuple(float(cast(Any, item)) for item in value)
    except (TypeError, ValueError) as exc:
        raise HoldCurrentError(f"{name} is not a numeric sequence") from exc
    if len(result) != expected_length or not all(
        math.isfinite(item) for item in result
    ):
        raise HoldCurrentError(f"{name} must contain {expected_length} finite values")
    return result


def _finite_scalar(name: str, value: object) -> float:
    try:
        result = float(cast(Any, value))
    except (TypeError, ValueError) as exc:
        raise HoldCurrentError(f"{name} is not numeric") from exc
    if not math.isfinite(result):
        raise HoldCurrentError(f"{name} must be finite")
    return result


def _timestamp_progress(samples: list[HoldCurrentSample]) -> bool:
    if len(samples) < 2:
        return True
    return all(
        current.sdk_timestamp_s > previous.sdk_timestamp_s
        for previous, current in pairwise(samples)
    )


def _positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="phase5-hold-current")
    parser.add_argument(
        "--model-config",
        type=Path,
        default=ROOT / "configs/phone/robots/arx5.yaml",
    )
    parser.add_argument("--model", default="X5")
    parser.add_argument("--robot-interface")
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--confirm-operator-present", action="store_true")
    parser.add_argument("--allow-command-writes", action="store_true")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--command-rate-hz", type=float, default=50.0)
    parser.add_argument("--max-joint-drift-rad", type=float, default=0.05)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = run_hold_current(
            model_config_path=args.model_config,
            model=args.model,
            robot_interface=args.robot_interface,
            enable_hardware=args.enable_hardware,
            confirm_operator_present=args.confirm_operator_present,
            allow_command_writes=args.allow_command_writes,
            duration_s=args.duration,
            command_rate_hz=args.command_rate_hz,
            max_joint_drift_rad=args.max_joint_drift_rad,
        )
    except (Arx5BackendError, HoldCurrentError, OSError, ValueError) as exc:
        raise SystemExit(f"phase5-hold-current: {exc}") from exc
    print(json.dumps(report, sort_keys=True))
    if report.get("hold_passed") is True or not args.enable_hardware:
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
