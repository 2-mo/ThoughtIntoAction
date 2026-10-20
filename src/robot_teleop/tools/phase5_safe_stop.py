"""Validate the ARX5 SDK damping primitive behind explicit operator gates."""

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

from robot_core.device_lease import DeviceLease
from typing import Any, Protocol, cast

from robot_teleop.kinematics.config import load_model_config
from robot_teleop.robots.arx5 import Arx5BackendError

ROOT = Path(__file__).resolve().parents[3]


class SafeStopValidationError(RuntimeError):
    """Raised when the safe-stop validation cannot complete cleanly."""


class JointStateLike(Protocol):
    timestamp: float

    def pos(self) -> Sequence[float]: ...

    def vel(self) -> Sequence[float]: ...

    def torque(self) -> Sequence[float]: ...


class SafeStopController(Protocol):
    def get_joint_state(self) -> JointStateLike: ...

    def set_to_damping(self) -> None: ...


@dataclass(frozen=True, slots=True)
class SafeStopSample:
    index: int
    phase: str
    host_timestamp_ns: int
    sdk_timestamp_s: float
    joint_position_rad: tuple[float, ...]
    joint_velocity_rad_s: tuple[float, ...]
    joint_torque_nm: tuple[float, ...]

    @property
    def max_abs_joint_velocity_rad_s(self) -> float:
        return max((abs(value) for value in self.joint_velocity_rad_s), default=0.0)

    def as_report(self) -> dict[str, object]:
        return {
            "index": self.index,
            "phase": self.phase,
            "host_timestamp_ns": self.host_timestamp_ns,
            "sdk_timestamp_s": self.sdk_timestamp_s,
            "joint_position_rad": self.joint_position_rad,
            "joint_velocity_rad_s": self.joint_velocity_rad_s,
            "joint_torque_nm": self.joint_torque_nm,
            "max_abs_joint_velocity_rad_s": self.max_abs_joint_velocity_rad_s,
        }


SdkLoader = Callable[[], object]
ControllerFactory = Callable[[object, str, str, int], SafeStopController]
Sleeper = Callable[[float], None]
Clock = Callable[[], int]


def run_safe_stop_validation(
    *,
    model_config_path: Path,
    model: str = "X5",
    robot_interface: str | None = None,
    enable_hardware: bool = False,
    confirm_operator_present: bool = False,
    allow_stop_command: bool = False,
    post_stop_samples: int = 3,
    sample_interval_ms: float = 100.0,
    max_post_stop_velocity_rad_s: float = 1.0,
    sdk_loader: SdkLoader | None = None,
    controller_factory: ControllerFactory | None = None,
    sleeper: Sleeper = time.sleep,
    clock: Clock = time.monotonic_ns,
) -> dict[str, object]:
    """Send exactly one SDK damping request and observe feedback afterward.

    This is not the production Servo loop and it does not generate Cartesian,
    joint-position, or gripper commands. The only SDK command in the armed path
    is ``set_to_damping()``, guarded by an explicit stop-command flag.
    """

    if isinstance(post_stop_samples, bool) or post_stop_samples <= 0:
        raise ValueError("post_stop_samples must be positive")
    if not math.isfinite(sample_interval_ms) or sample_interval_ms < 0.0:
        raise ValueError("sample_interval_ms must be non-negative and finite")
    _positive_finite("max_post_stop_velocity_rad_s", max_post_stop_velocity_rad_s)
    model_config = load_model_config(model_config_path)
    joint_count = model_config.model.joint_limits.joint_count
    base_report: dict[str, object] = {
        "event": "phase5_safe_stop",
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
        "allow_stop_command": allow_stop_command,
        "robot_interface": robot_interface,
        "controller_constructed": False,
        "can_interface_requested": False,
        "command_writes_enabled": False,
        "servo_command_writes_enabled": False,
        "post_stop_samples_requested": post_stop_samples,
        "sample_interval_ms": sample_interval_ms,
        "max_post_stop_velocity_rad_s": max_post_stop_velocity_rad_s,
        "sdk_background_send_recv": True,
        "sdk_interpolation_method": "cubic",
    }
    if not enable_hardware:
        return {
            **base_report,
            "dry_run": True,
            "safe_stop_passed": False,
            "blocking_reasons": ("hardware gate not enabled",),
            "next_required_flags": (
                "--enable-hardware",
                "--confirm-operator-present",
                "--allow-stop-command",
                "--robot-interface",
            ),
        }
    if not confirm_operator_present:
        raise ValueError(
            "--confirm-operator-present is required when hardware is enabled"
        )
    if not allow_stop_command:
        raise ValueError("--allow-stop-command is required for safe-stop validation")
    interface = "" if robot_interface is None else robot_interface.strip()
    if not interface:
        raise ValueError("--robot-interface is required when hardware is enabled")

    loader = _load_sdk if sdk_loader is None else sdk_loader
    factory = (
        _default_controller_factory
        if controller_factory is None
        else controller_factory
    )
    controller: SafeStopController | None = None
    # Real SDK only: share the arm service / kernel CAN lease; injected test controllers skip it.
    lease = None
    samples: list[SafeStopSample] = []
    read_errors: list[str] = []
    stop_error: str | None = None
    stop_command_sent = False
    try:
        if controller_factory is None:
            lease = DeviceLease(interface)
        sdk = loader()
        controller = factory(sdk, model, interface, joint_count)
        try:
            samples.append(
                _sample_report(
                    0,
                    "before_stop",
                    controller.get_joint_state(),
                    joint_count=joint_count,
                    clock=clock,
                )
            )
        except (
            AttributeError,
            TypeError,
            ValueError,
            RuntimeError,
            SafeStopValidationError,
        ) as exc:
            read_errors.append(f"pre-stop feedback failed: {exc}")
        try:
            controller.set_to_damping()
            stop_command_sent = True
        except (AttributeError, TypeError, RuntimeError) as exc:
            stop_error = str(exc)
        for offset in range(post_stop_samples):
            if sample_interval_ms > 0.0:
                sleeper(sample_interval_ms / 1_000.0)
            try:
                samples.append(
                    _sample_report(
                        len(samples),
                        "after_stop",
                        controller.get_joint_state(),
                        joint_count=joint_count,
                        clock=clock,
                    )
                )
            except (
                AttributeError,
                TypeError,
                ValueError,
                RuntimeError,
                SafeStopValidationError,
            ) as exc:
                read_errors.append(f"post-stop feedback {offset} failed: {exc}")
                break
    except (
        Arx5BackendError,
        AttributeError,
        TypeError,
        ValueError,
        RuntimeError,
        SafeStopValidationError,
    ) as exc:
        read_errors.append(str(exc))
    finally:
        if lease is not None:
            lease.close()

    timestamp_progress = _timestamp_progress(samples)
    host_sample_progress = _host_sample_progress(samples)
    max_post_stop_velocity = max(
        (
            sample.max_abs_joint_velocity_rad_s
            for sample in samples
            if sample.phase == "after_stop"
        ),
        default=math.inf,
    )
    blocking_reasons: list[str] = []
    blocking_reasons.extend(read_errors)
    if stop_error is not None:
        blocking_reasons.append(f"safe-stop command failed: {stop_error}")
    if not stop_command_sent:
        blocking_reasons.append("safe-stop command was not accepted")
    if not timestamp_progress:
        blocking_reasons.append("SDK feedback timestamp did not advance")
    if not host_sample_progress:
        blocking_reasons.append("host sample timestamp did not advance")
    if max_post_stop_velocity > max_post_stop_velocity_rad_s:
        blocking_reasons.append("post-stop joint velocity exceeded threshold")

    return {
        **base_report,
        "dry_run": False,
        "controller_constructed": controller is not None,
        "can_interface_requested": True,
        "command_writes_enabled": True,
        "stop_command_sent": stop_command_sent,
        "stop_command_error": stop_error,
        "safe_stop_passed": not blocking_reasons,
        "blocking_reasons": tuple(blocking_reasons),
        "sample_count": len(samples),
        "timestamp_progress": timestamp_progress,
        "host_sample_progress": host_sample_progress,
        "maximum_post_stop_joint_velocity_rad_s": max_post_stop_velocity,
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
) -> SafeStopController:
    try:
        module = cast(Any, sdk)
        robot_config = module.RobotConfigFactory.get_instance().get_config(model)
        controller_config = module.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", joint_count
        )
        controller_config.background_send_recv = True
        controller_config.interpolation_method = "cubic"
        return cast(
            SafeStopController,
            module.Arx5JointController(robot_config, controller_config, interface),
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise SafeStopValidationError(
            f"could not construct ARX5 SDK controller: {exc}"
        ) from exc


def _sample_report(
    index: int,
    phase: str,
    feedback: JointStateLike,
    *,
    joint_count: int,
    clock: Clock,
) -> SafeStopSample:
    return SafeStopSample(
        index=index,
        phase=phase,
        host_timestamp_ns=clock(),
        sdk_timestamp_s=_finite_scalar("SDK timestamp", feedback.timestamp),
        joint_position_rad=_finite_sequence(
            "joint position", feedback.pos(), joint_count
        ),
        joint_velocity_rad_s=_finite_sequence(
            "joint velocity", feedback.vel(), joint_count
        ),
        joint_torque_nm=_finite_sequence(
            "joint torque", feedback.torque(), joint_count
        ),
    )


def _finite_sequence(
    name: str,
    value: Iterable[object],
    expected_length: int,
) -> tuple[float, ...]:
    try:
        result = tuple(float(cast(Any, item)) for item in value)
    except (TypeError, ValueError) as exc:
        raise SafeStopValidationError(f"{name} is not a numeric sequence") from exc
    if len(result) != expected_length or not all(
        math.isfinite(item) for item in result
    ):
        raise SafeStopValidationError(
            f"{name} must contain {expected_length} finite values"
        )
    return result


def _finite_scalar(name: str, value: object) -> float:
    try:
        result = float(cast(Any, value))
    except (TypeError, ValueError) as exc:
        raise SafeStopValidationError(f"{name} is not numeric") from exc
    if not math.isfinite(result):
        raise SafeStopValidationError(f"{name} must be finite")
    return result


def _timestamp_progress(samples: list[SafeStopSample]) -> bool:
    if len(samples) < 2:
        return True
    return all(
        current.sdk_timestamp_s > previous.sdk_timestamp_s
        for previous, current in pairwise(samples)
    )


def _host_sample_progress(samples: list[SafeStopSample]) -> bool:
    if len(samples) < 2:
        return True
    return all(
        current.host_timestamp_ns > previous.host_timestamp_ns
        for previous, current in pairwise(samples)
    )


def _positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="phase5-safe-stop")
    parser.add_argument(
        "--model-config",
        type=Path,
        default=ROOT / "configs/phone/robots/arx5.yaml",
    )
    parser.add_argument("--model", default="X5")
    parser.add_argument("--robot-interface")
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--confirm-operator-present", action="store_true")
    parser.add_argument("--allow-stop-command", action="store_true")
    parser.add_argument("--post-stop-samples", type=int, default=3)
    parser.add_argument("--sample-interval-ms", type=float, default=100.0)
    parser.add_argument("--max-post-stop-velocity-rad-s", type=float, default=1.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = run_safe_stop_validation(
            model_config_path=args.model_config,
            model=args.model,
            robot_interface=args.robot_interface,
            enable_hardware=args.enable_hardware,
            confirm_operator_present=args.confirm_operator_present,
            allow_stop_command=args.allow_stop_command,
            post_stop_samples=args.post_stop_samples,
            sample_interval_ms=args.sample_interval_ms,
            max_post_stop_velocity_rad_s=args.max_post_stop_velocity_rad_s,
        )
    except (Arx5BackendError, SafeStopValidationError, OSError, ValueError) as exc:
        raise SystemExit(f"phase5-safe-stop: {exc}") from exc
    print(json.dumps(report, sort_keys=True))
    if report.get("safe_stop_passed") is True or not args.enable_hardware:
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
