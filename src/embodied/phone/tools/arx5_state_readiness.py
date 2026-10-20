"""ARX5 state-only readiness check with explicit hardware gates."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections.abc import Callable, Sequence
from itertools import pairwise
from pathlib import Path

from embodied.phone.kinematics.config import load_model_config
from embodied.phone.kinematics.models import KinematicsUnavailableError
from embodied.phone.kinematics.pinocchio import PinocchioKinematics
from embodied.phone.robots.arx5 import (
    Arx5Backend,
    Arx5BackendConfig,
    Arx5BackendError,
    Arx5RawFeedback,
    ControllerFactory,
    SdkLoader,
    TcpPoseProvider,
)

ROOT = Path(__file__).resolve().parents[4]


def run_state_readiness(
    *,
    model_config_path: Path,
    model: str = "X5",
    robot_interface: str | None = None,
    enable_hardware: bool = False,
    state_only: bool = False,
    confirm_operator_present: bool = False,
    samples: int = 5,
    sample_interval_ms: float = 100.0,
    sdk_loader: SdkLoader | None = None,
    controller_factory: ControllerFactory | None = None,
    tcp_pose_provider: TcpPoseProvider | None = None,
    clock: Callable[[], int] | None = None,
) -> dict[str, object]:
    """Run a dry or explicitly armed ARX5 state-readiness check.

    The dry path performs no SDK import, controller construction, CAN access, or
    command write. The hardware path is intentionally limited to controller
    construction plus state reads; the backend is configured not to call the
    damping/stop primitive during this readiness step.
    """

    if isinstance(samples, bool) or samples <= 0:
        raise ValueError("samples must be positive")
    if not math.isfinite(sample_interval_ms) or sample_interval_ms < 0.0:
        raise ValueError("sample_interval_ms must be non-negative and finite")
    model_config = load_model_config(model_config_path)
    base_report: dict[str, object] = {
        "event": "arx5_state_readiness",
        "model": model,
        "model_config": str(model_config_path),
        "offline_model": {
            "name": model_config.model.name,
            "joint_count": model_config.model.joint_limits.joint_count,
            "base_frame": model_config.model.base_frame,
            "end_effector_frame": model_config.model.end_effector_frame,
            "joint_names": model_config.model.joint_names,
        },
        "hardware_enabled": enable_hardware,
        "state_only": state_only,
        "operator_confirmed": confirm_operator_present,
        "robot_interface": robot_interface,
        "controller_constructed": False,
        "sdk_state_reads": 0,
        "command_writes_enabled": False,
        "disconnect_damping_enabled": False,
        "damping_on_connect_error": False,
        "feedback_fault_damping_enabled": False,
        "sdk_background_send_recv": True,
        "sdk_interpolation_method": "cubic",
        "sdk_lifecycle_warning": (
            "vendor SDK controller construction may open CAN, clear internal "
            "controller state, start background IO, and set damping on SDK exit; "
            "this tool sends no framework servo targets"
        ),
    }
    if not enable_hardware:
        return {
            **base_report,
            "ready_for_hardware_state_read": False,
            "dry_run": True,
            "can_interface_requested": False,
            "readiness_passed": False,
            "blocking_reasons": ("hardware gate not enabled",),
            "next_required_flags": (
                "--enable-hardware",
                "--state-only",
                "--confirm-operator-present",
                "--robot-interface",
            ),
        }
    if not state_only:
        raise ValueError("--state-only is required for ARX5 readiness hardware access")
    interface = "" if robot_interface is None else robot_interface.strip()
    if not interface:
        raise ValueError("--robot-interface is required when hardware is enabled")
    if not confirm_operator_present:
        raise ValueError(
            "--confirm-operator-present is required when hardware is enabled"
        )
    provider = tcp_pose_provider
    if provider is None:
        try:
            kinematics = PinocchioKinematics(model_config.model)
        except KinematicsUnavailableError as exc:
            raise Arx5BackendError(
                f"could not build state-only FK provider: {exc}"
            ) from exc
        provider = kinematics.forward_pose

    backend_config = Arx5BackendConfig(
        model=model,
        interface_name=interface,
        hardware_enabled=True,
        joint_count=model_config.model.joint_limits.joint_count,
        disconnect_damping_enabled=False,
        damping_on_connect_error=False,
        feedback_fault_damping_enabled=False,
    )
    backend_clock = time.monotonic_ns if clock is None else clock
    if sdk_loader is None and controller_factory is None:
        backend = Arx5Backend(
            backend_config,
            tcp_pose_provider=provider,
            clock=backend_clock,
        )
    elif sdk_loader is None:
        assert controller_factory is not None
        backend = Arx5Backend(
            backend_config,
            controller_factory=controller_factory,
            tcp_pose_provider=provider,
            clock=backend_clock,
        )
    elif controller_factory is None:
        assert sdk_loader is not None
        backend = Arx5Backend(
            backend_config,
            sdk_loader=sdk_loader,
            tcp_pose_provider=provider,
            clock=backend_clock,
        )
    else:
        backend = Arx5Backend(
            backend_config,
            sdk_loader=sdk_loader,
            controller_factory=controller_factory,
            tcp_pose_provider=provider,
            clock=backend_clock,
        )
    state_reports: list[dict[str, object]] = []
    read_error: str | None = None
    controller_constructed = False
    try:
        backend.connect()
        controller_constructed = True
        for index in range(samples):
            if index > 0 and sample_interval_ms > 0.0:
                time.sleep(sample_interval_ms / 1_000.0)
            state = backend.state()
            raw = backend.raw_feedback
            if raw is None:
                raise Arx5BackendError("ARX5 backend did not retain raw feedback")
            state_reports.append(_state_report(index, state.timestamp_ns, raw))
    except Arx5BackendError as exc:
        read_error = str(exc)
    finally:
        backend.disconnect()
    timestamp_progress = _timestamp_progress(state_reports)
    host_sample_progress = _host_sample_progress(state_reports)
    blocking_reasons: list[str] = []
    if read_error is not None:
        blocking_reasons.append(read_error)
    if not timestamp_progress:
        blocking_reasons.append("SDK feedback timestamp did not advance")
    if not host_sample_progress:
        blocking_reasons.append("host sample timestamp did not advance")
    return {
        **base_report,
        "dry_run": False,
        "ready_for_hardware_state_read": not blocking_reasons,
        "readiness_passed": not blocking_reasons,
        "blocking_reasons": tuple(blocking_reasons),
        "can_interface_requested": True,
        "controller_constructed": controller_constructed,
        "read_error": read_error,
        "sdk_state_reads": len(state_reports),
        "samples": state_reports,
        "timestamp_progress": timestamp_progress,
        "host_sample_progress": host_sample_progress,
    }


def _state_report(
    index: int,
    host_timestamp_ns: int,
    raw: Arx5RawFeedback,
) -> dict[str, object]:
    return {
        "index": index,
        "host_timestamp_ns": host_timestamp_ns,
        "sdk_timestamp_s": raw.sdk_timestamp_s,
        "joint_position_rad": raw.joint_position,
        "joint_velocity_rad_s": raw.joint_velocity,
        "joint_torque_nm": raw.joint_torque,
        "gripper_position_raw": raw.gripper_position_raw,
        "gripper_velocity_raw": raw.gripper_velocity_raw,
        "gripper_torque_raw": raw.gripper_torque_raw,
    }


def _timestamp_progress(samples: list[dict[str, object]]) -> bool:
    if len(samples) < 2:
        return True
    values: list[float] = []
    for sample in samples:
        value = sample["sdk_timestamp_s"]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise Arx5BackendError("state readiness report has invalid timestamps")
        values.append(float(value))
    return all(current > previous for previous, current in pairwise(values))


def _host_sample_progress(samples: list[dict[str, object]]) -> bool:
    if len(samples) < 2:
        return True
    values: list[int] = []
    for sample in samples:
        value = sample["host_timestamp_ns"]
        if isinstance(value, bool) or not isinstance(value, int):
            raise Arx5BackendError("state readiness report has invalid host timestamps")
        values.append(value)
    return all(current > previous for previous, current in pairwise(values))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="arx5-state-readiness")
    parser.add_argument(
        "--model-config",
        type=Path,
        default=ROOT / "configs/phone/robots/arx5.yaml",
    )
    parser.add_argument("--model", default="X5")
    parser.add_argument("--robot-interface")
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--state-only", action="store_true")
    parser.add_argument("--confirm-operator-present", action="store_true")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--sample-interval-ms", type=float, default=100.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = run_state_readiness(
            model_config_path=args.model_config,
            model=args.model,
            robot_interface=args.robot_interface,
            enable_hardware=args.enable_hardware,
            state_only=args.state_only,
            confirm_operator_present=args.confirm_operator_present,
            samples=args.samples,
            sample_interval_ms=args.sample_interval_ms,
        )
    except (Arx5BackendError, OSError, ValueError) as exc:
        raise SystemExit(f"arx5-state-readiness: {exc}") from exc
    print(json.dumps(report, sort_keys=True))
    if report.get("readiness_passed") is True or not args.enable_hardware:
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
