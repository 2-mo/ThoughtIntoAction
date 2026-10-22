"""Build a hardware-disabled ARX5 candidate safety-envelope report.

This tool reconciles the attributed offline X5 model, packaged SDK metadata,
and the conservative ``fine`` reference profile into one inspectable candidate
envelope.  It never constructs a controller or accepts a CAN interface.  The
output intentionally remains non-authoritative until the operator records
physical-arm, firmware, limit, gripper, and singularity evidence.
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any, cast

from embodied.devices.phone.core.frames import IDENTITY_QUATERNION
from embodied.devices.phone.core.models import Pose, Twist, Vector3
from embodied.devices.phone.kinematics.config import KinematicModelConfig, load_model_config
from embodied.devices.phone.robots.arx5_safety import Arx5SafetyEnvelope

from embodied.devices.phone.tools.arx5_sdk_inspect import Arx5SdkInspectionError, inspect_arx5_sdk_isolated
from embodied.devices.phone.tools.phase5_sdk_model_check import SdkModelMismatchError, compare_sdk_model_config

from embodied.core.project import PROJECT_ROOT as ROOT

_REFERENCE_SOURCE = "arx5_test/configs/teleoperation.json#profiles.fine"
_REFERENCE_WORKSPACE_MINIMUM = Vector3(0.15, -0.35, 0.0)
_REFERENCE_WORKSPACE_MAXIMUM = Vector3(0.65, 0.35, 0.55)


class Phase5SafetyContractError(RuntimeError):
    """Raised when static SDK evidence cannot form a bounded candidate envelope."""


def run_safety_contract(
    *,
    model_config_path: Path,
    model: str = "X5",
    joint_velocity_rad_s: float = 1.5,
    joint_acceleration_rad_s2: float = 0.30,
    joint_jerk_rad_s3: float = 1.0,
    tcp_linear_velocity_m_s: float = 0.03,
    tcp_angular_velocity_rad_s: float = 0.35,
    workspace_minimum_m: Vector3 = _REFERENCE_WORKSPACE_MINIMUM,
    workspace_maximum_m: Vector3 = _REFERENCE_WORKSPACE_MAXIMUM,
    inspection: dict[str, object] | None = None,
) -> dict[str, object]:
    """Report a conservative candidate ARX5 envelope without hardware access.

    The derivative and Cartesian values are reference candidates only.  Static
    SDK position, velocity, and torque values constrain the candidate; physical
    validation is still required before it can be injected into a live backend.
    """

    for name, value in (
        ("joint_velocity_rad_s", joint_velocity_rad_s),
        ("joint_acceleration_rad_s2", joint_acceleration_rad_s2),
        ("joint_jerk_rad_s3", joint_jerk_rad_s3),
        ("tcp_linear_velocity_m_s", tcp_linear_velocity_m_s),
        ("tcp_angular_velocity_rad_s", tcp_angular_velocity_rad_s),
    ):
        _positive_finite(name, value)
    config = load_model_config(model_config_path)
    static_inspection = (
        inspect_arx5_sdk_isolated(model) if inspection is None else inspection
    )
    static_model_report = compare_sdk_model_config(config, static_inspection)
    sdk_model = _mapping(static_inspection.get("model"), "inspection.model")
    sdk_torque = _vector(
        "inspection.model.joint_torque_max_nm",
        sdk_model.get("joint_torque_max_nm"),
        expected_length=config.model.joint_limits.joint_count,
    )
    sdk_end_effector_velocity = _vector(
        "inspection.model.end_effector_velocity_max",
        sdk_model.get("end_effector_velocity_max"),
        expected_length=6,
    )
    _validate_tcp_candidates(
        linear_limit=tcp_linear_velocity_m_s,
        angular_limit=tcp_angular_velocity_rad_s,
        sdk_end_effector_velocity=sdk_end_effector_velocity,
    )
    candidate = _candidate_envelope(
        config,
        joint_velocity_rad_s=joint_velocity_rad_s,
        joint_acceleration_rad_s2=joint_acceleration_rad_s2,
        joint_jerk_rad_s3=joint_jerk_rad_s3,
        tcp_linear_velocity_m_s=tcp_linear_velocity_m_s,
        tcp_angular_velocity_rad_s=tcp_angular_velocity_rad_s,
        workspace_minimum_m=workspace_minimum_m,
        workspace_maximum_m=workspace_maximum_m,
        sdk_torque=sdk_torque,
    )
    controller = _mapping(static_inspection.get("controller"), "inspection.controller")
    _positive_finite(
        "inspection.controller.period_s",
        _number("inspection.controller.period_s", controller.get("period_s")),
    )
    _boolean(
        "inspection.controller.background_send_recv",
        controller.get("background_send_recv"),
    )
    _text(
        "inspection.controller.interpolation_method",
        controller.get("interpolation_method"),
    )
    midpoint = Vector3(
        (workspace_minimum_m.x + workspace_maximum_m.x) / 2.0,
        (workspace_minimum_m.y + workspace_maximum_m.y) / 2.0,
        (workspace_minimum_m.z + workspace_maximum_m.z) / 2.0,
    )
    # Exercise every bounded numeric group without constructing an SDK or a
    # trajectory. This confirms that model/static values form a coherent shape.
    candidate.validate_feedback(
        joint_position=config.reference_joint_position,
        joint_velocity=(0.0,) * candidate.joint_count,
        joint_torque=(0.0,) * candidate.joint_count,
    )
    candidate.validate_joint_command(
        position=config.reference_joint_position,
        velocity=(0.0,) * candidate.joint_count,
        acceleration=(0.0,) * candidate.joint_count,
        jerk=(0.0,) * candidate.joint_count,
    )
    candidate.validate_pose(
        # A candidate workspace check only needs a valid canonical pose. The
        # static safety tool does not make any kinematics or command claim.
        _identity_pose(midpoint)
    )
    candidate.validate_twist(
        Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
    )
    candidate.validate_singularity(0.0)
    blocking_reasons = (
        "candidate limits are not verified on the physical arm",
        "arm model, firmware, units, and command modes require operator confirmation",
        "gripper calibration and command behavior remain unverified; "
        "keep gripper disabled",
        "workspace lower Z and singularity thresholds require measured "
        "robot-specific approval",
        "network-loss, tracking-loss, and clutch-release stops remain pending "
        "before speed increases",
    )
    return {
        "event": "phase5_safety_contract",
        "hardware_enabled": False,
        "controller_constructed": False,
        "can_interface_opened": False,
        "active_sdk_command_path_enabled": False,
        "candidate_verified": False,
        "state_readiness_evidence": "five-sample operator run passed on can0",
        "safe_stop_evidence": "gated SDK damping run passed on can0",
        "static_model_matches": static_model_report["matches"],
        "offline_model": static_model_report["offline_model"],
        "sdk_controller": {
            "period_s": controller["period_s"],
            "background_send_recv": controller["background_send_recv"],
            "interpolation_method": controller["interpolation_method"],
        },
        "sdk_end_effector_velocity_max": sdk_end_effector_velocity,
        "candidate_profile": {
            "source": _REFERENCE_SOURCE,
            "joint_velocity_rad_s": joint_velocity_rad_s,
            "joint_acceleration_rad_s2": joint_acceleration_rad_s2,
            "joint_jerk_rad_s3": joint_jerk_rad_s3,
            "tcp_linear_velocity_m_s": tcp_linear_velocity_m_s,
            "tcp_angular_velocity_rad_s": tcp_angular_velocity_rad_s,
            "workspace_minimum_m": workspace_minimum_m.as_tuple(),
            "workspace_maximum_m": workspace_maximum_m.as_tuple(),
            "note": "reference candidates only; not physical-arm limits",
        },
        "candidate_envelope": candidate.as_report(),
        "blocking_reasons": blocking_reasons,
    }


def _candidate_envelope(
    config: KinematicModelConfig,
    *,
    joint_velocity_rad_s: float,
    joint_acceleration_rad_s2: float,
    joint_jerk_rad_s3: float,
    tcp_linear_velocity_m_s: float,
    tcp_angular_velocity_rad_s: float,
    workspace_minimum_m: Vector3,
    workspace_maximum_m: Vector3,
    sdk_torque: tuple[float, ...],
) -> Arx5SafetyEnvelope:
    model_limits = config.model.joint_limits
    return Arx5SafetyEnvelope(
        position_lower_rad=model_limits.lower_rad,
        position_upper_rad=model_limits.upper_rad,
        max_velocity_rad_s=tuple(
            min(value, joint_velocity_rad_s)
            for value in model_limits.velocity_max_rad_s
        ),
        max_acceleration_rad_s2=(joint_acceleration_rad_s2,) * model_limits.joint_count,
        max_jerk_rad_s3=(joint_jerk_rad_s3,) * model_limits.joint_count,
        max_torque_nm=sdk_torque,
        workspace_minimum_m=workspace_minimum_m,
        workspace_maximum_m=workspace_maximum_m,
        tcp_linear_velocity_max_m_s=tcp_linear_velocity_m_s,
        tcp_angular_velocity_max_rad_s=tcp_angular_velocity_rad_s,
        evidence_source=(
            "static arx5-interface package plus "
            f"{_REFERENCE_SOURCE}; physical verification pending"
        ),
    )


def _identity_pose(position: Vector3) -> Pose:
    return Pose(position, IDENTITY_QUATERNION)


def _validate_tcp_candidates(
    *,
    linear_limit: float,
    angular_limit: float,
    sdk_end_effector_velocity: tuple[float, ...],
) -> None:
    if linear_limit > min(sdk_end_effector_velocity[:3]) + 1e-9:
        raise Phase5SafetyContractError(
            "candidate TCP linear velocity exceeds the static SDK limit"
        )
    if angular_limit > min(sdk_end_effector_velocity[3:]) + 1e-9:
        raise Phase5SafetyContractError(
            "candidate TCP angular velocity exceeds the static SDK limit"
        )


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise Phase5SafetyContractError(f"{name} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _vector(name: str, values: object, *, expected_length: int) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise Phase5SafetyContractError(f"{name} must be a numeric sequence")
    try:
        result = tuple(float(cast(Any, value)) for value in cast(Iterable[Any], values))
    except (TypeError, ValueError) as exc:
        raise Phase5SafetyContractError(f"{name} must be a numeric sequence") from exc
    if len(result) != expected_length or not all(
        math.isfinite(value) and value > 0.0 for value in result
    ):
        raise Phase5SafetyContractError(
            f"{name} must contain {expected_length} positive finite values"
        )
    return result


def _number(name: str, value: object) -> float:
    try:
        result = float(cast(Any, value))
    except (TypeError, ValueError) as exc:
        raise Phase5SafetyContractError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise Phase5SafetyContractError(f"{name} must be finite")
    return result


def _positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise Phase5SafetyContractError(f"{name} must be positive and finite")


def _boolean(name: str, value: object) -> bool:
    if not isinstance(value, bool):
        raise Phase5SafetyContractError(f"{name} must be boolean")
    return value


def _text(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise Phase5SafetyContractError(f"{name} must be a non-empty string")
    return value.strip()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="phase5-safety-contract")
    parser.add_argument(
        "--model-config",
        type=Path,
        default=ROOT / "configs/phone/robots/arx5.yaml",
    )
    parser.add_argument("--model", default="X5")
    parser.add_argument("--joint-velocity", type=float, default=1.5)
    parser.add_argument("--joint-acceleration", type=float, default=0.30)
    parser.add_argument("--joint-jerk", type=float, default=1.0)
    parser.add_argument("--tcp-linear-velocity", type=float, default=0.03)
    parser.add_argument("--tcp-angular-velocity", type=float, default=0.35)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = run_safety_contract(
            model_config_path=args.model_config,
            model=args.model,
            joint_velocity_rad_s=args.joint_velocity,
            joint_acceleration_rad_s2=args.joint_acceleration,
            joint_jerk_rad_s3=args.joint_jerk,
            tcp_linear_velocity_m_s=args.tcp_linear_velocity,
            tcp_angular_velocity_rad_s=args.tcp_angular_velocity,
        )
    except (
        Arx5SdkInspectionError,
        Phase5SafetyContractError,
        SdkModelMismatchError,
        ValueError,
    ) as exc:
        raise SystemExit(f"phase5-safety-contract: {exc}") from exc
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
