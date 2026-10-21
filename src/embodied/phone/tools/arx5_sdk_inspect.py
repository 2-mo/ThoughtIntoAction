"""Read static ARX5 SDK configuration without constructing a controller."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import subprocess
import sys
from collections.abc import Iterable, Sequence
from typing import Any, cast


class Arx5SdkInspectionError(RuntimeError):
    """Raised when static SDK metadata cannot be inspected safely."""


_WORKER_REPORT_PREFIX = "__robot_teleop_arx5_static_report__="


def inspect_arx5_sdk(
    model: str = "X5",
    *,
    sdk_module: Any | None = None,
) -> dict[str, object]:
    """Return one JSON-ready static SDK report without opening a CAN interface.

    This function deliberately does not accept an interface name and never
    resolves or constructs ``Arx5JointController``. The factories queried here
    expose packaged model/controller defaults only.
    """

    model_name = model.strip()
    if not model_name:
        raise ValueError("model must not be blank")
    sdk = _load_sdk() if sdk_module is None else sdk_module
    try:
        robot_config = sdk.RobotConfigFactory.get_instance().get_config(model_name)
        joint_count = _positive_integer("joint_dof", robot_config.joint_dof)
        controller_config = sdk.ControllerConfigFactory.get_instance().get_config(
            "joint_controller",
            joint_count,
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise Arx5SdkInspectionError(
            f"could not inspect static ARX5 SDK configuration: {exc}"
        ) from exc
    return {
        "event": "arx5_sdk_static_inspection",
        "hardware_enabled": False,
        "controller_constructed": False,
        "can_interface_opened": False,
        "sdk_module": str(getattr(sdk, "__file__", "unknown")),
        "model": {
            "name": _text("robot_model", robot_config.robot_model),
            "joint_count": joint_count,
            "joint_position_min_rad": _vector(
                "joint_pos_min", robot_config.joint_pos_min, joint_count
            ),
            "joint_position_max_rad": _vector(
                "joint_pos_max", robot_config.joint_pos_max, joint_count
            ),
            "joint_velocity_max_rad_s": _vector(
                "joint_vel_max", robot_config.joint_vel_max, joint_count
            ),
            "joint_torque_max_nm": _vector(
                "joint_torque_max", robot_config.joint_torque_max, joint_count
            ),
            "end_effector_velocity_max": _vector(
                "ee_vel_max", robot_config.ee_vel_max, 6
            ),
            "base_frame": _text("base_link_name", robot_config.base_link_name),
            "end_effector_frame": _text("eef_link_name", robot_config.eef_link_name),
            "urdf_path": _text("urdf_path", robot_config.urdf_path),
            "gripper_width_m": _finite_scalar(
                "gripper_width", robot_config.gripper_width
            ),
            "gripper_velocity_max": _finite_scalar(
                "gripper_vel_max", robot_config.gripper_vel_max
            ),
            "gripper_torque_max": _finite_scalar(
                "gripper_torque_max", robot_config.gripper_torque_max
            ),
        },
        "controller": {
            "type": _text("controller_type", controller_config.controller_type),
            "period_s": _positive_scalar(
                "controller_dt", controller_config.controller_dt
            ),
            "background_send_recv": _boolean(
                "background_send_recv", controller_config.background_send_recv
            ),
            "interpolation_method": _text(
                "interpolation_method", controller_config.interpolation_method
            ),
            "default_preview_time_s": _finite_scalar(
                "default_preview_time", controller_config.default_preview_time
            ),
        },
    }


def _load_sdk() -> Any:
    try:
        return importlib.import_module("arx5_interface")
    except ImportError as exc:
        raise Arx5SdkInspectionError(
            "arx5-interface is optional; run uv sync --extra arx5 on Ubuntu x86_64"
        ) from exc


def _text(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise Arx5SdkInspectionError(f"{name} must be a non-empty string")
    return value.strip()


def _positive_integer(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise Arx5SdkInspectionError(f"{name} must be a positive integer")
    return value


def _finite_scalar(name: str, value: object) -> float:
    try:
        number = float(cast(Any, value))
    except (TypeError, ValueError) as exc:
        raise Arx5SdkInspectionError(f"{name} must be numeric") from exc
    if not math.isfinite(number):
        raise Arx5SdkInspectionError(f"{name} must be finite")
    return number


def _positive_scalar(name: str, value: object) -> float:
    number = _finite_scalar(name, value)
    if number <= 0.0:
        raise Arx5SdkInspectionError(f"{name} must be positive")
    return number


def _boolean(name: str, value: object) -> bool:
    if not isinstance(value, bool):
        raise Arx5SdkInspectionError(f"{name} must be a boolean")
    return value


def _vector(name: str, values: object, expected_length: int) -> list[float]:
    if isinstance(values, (str, bytes)):
        raise Arx5SdkInspectionError(f"{name} must be a numeric sequence")
    try:
        sequence = tuple(float(value) for value in cast(Iterable[Any], values))
    except (TypeError, ValueError) as exc:
        raise Arx5SdkInspectionError(f"{name} must be a numeric sequence") from exc
    if len(sequence) != expected_length or not all(
        math.isfinite(value) for value in sequence
    ):
        raise Arx5SdkInspectionError(
            f"{name} must contain {expected_length} finite values"
        )
    return list(sequence)


def inspect_arx5_sdk_isolated(model: str = "X5") -> dict[str, object]:
    """Capture native SDK import chatter outside the public JSON stream."""

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "embodied.phone.tools.arx5_sdk_inspect",
            "--_worker",
            "--model",
            model,
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    payload = _worker_payload(completed.stderr)
    if completed.returncode != 0:
        detail = payload.get("error")
        if not isinstance(detail, str) or not detail:
            detail = "static SDK worker failed without a structured error"
        raise Arx5SdkInspectionError(detail)
    if "error" in payload:
        raise Arx5SdkInspectionError("static SDK worker reported an error")
    return payload


def _worker_payload(stderr: str) -> dict[str, object]:
    for line in reversed(stderr.splitlines()):
        if not line.startswith(_WORKER_REPORT_PREFIX):
            continue
        raw = line.removeprefix(_WORKER_REPORT_PREFIX)
        try:
            value = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise Arx5SdkInspectionError(
                "static SDK worker returned malformed JSON"
            ) from exc
        if not isinstance(value, dict):
            raise Arx5SdkInspectionError(
                "static SDK worker returned a non-object report"
            )
        return value
    raise Arx5SdkInspectionError("static SDK worker returned no structured report")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="arx5-sdk-inspect")
    parser.add_argument("--model", default="X5")
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args._worker:
        try:
            report = inspect_arx5_sdk(args.model)
        except (Arx5SdkInspectionError, ValueError) as exc:
            print(
                _WORKER_REPORT_PREFIX + json.dumps({"error": str(exc)}),
                file=sys.stderr,
                flush=True,
            )
            return 2
        print(
            _WORKER_REPORT_PREFIX + json.dumps(report, sort_keys=True),
            file=sys.stderr,
            flush=True,
        )
        return 0
    try:
        report = inspect_arx5_sdk_isolated(args.model)
    except (Arx5SdkInspectionError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
