"""Run a hardware-free ARX5 model, FK, Jacobian, and bounded IK preflight."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from dataclasses import asdict
from pathlib import Path

from embodied.phone.config import load_profile
from embodied.phone.core.frames import (
    IDENTITY_QUATERNION,
    compose_pose,
    quaternion_from_axis_angle,
    relative_pose,
)
from embodied.phone.core.models import Pose, Vector3
from embodied.phone.kinematics.checks import KinematicTarget, check_pose_path
from embodied.phone.kinematics.config import load_model_config
from embodied.phone.kinematics.pinocchio import PinocchioKinematics

ROOT = Path(__file__).resolve().parents[4]


def build_relative_targets(
    *,
    profile_path: Path,
    reference_robot_pose: Pose,
) -> tuple[KinematicTarget, ...]:
    """Create a clutch/recenter trajectory relative to an offline TCP seed."""

    profile = load_profile(profile_path)
    input_reference = Pose(Vector3(0.0, 0.0, 0.0), IDENTITY_QUATERNION)
    input_current = input_reference
    robot_reference = reference_robot_pose
    targets: list[KinematicTarget] = []

    def append(label: str) -> Pose:
        target = compose_pose(
            robot_reference,
            profile.mapping.map_delta(relative_pose(input_reference, input_current)),
        )
        targets.append(KinematicTarget(label, target))
        return target

    append("clutch reference")
    input_current = Pose(Vector3(0.0, 0.0, 0.015), IDENTITY_QUATERNION)
    append("move input_z")
    input_current = Pose(Vector3(0.0, 0.0, 0.030), IDENTITY_QUATERNION)
    append("move input_z farther")
    input_current = Pose(Vector3(0.010, 0.0, 0.030), IDENTITY_QUATERNION)
    last_target = append("move input_x")

    robot_reference = last_target
    input_reference = input_current
    append("recenter at current pose")
    input_current = Pose(Vector3(0.010, 0.012, 0.030), IDENTITY_QUATERNION)
    last_target = append("move input_y after recenter")

    for label, axis in (
        ("rotate input_x", Vector3(1.0, 0.0, 0.0)),
        ("rotate input_y", Vector3(0.0, 1.0, 0.0)),
        ("rotate input_z", Vector3(0.0, 0.0, 1.0)),
    ):
        robot_reference = last_target
        input_reference = input_current
        input_current = compose_pose(
            input_reference,
            Pose(
                Vector3(0.0, 0.0, 0.0),
                quaternion_from_axis_angle(axis, 0.10),
            ),
        )
        last_target = append(label)

    return tuple(targets)


def write_kinematics_check(
    *,
    model_config_path: Path,
    profile_path: Path,
    output_dir: Path,
) -> dict[str, object]:
    """Write one deterministic no-hardware Phase 4 kinematics report."""

    model_config = load_model_config(model_config_path)
    kinematics = PinocchioKinematics(model_config.model)
    reference_pose = kinematics.forward_pose(model_config.reference_joint_position)
    targets = build_relative_targets(
        profile_path=profile_path,
        reference_robot_pose=reference_pose,
    )
    report = check_pose_path(
        kinematics,
        targets,
        reference_joint_position=model_config.reference_joint_position,
        maximum_branch_step_rad=model_config.maximum_branch_step_rad,
    )
    diagnostics = kinematics.jacobian_diagnostics(
        model_config.reference_joint_position,
    )
    result: dict[str, object] = {
        "event": "phase4_kinematics_check_written",
        "hardware_enabled": False,
        "model": {
            "name": model_config.model.name,
            "urdf": str(model_config.model.urdf_path),
            "base_frame": model_config.model.base_frame,
            "end_effector_frame": model_config.model.end_effector_frame,
            "joint_names": model_config.model.joint_names,
            "source_repository": model_config.model.source_repository,
            "source_commit": model_config.model.source_commit,
            "source_license": model_config.model.source_license,
        },
        "profile": str(profile_path),
        "reference_tcp_pose": {
            "position_m": reference_pose.position.as_tuple(),
            "orientation_xyzw": reference_pose.orientation.as_tuple(),
        },
        "jacobian": asdict(diagnostics),
        "path": report.as_dict(),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "phase4_kinematics_check.json"
    output_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    result["json"] = str(output_path)
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model-config",
        type=Path,
        default=ROOT / "configs/phone/robots/arx5.yaml",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default=ROOT / "configs/phone/profiles/arkit_mock.yaml",
    )
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = write_kinematics_check(
        model_config_path=args.model_config,
        profile_path=args.profile,
        output_dir=args.output_dir,
    )
    print(json.dumps(result, sort_keys=True))
    path = result["path"]
    assert isinstance(path, dict)
    return 0 if path["all_reachable"] and path["branch_continuous"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
