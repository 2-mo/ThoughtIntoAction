"""Run a hardware-free X5 Cartesian IK and online Ruckig preflight."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from embodied.phone.controllers.cartesian_ruckig import (
    CartesianRuckigServo,
    CartesianServoConfig,
    CartesianTargetRejected,
)
from embodied.phone.controllers.ruckig_servo import (
    JointServoLimits,
    JointServoState,
    RuckigJointServo,
)
from embodied.phone.core.frames import quaternion_angular_distance
from embodied.phone.kinematics.config import load_model_config
from embodied.phone.kinematics.pinocchio import PinocchioKinematics
from embodied.phone.tools.phase4_kinematics_check import build_relative_targets

ROOT = Path(__file__).resolve().parents[4]


def run_motion_preflight(
    *,
    model_config_path: Path,
    profile_path: Path,
    rate_hz: float = 200.0,
    joint_velocity_rad_s: float = 3.5,
    joint_acceleration_rad_s2: float = 0.70,
    joint_jerk_rad_s3: float = 3.0,
    maximum_steps_per_target: int = 5000,
) -> dict[str, object]:
    """Run a deterministic path without importing or constructing an SDK.

    The default derivative values mirror the reviewed ``fast`` offline profile
    in ``arx5_test/configs/teleoperation.json``. They are benchmark inputs, not
    verified limits for a physical arm.
    """

    _positive_finite("rate_hz", rate_hz)
    _positive_finite("joint_velocity_rad_s", joint_velocity_rad_s)
    _positive_finite("joint_acceleration_rad_s2", joint_acceleration_rad_s2)
    _positive_finite("joint_jerk_rad_s3", joint_jerk_rad_s3)
    if isinstance(maximum_steps_per_target, bool) or maximum_steps_per_target <= 0:
        raise ValueError("maximum_steps_per_target must be positive")

    model_config = load_model_config(model_config_path)
    kinematics = PinocchioKinematics(model_config.model)
    joint_count = model_config.model.joint_limits.joint_count
    limits = JointServoLimits(
        minimum_position=model_config.model.joint_limits.lower_rad,
        maximum_position=model_config.model.joint_limits.upper_rad,
        maximum_velocity=tuple(
            min(value, joint_velocity_rad_s)
            for value in model_config.model.joint_limits.velocity_max_rad_s
        ),
        maximum_acceleration=(joint_acceleration_rad_s2,) * joint_count,
        maximum_jerk=(joint_jerk_rad_s3,) * joint_count,
    )
    joint_servo = RuckigJointServo(
        JointServoState(
            position=model_config.reference_joint_position,
            velocity=(0.0,) * joint_count,
            acceleration=(0.0,) * joint_count,
        ),
        limits,
        period_s=1.0 / rate_hz,
    )
    servo = CartesianRuckigServo(
        kinematics=kinematics,
        joint_servo=joint_servo,
        config=CartesianServoConfig(
            maximum_joint_target_step_rad=model_config.maximum_branch_step_rad,
        ),
    )
    reference_pose = kinematics.forward_pose(model_config.reference_joint_position)
    targets = build_relative_targets(
        profile_path=profile_path,
        reference_robot_pose=reference_pose,
    )
    segments: list[dict[str, object]] = []
    all_finished = True
    all_accepted = True
    maximum_velocity = 0.0
    maximum_acceleration = 0.0
    maximum_jerk = 0.0
    for target in targets:
        try:
            decision = servo.set_target(target.pose)
        except CartesianTargetRejected as exc:
            all_accepted = False
            segments.append(
                {
                    "label": target.label,
                    "accepted": False,
                    "reason": str(exc),
                }
            )
            continue
        final_step = None
        sample_count = 0
        for sample_index in range(1, maximum_steps_per_target + 1):
            sample_count = sample_index
            final_step = servo.step()
            maximum_velocity = max(
                maximum_velocity,
                *(abs(value) for value in final_step.joint.velocity),
            )
            maximum_acceleration = max(
                maximum_acceleration,
                *(abs(value) for value in final_step.joint.acceleration),
            )
            maximum_jerk = max(
                maximum_jerk,
                *(abs(value) for value in final_step.joint.jerk),
            )
            if final_step.joint.finished:
                break
        finished = final_step is not None and final_step.joint.finished
        all_finished = all_finished and finished
        if final_step is None:
            position_error = math.inf
            orientation_error = math.inf
            final_position = None
        else:
            position_error = (
                final_step.tcp_pose.position - target.pose.position
            ).norm()
            orientation_error = quaternion_angular_distance(
                final_step.tcp_pose.orientation,
                target.pose.orientation,
            )
            final_position = final_step.joint.position
        segments.append(
            {
                "label": target.label,
                "accepted": True,
                "finished": finished,
                "samples": sample_count,
                "ik_iterations": decision.ik.iterations,
                "joint_target": decision.joint_target,
                "final_joint_position": final_position,
                "position_error_m": position_error,
                "orientation_error_rad": orientation_error,
                "minimum_singular_value": decision.jacobian.minimum_singular_value,
                "derivative_scale": decision.derivative_scale,
            }
        )

    return {
        "event": "phase5_motion_preflight",
        "hardware_enabled": False,
        "controller_constructed": False,
        "can_interface_opened": False,
        "all_targets_accepted": all_accepted,
        "all_segments_finished": all_finished,
        "model": {
            "name": model_config.model.name,
            "urdf": str(model_config.model.urdf_path),
            "base_frame": model_config.model.base_frame,
            "end_effector_frame": model_config.model.end_effector_frame,
            "joint_count": joint_count,
        },
        "motion_profile": {
            "source": "arx5_test/configs/teleoperation.json#profiles.fast",
            "rate_hz": rate_hz,
            "joint_velocity_rad_s": joint_velocity_rad_s,
            "joint_acceleration_rad_s2": joint_acceleration_rad_s2,
            "joint_jerk_rad_s3": joint_jerk_rad_s3,
            "note": "offline benchmark values; not physical-arm verification",
        },
        "maximum_observed": {
            "joint_velocity_rad_s": maximum_velocity,
            "joint_acceleration_rad_s2": maximum_acceleration,
            "joint_jerk_rad_s3": maximum_jerk,
        },
        "segments": segments,
    }


def _positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="phase5-motion-preflight")
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
    parser.add_argument("--rate-hz", type=float, default=200.0)
    parser.add_argument("--joint-velocity", type=float, default=3.5)
    parser.add_argument("--joint-acceleration", type=float, default=0.70)
    parser.add_argument("--joint-jerk", type=float, default=3.0)
    parser.add_argument("--maximum-steps-per-target", type=int, default=5000)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = run_motion_preflight(
            model_config_path=args.model_config,
            profile_path=args.profile,
            rate_hz=args.rate_hz,
            joint_velocity_rad_s=args.joint_velocity,
            joint_acceleration_rad_s2=args.joint_acceleration,
            joint_jerk_rad_s3=args.joint_jerk,
            maximum_steps_per_target=args.maximum_steps_per_target,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        raise SystemExit(f"phase5-motion-preflight: {exc}") from exc
    print(json.dumps(report, sort_keys=True))
    if report["all_targets_accepted"] and report["all_segments_finished"]:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
