from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("pinocchio")
pytest.importorskip("ruckig")

from embodied.devices.phone.controllers.cartesian_ruckig import (
    CartesianRuckigServo,
    CartesianTargetRejected,
)
from embodied.devices.phone.controllers.ruckig_servo import (
    JointServoLimits,
    JointServoState,
    RuckigJointServo,
)
from embodied.devices.phone.core.models import Pose, Vector3
from embodied.devices.phone.kinematics.config import load_model_config
from embodied.devices.phone.kinematics.pinocchio import PinocchioKinematics
from embodied.devices.phone.tools.phase4_kinematics_check import build_relative_targets
from embodied.devices.phone.tools.phase5_motion_preflight import (
    run_motion_preflight,
)


def test_x5_cartesian_ik_ruckig_preflight_is_latest_only() -> None:
    report = run_motion_preflight(
        model_config_path=Path("configs/phone/robots/arx5.yaml"),
        profile_path=Path("configs/phone/profiles/arkit_mock.yaml"),
    )

    assert report["hardware_enabled"] is False
    assert report["controller_constructed"] is False
    assert report["can_interface_opened"] is False
    assert report["all_targets_accepted"] is True
    assert report["all_segments_finished"] is True
    segments = report["segments"]
    assert isinstance(segments, list)
    assert len(segments) == 9
    assert all(segment["accepted"] for segment in segments)
    assert all(segment["finished"] for segment in segments)


def test_rejected_cartesian_target_keeps_previous_target_and_brakes() -> None:
    config = load_model_config(Path("configs/phone/robots/arx5.yaml"))
    kinematics = PinocchioKinematics(config.model)
    joint_count = config.model.joint_limits.joint_count
    joint_servo = RuckigJointServo(
        JointServoState(
            config.reference_joint_position,
            (0.0,) * joint_count,
            (0.0,) * joint_count,
        ),
        JointServoLimits(
            minimum_position=config.model.joint_limits.lower_rad,
            maximum_position=config.model.joint_limits.upper_rad,
            maximum_velocity=(3.5,) * joint_count,
            maximum_acceleration=(0.7,) * joint_count,
            maximum_jerk=(3.0,) * joint_count,
        ),
        period_s=0.005,
    )
    servo = CartesianRuckigServo(
        kinematics=kinematics,
        joint_servo=joint_servo,
    )
    original = servo.target_pose
    invalid = Pose(Vector3(10.0, 10.0, 10.0), original.orientation)

    with pytest.raises(CartesianTargetRejected):
        servo.set_target(invalid)
    assert servo.target_pose == original

    next_target = build_relative_targets(
        profile_path=Path("configs/phone/profiles/arkit_mock.yaml"),
        reference_robot_pose=original,
    )[1]
    servo.set_target(next_target.pose)
    moving_samples = [servo.step() for _ in range(20)]
    assert any(
        abs(value) > 1e-6
        for sample in moving_samples
        for value in sample.joint.velocity
    )
    servo.brake()
    samples = [servo.step() for _ in range(200)]
    assert samples[-1].braking is True
    assert max(abs(value) for value in samples[-1].joint.velocity) < 1e-3
