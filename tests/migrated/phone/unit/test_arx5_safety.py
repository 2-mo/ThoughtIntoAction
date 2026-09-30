from __future__ import annotations

import pytest

from robot_teleop.core.frames import IDENTITY_QUATERNION
from robot_teleop.core.models import Pose, Twist, Vector3
from robot_teleop.robots.arx5_safety import (
    Arx5SafetyEnvelope,
    Arx5SafetyViolation,
    validate_interface_name,
)


def envelope() -> Arx5SafetyEnvelope:
    return Arx5SafetyEnvelope(
        position_lower_rad=(-1.0, -1.0),
        position_upper_rad=(1.0, 1.0),
        max_velocity_rad_s=(2.0, 2.0),
        max_acceleration_rad_s2=(3.0, 3.0),
        max_jerk_rad_s3=(4.0, 4.0),
        max_torque_nm=(5.0, 5.0),
        workspace_minimum_m=Vector3(0.1, -0.2, 0.2),
        workspace_maximum_m=Vector3(0.6, 0.2, 0.7),
        tcp_linear_velocity_max_m_s=0.3,
        tcp_angular_velocity_max_rad_s=0.4,
        singularity_warn=0.2,
        singularity_stop=0.1,
        evidence_source="fake safety evidence",
    )


def test_envelope_validates_feedback_and_a_full_joint_trajectory() -> None:
    limits = envelope()

    limits.validate_feedback(
        joint_position=(0.0, 0.5),
        joint_velocity=(1.0, -1.0),
        joint_torque=(2.0, -2.0),
    )
    limits.validate_joint_command(
        position=(0.2, -0.2),
        velocity=(1.0, -1.0),
        acceleration=(2.0, -2.0),
        jerk=(3.0, -3.0),
    )

    with pytest.raises(Arx5SafetyViolation, match="joint torque"):
        limits.validate_feedback(
            joint_position=(0.0, 0.5),
            joint_velocity=(1.0, -1.0),
            joint_torque=(5.1, 0.0),
        )
    with pytest.raises(Arx5SafetyViolation, match="acceleration"):
        limits.validate_joint_command(
            position=(0.2, -0.2),
            velocity=(1.0, -1.0),
            acceleration=(3.1, 0.0),
            jerk=(3.0, -3.0),
        )


def test_envelope_validates_cartesian_workspace_velocity_and_singularity() -> None:
    limits = envelope()
    limits.validate_pose(Pose(Vector3(0.4, 0.0, 0.3), IDENTITY_QUATERNION))
    limits.validate_twist(
        Twist(
            linear=Vector3(0.2, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.3),
        )
    )
    limits.validate_singularity(0.15)

    with pytest.raises(Arx5SafetyViolation, match="workspace"):
        limits.validate_pose(Pose(Vector3(0.7, 0.0, 0.3), IDENTITY_QUATERNION))
    with pytest.raises(Arx5SafetyViolation, match="linear velocity"):
        limits.validate_twist(
            Twist(
                linear=Vector3(0.31, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, 0.0),
            )
        )
    with pytest.raises(Arx5SafetyViolation, match="singularity"):
        limits.validate_singularity(0.09)


@pytest.mark.parametrize("value", ["", "../../can0", "can 0", "can0/evil"])
def test_interface_names_reject_paths_and_whitespace(value: str) -> None:
    with pytest.raises(Arx5SafetyViolation, match="simple CAN name"):
        validate_interface_name(value)


def test_envelope_requires_coherent_optional_limits_and_reports_evidence() -> None:
    with pytest.raises(ValueError, match="configured together"):
        Arx5SafetyEnvelope(
            position_lower_rad=(-1.0,),
            position_upper_rad=(1.0,),
            max_velocity_rad_s=(1.0,),
            max_acceleration_rad_s2=(1.0,),
            max_jerk_rad_s3=(1.0,),
            workspace_minimum_m=Vector3(0.0, 0.0, 0.0),
        )

    report = envelope().as_report()

    assert report["verified"] is False
    assert report["evidence_source"] == "fake safety evidence"
    assert report["joint_count"] == 2
