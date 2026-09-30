from __future__ import annotations

from pathlib import Path

import pytest

from robot_teleop.kinematics.config import load_model_config
from robot_teleop.tools.phase5_safety_contract import (
    Phase5SafetyContractError,
    run_safety_contract,
)

MODEL_CONFIG = Path("configs/phone/robots/arx5.yaml")


def inspection_from_config() -> dict[str, object]:
    config = load_model_config(MODEL_CONFIG)
    limits = config.model.joint_limits
    return {
        "model": {
            "joint_count": limits.joint_count,
            "base_frame": config.model.base_frame,
            "end_effector_frame": config.model.end_effector_frame,
            "joint_position_min_rad": list(limits.lower_rad),
            "joint_position_max_rad": list(limits.upper_rad),
            "joint_velocity_max_rad_s": list(limits.velocity_max_rad_s),
            "joint_torque_max_nm": [8.0] * limits.joint_count,
            "end_effector_velocity_max": [0.6, 0.6, 0.6, 1.8, 1.8, 1.8],
        },
        "controller": {
            "period_s": 0.002,
            "background_send_recv": True,
            "interpolation_method": "linear",
        },
    }


def test_safety_contract_builds_candidate_without_sdk_or_can() -> None:
    report = run_safety_contract(
        model_config_path=MODEL_CONFIG,
        inspection=inspection_from_config(),
    )

    assert report["hardware_enabled"] is False
    assert report["controller_constructed"] is False
    assert report["can_interface_opened"] is False
    assert report["active_sdk_command_path_enabled"] is False
    assert report["candidate_verified"] is False
    assert report["static_model_matches"] is True
    envelope = report["candidate_envelope"]
    assert isinstance(envelope, dict)
    assert envelope["max_velocity_rad_s"] == pytest.approx((1.5,) * 6)
    assert envelope["max_torque_nm"] == pytest.approx((8.0,) * 6)
    assert envelope["workspace_minimum_m"] == pytest.approx((0.15, -0.35, 0.0))
    assert report["sdk_end_effector_velocity_max"] == pytest.approx(
        (0.6, 0.6, 0.6, 1.8, 1.8, 1.8)
    )
    assert "singularity thresholds" in " ".join(report["blocking_reasons"])


def test_safety_contract_rejects_missing_static_torque_evidence() -> None:
    inspection = inspection_from_config()
    model = inspection["model"]
    assert isinstance(model, dict)
    model.pop("joint_torque_max_nm")

    with pytest.raises(Phase5SafetyContractError, match="joint_torque_max_nm"):
        run_safety_contract(
            model_config_path=MODEL_CONFIG,
            inspection=inspection,
        )


def test_safety_contract_rejects_tcp_candidate_above_static_sdk_limit() -> None:
    with pytest.raises(Phase5SafetyContractError, match="linear velocity"):
        run_safety_contract(
            model_config_path=MODEL_CONFIG,
            tcp_linear_velocity_m_s=0.61,
            inspection=inspection_from_config(),
        )
