from __future__ import annotations

from pathlib import Path

import pytest

from robot_teleop.kinematics.config import load_model_config
from robot_teleop.tools.phase5_sdk_model_check import (
    SdkModelMismatchError,
    compare_sdk_model_config,
)


def inspection_from_config() -> dict[str, object]:
    config = load_model_config(Path("configs/phone/robots/arx5.yaml"))
    limits = config.model.joint_limits
    return {
        "model": {
            "joint_count": limits.joint_count,
            "base_frame": config.model.base_frame,
            "end_effector_frame": config.model.end_effector_frame,
            "joint_position_min_rad": list(limits.lower_rad),
            "joint_position_max_rad": list(limits.upper_rad),
            "joint_velocity_max_rad_s": list(limits.velocity_max_rad_s),
        }
    }


def test_static_sdk_model_matches_attributed_offline_config() -> None:
    config = load_model_config(Path("configs/phone/robots/arx5.yaml"))

    report = compare_sdk_model_config(config, inspection_from_config())

    assert report["matches"] is True
    assert report["hardware_enabled"] is False
    assert report["controller_constructed"] is False


def test_static_sdk_model_mismatch_is_rejected() -> None:
    config = load_model_config(Path("configs/phone/robots/arx5.yaml"))
    inspection = inspection_from_config()
    model = inspection["model"]
    assert isinstance(model, dict)
    model["joint_velocity_max_rad_s"] = [9.0] * 6

    with pytest.raises(SdkModelMismatchError, match="joint_velocity"):
        compare_sdk_model_config(config, inspection)
