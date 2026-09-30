from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("pinocchio")

from robot_teleop.kinematics.config import load_model_config
from robot_teleop.kinematics.pinocchio import PinocchioKinematics
from robot_teleop.tools.phase4_kinematics_check import write_kinematics_check


def test_x5_pinocchio_model_runs_fk_and_seeded_ik(tmp_path: Path) -> None:
    config = load_model_config(Path("configs/phone/robots/arx5.yaml"))
    kinematics = PinocchioKinematics(config.model)

    pose = kinematics.forward_pose(config.reference_joint_position)
    result = kinematics.solve_ik(pose, config.reference_joint_position)
    diagnostics = kinematics.jacobian_diagnostics(config.reference_joint_position)

    assert kinematics.backend_name.startswith("pinocchio-")
    assert result.reachable is True
    assert result.joint_position == pytest.approx(config.reference_joint_position)
    assert result.position_error_m < 1e-9
    assert diagnostics.minimum_singular_value > 0.0
    assert diagnostics.condition_number < 100.0


def test_phase4_kinematics_tool_exports_a_continuous_path(tmp_path: Path) -> None:
    result = write_kinematics_check(
        model_config_path=Path("configs/phone/robots/arx5.yaml"),
        profile_path=Path("configs/phone/profiles/arkit_mock.yaml"),
        output_dir=tmp_path,
    )

    path = result["path"]
    assert isinstance(path, dict)
    assert path["all_reachable"] is True
    assert path["branch_continuous"] is True
    assert len(path["steps"]) == 9
    assert Path(str(result["json"])).is_file()
