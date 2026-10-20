from __future__ import annotations

from pathlib import Path

import pytest

from embodied.phone.core.frames import IDENTITY_QUATERNION
from embodied.phone.core.models import Pose, Vector3
from embodied.phone.kinematics.checks import KinematicTarget, check_pose_path
from embodied.phone.kinematics.config import (
    ModelConfigurationError,
    load_model_config,
)
from embodied.phone.kinematics.models import (
    IKResult,
    JacobianDiagnostics,
    JointLimits,
    RobotModel,
)
from embodied.phone.tools.phase4_kinematics_check import build_relative_targets


def test_x5_model_config_is_offline_and_matches_the_copied_urdf() -> None:
    config = load_model_config(Path("configs/phone/robots/arx5.yaml"))

    assert config.model.name == "arx5-x5"
    assert config.model.urdf_path.is_file()
    assert config.model.base_frame == "base_link"
    assert config.model.end_effector_frame == "eef_link"
    assert config.model.joint_names == (
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    )
    assert config.model.joint_limits.lower_rad == pytest.approx(
        (-3.14, -0.05, -0.1, -1.6, -1.57, -2.0)
    )
    assert config.reference_joint_position == pytest.approx(
        (0.0, 1.0, 1.0, 0.0, 0.0, 0.0)
    )


def test_model_config_rejects_hardware_enablement(tmp_path: Path) -> None:
    source = Path("configs/phone/robots/arx5.yaml").read_text(encoding="utf-8")
    config_path = tmp_path / "unsafe.yaml"
    config_path.write_text(
        source.replace("hardware_enabled: false", "hardware_enabled: true"),
        encoding="utf-8",
    )

    with pytest.raises(ModelConfigurationError, match="hardware_enabled"):
        load_model_config(config_path)


def test_relative_targets_preserve_clutch_and_recenter_continuity() -> None:
    reference = Pose(Vector3(0.4, 0.0, 0.3), IDENTITY_QUATERNION)
    targets = build_relative_targets(
        profile_path=Path("configs/phone/profiles/arkit_mock.yaml"),
        reference_robot_pose=reference,
    )

    positions = [target.pose.position.as_tuple() for target in targets]
    assert [target.label for target in targets] == [
        "clutch reference",
        "move input_z",
        "move input_z farther",
        "move input_x",
        "recenter at current pose",
        "move input_y after recenter",
        "rotate input_x",
        "rotate input_y",
        "rotate input_z",
    ]
    assert positions[0] == pytest.approx(reference.position.as_tuple())
    assert positions[1] == pytest.approx((0.4, 0.0, 0.33))
    assert positions[3] == pytest.approx((0.4, -0.02, 0.36))
    assert positions[4] == pytest.approx(positions[3])
    assert positions[5] == pytest.approx((0.424, -0.02, 0.36))


def test_path_check_uses_prior_solution_and_flags_branch_jumps() -> None:
    model = _fake_model()
    kinematics = _FakeKinematics(
        model,
        results=(
            _ik_result((0.1, 0.0)),
            _ik_result((0.8, 0.0)),
        ),
    )
    targets = (_target("first"), _target("second"))

    report = check_pose_path(
        kinematics,
        targets,
        reference_joint_position=(0.0, 0.0),
        maximum_branch_step_rad=0.2,
    )

    assert kinematics.seeds == [(0.0, 0.0), (0.1, 0.0)]
    assert report.all_reachable is True
    assert report.branch_continuous is False
    assert report.steps[0].branch_continuous is True
    assert report.steps[1].joint_step_rad == pytest.approx(0.7)
    assert report.steps[1].branch_continuous is False


def _fake_model() -> RobotModel:
    return RobotModel(
        name="fake",
        urdf_path=Path("fake.urdf"),
        base_frame="base",
        end_effector_frame="tool",
        joint_names=("joint_a", "joint_b"),
        joint_limits=JointLimits(
            lower_rad=(-1.0, -1.0),
            upper_rad=(1.0, 1.0),
            velocity_max_rad_s=(1.0, 1.0),
        ),
        source_repository="https://example.com/model",
        source_commit="test",
        source_license="MIT",
    )


def _target(label: str) -> KinematicTarget:
    return KinematicTarget(
        label,
        Pose(Vector3(0.0, 0.0, 0.0), IDENTITY_QUATERNION),
    )


def _ik_result(joints: tuple[float, float]) -> IKResult:
    return IKResult(
        reachable=True,
        joint_position=joints,
        iterations=1,
        position_error_m=0.0,
        orientation_error_rad=0.0,
        reason=None,
    )


class _FakeKinematics:
    def __init__(self, model: RobotModel, *, results: tuple[IKResult, ...]) -> None:
        self.model = model
        self.backend_name = "fake"
        self._results = iter(results)
        self.seeds: list[tuple[float, ...]] = []

    def forward_pose(self, joint_position: tuple[float, ...]) -> Pose:
        del joint_position
        return Pose(Vector3(0.0, 0.0, 0.0), IDENTITY_QUATERNION)

    def solve_ik(
        self,
        target_pose: Pose,
        seed_joint_position: tuple[float, ...],
        *,
        config: object = None,
    ) -> IKResult:
        del target_pose, config
        self.seeds.append(seed_joint_position)
        return next(self._results)

    def jacobian_diagnostics(
        self,
        joint_position: tuple[float, ...],
        *,
        orientation_scale_m: float = 0.10,
    ) -> JacobianDiagnostics:
        del joint_position
        return JacobianDiagnostics(
            singular_values=(1.0, 1.0),
            minimum_singular_value=1.0,
            condition_number=1.0,
            manipulability=1.0,
            orientation_scale_m=orientation_scale_m,
        )
