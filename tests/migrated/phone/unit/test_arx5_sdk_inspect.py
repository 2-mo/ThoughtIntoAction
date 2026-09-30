from __future__ import annotations

from dataclasses import dataclass

import pytest

from robot_teleop.tools.arx5_sdk_inspect import (
    Arx5SdkInspectionError,
    _worker_payload,
    inspect_arx5_sdk,
)


@dataclass
class FakeRobotConfig:
    robot_model: str = "X5"
    joint_dof: int = 6
    joint_pos_min: tuple[float, ...] = (-1.0,) * 6
    joint_pos_max: tuple[float, ...] = (1.0,) * 6
    joint_vel_max: tuple[float, ...] = (2.0,) * 6
    joint_torque_max: tuple[float, ...] = (3.0,) * 6
    ee_vel_max: tuple[float, ...] = (1.0,) * 6
    base_link_name: str = "base_link"
    eef_link_name: str = "eef_link"
    urdf_path: str = "/models/X5.urdf"
    gripper_width: float = 0.08
    gripper_vel_max: float = 0.2
    gripper_torque_max: float = 1.0


@dataclass
class FakeControllerConfig:
    controller_type: str = "joint_controller"
    controller_dt: float = 0.002
    background_send_recv: bool = True
    interpolation_method: str = "cubic"
    default_preview_time: float = 0.1


class FakeRobotConfigFactory:
    @classmethod
    def get_instance(cls) -> FakeRobotConfigFactory:
        return cls()

    def get_config(self, model: str) -> FakeRobotConfig:
        assert model == "X5"
        return FakeRobotConfig()


class FakeControllerConfigFactory:
    @classmethod
    def get_instance(cls) -> FakeControllerConfigFactory:
        return cls()

    def get_config(
        self, controller_type: str, joint_count: int
    ) -> FakeControllerConfig:
        assert controller_type == "joint_controller"
        assert joint_count == 6
        return FakeControllerConfig()


class FakeSdk:
    __file__ = "/fake/arx5_interface.so"
    RobotConfigFactory = FakeRobotConfigFactory
    ControllerConfigFactory = FakeControllerConfigFactory


def test_static_inspection_never_constructs_a_controller() -> None:
    report = inspect_arx5_sdk(sdk_module=FakeSdk)

    assert report["hardware_enabled"] is False
    assert report["controller_constructed"] is False
    assert report["can_interface_opened"] is False
    model = report["model"]
    assert isinstance(model, dict)
    assert model["joint_count"] == 6
    assert model["base_frame"] == "base_link"
    controller = report["controller"]
    assert isinstance(controller, dict)
    assert controller["period_s"] == pytest.approx(0.002)


def test_static_inspection_rejects_bad_static_config() -> None:
    class InvalidRobotConfig(FakeRobotConfig):
        joint_dof = 0

    class InvalidFactory(FakeRobotConfigFactory):
        def get_config(self, model: str) -> InvalidRobotConfig:
            assert model == "X5"
            return InvalidRobotConfig(joint_dof=0)

    class InvalidSdk(FakeSdk):
        RobotConfigFactory = InvalidFactory

    with pytest.raises(Arx5SdkInspectionError, match="joint_dof"):
        inspect_arx5_sdk(sdk_module=InvalidSdk)


def test_worker_payload_ignores_native_stdout_chatter() -> None:
    payload = _worker_payload(
        "other stderr\n"
        '__robot_teleop_arx5_static_report__={"hardware_enabled": false}\n'
    )

    assert payload == {"hardware_enabled": False}

    with pytest.raises(Arx5SdkInspectionError, match="no structured"):
        _worker_payload("native message only\n")
