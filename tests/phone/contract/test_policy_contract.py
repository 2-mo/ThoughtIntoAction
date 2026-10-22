import math
from collections.abc import Callable
from dataclasses import replace

import pytest

from embodied.devices.phone.core.capabilities import InputCapabilities
from embodied.devices.phone.core.frames import (
    quaternion_angular_distance,
    quaternion_from_axis_angle,
)
from embodied.devices.phone.core.mapping import AxisMapping, MappingScales, PoseMapping
from embodied.devices.phone.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    RobotState,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.devices.phone.core.pipeline import TeleopPipeline
from embodied.devices.phone.core.safety import TargetSafetyLimits
from embodied.devices.phone.core.state_machine import TeleopState
from embodied.devices.phone.policies.no_compliance import NoCompliancePolicy
from embodied.devices.phone.policies.relative_pose import RelativePosePolicy
from embodied.devices.phone.runtime.mapping_settings import MappingSettingsStore

IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)
ZERO_POSITION = Vector3(0.0, 0.0, 0.0)
CAPABILITIES = InputCapabilities(
    pose_6dof=True,
    linear_velocity=False,
    angular_velocity=False,
    analog_gripper=True,
    haptics=False,
    dual_hand=False,
)


def make_robot(pose: Pose | None = None, *, timestamp_ns: int = 1) -> RobotState:
    return RobotState(
        timestamp_ns=timestamp_ns,
        joint_position=(0.0,) * 6,
        joint_velocity=(0.0,) * 6,
        joint_torque=None,
        tcp_pose=pose or Pose(Vector3(0.4, 0.0, 0.3), IDENTITY),
        tcp_twist=None,
        gripper=0.5,
        external_wrench=None,
    )


def make_input(
    sequence: int,
    *,
    session_id: int = 10,
    position: Vector3 = ZERO_POSITION,
    orientation: Quaternion = IDENTITY,
    clutch: bool = True,
    recenter: bool = False,
    estop: bool = False,
    tracking: TrackingQuality = TrackingQuality.NORMAL,
) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=session_id,
        timestamp_ns=sequence * 1_000_000,
        pose=Pose(position, orientation),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=clutch,
        recenter_requested=recenter,
        estop_requested=estop,
        gripper=0.25,
        tracking_quality=tracking,
        source_name="contract",
    )


def make_pipeline(
    mapping: PoseMapping | None = None,
    *,
    mapping_provider: Callable[[], PoseMapping] | None = None,
) -> TeleopPipeline:
    return TeleopPipeline(
        policy=RelativePosePolicy(
            mapping or PoseMapping(),
            mapping_provider=mapping_provider,
        ),
        compliance=NoCompliancePolicy(),
        input_capabilities=CAPABILITIES,
        safety_limits=TargetSafetyLimits(
            maximum_target_step_m=1.0,
            maximum_rotation_step_rad=math.pi,
            maximum_linear_velocity_m_s=10.0,
            maximum_angular_velocity_rad_s=10.0,
        ),
    )


def assert_pose_equal(actual: Pose, expected: Pose) -> None:
    assert actual.position.as_tuple() == pytest.approx(expected.position.as_tuple())
    assert quaternion_angular_distance(actual.orientation, expected.orientation) < 1e-8


def active_pose(target: object) -> Pose:
    assert target.mode is ControlMode.CARTESIAN_POSITION
    pose = target.target_pose
    assert isinstance(pose, Pose)
    return pose


def test_first_clutch_activation_has_no_jump_and_release_holds() -> None:
    pipeline = make_pipeline()
    robot = make_robot()

    first = pipeline.process(make_input(1), robot, dt=0.01, now_ns=10)
    released = pipeline.process(
        make_input(2, position=Vector3(0.2, 0.0, 0.0), clutch=False),
        robot,
        dt=0.01,
        now_ns=20,
    )

    assert_pose_equal(active_pose(first), robot.tcp_pose)
    assert released.mode is ControlMode.HOLD
    assert pipeline.state is TeleopState.HOLD


def test_position_and_rotation_scale_are_applied() -> None:
    pipeline = make_pipeline(PoseMapping(position_scale=0.5, rotation_scale=0.5))
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)
    quarter_turn = quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), math.pi / 2.0)

    target = pipeline.process(
        make_input(2, position=Vector3(0.2, 0.0, 0.0), orientation=quarter_turn),
        robot,
        dt=0.1,
        now_ns=20,
    )

    pose = active_pose(target)
    assert pose.position.as_tuple() == pytest.approx((0.5, 0.0, 0.3))
    expected = quaternion_from_axis_angle(Vector3(0.0, 0.0, 1.0), math.pi / 4.0)
    assert quaternion_angular_distance(pose.orientation, expected) < 1e-8


def test_dynamic_scale_is_latched_only_on_next_activation() -> None:
    settings = MappingSettingsStore(PoseMapping())
    pipeline = make_pipeline(mapping_provider=settings.load)
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)

    settings.update(MappingScales(position_scale=0.5, rotation_scale=1.0))
    active = pipeline.process(
        make_input(2, position=Vector3(0.2, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=20,
    )
    active_robot = make_robot(active_pose(active), timestamp_ns=2)
    pipeline.process(
        make_input(3, position=Vector3(0.2, 0.0, 0.0), clutch=False),
        active_robot,
        dt=0.1,
        now_ns=30,
    )
    reactivated = pipeline.process(
        make_input(4, position=Vector3(0.2, 0.0, 0.0)),
        active_robot,
        dt=0.1,
        now_ns=40,
    )
    scaled = pipeline.process(
        make_input(5, position=Vector3(0.4, 0.0, 0.0)),
        active_robot,
        dt=0.1,
        now_ns=50,
    )

    assert active_pose(active).position.x == pytest.approx(0.6)
    assert_pose_equal(active_pose(reactivated), active_robot.tcp_pose)
    assert active_pose(scaled).position.x == pytest.approx(0.7)


def test_axis_mapping_is_policy_configuration_not_source_branching() -> None:
    pipeline = make_pipeline(
        PoseMapping(
            axes=AxisMapping(robot_from_input=(2, 0, 1), signs=(-1, -1, 1)),
        )
    )
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)

    target = pipeline.process(
        make_input(2, position=Vector3(0.1, 0.2, 0.3)),
        robot,
        dt=0.1,
        now_ns=20,
    )

    assert active_pose(target).position.as_tuple() == pytest.approx((0.1, -0.1, 0.5))


def test_recenter_and_reactivation_remain_continuous() -> None:
    pipeline = make_pipeline()
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)
    moved = pipeline.process(
        make_input(2, position=Vector3(0.1, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=20,
    )
    moved_robot = make_robot(active_pose(moved), timestamp_ns=2)

    recentered = pipeline.process(
        make_input(3, position=Vector3(0.1, 0.0, 0.0), recenter=True),
        moved_robot,
        dt=0.1,
        now_ns=30,
    )
    pipeline.process(
        make_input(4, position=Vector3(1.0, 0.0, 0.0), clutch=False),
        moved_robot,
        dt=0.1,
        now_ns=40,
    )
    reactivated = pipeline.process(
        make_input(5, position=Vector3(1.0, 0.0, 0.0), clutch=True),
        moved_robot,
        dt=0.1,
        now_ns=50,
    )

    assert_pose_equal(active_pose(recentered), moved_robot.tcp_pose)
    assert_pose_equal(active_pose(reactivated), moved_robot.tcp_pose)


def test_session_change_while_pressed_requires_release_and_repress() -> None:
    pipeline = make_pipeline()
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)

    changed = pipeline.process(
        make_input(0, session_id=11, position=Vector3(2.0, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=20,
    )
    still_pressed = pipeline.process(
        make_input(1, session_id=11, position=Vector3(2.1, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=30,
    )
    released = pipeline.process(
        make_input(2, session_id=11, clutch=False), robot, dt=0.1, now_ns=40
    )
    reactivated = pipeline.process(
        make_input(3, session_id=11, position=Vector3(4.0, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=50,
    )

    assert changed.mode is ControlMode.HOLD
    assert still_pressed.mode is ControlMode.HOLD
    assert released.mode is ControlMode.HOLD
    assert_pose_equal(active_pose(reactivated), robot.tcp_pose)


def test_tracking_loss_and_recovery_cannot_auto_reactivate() -> None:
    pipeline = make_pipeline()
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)

    degraded = pipeline.process(
        make_input(2, tracking=TrackingQuality.LIMITED), robot, dt=0.1, now_ns=20
    )
    recovered_pressed = pipeline.process(make_input(3), robot, dt=0.1, now_ns=30)
    pipeline.process(make_input(4, clutch=False), robot, dt=0.1, now_ns=40)
    recovered = pipeline.process(make_input(5), robot, dt=0.1, now_ns=50)

    assert degraded.mode is ControlMode.HOLD
    assert recovered_pressed.mode is ControlMode.HOLD
    assert_pose_equal(active_pose(recovered), robot.tcp_pose)


def test_network_suspend_requires_release_and_recaptures_references() -> None:
    pipeline = make_pipeline()
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)
    moved = pipeline.process(
        make_input(2, position=Vector3(0.1, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=20,
    )
    moved_robot = make_robot(active_pose(moved), timestamp_ns=2)

    pipeline.suspend()
    still_pressed = pipeline.process(
        make_input(3, position=Vector3(0.2, 0.0, 0.0)),
        moved_robot,
        dt=0.1,
        now_ns=30,
    )
    pipeline.process(
        make_input(4, position=Vector3(0.2, 0.0, 0.0), clutch=False),
        moved_robot,
        dt=0.1,
        now_ns=40,
    )
    reactivated = pipeline.process(
        make_input(5, position=Vector3(0.2, 0.0, 0.0)),
        moved_robot,
        dt=0.1,
        now_ns=50,
    )

    assert still_pressed.mode is ControlMode.HOLD
    assert_pose_equal(active_pose(reactivated), moved_robot.tcp_pose)


def test_disconnect_requires_release_and_recaptures_references() -> None:
    pipeline = make_pipeline()
    robot = make_robot()
    pipeline.process(make_input(1), robot, dt=0.1, now_ns=10)

    pipeline.disconnect()
    still_pressed = pipeline.process(
        make_input(2, position=Vector3(0.2, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=20,
    )
    pipeline.process(
        make_input(3, position=Vector3(0.2, 0.0, 0.0), clutch=False),
        robot,
        dt=0.1,
        now_ns=30,
    )
    reactivated = pipeline.process(
        make_input(4, position=Vector3(0.2, 0.0, 0.0)),
        robot,
        dt=0.1,
        now_ns=40,
    )

    assert still_pressed.mode is ControlMode.HOLD
    assert_pose_equal(active_pose(reactivated), robot.tcp_pose)


def test_estop_latches_until_pipeline_reset() -> None:
    pipeline = make_pipeline()
    robot = make_robot()

    stopped = pipeline.process(make_input(1, estop=True), robot, dt=0.1, now_ns=10)
    after = pipeline.process(make_input(2, estop=False), robot, dt=0.1, now_ns=20)

    assert stopped.mode is ControlMode.STOP
    assert after.mode is ControlMode.STOP
    assert pipeline.state is TeleopState.ESTOP
    pipeline.reset_latches()
    assert pipeline.state is TeleopState.READY


def test_policy_uses_local_target_generation_timestamp() -> None:
    pipeline = make_pipeline()
    target = pipeline.process(make_input(1), make_robot(), dt=0.1, now_ns=999)

    assert target.timestamp_ns == 999


def test_relative_policy_requires_pose_capability() -> None:
    no_pose = replace(CAPABILITIES, pose_6dof=False)

    with pytest.raises(ValueError, match="6DoF"):
        TeleopPipeline(
            policy=RelativePosePolicy(PoseMapping()),
            compliance=NoCompliancePolicy(),
            input_capabilities=no_pose,
            safety_limits=TargetSafetyLimits(),
        )
