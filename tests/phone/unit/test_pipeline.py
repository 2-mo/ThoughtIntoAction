from dataclasses import replace

from embodied.phone.core.capabilities import InputCapabilities
from embodied.phone.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    RobotState,
    RobotTarget,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.phone.core.pipeline import TeleopPipeline
from embodied.phone.core.safety import TargetSafetyLimits
from embodied.phone.policies.no_compliance import NoCompliancePolicy

IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)
ZERO_POSE = Pose(Vector3(0.0, 0.0, 0.0), IDENTITY)
CAPABILITIES = InputCapabilities(
    pose_6dof=True,
    linear_velocity=False,
    angular_velocity=False,
    analog_gripper=True,
    haptics=False,
    dual_hand=False,
)


class DummyPolicy:
    def reset(self, input_state: TeleopInput, robot_state: RobotState) -> None:
        del input_state, robot_state

    def update(
        self,
        input_state: TeleopInput,
        robot_state: RobotState,
        dt: float,
    ) -> RobotTarget:
        del input_state, dt
        return RobotTarget(
            mode=ControlMode.CARTESIAN_POSITION,
            target_pose=robot_state.tcp_pose,
            target_twist=None,
            target_joint_position=None,
            gripper=None,
            velocity_scale=1.0,
            timestamp_ns=robot_state.timestamp_ns,
        )


def make_pipeline() -> TeleopPipeline:
    return TeleopPipeline(
        policy=DummyPolicy(),
        compliance=NoCompliancePolicy(),
        input_capabilities=CAPABILITIES,
        safety_limits=TargetSafetyLimits(),
    )


def make_robot() -> RobotState:
    return RobotState(
        timestamp_ns=100,
        joint_position=(0.0,) * 6,
        joint_velocity=(0.0,) * 6,
        joint_torque=None,
        tcp_pose=ZERO_POSE,
        tcp_twist=None,
        gripper=0.0,
        external_wrench=None,
    )


def make_home_input() -> TeleopInput:
    return TeleopInput(
        sequence=10,
        session_id=22,
        timestamp_ns=1_000,
        pose=ZERO_POSE,
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=False,
        recenter_requested=False,
        estop_requested=False,
        gripper=0.5,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="arkit",
        home_requested=True,
    )


def test_home_request_holds_joint_zero_until_feedback_reaches_zero() -> None:
    pipeline = make_pipeline()
    robot = replace(
        make_robot(),
        joint_position=(0.041, 0.002, 0.006, 0.368, -0.001, -0.011),
    )
    home_input = make_home_input()

    first = pipeline.process(
        home_input,
        robot,
        dt=0.005,
        now_ns=2_000,
        input_dt=0.016,
        is_new_input=True,
    )
    stale = pipeline.process(
        home_input,
        robot,
        dt=0.005,
        now_ns=2_005,
        input_dt=None,
        is_new_input=False,
    )
    arrived = pipeline.process(
        home_input,
        replace(robot, joint_position=(0.001,) * 6),
        dt=0.005,
        now_ns=2_010,
        input_dt=None,
        is_new_input=False,
    )

    assert first.mode is ControlMode.JOINT_POSITION
    assert first.target_joint_position == (0.0,) * 6
    assert stale.mode is ControlMode.JOINT_POSITION
    assert stale.target_joint_position == (0.0,) * 6
    assert arrived.mode is ControlMode.HOLD
    assert arrived.gripper == 0.5


def test_joint_zero_request_holds_target_until_feedback_reaches_zero() -> None:
    pipeline = make_pipeline()
    robot = replace(
        make_robot(),
        joint_position=(0.041, 0.002, 0.006, 0.368, -0.001, -0.011),
    )
    request = replace(
        make_home_input(),
        home_requested=False,
        joint_zero_requested=True,
    )

    first = pipeline.process(request, robot, dt=0.005, now_ns=2_000)
    continuing = pipeline.process(
        replace(request, joint_zero_requested=False),
        robot,
        dt=0.005,
        now_ns=2_005,
    )
    arrived = pipeline.process(
        replace(request, joint_zero_requested=False),
        replace(robot, joint_position=(0.001,) * 6),
        dt=0.005,
        now_ns=2_010,
    )

    assert first.mode is ControlMode.JOINT_POSITION
    assert first.target_joint_position == (0.0,) * 6
    assert continuing.mode is ControlMode.JOINT_POSITION
    assert arrived.mode is ControlMode.HOLD
