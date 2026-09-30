import time
from collections.abc import Callable

from robot_teleop.controllers.cartesian_position import CartesianPositionController
from robot_teleop.core.capabilities import InputCapabilities
from robot_teleop.core.mapping import PoseMapping
from robot_teleop.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from robot_teleop.core.pipeline import TeleopPipeline
from robot_teleop.core.safety import TargetSafetyLimits
from robot_teleop.inputs.mock import MockInputSource
from robot_teleop.policies.no_compliance import NoCompliancePolicy
from robot_teleop.policies.relative_pose import RelativePosePolicy
from robot_teleop.robots.mock import MockRobotBackend
from robot_teleop.runtime.application import TeleopApplication
from robot_teleop.runtime.latest_value import LatestValue
from robot_teleop.runtime.metrics import MetricsWorker, RuntimeMetrics
from robot_teleop.runtime.receiver import InputReceiver
from robot_teleop.runtime.servo_loop import ServoLoop, ServoLoopConfig
from robot_teleop.runtime.watchdog import Watchdog, WatchdogConfig

CAPABILITIES = InputCapabilities(
    pose_6dof=True,
    linear_velocity=False,
    angular_velocity=False,
    analog_gripper=True,
    haptics=False,
    dual_hand=False,
)


def make_input(
    sequence: int,
    *,
    position_x: float = 0.0,
    clutch: bool = True,
    estop: bool = False,
) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=42,
        timestamp_ns=sequence * 5_000_000,
        pose=Pose(
            Vector3(position_x, 0.0, 0.0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=clutch,
        recenter_requested=False,
        estop_requested=estop,
        gripper=0.5,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="mock",
    )


def wait_until(predicate: Callable[[], bool], timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError("condition was not reached before timeout")


def latest_sequence(application: TeleopApplication) -> int | None:
    snapshot = application.latest_input.load()
    return None if snapshot is None else snapshot.value.sequence


def build_application(
    *,
    state_stale_timeout_ms: float = 100.0,
    watchdog_config: WatchdogConfig | None = None,
    target_safety_limits: TargetSafetyLimits | None = None,
) -> tuple[TeleopApplication, MockInputSource, MockRobotBackend, RuntimeMetrics]:
    source = MockInputSource(CAPABILITIES)
    robot = MockRobotBackend()
    latest: LatestValue[TeleopInput] = LatestValue()
    metrics = RuntimeMetrics()
    receiver = InputReceiver(source, latest, metrics, poll_interval_s=0.0005)
    pipeline = TeleopPipeline(
        policy=RelativePosePolicy(PoseMapping()),
        compliance=NoCompliancePolicy(),
        input_capabilities=CAPABILITIES,
        safety_limits=target_safety_limits
        or TargetSafetyLimits(
            maximum_target_step_m=0.1,
            maximum_rotation_step_rad=1.0,
            maximum_linear_velocity_m_s=100.0,
            maximum_angular_velocity_rad_s=100.0,
        ),
    )
    servo = ServoLoop(
        latest=latest,
        pipeline=pipeline,
        controller=CartesianPositionController(robot.capabilities),
        backend=robot,
        watchdog=Watchdog(watchdog_config or WatchdogConfig()),
        metrics=metrics,
        config=ServoLoopConfig(
            rate_hz=200.0,
            state_stale_timeout_ms=state_stale_timeout_ms,
        ),
    )
    metrics_worker = MetricsWorker(metrics, interval_s=0.01)
    application = TeleopApplication(
        receiver=receiver,
        servo=servo,
        metrics_worker=metrics_worker,
        backend=robot,
    )
    return application, source, robot, metrics


def test_mock_to_mock_tracks_latest_target_and_stops_cleanly() -> None:
    application, source, robot, metrics = build_application()
    application.start()
    try:
        source.push(make_input(1))
        for sequence in range(2, 15):
            source.push(make_input(sequence, position_x=sequence * 0.001))
            time.sleep(0.002)
        wait_until(lambda: robot.state().tcp_pose.position.x > 0.405)
        source.push(make_input(15, position_x=0.015, clutch=False))
        wait_until(lambda: latest_sequence(application) == 15)
        wait_until(lambda: robot.hold_count > 0)

        snapshot = application.latest_input.load()
        assert snapshot is not None
        assert snapshot.value.sequence == 15
        assert metrics.report()["cycle_count"] > 0
        assert application.worker_count == 3
    finally:
        application.stop()

    assert application.worker_count == 0
    assert not robot.connected
    assert robot.safe_stop_count == 0


def test_feedback_stall_faults_and_safe_stops_backend() -> None:
    application, source, robot, _ = build_application(
        state_stale_timeout_ms=20.0,
        watchdog_config=WatchdogConfig(
            hold_packet_age_ms=1_000.0,
            brake_packet_age_ms=2_000.0,
            stop_packet_age_ms=3_000.0,
        ),
    )
    application.start()
    try:
        source.push(make_input(1))
        wait_until(lambda: len(robot.command_history) > 0)
        robot.set_feedback_frozen(True)
        wait_until(lambda: robot.safe_stop_count == 1)
        assert application.servo_error is not None
        assert "feedback timestamp" in application.servo_error
    finally:
        application.stop()


def test_watchdog_arms_only_after_active_control_begins() -> None:
    application, source, robot, metrics = build_application(
        watchdog_config=WatchdogConfig(
            hold_packet_age_ms=20.0,
            brake_packet_age_ms=40.0,
            stop_packet_age_ms=60.0,
        )
    )
    application.start()
    try:
        source.push(make_input(1, clutch=False))
        time.sleep(0.10)
        assert robot.safe_stop_count == 0
        assert metrics.report()["watchdog_stages"] == {}

        source.push(make_input(2, clutch=True))
        wait_until(lambda: application.state.value == "active")
        wait_until(lambda: robot.safe_stop_count == 1)
        assert application.state.value == "fault"
        assert metrics.report()["watchdog_stages"]["safe_stop"] >= 1
    finally:
        application.stop()


def test_estop_request_invokes_backend_safe_stop() -> None:
    application, source, robot, _ = build_application()
    application.start()
    try:
        source.push(make_input(1, estop=True))
        wait_until(lambda: robot.safe_stop_count == 1)
    finally:
        application.stop()


def test_target_velocity_uses_new_input_cadence_not_servo_rate() -> None:
    application, source, robot, _ = build_application(
        state_stale_timeout_ms=2_000.0,
        watchdog_config=WatchdogConfig(
            hold_packet_age_ms=500.0,
            brake_packet_age_ms=750.0,
            stop_packet_age_ms=1_000.0,
        ),
        target_safety_limits=TargetSafetyLimits(
            maximum_target_step_m=0.1,
            maximum_rotation_step_rad=1.0,
            maximum_linear_velocity_m_s=0.5,
            maximum_angular_velocity_rad_s=100.0,
        ),
    )
    application.start()
    try:
        source.push(make_input(1, position_x=0.0))
        wait_until(
            lambda: any(
                command.mode is ControlMode.CARTESIAN_POSITION
                for command in robot.command_history
            )
        )
        time.sleep(0.020)

        # 8 mm per 20 ms is 0.4 m/s: within the 0.5 m/s target limit.
        for sequence in range(2, 8):
            source.push(make_input(sequence, position_x=(sequence - 1) * 0.008))
            time.sleep(0.020)

        wait_until(lambda: latest_sequence(application) == 7)
        time.sleep(0.030)
        assert application.servo_error is None
        assert robot.safe_stop_count == 0
    finally:
        application.stop()


def test_target_velocity_limits_fast_new_input_without_fault() -> None:
    application, source, robot, _ = build_application(
        state_stale_timeout_ms=2_000.0,
        watchdog_config=WatchdogConfig(
            hold_packet_age_ms=500.0,
            brake_packet_age_ms=750.0,
            stop_packet_age_ms=1_000.0,
        ),
        target_safety_limits=TargetSafetyLimits(
            maximum_target_step_m=0.1,
            maximum_rotation_step_rad=1.0,
            maximum_linear_velocity_m_s=0.5,
            maximum_angular_velocity_rad_s=100.0,
        ),
    )
    application.start()
    try:
        source.push(make_input(1, position_x=0.0))
        wait_until(
            lambda: any(
                command.mode is ControlMode.CARTESIAN_POSITION
                for command in robot.command_history
            )
        )
        time.sleep(0.025)
        source.push(make_input(2, position_x=0.0))
        wait_until(lambda: latest_sequence(application) == 2)
        time.sleep(0.025)

        # 30 mm per roughly 25 ms exceeds the limit, so the common safety gate
        # advances only by the permitted distance instead of faulting the session.
        source.push(make_input(3, position_x=0.03))
        wait_until(lambda: latest_sequence(application) == 3)
        wait_until(lambda: robot.state().tcp_pose.position.x > 0.4)
        target_x = robot.state().tcp_pose.position.x
        assert target_x < 0.43
        assert application.servo_error is None
        assert robot.safe_stop_count == 0
    finally:
        application.stop()


def test_watchdog_recovery_requires_release_before_motion_resumes() -> None:
    application, source, robot, _ = build_application(
        state_stale_timeout_ms=2_000.0,
        watchdog_config=WatchdogConfig(
            hold_packet_age_ms=50.0,
            brake_packet_age_ms=500.0,
            stop_packet_age_ms=1_000.0,
        ),
    )
    application.start()
    try:
        source.push(make_input(1))
        wait_until(
            lambda: any(
                command.mode is ControlMode.CARTESIAN_POSITION
                for command in robot.command_history
            )
        )
        source.push(make_input(2, position_x=0.01))
        wait_until(lambda: robot.state().tcp_pose.position.x > 0.405)
        interrupted_position = robot.state().tcp_pose.position.x
        holds_before_timeout = robot.hold_count

        wait_until(lambda: robot.hold_count > holds_before_timeout)
        source.push(make_input(3, position_x=0.08, clutch=True))
        wait_until(lambda: latest_sequence(application) == 3)
        time.sleep(0.01)
        assert robot.state().tcp_pose.position.x == interrupted_position

        source.push(make_input(4, position_x=0.08, clutch=False))
        wait_until(lambda: latest_sequence(application) == 4)
        time.sleep(0.01)
        source.push(make_input(5, position_x=0.08, clutch=True))
        wait_until(lambda: latest_sequence(application) == 5)
        time.sleep(0.01)
        assert robot.state().tcp_pose.position.x == interrupted_position

        source.push(make_input(6, position_x=0.09, clutch=True))
        wait_until(
            lambda: robot.state().tcp_pose.position.x > interrupted_position + 0.005
        )
    finally:
        application.stop()
