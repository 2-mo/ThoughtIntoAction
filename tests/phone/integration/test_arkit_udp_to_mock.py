import socket
import time
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path

import pytest

from embodied.phone.config import load_profile
from embodied.phone.core.models import Pose, Quaternion, TrackingQuality, Vector3
from embodied.phone.core.state_machine import TeleopState
from embodied.phone.protocol.packet import (
    EStopResetAckPacket,
    EStopResetRequestPacket,
    Packet,
    PingPacket,
    PongPacket,
    PosePacket,
    SettingsAckPacket,
    SettingsPacket,
    SourceType,
    decode_packet,
    encode_packet,
)
from embodied.phone.registry import create_application
from embodied.phone.transports.udp import UdpTransport

ROOT = Path(__file__).parents[3]


def wait_until(predicate: Callable[[], bool], timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError("condition was not reached before timeout")


def pose_packet(
    sequence: int,
    *,
    session_id: int = 7,
    timestamp_ns: int | None = None,
    input_x: float = 0.0,
    clutch: bool = False,
    estop: bool = False,
    tracking: TrackingQuality = TrackingQuality.NORMAL,
) -> PosePacket:
    return PosePacket(
        source_type=SourceType.ARKIT,
        sequence=sequence,
        session_id=session_id,
        client_time_ns=(sequence + 1) * 1_000 if timestamp_ns is None else timestamp_ns,
        pose=Pose(
            Vector3(input_x, 0.0, 0.0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        linear_velocity=None,
        angular_velocity=None,
        connected=True,
        clutch_active=clutch,
        recenter_requested=False,
        estop_requested=estop,
        gripper=0.5,
        tracking_quality=tracking,
    )


def test_arkit_udp_arms_watchdog_only_after_active_clutch() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")
    assert profile.udp is not None
    profile = replace(
        profile,
        udp=replace(profile.udp, bind_host="127.0.0.1", bind_port=0),
        watchdog=replace(
            profile.watchdog,
            hold_packet_age_ms=25.0,
            brake_packet_age_ms=50.0,
            stop_packet_age_ms=75.0,
        ),
        state_stale_timeout_ms=1_000.0,
    )
    bundle = create_application(profile, enable_hardware=False)
    assert isinstance(bundle.transport, UdpTransport)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    bundle.application.start()
    address = bundle.transport.local_address
    try:
        # A local operator can take longer than the watchdog threshold to connect.
        time.sleep(0.125)
        assert bundle.application.state is TeleopState.DISCONNECTED
        assert bundle.application.worker_count == 3
        assert bundle.robot.safe_stop_count == 0

        client.sendto(encode_packet(pose_packet(0)), address)
        wait_until(
            lambda: (
                (snapshot := bundle.application.latest_input.load()) is not None
                and snapshot.value.sequence == 0
            )
        )
        wait_until(lambda: bundle.application.state is TeleopState.READY)
        assert bundle.robot.safe_stop_count == 0

        # A release-only baseline does not arm the timeout. Active control does.
        client.sendto(encode_packet(pose_packet(1, clutch=True)), address)
        wait_until(lambda: bundle.application.state is TeleopState.ACTIVE)
        wait_until(lambda: bundle.robot.safe_stop_count == 1)
        assert bundle.application.state is TeleopState.FAULT
    finally:
        bundle.application.stop()
        client.close()


def test_arkit_udp_fast_continuous_motion_is_slew_limited() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")
    assert profile.udp is not None
    profile = replace(
        profile,
        udp=replace(profile.udp, bind_host="127.0.0.1", bind_port=0),
        watchdog=replace(
            profile.watchdog,
            hold_packet_age_ms=500.0,
            brake_packet_age_ms=750.0,
            stop_packet_age_ms=1_000.0,
        ),
        state_stale_timeout_ms=1_000.0,
    )
    bundle = create_application(profile, enable_hardware=False)
    assert isinstance(bundle.transport, UdpTransport)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    bundle.application.start()
    address = bundle.transport.local_address
    try:
        client.sendto(encode_packet(pose_packet(0, clutch=False)), address)
        wait_until(
            lambda: (
                (snapshot := bundle.application.latest_input.load()) is not None
                and snapshot.value.sequence == 0
            )
        )
        client.sendto(encode_packet(pose_packet(1, clutch=True)), address)
        wait_until(lambda: bundle.application.state is TeleopState.ACTIVE)

        for sequence in range(2, 8):
            client.sendto(
                encode_packet(
                    pose_packet(sequence, input_x=(sequence - 1) * 0.03, clutch=True)
                ),
                address,
            )
            time.sleep(0.016)

        wait_until(
            lambda: (
                (snapshot := bundle.application.latest_input.load()) is not None
                and snapshot.value.sequence == 7
            )
        )
        time.sleep(0.05)
        assert bundle.application.servo_error is None
        assert bundle.robot.safe_stop_count == 0
        assert bundle.application.state is TeleopState.ACTIVE
    finally:
        bundle.application.stop()
        client.close()


def test_arkit_udp_to_mock_complete_session_and_watchdog() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")
    assert profile.udp is not None
    profile = replace(
        profile,
        udp=replace(profile.udp, bind_host="127.0.0.1", bind_port=0),
        watchdog=replace(
            profile.watchdog,
            hold_packet_age_ms=100.0,
            brake_packet_age_ms=200.0,
            stop_packet_age_ms=350.0,
        ),
        state_stale_timeout_ms=500.0,
    )
    bundle = create_application(profile, enable_hardware=False)
    assert isinstance(bundle.transport, UdpTransport)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(0.5)

    bundle.application.start()
    address = bundle.transport.local_address
    try:

        def send(packet: Packet) -> None:
            client.sendto(encode_packet(packet), address)

        def latest_sequence() -> int | None:
            snapshot = bundle.application.latest_input.load()
            return None if snapshot is None else snapshot.value.sequence

        send(pose_packet(0, clutch=False))
        wait_until(lambda: latest_sequence() == 0)
        wait_until(
            lambda: any(
                command.gripper == 0.5 for command in bundle.robot.command_history
            )
        )
        send(
            SettingsPacket(
                source_type=SourceType.ARKIT,
                sequence=0,
                session_id=7,
                client_time_ns=1_500,
                position_scale=0.5,
                rotation_scale=0.75,
            )
        )
        settings_ack = decode_packet(client.recvfrom(256)[0])
        assert isinstance(settings_ack, SettingsAckPacket)
        assert settings_ack.position_scale == 0.5
        assert settings_ack.rotation_scale == 0.75
        assert bundle.mapping_settings.load().position_scale == 0.5
        send(pose_packet(1, clutch=True))
        wait_until(lambda: latest_sequence() == 1)
        wait_until(
            lambda: (
                bundle.application.servo_error is not None
                or any(
                    command.target_pose is not None
                    for command in bundle.robot.command_history
                )
            )
        )
        assert bundle.application.receiver_error is None
        assert bundle.application.servo_error is None

        send(pose_packet(2, input_x=0.001, clutch=True))
        wait_until(lambda: bundle.robot.state().tcp_pose.position.y < -0.0003)

        send(
            PingPacket(
                source_type=SourceType.ARKIT,
                sequence=99,
                session_id=7,
                client_time_ns=123_456,
            )
        )
        pong = decode_packet(client.recvfrom(256)[0])
        assert isinstance(pong, PongPacket)
        assert pong.client_time_ns == 123_456
        assert pong.server_send_time_ns >= pong.server_receive_time_ns

        send(pose_packet(2, timestamp_ns=4_000, clutch=True))
        wait_until(
            lambda: (
                bundle.metrics.report()["protocol_events"].get(
                    "sequence.reject.duplicate"
                )
                == 1
            )
        )
        send(pose_packet(1, timestamp_ns=5_000, clutch=True))
        wait_until(
            lambda: (
                bundle.metrics.report()["protocol_events"].get(
                    "sequence.reject.out_of_order"
                )
                == 1
            )
        )

        send(pose_packet(0, session_id=8, timestamp_ns=100, input_x=0.001))
        wait_until(lambda: latest_sequence() == 0)
        time.sleep(0.02)
        send(
            pose_packet(
                1,
                session_id=8,
                timestamp_ns=200,
                input_x=0.001,
                clutch=True,
            )
        )
        wait_until(lambda: latest_sequence() == 1)
        send(pose_packet(3, session_id=7, timestamp_ns=6_000))
        wait_until(
            lambda: (
                bundle.metrics.report()["protocol_events"].get(
                    "sequence.reject.retired_session"
                )
                == 1
            )
        )

        for sequence in range(2, 31):
            send(
                pose_packet(
                    sequence,
                    session_id=8,
                    timestamp_ns=(sequence + 1) * 100,
                    input_x=0.001 + sequence * 0.00001,
                    clutch=True,
                )
            )
        wait_until(lambda: latest_sequence() == 30)

        holds_before_release = bundle.robot.hold_count
        send(
            pose_packet(
                31,
                session_id=8,
                timestamp_ns=3_200,
                input_x=0.0013,
                clutch=False,
            )
        )
        wait_until(lambda: bundle.robot.hold_count > holds_before_release)
        send(
            pose_packet(
                32,
                session_id=8,
                timestamp_ns=3_300,
                input_x=0.0013,
                tracking=TrackingQuality.UNAVAILABLE,
            )
        )
        wait_until(lambda: latest_sequence() == 32)

        wait_until(lambda: bundle.robot.safe_stop_count == 1)
        report = bundle.metrics.report()
        assert report["protocol_events"]["packet.settings"] == 1
        assert report["protocol_events"]["packet.settings_ack_sent"] == 1
        assert report["protocol_events"]["packet.ping"] == 1
        assert report["protocol_events"]["packet.pong_sent"] == 1
        assert report["protocol_events"]["sequence.reject.duplicate"] == 1
        assert report["watchdog_stages"]["hold"] > 0
        assert report["watchdog_stages"]["brake"] > 0
        assert report["watchdog_stages"]["safe_stop"] == 1
        assert not bundle.hardware_enabled
    finally:
        bundle.application.stop()
        client.close()

    assert bundle.application.worker_count == 0
    assert not bundle.robot.connected


def test_estop_reset_requires_explicit_local_completion() -> None:
    profile = load_profile(ROOT / "configs/phone/profiles/arkit_mock.yaml")
    assert profile.udp is not None
    profile = replace(
        profile,
        udp=replace(profile.udp, bind_host="127.0.0.1", bind_port=0),
        watchdog=replace(
            profile.watchdog,
            hold_packet_age_ms=500.0,
            brake_packet_age_ms=750.0,
            stop_packet_age_ms=1_000.0,
        ),
        state_stale_timeout_ms=2_000.0,
    )
    bundle = create_application(profile, enable_hardware=False)
    assert isinstance(bundle.transport, UdpTransport)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(0.1)
    bundle.application.start()
    address = bundle.transport.local_address
    try:

        def send(packet: Packet) -> None:
            client.sendto(encode_packet(packet), address)

        send(pose_packet(0, timestamp_ns=100))
        send(pose_packet(1, timestamp_ns=200, estop=True))
        wait_until(lambda: bundle.robot.safe_stop_count == 1)
        assert bundle.application.state is TeleopState.ESTOP

        request = EStopResetRequestPacket(
            source_type=SourceType.ARKIT,
            sequence=0,
            session_id=7,
            client_time_ns=250,
        )
        send(request)
        send(pose_packet(2, timestamp_ns=251, clutch=False, estop=False))
        wait_until(lambda: bundle.application.estop_reset_ready)

        time.sleep(0.02)
        assert bundle.application.state is TeleopState.ESTOP
        assert bundle.robot.safe_stopped
        with pytest.raises(TimeoutError):
            client.recvfrom(256)

        assert bundle.application.reset_estop()
        acknowledgement = decode_packet(client.recvfrom(256)[0])
        assert acknowledgement == EStopResetAckPacket(
            source_type=SourceType.ARKIT,
            sequence=request.sequence,
            session_id=request.session_id,
            client_time_ns=request.client_time_ns,
        )
        wait_until(lambda: bundle.application.state is TeleopState.READY)
        assert not bundle.robot.safe_stopped

        send(request)
        repeated_acknowledgement = decode_packet(client.recvfrom(256)[0])
        assert repeated_acknowledgement == acknowledgement
        assert bundle.robot.safe_stop_count == 1
    finally:
        bundle.application.stop()
        client.close()
