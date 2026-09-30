import json
import socket
import threading
from pathlib import Path

import pytest

from robot_teleop.protocol.golden import GOLDEN_POSE
from robot_teleop.protocol.packet import (
    SettingsAckPacket,
    SettingsPacket,
    decode_packet,
    encode_packet,
)
from robot_teleop.tools import latency_report, network_test, udp_loopback
from robot_teleop.tools.pose_monitor import PoseMonitorConfig, run_monitor
from robot_teleop.tools.pose_simulator import PoseSimulatorConfig, run_simulator


def unused_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return int(port)


def test_protocol_udp_loopback_reports_rtt(capsys: object) -> None:
    exit_code = udp_loopback.main(
        ["--count", "5", "--interval-ms", "0", "--timeout-ms", "100"]
    )
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)

    assert exit_code == 0
    assert report["sent"] == 5
    assert report["received"] == 5
    assert report["lost"] == 0
    assert report["rtt_ms"]["count"] == 5
    assert report["clock_offset_ns"] is not None


def test_local_network_test_reaches_latest_sequence(capsys: object) -> None:
    exit_code = network_test.main(
        ["--duration", "0.05", "--rate-hz", "200", "--seed", "7"]
    )
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)

    assert exit_code == 0
    assert report["attempted"] == 10
    assert report["sent"] == 10
    assert report["received"] > 0
    assert report["last_sequence"] == 9
    assert report["latest_reached"] is True


def test_network_test_can_simulate_complete_loss(capsys: object) -> None:
    exit_code = network_test.main(
        [
            "--duration",
            "0.02",
            "--rate-hz",
            "200",
            "--loss-rate",
            "1",
        ]
    )
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)

    assert exit_code == 0
    assert report["attempted"] == 4
    assert report["simulated_lost"] == 4
    assert report["sent"] == 0
    assert report["received"] == 0


def test_pose_simulator_and_monitor_interoperate() -> None:
    port = unused_udp_port()
    ready = threading.Event()
    monitor_result: list[dict[str, object]] = []

    def monitor() -> None:
        monitor_result.append(
            run_monitor(
                PoseMonitorConfig(
                    bind_host="127.0.0.1",
                    port=port,
                    duration_s=0.25,
                    receive_timeout_s=0.02,
                ),
                ready_event=ready,
            )
        )

    thread = threading.Thread(target=monitor)
    thread.start()
    assert ready.wait(0.5)
    simulator_report = run_simulator(
        PoseSimulatorConfig(
            host="127.0.0.1",
            port=port,
            rate_hz=100.0,
            duration_s=0.08,
            session_id=123,
            ping_interval_s=0.02,
        )
    )
    thread.join(1.0)

    assert not thread.is_alive()
    assert simulator_report["pose_sent"] == 9
    assert monitor_result[0]["pose_received"] > 0
    assert monitor_result[0]["last_session_id"] == 123
    control_observations = monitor_result[0]["control_observations"]
    assert isinstance(control_observations, dict)
    assert control_observations["sessions_observed"] == 1
    assert control_observations["tracking"]["normal"] > 0


def test_pose_monitor_acknowledges_mapping_settings() -> None:
    port = unused_udp_port()
    ready = threading.Event()
    monitor_result: list[dict[str, object]] = []

    def monitor() -> None:
        monitor_result.append(
            run_monitor(
                PoseMonitorConfig(
                    bind_host="127.0.0.1",
                    port=port,
                    duration_s=0.2,
                    receive_timeout_s=0.01,
                ),
                ready_event=ready,
            )
        )

    thread = threading.Thread(target=monitor)
    thread.start()
    assert ready.wait(0.5)

    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(0.2)
    try:
        client.sendto(encode_packet(GOLDEN_POSE), ("127.0.0.1", port))
        threading.Event().wait(0.02)
        settings = SettingsPacket(
            source_type=GOLDEN_POSE.source_type,
            sequence=0,
            session_id=GOLDEN_POSE.session_id,
            client_time_ns=GOLDEN_POSE.client_time_ns + 1,
            position_scale=0.7,
            rotation_scale=0.8,
        )
        client.sendto(encode_packet(settings), ("127.0.0.1", port))
        acknowledgement = decode_packet(client.recvfrom(512)[0])
    finally:
        client.close()

    thread.join(1.0)
    assert not thread.is_alive()
    assert isinstance(acknowledgement, SettingsAckPacket)
    assert acknowledgement.source_type == settings.source_type
    assert acknowledgement.sequence == settings.sequence
    assert acknowledgement.session_id == settings.session_id
    assert acknowledgement.client_time_ns == settings.client_time_ns
    assert acknowledgement.position_scale == pytest.approx(0.7)
    assert acknowledgement.rotation_scale == pytest.approx(0.8)
    applied = monitor_result[0]["last_mapping_scales"]
    assert isinstance(applied, dict)
    assert applied["position"] == pytest.approx(0.7)
    assert applied["rotation"] == pytest.approx(0.8)


def test_latency_report_summarizes_json_samples(
    tmp_path: Path,
    capsys: object,
) -> None:
    source = tmp_path / "samples.json"
    source.write_text(json.dumps({"samples_ms": [1.0, 2.0, 3.0, 4.0]}))

    exit_code = latency_report.main(["--input", str(source)])
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)

    assert exit_code == 0
    assert report["count"] == 4
    assert report["median"] == 2.5
    assert report["p95"] == 4.0
