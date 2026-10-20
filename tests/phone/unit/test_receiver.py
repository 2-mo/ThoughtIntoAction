import threading
import time

from embodied.phone.core.capabilities import InputCapabilities
from embodied.phone.core.models import (
    Pose,
    Quaternion,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.phone.inputs.mock import MockInputSource
from embodied.phone.runtime.latest_value import LatestValue
from embodied.phone.runtime.metrics import RuntimeMetrics
from embodied.phone.runtime.receiver import InputReceiver


class BlockingInputSource:
    def __init__(self) -> None:
        self._stop = threading.Event()
        self.entered = threading.Event()
        self._capabilities = InputCapabilities(
            pose_6dof=True,
            linear_velocity=False,
            angular_velocity=False,
            analog_gripper=False,
            haptics=False,
            dual_hand=False,
        )

    @property
    def capabilities(self) -> InputCapabilities:
        return self._capabilities

    def start(self) -> None:
        self._stop.clear()

    def latest(self) -> TeleopInput | None:
        self.entered.set()
        self._stop.wait()
        return None

    def stop(self) -> None:
        self._stop.set()


def _make_input(sequence: int) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=1,
        timestamp_ns=sequence,
        pose=Pose(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=False,
        recenter_requested=False,
        estop_requested=False,
        gripper=0.5,
        tracking_quality=TrackingQuality.NORMAL,
        source_name="mock",
    )


def test_receiver_allows_blocking_source_and_stops_it_before_join() -> None:
    source = BlockingInputSource()
    receiver = InputReceiver(
        source,
        LatestValue(),
        RuntimeMetrics(),
        poll_interval_s=0.0,
    )
    receiver.start()
    assert source.entered.wait(0.5)

    receiver.stop(timeout_s=0.5)

    assert not receiver.alive


def test_receiver_records_host_input_intervals() -> None:
    source = MockInputSource()
    metrics = RuntimeMetrics()
    latest: LatestValue[TeleopInput] = LatestValue()
    receiver = InputReceiver(source, latest, metrics, poll_interval_s=0.0005)
    receiver.start()
    try:
        source.push(_make_input(1))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if metrics.report()["accepted_inputs"] == 1:
                break
            time.sleep(0.002)
        source.push(_make_input(2))
        while time.monotonic() < deadline:
            if metrics.report()["accepted_inputs"] == 2:
                break
            time.sleep(0.002)
        report = metrics.report()
        assert report["accepted_inputs"] == 2
        assert report["input_interval_ms"]["max"] >= 0.0
        assert report["sample_window"]["input_interval"]["sample_count"] == 1
    finally:
        receiver.stop()
