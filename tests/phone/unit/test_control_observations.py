from embodied.devices.phone.core.models import (
    Pose,
    Quaternion,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.devices.phone.tools.control_observations import ControlObservations


def sample(
    sequence: int,
    *,
    session_id: int = 1,
    clutch: bool = False,
    recenter: bool = False,
    estop: bool = False,
    gripper: float | None = 0.2,
    tracking: TrackingQuality = TrackingQuality.NORMAL,
) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=session_id,
        timestamp_ns=sequence,
        pose=Pose(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=clutch,
        recenter_requested=recenter,
        estop_requested=estop,
        gripper=gripper,
        tracking_quality=tracking,
        source_name="test",
    )


def test_control_observations_report_edges_ranges_and_tracking() -> None:
    observations = ControlObservations()
    inputs = (
        sample(1),
        sample(2, clutch=True, recenter=True),
        sample(3, clutch=True, gripper=0.8, tracking=TrackingQuality.LIMITED),
        sample(
            4,
            estop=True,
            gripper=0.8,
            tracking=TrackingQuality.UNAVAILABLE,
        ),
        sample(5, estop=True, gripper=None),
    )

    for input_state in inputs:
        observations.observe(input_state)

    assert observations.report() == {
        "sessions_observed": 1,
        "clutch": {
            "active_packets": 2,
            "activation_edges": 1,
            "active_session_starts": 0,
            "release_edges": 1,
            "last_active": False,
        },
        "recenter": {
            "requested_packets": 1,
            "activation_edges": 1,
            "requested_session_starts": 0,
            "last_requested": False,
        },
        "estop": {
            "requested_packets": 2,
            "activation_edges": 1,
            "requested_session_starts": 0,
            "release_edges": 0,
            "ever_requested": True,
            "last_requested": True,
        },
        "gripper": {
            "samples": 4,
            "changes": 1,
            "minimum": 0.2,
            "maximum": 0.8,
            "last": 0.8,
        },
        "tracking": {"unavailable": 1, "limited": 1, "normal": 3},
    }


def test_repeated_recenter_is_visible_without_counting_extra_edges() -> None:
    observations = ControlObservations()

    observations.observe(sample(1))
    observations.observe(sample(2, recenter=True))
    observations.observe(sample(3, recenter=True))
    observations.observe(sample(4))

    report = observations.report()
    assert report["recenter"] == {
        "requested_packets": 2,
        "activation_edges": 1,
        "requested_session_starts": 0,
        "last_requested": False,
    }


def test_new_session_resets_edge_and_gripper_baselines() -> None:
    observations = ControlObservations()

    observations.observe(sample(10, session_id=1, gripper=0.4))
    observations.observe(sample(11, session_id=1, clutch=True, gripper=0.8))
    observations.observe(sample(0, session_id=2, clutch=True, gripper=0.8))

    report = observations.report()
    assert report["sessions_observed"] == 2
    assert report["clutch"] == {
        "active_packets": 2,
        "activation_edges": 1,
        "active_session_starts": 1,
        "release_edges": 0,
        "last_active": True,
    }
    assert report["gripper"] == {
        "samples": 3,
        "changes": 1,
        "minimum": 0.4,
        "maximum": 0.8,
        "last": 0.8,
    }


def test_estop_active_at_reconnect_is_not_reported_as_another_press() -> None:
    observations = ControlObservations()

    observations.observe(sample(1, session_id=1))
    observations.observe(sample(2, session_id=1, estop=True))
    observations.observe(sample(0, session_id=2, estop=True))

    report = observations.report()
    assert report["estop"] == {
        "requested_packets": 2,
        "activation_edges": 1,
        "requested_session_starts": 1,
        "release_edges": 0,
        "ever_requested": True,
        "last_requested": True,
    }
