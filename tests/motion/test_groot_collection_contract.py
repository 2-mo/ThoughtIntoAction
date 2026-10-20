import json
import re
import time
from types import SimpleNamespace

import numpy as np
import pytest

from embodied.data.groot.config import load_collection_config
from embodied.data.groot.exporter import (
    _build_absolute_action,
    _initial_arm_motion_trim_index,
)
from embodied.data.groot.raw_episode import RawEpisodeStore, atomic_json
from embodied.data.groot.recording import (
    ArmStateRecorder,
    CameraEpisodeRecorder,
    SharedFrameTimeline,
)
from embodied.data.groot.validator import (
    shared_timeline_frame_selection,
    uniform_frame_timestamps,
    unique_source_frame_indices,
    validate_raw_episode,
)


def write_validation_fixture(tmp_path, *, gripper_position: float):
    config = load_collection_config()
    episode_path = tmp_path / "episode-test"
    state_path = episode_path / "state"
    state_path.mkdir(parents=True)
    sample_count = 401
    states = []
    for index in range(sample_count):
        if index <= 30:
            joint_position = 0.0
        elif index < 350:
            joint_position = 0.001 * (index - 30)
        else:
            joint_position = 0.32
        states.append(
            {
                "host_monotonic_ns": index * 10_000_000,
                "joint_position": [joint_position, 0.0, 0.0, 0.0, 0.0, 0.0],
                "joint_velocity": [0.0] * 6,
                "joint_torque": [0.0] * 6,
                "gripper_position": gripper_position,
                "eef_pose_6d": [0.0] * 6,
            }
        )
    (state_path / "states.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in states),
        encoding="utf-8",
    )
    (state_path / "summary.json").write_text(
        json.dumps({"sample_count": sample_count, "error": None}),
        encoding="utf-8",
    )
    for camera_index, camera in enumerate(config.cameras):
        camera_path = episode_path / "cameras" / camera.video_key
        camera_path.mkdir(parents=True)
        frame_records = [
            {
                "host_monotonic_ns": index * 33_000_000 + camera_index * 2_000_000,
                "frame_index": index,
            }
            for index in range(121)
        ]
        (camera_path / "frames.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in frame_records),
            encoding="utf-8",
        )
        (camera_path / "video.mkv").write_bytes(b"\x1aE\xdf\xa3")
        (camera_path / "camera.json").write_text(
            json.dumps(
                {
                    "stage": "complete",
                    "video_key": camera.video_key,
                    "width": camera.width,
                    "height": camera.height,
                    "fps": camera.fps,
                    "frame_count": len(frame_records),
                    "dropped_queue_frames": 0,
                    "storage": {
                        "container": "matroska",
                        "codec": "h264",
                        "video_file": "video.mkv",
                        "frame_metadata_file": "frames.jsonl",
                    },
                    "error": None,
                }
            ),
            encoding="utf-8",
        )
    (episode_path / "episode.json").write_text(
        json.dumps(
            {
                "session_id": "episode-test",
                "dataset_id": config.dataset_id,
                "batch_id": config.batch_id,
                "task_instruction": config.task_instruction,
                "limits": {
                    "joint_position_min_rad": [-2.0] * 6,
                    "joint_position_max_rad": [2.0] * 6,
                    "gripper_width_m": 0.088,
                },
            }
        ),
        encoding="utf-8",
    )
    return episode_path, config


def test_default_config_matches_arx5_groot_contract() -> None:
    config = load_collection_config()

    assert re.fullmatch(r"\d{8}-\d{6}[a-z0-9_-]*", config.batch_id)
    assert [camera.video_key for camera in config.cameras] == [
        "exterior_rgb",
        "wrist_rgb",
    ]
    assert (config.camera.width, config.camera.height, config.camera.fps) == (
        1280,
        720,
        30,
    )
    assert (
        config.wrist_camera.width,
        config.wrist_camera.height,
        config.wrist_camera.fps,
    ) == (1280, 720, 30)
    assert (config.camera.exposure_us, config.camera.gain) == (8000, 64)
    assert (config.wrist_camera.exposure_us, config.wrist_camera.gain) == (
        10000,
        32,
    )
    assert config.camera.white_balance_k == config.wrist_camera.white_balance_k == 4500
    assert config.sampling.arm_rate_hz == 100
    assert config.sampling.action_horizon == 16
    assert config.task_instruction


def test_absolute_action_clips_gripper_without_changing_observation() -> None:
    state = np.zeros((4, 16), dtype=np.float32)
    state[:, 9] = [0.01, -0.0037, 0.091, 0.02]

    action, metrics = _build_absolute_action(state, gripper_width_m=0.088)

    assert state[:, 9].tolist() == pytest.approx([0.01, -0.0037, 0.091, 0.02])
    assert action[:, 9].tolist() == pytest.approx([0.0, 0.088, 0.02, 0.02])
    assert metrics == {
        "control_range_m": [0.0, 0.088],
        "raw_action_min_m": pytest.approx(-0.0037),
        "raw_action_max_m": pytest.approx(0.091),
        "clipped_low_frames": 1,
        "clipped_high_frames": 1,
        "clipped_frames": 2,
    }


def test_camera_live_summary_exposes_collection_health(tmp_path) -> None:
    recorder = CameraEpisodeRecorder(
        tmp_path,
        video_key="exterior_rgb",
        width=640,
        height=480,
        fps=30,
        video_crf=12,
        camera_metadata={"camera": "test"},
        queue_size=12,
    )

    assert recorder.summary() == {
        "frame_count": 0,
        "dropped_queue_frames": 0,
        "drop_fraction": 0.0,
        "source_callbacks": 0,
        "duplicate_source_frames": 0,
        "duplicate_source_fraction": 0.0,
        "rate_limited_source_frames": 0,
        "source_frame_number_gaps": 0,
        "source_nonmonotonic_frames": 0,
        "source_duration_s": 0.0,
        "source_effective_fps": 0.0,
        "recording_effective_fps": 0.0,
        "queue_depth": 0,
        "queue_capacity": 12,
        "first_host_monotonic_ns": None,
        "last_host_monotonic_ns": None,
        "maximum_frame_gap_ms": 0.0,
        "duration_s": 0.0,
        "error": None,
    }


def test_camera_recorder_omits_repeated_source_frame(tmp_path) -> None:
    recorder = CameraEpisodeRecorder(
        tmp_path,
        video_key="exterior_rgb",
        width=8,
        height=6,
        fps=30,
        video_crf=12,
        camera_metadata={"camera": "test"},
        queue_size=4,
    )
    image = np.zeros((6, 8, 3), dtype=np.uint8)

    recorder.capture(
        image,
        host_monotonic_ns=1_000_000,
        device_timestamp_ms=10.0,
        frame_number=1,
    )
    recorder.capture(
        image,
        host_monotonic_ns=2_000_000,
        device_timestamp_ms=10.0,
        frame_number=1,
    )

    summary = recorder.summary()
    assert summary["source_callbacks"] == 2
    assert summary["duplicate_source_frames"] == 1
    assert summary["duplicate_source_fraction"] == pytest.approx(0.5)
    assert summary["queue_depth"] == 1


def test_shared_camera_timeline_uses_wrist_as_30hz_master() -> None:
    timeline = SharedFrameTimeline(30, master_video_key="wrist_rgb")

    assert timeline.select("exterior_rgb", 990_000_000) is None
    assert timeline.select("wrist_rgb", 1_000_000_000) == (0, 1_000_000_000)
    assert timeline.select("exterior_rgb", 1_005_000_000) == (0, 1_000_000_000)
    assert timeline.select("exterior_rgb", 1_030_000_000) == (1, 1_033_333_333)
    assert timeline.select("exterior_rgb", 1_031_000_000) is None
    assert timeline.select("wrist_rgb", 1_033_400_000) == (1, 1_033_333_333)


def test_shared_timeline_export_trims_edges_without_repeating_frames() -> None:
    origin_ns = 1_000_000_000

    def record(slot: int) -> dict[str, int]:
        target_ns = origin_ns + (slot * 1_000_000_000 + 15) // 30
        return {
            "timeline_slot": slot,
            "timeline_target_monotonic_ns": target_ns,
        }

    exterior = [record(slot) for slot in range(8, 11)]
    wrist = [record(slot) for slot in range(11)]
    selection = shared_timeline_frame_selection(
        {"exterior_rgb": exterior, "wrist_rgb": wrist},
        {
            "exterior_rgb": np.arange(len(exterior), dtype=np.int64),
            "wrist_rgb": np.arange(len(wrist), dtype=np.int64),
        },
        state_first_ns=origin_ns,
        state_last_ns=record(10)["timeline_target_monotonic_ns"],
        fps=30,
    )

    assert selection is not None
    frame_ns, source_indices = selection
    assert frame_ns.tolist() == [
        record(slot)["timeline_target_monotonic_ns"] for slot in range(8, 11)
    ]
    assert source_indices["exterior_rgb"].tolist() == [0, 1, 2]
    assert source_indices["wrist_rgb"].tolist() == [8, 9, 10]


def test_uniform_export_timeline_and_source_deduplication() -> None:
    timeline = uniform_frame_timestamps(1_000_000_000, 2_000_000_000, 30)
    assert len(timeline) == 31
    assert timeline[0] == 1_000_000_000
    assert timeline[-1] == 2_000_000_000
    assert set(np.diff(timeline).tolist()) == {33_333_333, 33_333_334}

    indices = unique_source_frame_indices(
        [
            {"frame_number": 1, "device_timestamp_ms": 10.0},
            {"frame_number": 1, "device_timestamp_ms": 10.0},
            {"frame_number": 2, "device_timestamp_ms": 43.3},
        ]
    )
    assert indices.tolist() == [0, 2]


def test_initial_arm_motion_trim_keeps_requested_lead_window() -> None:
    state = np.zeros((40, 16), dtype=np.float32)
    state[20:, 0] = 0.002

    trim_index = _initial_arm_motion_trim_index(
        state,
        fps=10,
        lead_seconds=0.5,
    )

    assert trim_index == 15


def test_arm_live_summary_exposes_latest_joint_position(tmp_path) -> None:
    snapshot = SimpleNamespace(
        timestamp=1.0,
        joint_position=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        joint_velocity=[0.0] * 6,
        joint_torque=[0.0] * 6,
        gripper_position=0.02,
        gripper_velocity=0.0,
        gripper_torque=0.0,
        eef_pose_6d=[0.0] * 6,
    )
    controller = SimpleNamespace(get_state=lambda: snapshot)
    recorder = ArmStateRecorder(tmp_path, controller, rate_hz=100)
    recorder.start()
    deadline = time.monotonic() + 1.0
    while recorder.summary()["sample_count"] < 1 and time.monotonic() < deadline:
        time.sleep(0.005)
    summary = recorder.stop()

    assert summary["sample_count"] >= 1
    assert summary["latest_joint_position_rad"] == pytest.approx(
        snapshot.joint_position
    )


def test_validation_treats_edge_stillness_as_advice_and_uses_gripper_tolerance(
    tmp_path,
) -> None:
    episode_path, config = write_validation_fixture(
        tmp_path,
        gripper_position=0.091,
    )

    report = validate_raw_episode(episode_path, config, write_report=False)

    assert report["ok"] is True
    assert report["errors"] == []
    assert any("initial still window" in warning for warning in report["warnings"])


def test_validation_rejects_gripper_feedback_beyond_controller_tolerance(
    tmp_path,
) -> None:
    episode_path, config = write_validation_fixture(
        tmp_path,
        gripper_position=0.094,
    )

    report = validate_raw_episode(episode_path, config, write_report=False)

    assert report["ok"] is False
    assert "gripper position exceeds recorded controller limits" in report["errors"]


def test_validation_keeps_recoverable_camera_timing_as_advice(tmp_path) -> None:
    episode_path, config = write_validation_fixture(
        tmp_path,
        gripper_position=0.04,
    )
    for camera_index, camera in enumerate(config.cameras):
        camera_path = episode_path / "cameras" / camera.video_key
        frames_path = camera_path / "frames.jsonl"
        frames = [json.loads(line) for line in frames_path.read_text().splitlines()]
        for index, frame in enumerate(frames):
            timestamp = index * 50_000_000 + camera_index * 25_000_000
            if camera_index == 1 and index >= 40:
                timestamp += 40_000_000
            frame["host_monotonic_ns"] = timestamp
        frames_path.write_text(
            "".join(json.dumps(frame) + "\n" for frame in frames),
            encoding="utf-8",
        )
        summary_path = camera_path / "camera.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["dropped_queue_frames"] = 3
        summary_path.write_text(json.dumps(summary), encoding="utf-8")

    report = validate_raw_episode(episode_path, config, write_report=False)

    assert report["ok"] is True
    assert report["errors"] == []
    assert any("maximum video gap" in warning for warning in report["warnings"])
    assert any("camera queue drop fraction" in warning for warning in report["warnings"])
    assert any("D455/D405 p99 alignment" in warning for warning in report["warnings"])


def test_validation_rejects_camera_rate_below_training_floor(tmp_path) -> None:
    episode_path, config = write_validation_fixture(
        tmp_path,
        gripper_position=0.04,
    )
    camera = config.wrist_camera
    camera_path = episode_path / "cameras" / camera.video_key
    frames_path = camera_path / "frames.jsonl"
    frames = [
        {"host_monotonic_ns": index * 1_000_000_000, "frame_index": index}
        for index in range(5)
    ]
    frames_path.write_text(
        "".join(json.dumps(frame) + "\n" for frame in frames),
        encoding="utf-8",
    )
    summary_path = camera_path / "camera.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["frame_count"] = len(frames)
    summary_path.write_text(json.dumps(summary), encoding="utf-8")

    report = validate_raw_episode(episode_path, config, write_report=False)

    assert report["ok"] is False
    assert any("effective frame rate" in error for error in report["errors"])


def test_reject_waits_for_camera_writer_to_finish(tmp_path) -> None:
    store = RawEpisodeStore(tmp_path)
    episode = store.begin()
    camera_manifest = (
        episode.path / "cameras" / "exterior_rgb" / "camera.json"
    )
    atomic_json(camera_manifest, {"stage": "recording"})

    with pytest.raises(RuntimeError, match="相机录制仍在写入"):
        store.reject(episode.session_id, "operator discarded episode")

    atomic_json(camera_manifest, {"stage": "aborted"})
    rejected = store.reject(episode.session_id, "operator discarded episode")
    assert rejected.stage == "rejected"
    assert rejected.path.parent == tmp_path / "discarded"


def test_episode_layout_uses_timestamp_and_id(tmp_path) -> None:
    store = RawEpisodeStore(tmp_path)
    accepted_episode = store.begin()

    assert re.fullmatch(
        r"\d{8}-\d{6}-\d{6}_id-[0-9a-f]{8}",
        accepted_episode.session_id,
    )
    assert accepted_episode.path.parent == tmp_path / "in-progress"

    accepted = store.accept(accepted_episode.session_id, {"ok": True})
    discarded_episode = store.begin()
    discarded = store.reject(discarded_episode.session_id, "operator redo")

    assert accepted.path.parent == tmp_path / "success"
    assert discarded.path.parent == tmp_path / "discarded"
    assert store.counts() == {
        "accepted": 1,
        "rejected": 1,
        "inprogress": 0,
        "target": store.config.target_accepted_episodes,
    }
