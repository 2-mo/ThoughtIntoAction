from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import msgpack
import numpy as np
import zmq


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ZERO_SHOT_ROOT = PROJECT_ROOT / "src" / "gr-zeroshot"
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(ZERO_SHOT_ROOT))

from embodied.policy.groot.checkpoint import (  # noqa: E402
    CheckpointError,
    select_checkpoint,
)
from embodied.policy.groot.cameras import (  # noqa: E402
    CameraError,
    DualFrameBuffer,
    RgbFrame,
    default_camera_specs,
)
from embodied.policy.groot.policy import (  # noqa: E402
    NumericSerializer,
    PolicyClient,
    PolicyWorker,
)
from embodied.policy.groot.runtime import (  # noqa: E402
    ACTION_PERIOD_NS,
    ActionRejected,
    ScheduledActionTarget,
    build_observation,
    decode_action_chunk,
    merge_scheduled_actions,
    rtc_options,
    synthetic_observation,
    to_servo_target,
)
import embodied.policy.groot.runtime as runtime_module  # noqa: E402
from embodied.policy.groot.cli import parse_args, parse_live_key  # noqa: E402
from legacy.arx5_control import GripperState, WorkspaceBounds  # noqa: E402
from legacy.arx5_control.cartesian import rpy_to_quaternion_xyzw  # noqa: E402
from legacy.arx5_control.teleoperation import quaternion_multiply_xyzw  # noqa: E402
from embodied.data.groot.geometry import pose_6d_to_eef_9d  # noqa: E402


class FineTunedCheckpointTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "outputs" / "groot-training"
        self.root.mkdir(parents=True)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def make_checkpoint(
        self,
        step: int,
        *,
        complete: bool = True,
        video_keys: list[str] | None = None,
        state_action_keys: list[str] | None = None,
    ) -> Path:
        path = self.root / "run" / f"checkpoint-{step}"
        path.mkdir(parents=True)
        keys = state_action_keys or ["eef_9d", "gripper_position", "joint_position"]
        action_configs = [
            {
                "rep": "RELATIVE",
                "type": "EEF",
                "format": "XYZ_ROT6D",
                "state_key": "eef_9d",
            },
            {
                "rep": "ABSOLUTE",
                "type": "NON_EEF",
                "format": "DEFAULT",
                "state_key": "gripper_position",
            },
            {
                "rep": "RELATIVE",
                "type": "NON_EEF",
                "format": "DEFAULT",
                "state_key": "joint_position",
            },
        ]
        config = {
            "processor_kwargs": {
                "modality_configs": {
                    "new_embodiment": {
                        "video": {
                            "delta_indices": [0],
                            "modality_keys": video_keys
                            or ["exterior_rgb", "wrist_rgb"],
                        },
                        "state": {
                            "delta_indices": [0],
                            "modality_keys": keys,
                        },
                        "action": {
                            "delta_indices": list(range(16)),
                            "modality_keys": keys,
                            "action_configs": action_configs[: len(keys)],
                        },
                        "language": {
                            "delta_indices": [0],
                            "modality_keys": [
                                "annotation.human.task_description"
                            ],
                        },
                    }
                }
            }
        }
        (path / "config.json").write_text("{}", encoding="utf-8")
        (path / "processor_config.json").write_text(
            json.dumps(config), encoding="utf-8"
        )
        (path / "statistics.json").write_text(
            json.dumps({"new_embodiment": {"state": {}, "action": {}}}),
            encoding="utf-8",
        )
        (path / "model.safetensors.index.json").write_text(
            json.dumps({"weight_map": {"model.x": "model-1.safetensors"}}),
            encoding="utf-8",
        )
        (path / "trainer_state.json").write_text(
            json.dumps({"global_step": step}),
            encoding="utf-8",
        )
        if complete:
            (path / "model-1.safetensors").write_bytes(b"weights")
        return path

    def test_selects_highest_complete_checkpoint(self) -> None:
        self.make_checkpoint(500)
        expected = self.make_checkpoint(1000)
        self.make_checkpoint(1500, complete=False)

        selected = select_checkpoint(self.root)

        self.assertEqual(selected.path, expected.resolve())
        self.assertEqual(selected.step, 1000)
        self.assertEqual(selected.embodiment, "new_embodiment")
        self.assertEqual(selected.action_horizon, 16)
        self.assertEqual(selected.video_keys, ("exterior_rgb", "wrist_rgb"))

    def test_accepts_wrist_only_checkpoint(self) -> None:
        path = self.make_checkpoint(2000, video_keys=["wrist_rgb"])

        selected = select_checkpoint(self.root, path)

        self.assertEqual(selected.video_keys, ("wrist_rgb",))

    def test_accepts_wrist_cartesian_only_checkpoint(self) -> None:
        path = self.make_checkpoint(
            1000,
            video_keys=["wrist_rgb"],
            state_action_keys=["eef_9d", "gripper_position"],
        )

        selected = select_checkpoint(self.root, path)

        self.assertEqual(selected.state_keys, ("eef_9d", "gripper_position"))
        self.assertEqual(selected.action_keys, ("eef_9d", "gripper_position"))

    def test_rejects_incompatible_camera_contract(self) -> None:
        path = self.make_checkpoint(1000)
        processor_path = path / "processor_config.json"
        value = json.loads(processor_path.read_text(encoding="utf-8"))
        value["processor_kwargs"]["modality_configs"]["new_embodiment"][
            "video"
        ]["modality_keys"] = ["wrong"]
        processor_path.write_text(json.dumps(value), encoding="utf-8")

        with self.assertRaisesRegex(CheckpointError, "video.modality_keys"):
            select_checkpoint(self.root)

    def test_explicit_checkpoint_must_be_complete(self) -> None:
        path = self.make_checkpoint(500, complete=False)

        with self.assertRaisesRegex(CheckpointError, "incomplete"):
            select_checkpoint(self.root, path)


class PolicyProtocolTest(unittest.TestCase):
    def test_numeric_serializer_round_trip_and_pickle_rejection(self) -> None:
        value = {"image": np.arange(18, dtype=np.uint8).reshape(2, 3, 3)}

        decoded = NumericSerializer.from_bytes(NumericSerializer.to_bytes(value))

        np.testing.assert_array_equal(decoded["image"], value["image"])
        with self.assertRaisesRegex(TypeError, "object-dtype"):
            NumericSerializer.to_bytes(np.array([object()], dtype=object))
        forged = msgpack.packb(
            {b"nd": True, b"kind": b"O", b"data": b"not-a-pickle"}
        )
        with self.assertRaisesRegex(ValueError, "pickle-bearing"):
            NumericSerializer.from_bytes(forged)

    def test_client_uses_official_get_action_envelope(self) -> None:
        requests: list[dict] = []
        context = zmq.Context()
        server = context.socket(zmq.REP)
        server.setsockopt(zmq.LINGER, 0)
        port = server.bind_to_random_port("tcp://127.0.0.1")

        def serve() -> None:
            request = NumericSerializer.from_bytes(server.recv())
            requests.append(request)
            response = [
                {
                    "eef_9d": np.zeros((1, 16, 9), dtype=np.float32),
                    "gripper_position": np.zeros((1, 16, 1), dtype=np.float32),
                    "joint_position": np.zeros((1, 16, 6), dtype=np.float32),
                },
                {},
            ]
            server.send(NumericSerializer.to_bytes(response))

        thread = threading.Thread(target=serve)
        thread.start()
        try:
            with PolicyClient("127.0.0.1", port, timeout_s=1.0) as client:
                actions, info = client.get_action({"video": {}, "state": {}})
        finally:
            thread.join(2.0)
            server.close()
            context.term()

        self.assertFalse(thread.is_alive())
        self.assertEqual(actions["eef_9d"].shape, (1, 16, 9))
        self.assertEqual(info, {})
        self.assertEqual(requests[0]["endpoint"], "get_action")
        self.assertEqual(
            requests[0]["data"],
            {"observation": {"video": {}, "state": {}}, "options": None},
        )

    def test_worker_keeps_latest_pending_observation(self) -> None:
        calls: list[int] = []
        context = zmq.Context()
        server = context.socket(zmq.REP)
        server.setsockopt(zmq.LINGER, 0)
        port = server.bind_to_random_port("tcp://127.0.0.1")
        release_first = threading.Event()

        def serve() -> None:
            for _ in range(2):
                request = NumericSerializer.from_bytes(server.recv())
                identifier = request["data"]["observation"]["id"]
                calls.append(identifier)
                if identifier == 1:
                    release_first.wait(2.0)
                actions = {
                    "eef_9d": np.zeros((1, 16, 9), dtype=np.float32),
                    "gripper_position": np.zeros((1, 16, 1), dtype=np.float32),
                    "joint_position": np.zeros((1, 16, 6), dtype=np.float32),
                }
                server.send(NumericSerializer.to_bytes([actions, {}]))

        thread = threading.Thread(target=serve)
        thread.start()
        worker = PolicyWorker("127.0.0.1", port, timeout_s=2.0)
        try:
            worker.submit({"id": 1}, 1)
            deadline = time.monotonic() + 1.0
            while not calls and time.monotonic() < deadline:
                time.sleep(0.005)
            worker.submit({"id": 2}, 2)
            worker.submit({"id": 3}, 3)
            release_first.set()
            deadline = time.monotonic() + 2.0
            while len(calls) < 2 and time.monotonic() < deadline:
                time.sleep(0.005)
        finally:
            worker.close()
            thread.join(2.0)
            server.close()
            context.term()

        self.assertEqual(calls, [1, 3])


class DualCameraFrameTest(unittest.TestCase):
    @staticmethod
    def frame(timestamp_ns: int, number: int) -> RgbFrame:
        return RgbFrame(
            np.full((4, 6, 3), number, dtype=np.uint8),
            timestamp_ns,
            timestamp_ns / 1e6,
            number,
        )

    def test_uses_fresh_closest_current_pair(self) -> None:
        frames = DualFrameBuffer()
        frames.add("exterior", self.frame(900_000_000, 1))
        frames.add("wrist", self.frame(905_000_000, 2))
        frames.add("exterior", self.frame(1_000_000_000, 3))
        frames.add("wrist", self.frame(1_018_000_000, 4))

        pair = frames.latest_pair(now_ns=1_100_000_000)

        self.assertEqual(pair.exterior.number, 3)
        self.assertEqual(pair.wrist.number, 4)

    def test_rejects_skew_and_stale_frames(self) -> None:
        frames = DualFrameBuffer()
        frames.add("exterior", self.frame(1_000_000_000, 1))
        frames.add("wrist", self.frame(1_051_000_000, 2))
        with self.assertRaisesRegex(CameraError, "超过 50 ms"):
            frames.latest_pair(now_ns=1_100_000_000)

        frames = DualFrameBuffer()
        frames.add("exterior", self.frame(1_000_000_000, 1))
        frames.add("wrist", self.frame(1_010_000_000, 2))
        with self.assertRaisesRegex(CameraError, "过期"):
            frames.latest_pair(now_ns=1_210_000_000)

    def test_default_specs_match_collection_contract(self) -> None:
        exterior, wrist = default_camera_specs()

        self.assertEqual((exterior.width, exterior.height, exterior.fps), (1280, 720, 30))
        self.assertEqual((exterior.exposure_us, exterior.gain), (8000, 64))
        self.assertEqual((wrist.exposure_us, wrist.gain), (10000, 32))
        self.assertEqual(exterior.white_balance_k, wrist.white_balance_k)


class NativePolicyContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.pose = np.asarray([0.30, 0.0, 0.20, 0.0, 0.0, 0.0])
        self.snapshot = SimpleNamespace(
            eef_pose_6d=self.pose,
            joint_position=np.zeros(6),
            gripper_position=0.02,
        )
        exterior = DualCameraFrameTest.frame(1_000_000_000, 1)
        wrist = DualCameraFrameTest.frame(1_010_000_000, 2)
        self.frames = SimpleNamespace(exterior=exterior, wrist=wrist)
        self.workspace = WorkspaceBounds(
            minimum_m=np.asarray([0.15, -0.35, 0.08]),
            maximum_m=np.asarray([0.65, 0.35, 0.55]),
            soft_boundary_m=0.03,
        )

    def actions(self, delta_x: float = 0.0) -> dict[str, np.ndarray]:
        eef = np.repeat(
            pose_6d_to_eef_9d(self.pose)[None, :],
            16,
            axis=0,
        ).astype(np.float32)
        eef[:, 0] += np.linspace(0.0, delta_x, 16, dtype=np.float32)
        return {
            "eef_9d": eef[None],
            "gripper_position": np.full((1, 16, 1), 0.02, dtype=np.float32),
            "joint_position": np.zeros((1, 16, 6), dtype=np.float32),
        }

    def test_observation_matches_finetuned_processor_shapes(self) -> None:
        observation = build_observation(self.frames, self.snapshot, "stack cubes")

        self.assertEqual(observation["video"]["exterior_rgb"].shape, (1, 1, 4, 6, 3))
        self.assertEqual(observation["video"]["wrist_rgb"].dtype, np.uint8)
        self.assertEqual(observation["state"]["eef_9d"].shape, (1, 1, 9))
        self.assertEqual(observation["state"]["joint_position"].shape, (1, 1, 6))
        self.assertEqual(
            observation["language"]["annotation.human.task_description"],
            [["stack cubes"]],
        )

    def test_synthetic_observation_has_same_schema(self) -> None:
        observation = synthetic_observation()

        self.assertEqual(set(observation), {"video", "state", "language"})
        self.assertEqual(observation["video"]["exterior_rgb"].shape[:2], (1, 1))
        self.assertEqual(observation["state"]["joint_position"].dtype, np.float32)

    def test_wrist_only_observation_omits_exterior_camera(self) -> None:
        frames = SimpleNamespace(exterior=None, wrist=self.frames.wrist)

        observation = build_observation(
            frames,
            self.snapshot,
            "stack cubes",
            ("wrist_rgb",),
        )

        self.assertEqual(set(observation["video"]), {"wrist_rgb"})

    def test_decodes_valid_absolute_eef_chunk_and_latency_skip(self) -> None:
        targets = decode_action_chunk(
            self.actions(delta_x=0.015),
            current_pose_6d=self.pose,
            workspace=self.workspace,
            previous_gripper_state=GripperState.CLOSE,
            latency_s=0.10,
        )

        self.assertEqual(len(targets), 13)
        self.assertEqual(targets[0].gripper_state, GripperState.OPEN)
        np.testing.assert_allclose(targets[-1].xyz_m, [0.315, 0.0, 0.20])

    def test_rejects_invalid_arrays_and_limits_unsafe_targets(self) -> None:
        invalid = self.actions()
        invalid["eef_9d"][0, 0, 0] = np.nan
        with self.assertRaisesRegex(ActionRejected, "无效数值"):
            self.decode(invalid)

        invalid = self.actions()
        invalid["joint_position"] = invalid["joint_position"][:, :, :5]
        with self.assertRaisesRegex(ActionRejected, "形状"):
            self.decode(invalid)

        invalid = self.actions()
        invalid["eef_9d"][:, :, 0] = 0.70
        targets = self.decode(invalid)
        self.assertTrue(all(self.workspace.contains(target.xyz_m) for target in targets))

        invalid = self.actions()
        invalid["eef_9d"][0, 1:, 0] += 0.03
        targets = self.decode(invalid)
        previous_xyz = self.pose[:3]
        for target in targets:
            self.assertLessEqual(
                float(np.linalg.norm(target.xyz_m - previous_xyz)),
                0.020,
            )
            previous_xyz = target.xyz_m

    def test_decodes_cartesian_only_chunk(self) -> None:
        actions = self.actions()
        actions.pop("joint_position")

        targets = self.decode(actions)

        self.assertEqual(len(targets), 16)

    def test_clamps_high_latency_to_a_usable_action_tail(self) -> None:
        targets = self.decode(self.actions(), latency_s=0.47)

        self.assertEqual(len(targets), 8)

    def test_latency_skip_does_not_compare_expired_prefix_to_current_pose(self) -> None:
        actions = self.actions()
        actions["eef_9d"][0, :3, 0] = 0.285

        targets = self.decode(actions, latency_s=0.10)

        self.assertEqual(len(targets), 13)
        np.testing.assert_allclose(targets[0].xyz_m, self.pose[:3])

    def test_absolute_target_converts_to_servo_session_delta(self) -> None:
        start_pose = np.asarray([0.30, 0.0, 0.20, 0.2, -0.1, 0.3])
        absolute = rpy_to_quaternion_xyzw(np.asarray([-0.1, 0.2, 0.4]))
        target = self.decode(self.actions())[0]
        target = type(target)(
            np.asarray([0.31, 0.01, 0.21]),
            absolute,
            target.gripper_state,
            target.raw_gripper_m,
        )

        relative = to_servo_target(target, start_pose)
        reconstructed = quaternion_multiply_xyzw(
            relative.delta_orientation_xyzw,
            rpy_to_quaternion_xyzw(start_pose[3:]),
        )

        np.testing.assert_allclose(relative.delta_xyz_m, [0.01, 0.01, 0.01])
        self.assertAlmostEqual(abs(float(np.dot(reconstructed, absolute))), 1.0)

    def test_rtc_options_use_observation_timestamps_and_latency(self) -> None:
        previous = self.actions(delta_x=0.010)

        options = rtc_options(
            previous,
            previous_observation_ns=1_000_000_000,
            observation_ns=1_000_000_000 + 8 * ACTION_PERIOD_NS,
            estimated_latency_s=0.10,
        )

        self.assertIsNotNone(options)
        assert options is not None
        self.assertIs(options["previous_action"], previous)
        self.assertEqual(options["rtc_overlap_steps"], 8)
        self.assertEqual(options["rtc_frozen_steps"], 3)

    def test_timestamp_merge_blends_overlap_without_dropping_safe_tail(self) -> None:
        previous_targets = self.decode(self.actions(delta_x=0.015))
        current_actions = self.actions(delta_x=0.015)
        current_actions["eef_9d"][0, :, 1] += 0.006
        current_targets = self.decode(current_actions)
        origin = 1_000_000_000
        queued = deque(
            [
                ScheduledActionTarget(origin, previous_targets[0]),
                ScheduledActionTarget(origin + ACTION_PERIOD_NS, previous_targets[1]),
                ScheduledActionTarget(origin + 2 * ACTION_PERIOD_NS, previous_targets[2]),
            ]
        )
        incoming = [
            ScheduledActionTarget(origin, current_targets[0]),
            ScheduledActionTarget(origin + ACTION_PERIOD_NS, current_targets[1]),
        ]

        merged = merge_scheduled_actions(queued, incoming, now_ns=origin)

        self.assertEqual(len(merged), 3)
        np.testing.assert_allclose(merged[0].target.xyz_m[1], 0.002, atol=1e-8)
        np.testing.assert_allclose(merged[1].target.xyz_m[1], 0.004, atol=1e-8)
        np.testing.assert_allclose(
            merged[2].target.xyz_m,
            previous_targets[2].xyz_m,
        )

    def decode(
        self,
        actions: dict[str, np.ndarray],
        *,
        latency_s: float = 0.0,
    ):
        return decode_action_chunk(
            actions,
            current_pose_6d=self.pose,
            workspace=self.workspace,
            previous_gripper_state=GripperState.CLOSE,
            latency_s=latency_s,
        )


class ZeroShotCliTest(unittest.TestCase):
    def test_terminal_live_keys_are_minimal_and_case_insensitive(self) -> None:
        self.assertEqual(
            [parse_live_key(value) for value in "sHnQx"],
            ["S", "H", "N", "Q", None],
        )

    def test_cli_defaults_to_latest_finetuned_checkpoint(self) -> None:
        options = parse_args([])

        self.assertIsNone(options.checkpoint)
        self.assertFalse(options.check)
        self.assertFalse(options.smoke)


class RuntimeLifecycleTest(unittest.TestCase):
    @staticmethod
    def snapshot():
        return SimpleNamespace(
            eef_pose_6d=np.asarray([0.30, 0.0, 0.20, 0.0, 0.0, 0.0]),
            joint_position=np.zeros(6),
            joint_velocity=np.zeros(6),
            joint_torque=np.zeros(6),
            gripper_position=0.02,
            gripper_velocity=0.0,
            gripper_torque=0.0,
            timestamp=1.0,
        )

    def make_controller(self):
        snapshot = self.snapshot()

        class Controller:
            def __init__(self):
                self.mode = runtime_module.ControllerMode.DISCONNECTED
                self.calls: list[str] = []

            def connect(self, *, stabilize: bool):
                self.calls.append(f"connect:{stabilize}")
                self.mode = runtime_module.ControllerMode.DAMPING
                return snapshot

            def activate(self):
                self.calls.append("activate")
                self.mode = runtime_module.ControllerMode.ACTIVE
                return snapshot

            def damping(self):
                self.calls.append("damping")
                self.mode = runtime_module.ControllerMode.DAMPING
                return snapshot

            def close(self, *, emergency: bool):
                self.calls.append(f"close:{emergency}")
                self.mode = runtime_module.ControllerMode.CLOSED

        return Controller()

    def test_quit_stops_servo_worker_and_returns_home(self) -> None:
        controller = self.make_controller()
        snapshot = self.snapshot()
        lifecycle: list[str] = []
        start_options: dict[str, float] = {}

        class Servo:
            def __init__(self, *_args, **_kwargs):
                pass

            def start(self, **_kwargs):
                start_options.update(_kwargs)
                lifecycle.append("servo-start")
                return snapshot

            def stop(self, reason, *, emergency):
                lifecycle.append(f"servo-stop:{reason}:{emergency}")

        class Worker:
            def __init__(self, *_args, **_kwargs):
                lifecycle.append("worker-start")

            def close(self):
                lifecycle.append("worker-close")

        runtime = runtime_module.FineTunedRuntime(
            SimpleNamespace(),
            policy_host="127.0.0.1",
            policy_port=5555,
            controller=controller,
        )

        def return_home() -> None:
            lifecycle.append("return-home")
            controller.damping()
            runtime._return_home_required = False

        with (
            patch.object(runtime_module, "TeleoperationServo", Servo),
            patch.object(runtime_module, "PolicyWorker", Worker),
            patch.object(runtime, "return_home", side_effect=return_home),
        ):
            command = runtime.run_task("task", lambda: "Q")

        self.assertEqual(command, "Q")
        self.assertEqual(start_options["open_width_m"], 0.080)
        self.assertEqual(
            controller.calls,
            ["connect:False", "activate", "damping"],
        )
        self.assertEqual(
            lifecycle,
            [
                "servo-start",
                "worker-start",
                "worker-close",
                "servo-stop:终端命令 Q:False",
                "return-home",
            ],
        )

    def test_close_returns_home_before_disconnect(self) -> None:
        controller = self.make_controller()
        runtime = runtime_module.FineTunedRuntime(
            SimpleNamespace(),
            policy_host="127.0.0.1",
            policy_port=5555,
            controller=controller,
        )
        runtime._connected = True
        runtime._return_home_required = True

        def return_home() -> None:
            controller.calls.append("return-home")
            runtime._return_home_required = False

        with patch.object(runtime, "return_home", side_effect=return_home):
            runtime.close()

        self.assertEqual(controller.calls, ["return-home", "close:False"])

    def test_servo_start_failure_still_enters_damping(self) -> None:
        controller = self.make_controller()
        lifecycle: list[str] = []

        class FailingServo:
            def __init__(self, *_args, **_kwargs):
                pass

            def start(self, **_kwargs):
                raise RuntimeError("start failed")

            def stop(self, reason, *, emergency):
                lifecycle.append(f"stop:{reason}:{emergency}")

        runtime = runtime_module.FineTunedRuntime(
            SimpleNamespace(),
            policy_host="127.0.0.1",
            policy_port=5555,
            controller=controller,
        )
        with patch.object(runtime_module, "TeleoperationServo", FailingServo):
            with self.assertRaisesRegex(RuntimeError, "start failed"):
                runtime.run_task("task", lambda: None)

        self.assertEqual(
            controller.calls,
            ["connect:False", "activate", "damping"],
        )
        self.assertEqual(lifecycle, ["stop:start failed:False"])


if __name__ == "__main__":
    unittest.main()
