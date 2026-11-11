"""Remote π0.5 client: wire format, observation building and the websocket handshake."""
import threading
import unittest

import numpy as np

from embodied.policy.remote.client import PolicyError, RemotePolicy
from embodied.policy.remote.runner import ChunkQueue, blend_into, build_state, clip_actions
from embodied.policy.remote.wire import packb, unpackb


class WireTests(unittest.TestCase):
    def test_ndarray_and_scalar_round_trip(self):
        payload = {
            "state": np.arange(14, dtype=np.float32),
            "image": np.zeros((4, 5, 3), dtype=np.uint8),
            "valid": np.bool_(True),
            "prompt": "sweep",
            "count": 3,
        }
        decoded = unpackb(packb(payload))
        self.assertIsInstance(decoded["state"], np.ndarray)
        self.assertEqual(decoded["state"].dtype, np.float32)
        np.testing.assert_array_equal(decoded["state"], payload["state"])
        self.assertEqual(decoded["image"].shape, (4, 5, 3))
        self.assertTrue(bool(decoded["valid"]))
        self.assertEqual(decoded["prompt"], "sweep")
        self.assertEqual(decoded["count"], 3)

    def test_encoded_keys_match_the_server_extension(self):
        import msgpack

        raw = msgpack.unpackb(packb({"a": np.zeros(2, dtype=np.float64)}), raw=False)
        self.assertTrue(raw["a"][b"__ndarray__"])
        self.assertEqual(raw["a"][b"dtype"], b"float64")
        self.assertEqual(list(raw["a"][b"shape"]), [2])


class StateTests(unittest.TestCase):
    def test_state_is_twelve_joints_and_two_grippers(self):
        status = {"q": [0.1] * 12, "gripper_widths": [0.08, 0.01], "gripper_width": 0.08}
        state = build_state(status)
        self.assertEqual(state.shape, (14,))
        self.assertAlmostEqual(float(state[12]), 0.08, places=6)
        self.assertAlmostEqual(float(state[13]), 0.01, places=6)

    def test_single_gripper_falls_back_to_both_arms(self):
        state = build_state({"q": [0.0] * 12, "gripper_width": 0.03})
        self.assertEqual(state.shape, (14,))
        self.assertAlmostEqual(float(state[13]), 0.03, places=6)

    def test_single_arm_kernel_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "12 关节"):
            build_state({"q": [0.0] * 6, "gripper_width": 0.03})

    def test_actions_are_clipped_to_kernel_limits(self):
        actions = np.array([[9.0] * 12 + [5.0, -1.0]], dtype=np.float32)
        clipped = clip_actions(actions, np.zeros(12), np.ones(12), 0.082)
        self.assertTrue(np.all(clipped[0, :12] == 1.0))
        self.assertAlmostEqual(float(clipped[0, 12]), 0.082, places=6)
        self.assertAlmostEqual(float(clipped[0, 13]), 0.0, places=6)


class ObservationTests(unittest.TestCase):
    def test_frames_are_sent_as_rgb(self):
        from embodied.policy.remote.runner import build_observation

        bgr = np.zeros((4, 5, 3), dtype=np.uint8)
        bgr[..., 0], bgr[..., 1], bgr[..., 2] = 10, 20, 30

        class Camera:
            def read(self):
                return {"image": bgr}

        class Session:
            def status(self):
                return {"q": [0.0] * 12, "gripper_widths": [0.0, 0.0]}

        observation, _status = build_observation(Session(), {"wrist_rgb": Camera()}, "task")
        # Training frames are RGB (the export flips BGR before writing).
        self.assertEqual(int(observation["wrist_rgb"][0, 0, 0]), 30)
        self.assertEqual(int(observation["wrist_rgb"][0, 0, 2]), 10)


class QueueTests(unittest.TestCase):
    def test_adopted_chunk_keeps_the_request_tick(self):
        queue = ChunkQueue()
        queue.request(12)
        queue.deliver(np.zeros((2, 14), dtype=np.float32), queue.request_tick)
        adopted = queue.adopt(20, 0)
        self.assertEqual(adopted["request_tick"], 12)
        self.assertEqual(adopted["arrival_tick"], 20)
        self.assertEqual(adopted["skipped"], 0)  # too short to skip safely

    def test_adopt_skips_the_steps_spent_waiting_for_the_chunk(self):
        queue = ChunkQueue()
        queue.request(0)
        actions = np.arange(50, dtype=np.float32).reshape(50, 1).repeat(14, axis=1)
        queue.deliver(actions, queue.request_tick)
        adopted = queue.adopt(20, 0)
        self.assertEqual(adopted["skipped"], 20)
        # The arm already executed the first 20 steps while inference ran.
        self.assertEqual(float(queue.take()[0]), 20.0)


class BlendTests(unittest.TestCase):
    def test_blend_pulls_only_the_head_toward_the_old_chunk(self):
        new = np.zeros((4, 14), dtype=np.float32)
        old = np.ones((2, 14), dtype=np.float32)
        blended = blend_into(new, old, 2)
        self.assertAlmostEqual(float(blended[0, 0]), 1.0, places=6)  # full weight on the first step
        self.assertAlmostEqual(float(blended[1, 0]), 0.5, places=6)
        self.assertAlmostEqual(float(blended[2, 0]), 0.0, places=6)

    def test_blend_is_a_no_op_without_old_actions(self):
        new = np.zeros((3, 14), dtype=np.float32)
        np.testing.assert_array_equal(blend_into(new, None, 4), new)


class _FakeServer:
    """Minimal openpi policy server: one metadata message, then one reply per request."""

    def __init__(self, actions):
        self.actions = actions
        self.requests = []

    def serve(self, connection):
        connection.send(packb({}))
        for message in connection:
            self.requests.append(unpackb(message))
            connection.send(packb({"actions": self.actions}))

    def __enter__(self):
        from websockets.sync.server import serve

        self._server = serve(self.serve, "127.0.0.1", 0)
        port = self._server.socket.getsockname()[1]
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self.url = f"ws://127.0.0.1:{port}"
        return self

    def __exit__(self, *exc):
        self._server.shutdown()


class ClientTests(unittest.TestCase):
    def test_handshake_and_single_inference(self):
        actions = np.zeros((50, 14), dtype=np.float32)
        with _FakeServer(actions) as server:
            with RemotePolicy(server.url) as client:
                self.assertEqual(client.metadata, {})
                reply = client.infer({"state": np.zeros(14, dtype=np.float32), "prompt": "p"})
        self.assertEqual(reply["actions"].shape, (50, 14))
        self.assertEqual(len(server.requests), 1)
        self.assertEqual(server.requests[0]["prompt"], "p")

    def test_actions_with_wrong_width_are_rejected(self):
        with _FakeServer(np.zeros((50, 7), dtype=np.float32)) as server:
            with RemotePolicy(server.url) as client:
                with self.assertRaises(PolicyError):
                    client.infer({"state": np.zeros(14, dtype=np.float32)})


if __name__ == "__main__":
    unittest.main()
