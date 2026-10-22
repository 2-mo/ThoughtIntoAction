from __future__ import annotations

import json
import unittest

from starlette.websockets import WebSocketState

from embodied.applications.console.features import feature_catalog
from embodied.applications.console.transport.websocket_adapter import ApiWebSocket


class FakeSocket:
    def __init__(self, messages: list[object]) -> None:
        self.messages = [json.dumps(message) for message in messages]
        self.sent: list[dict[str, object]] = []
        self.url = "ws://localhost:8766/api/v1/control"
        self.client = ("127.0.0.1", 50000)
        self.application_state = WebSocketState.CONNECTED

    async def receive_text(self) -> str:
        return self.messages.pop(0)

    async def send_json(self, payload: dict[str, object]) -> None:
        self.sent.append(payload)

    async def close(self, code: int = 1000, reason: str = "") -> None:
        del code, reason


class ApiWebSocketTest(unittest.IsolatedAsyncioTestCase):
    async def test_invalid_message_reports_error_then_keeps_session_alive(self) -> None:
        socket = FakeSocket([
            [],
            {
                "type": "command",
                "request_id": "r2",
                "feature": "arm",
                "command": "state",
            },
        ])
        adapter = ApiWebSocket(socket, feature_catalog())  # type: ignore[arg-type]
        message = json.loads(await adapter.__anext__())
        self.assertEqual(message["action"], "state")
        self.assertEqual(socket.sent[0]["type"], "protocol.error")

    async def test_public_command_is_routed_to_control_kernel(self) -> None:
        socket = FakeSocket(
            [{
                "type": "command",
                "request_id": "r1",
                "feature": "gripper",
                "command": "force-close",
                "payload": {"torque_nm": 0.15},
            }]
        )
        adapter = ApiWebSocket(socket, feature_catalog())  # type: ignore[arg-type]
        message = json.loads(await adapter.__anext__())
        self.assertEqual(message["action"], "gripper_force")
        self.assertEqual(message["id"], "r1")
        self.assertEqual(message["params"], {"torque_nm": 0.15})

    async def test_closed_takeover_socket_ends_iteration_cleanly(self) -> None:
        socket = FakeSocket([])
        socket.application_state = WebSocketState.DISCONNECTED

        async def receive_closed() -> str:
            raise RuntimeError(
                'WebSocket is not connected. Need to call "accept" first.'
            )

        socket.receive_text = receive_closed  # type: ignore[method-assign]
        adapter = ApiWebSocket(socket, feature_catalog())  # type: ignore[arg-type]
        with self.assertRaises(StopAsyncIteration):
            await adapter.__anext__()

    async def test_kernel_result_is_exposed_as_versioned_event(self) -> None:
        socket = FakeSocket([])
        adapter = ApiWebSocket(socket, feature_catalog())  # type: ignore[arg-type]
        await adapter.send(json.dumps({
            "type": "command_result",
            "id": 7,
            "action": "preview_xyz",
            "ok": True,
            "result": {"kind": "cartesian"},
        }))
        self.assertEqual(socket.sent[0], {
            "type": "command.succeeded",
            "request_id": 7,
            "feature": "arm",
            "command": "preview-pose",
            "ok": True,
            "data": {"result": {"kind": "cartesian"}},
        })

    async def test_teleoperation_metadata_stays_outside_target_payload(self) -> None:
        socket = FakeSocket(
            [{
                "type": "stream",
                "feature": "teleoperation",
                "stream": "target",
                "command": "update",
                "sequence": 9,
                "payload": {
                    "session_id": "session-1",
                    "client_time_ms": 1234,
                    "delta_xyz_m": [0, 0, 0],
                },
            }]
        )
        adapter = ApiWebSocket(socket, feature_catalog())  # type: ignore[arg-type]
        message = json.loads(await adapter.__anext__())
        self.assertEqual(message["session_id"], "session-1")
        self.assertEqual(message["client_time_ms"], 1234)
        self.assertEqual(message["params"], {"delta_xyz_m": [0, 0, 0]})


if __name__ == "__main__":
    unittest.main()
