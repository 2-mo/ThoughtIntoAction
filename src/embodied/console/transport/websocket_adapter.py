from __future__ import annotations

import json
from types import SimpleNamespace
from typing import Any

from fastapi import WebSocket
from starlette.websockets import WebSocketDisconnect, WebSocketState

from embodied.console.protocol import CommandCatalog, CommandRequest, StreamRequest


EVENT_NAMES = {
    "hello": "session.ready",
    "fatal": "session.fatal",
    "state": "state.snapshot",
    "motion_state": "motion.progress",
    "command_started": "command.started",
    "teleop_state": "teleoperation.state",
    "servo_state": "servo.state",
    "payload_calibration": "calibration.payload",
    "gripper_calibration": "calibration.gripper",
    "handeye_calibration": "calibration.hand-eye",
    "handeye_plan": "calibration.hand-eye-plan",
    "handeye_preview": "calibration.hand-eye-preview",
    "handeye_validation": "calibration.hand-eye-validation",
}


class ApiWebSocket:
    """Translate the public v1 protocol into the proven control kernel calls."""

    def __init__(self, websocket: WebSocket, catalog: CommandCatalog) -> None:
        self.websocket = websocket
        self.catalog = catalog
        self.request = SimpleNamespace(path=str(websocket.url))
        self.remote_address = websocket.client

    def __aiter__(self) -> "ApiWebSocket":
        return self

    async def __anext__(self) -> str:
        while True:
            try:
                data = await self.websocket.receive_text()
            except WebSocketDisconnect as exc:
                raise StopAsyncIteration from exc
            except RuntimeError as exc:
                if (
                    getattr(self.websocket, "application_state", None)
                    == WebSocketState.DISCONNECTED
                ):
                    raise StopAsyncIteration from exc
                raise
            try:
                raw = json.loads(data)
                if raw.get("type") == "command":
                    request = CommandRequest.model_validate(raw)
                    spec = self.catalog.resolve(request.feature, request.command)
                    message = {
                        "type": "command",
                        "id": request.request_id,
                        "action": spec.kernel_action,
                        "params": request.payload,
                    }
                else:
                    request = StreamRequest.model_validate(raw)
                    message = self._stream_message(request)
            except (
                AttributeError,
                ValueError,
                TypeError,
                json.JSONDecodeError,
            ) as exc:
                await self.websocket.send_json(
                    {"type": "protocol.error", "data": {"message": str(exc)}}
                )
                continue
            return json.dumps(message, separators=(",", ":"))

    def _stream_message(self, request: StreamRequest) -> dict[str, Any]:
        if request.feature == "teleoperation" and request.stream == "target":
            payload = dict(request.payload)
            return {
                "type": "teleop",
                "seq": request.sequence,
                "action": request.command,
                **(
                    {"session_id": payload.pop("session_id")}
                    if "session_id" in payload
                    else {}
                ),
                **(
                    {"client_time_ms": payload.pop("client_time_ms")}
                    if "client_time_ms" in payload
                    else {}
                ),
                "params": payload,
            }
        if request.feature == "arm" and request.stream == "cartesian":
            kind = "cartesian"
        elif request.feature == "gripper" and request.stream == "position":
            kind = "gripper"
        else:
            raise ValueError(
                f"Unsupported stream: {request.feature}.{request.stream}"
            )
        return {
            "type": "servo",
            "kind": kind,
            "seq": request.sequence,
            "action": request.command,
            "params": request.payload,
        }

    async def send(self, data: str | bytes) -> None:
        raw = json.loads(data)
        try:
            await self.websocket.send_json(self._public_event(raw))
        except (WebSocketDisconnect, RuntimeError):
            pass

    async def close(self, code: int = 1000, reason: str = "") -> None:
        try:
            await self.websocket.close(code=code, reason=reason)
        except (WebSocketDisconnect, RuntimeError):
            pass

    def _public_event(self, raw: dict[str, Any]) -> dict[str, Any]:
        event_type = raw.get("type")
        if event_type == "command_result":
            name = "command.succeeded" if raw.get("ok") else "command.failed"
        else:
            name = EVENT_NAMES.get(str(event_type), f"runtime.{event_type}")
        feature, command = self.catalog.public_name(raw.get("action"))
        payload = {
            key: value
            for key, value in raw.items()
            if key not in {"type", "id", "action", "ok"}
        }
        if event_type == "hello":
            payload.pop("protocol", None)
            payload["api_version"] = 1
        event: dict[str, Any] = {"type": name, "data": payload}
        if "id" in raw:
            event["request_id"] = raw["id"]
        if "action" in raw:
            event["feature"] = feature
            event["command"] = command
        if event_type == "command_result":
            event["ok"] = bool(raw.get("ok"))
        return event
