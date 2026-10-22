from __future__ import annotations

import asyncio
import json
from pathlib import Path
import ssl
from typing import Any
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

from websockets.asyncio.client import ClientConnection, connect
from websockets.exceptions import ConnectionClosed


class CollectionCameraClient:
    """Own the host-side control connection for dual-camera episode recording."""

    def __init__(
        self,
        url: str,
        *,
        token: str | None,
        origin: str | None,
        ca_cert: Path | None,
    ) -> None:
        self._url = self._with_token(url, token)
        self._origin = origin
        self._ssl = self._ssl_context(url, ca_cert)
        self._connection: ClientConnection | None = None
        self._lock = asyncio.Lock()
        self._sequence = 0

    @staticmethod
    def _with_token(url: str, token: str | None) -> str:
        if token is None:
            return url
        parts = urlsplit(url)
        query = parse_qsl(parts.query, keep_blank_values=True)
        query.append(("token", token))
        return urlunsplit(parts._replace(query=urlencode(query)))

    @staticmethod
    def _ssl_context(url: str, ca_cert: Path | None) -> ssl.SSLContext | None:
        if urlsplit(url).scheme != "wss":
            return None
        return ssl.create_default_context(
            cafile=None if ca_cert is None else str(ca_cert)
        )

    async def _ensure_connected(self) -> ClientConnection:
        connection = self._connection
        if connection is not None and connection.state.name == "OPEN":
            return connection
        self._connection = await connect(
            self._url,
            origin=self._origin,
            ssl=self._ssl,
            proxy=None,
            open_timeout=8.0,
            ping_interval=10.0,
            ping_timeout=10.0,
            max_size=256 * 1024,
        )
        return self._connection

    async def request(
        self,
        message_type: str,
        *,
        session_id: str | None = None,
        video_key: str | None = None,
        timeout_s: float = 20.0,
    ) -> dict[str, Any]:
        async with self._lock:
            self._sequence += 1
            request_id = f"control-camera-{self._sequence}"
            payload: dict[str, Any] = {
                "type": message_type,
                "id": request_id,
            }
            if session_id is not None:
                payload["session_id"] = session_id
            if video_key is not None:
                payload["video_key"] = video_key
            try:
                async with asyncio.timeout(timeout_s):
                    connection = await self._ensure_connected()
                    await connection.send(
                        json.dumps(payload, separators=(",", ":"))
                    )
                    raw_response = await connection.recv()
                response = json.loads(raw_response)
            except (
                asyncio.TimeoutError,
                ConnectionClosed,
                OSError,
                TypeError,
                ValueError,
                json.JSONDecodeError,
            ) as exc:
                await self._drop_connection()
                raise RuntimeError(f"主机内部双相机服务不可用：{exc}") from exc
            if not isinstance(response, dict) or response.get("id") != request_id:
                await self._drop_connection()
                raise RuntimeError("主机内部双相机服务返回了无效响应")
            if response.get("ok") is False:
                raise RuntimeError(str(response.get("error") or "双相机请求失败"))
            return response

    async def _drop_connection(self) -> None:
        connection = self._connection
        self._connection = None
        if connection is not None:
            try:
                await connection.close()
            except (ConnectionClosed, OSError):
                pass

    async def close(self) -> None:
        async with self._lock:
            await self._drop_connection()
