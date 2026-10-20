#!/usr/bin/env python3
"""Receive browser landmarks without importing or controlling the ARX SDK."""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
import hmac
import json
import math
from pathlib import Path
import ssl
import time
from typing import Any
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import ServerConnection, serve


LANDMARK_PATH = "/api/v1/vision/landmarks"


@dataclass
class ClientStats:
    accepted: int = 0
    rejected: int = 0
    last_seq: int = -1
    last_report_at: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mac 浏览器 MediaPipe 数据接收桥（不会连接机械臂）"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--origin", action="append", dest="origins")
    parser.add_argument("--token", help="局域网配对令牌")
    args = parser.parse_args()
    if (args.cert is None) != (args.key is None):
        parser.error("--cert 和 --key 必须同时提供")
    return args


def finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def validate_landmark_list(value: Any, expected_count: int) -> bool:
    if not isinstance(value, list) or len(value) != expected_count:
        return False
    for landmark in value:
        if not isinstance(landmark, dict):
            return False
        if not all(finite_number(landmark.get(axis)) for axis in ("x", "y", "z")):
            return False
    return True


def validate_payload(payload: Any, stats: ClientStats) -> tuple[bool, str]:
    if not isinstance(payload, dict):
        return False, "payload must be an object"
    if payload.get("version") != 1 or payload.get("type") != "landmarks":
        return False, "unsupported message version or type"
    seq = payload.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq <= stats.last_seq:
        return False, "sequence must increase"
    if not finite_number(payload.get("client_time_ms")):
        return False, "invalid client timestamp"
    if payload.get("selected_side") not in ("left", "right"):
        return False, "selected_side must be left or right"
    pose = payload.get("pose")
    if pose is not None and not validate_landmark_list(pose, 33):
        return False, "pose must contain 33 finite landmarks"
    pose_world = payload.get("pose_world")
    if pose_world is not None and not validate_landmark_list(pose_world, 33):
        return False, "pose_world must contain 33 finite landmarks"
    hand = payload.get("hand")
    if hand is not None and not validate_landmark_list(hand, 21):
        return False, "hand must contain 21 finite landmarks"
    hand_world = payload.get("hand_world")
    if hand_world is not None and not validate_landmark_list(hand_world, 21):
        return False, "hand_world must contain 21 finite landmarks"
    if pose is None and hand is None:
        return False, "message has no tracked landmarks"
    pinch_ratio = payload.get("pinch_ratio")
    if pinch_ratio is not None and not finite_number(pinch_ratio):
        return False, "invalid pinch ratio"
    stats.last_seq = seq
    return True, "ok"


def authorized(websocket: ServerConnection, token: str | None) -> bool:
    if token is None:
        return True
    request = websocket.request
    if request is None:
        return False
    values = parse_qs(urlsplit(request.path).query).get("token", [])
    return len(values) == 1 and hmac.compare_digest(values[0], token)


def canonical_path(websocket: ServerConnection) -> bool:
    request = websocket.request
    return request is not None and urlsplit(request.path).path == LANDMARK_PATH


async def handle_client(
    websocket: ServerConnection, token: str | None = None
) -> None:
    if not canonical_path(websocket):
        await websocket.close(code=1008, reason="unsupported landmark path")
        return
    if not authorized(websocket, token):
        await websocket.close(code=1008, reason="invalid pairing token")
        return
    peer = websocket.remote_address
    stats = ClientStats(last_report_at=time.monotonic())
    print(f"浏览器已连接：{peer}")
    try:
        async for message in websocket:
            received_at = time.time() * 1000.0
            try:
                payload = json.loads(message)
            except (json.JSONDecodeError, TypeError):
                stats.rejected += 1
                await websocket.send(
                    json.dumps({"type": "ack", "accepted": False, "error": "bad json"})
                )
                continue

            accepted, reason = validate_payload(payload, stats)
            if accepted:
                stats.accepted += 1
            else:
                stats.rejected += 1
            await websocket.send(
                json.dumps(
                    {
                        "type": "ack",
                        "accepted": accepted,
                        "seq": payload.get("seq"),
                        "server_time_ms": received_at,
                        "error": None if accepted else reason,
                    },
                    separators=(",", ":"),
                )
            )

            now = time.monotonic()
            if now - stats.last_report_at >= 1.0:
                pose_ok = payload.get("pose") is not None
                hand_ok = payload.get("hand") is not None
                print(
                    f"seq={stats.last_seq} accepted={stats.accepted} "
                    f"rejected={stats.rejected} pose={pose_ok} hand={hand_ok}"
                )
                stats.last_report_at = now
    finally:
        print(
            f"浏览器已断开：{peer}；accepted={stats.accepted}, "
            f"rejected={stats.rejected}"
        )


async def run_server(args: argparse.Namespace) -> None:
    origins = args.origins or [
        "http://localhost:8000",
        "http://127.0.0.1:8000",
    ]
    ssl_context = None
    if args.cert is not None:
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.minimum_version = ssl.TLSVersion.TLSv1_2
        ssl_context.load_cert_chain(args.cert, args.key)
    async with serve(
        lambda websocket: handle_client(websocket, args.token),
        args.host,
        args.port,
        origins=origins,
        ssl=ssl_context,
        max_size=64 * 1024,
        max_queue=2,
        ping_interval=10,
        ping_timeout=10,
    ):
        scheme = "wss" if ssl_context is not None else "ws"
        print(f"视觉桥监听 {scheme}://{args.host}:{args.port}{LANDMARK_PATH}")
        print("只校验并显示关键点状态；不会连接或驱动机械臂。")
        await asyncio.Future()


def main() -> int:
    args = parse_args()
    try:
        asyncio.run(run_server(args))
    except KeyboardInterrupt:
        print("\n视觉桥已停止。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
