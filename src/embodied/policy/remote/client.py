"""Client for the openpi π0.5 policy server (websocket, msgpack_numpy, request/response).

The server sends one metadata message on connect and then answers each
observation with exactly one prediction; there is no streaming or batching.
See docs/guides/policy-remote.md for the field contract.
"""
from __future__ import annotations

import time
from typing import Any

import numpy as np

from .wire import packb, unpackb


class PolicyError(RuntimeError):
    """Server refused the request or answered with an unusable payload."""


class RemotePolicy:
    def __init__(self, url: str, *, timeout_s: float = 30.0) -> None:
        self.url = url
        self.timeout_s = timeout_s
        self.metadata: dict[str, Any] | None = None
        self._connection = None
        self.last_latency_s: float | None = None

    def __enter__(self) -> "RemotePolicy":
        from websockets.sync.client import connect

        self._connection = connect(self.url, open_timeout=10, close_timeout=2,
                                   max_size=None, compression=None)
        self.metadata = unpackb(self._connection.recv(timeout=self.timeout_s))
        if not isinstance(self.metadata, dict):
            raise PolicyError("模型服务返回的 metadata 不是字典")
        return self

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        if self._connection is None:
            raise PolicyError("尚未连接模型服务")
        started = time.monotonic()
        self._connection.send(packb(observation))
        reply = unpackb(self._connection.recv(timeout=self.timeout_s))
        self.last_latency_s = time.monotonic() - started
        if not isinstance(reply, dict) or "actions" not in reply:
            raise PolicyError("模型服务返回里没有 actions")
        actions = np.asarray(reply["actions"], dtype=np.float32)
        if actions.ndim != 2 or not actions.shape[1] == len(observation["state"]):
            raise PolicyError(f"动作形状 {actions.shape} 与状态维度 {len(observation['state'])} 不一致")
        if not np.isfinite(actions).all():
            raise PolicyError("动作里包含非有限数值")
        reply["actions"] = actions
        return reply

    def close(self) -> None:
        if self._connection is not None:
            try:
                self._connection.close()
            finally:
                self._connection = None

    def __exit__(self, *exc) -> None:
        self.close()
