from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import functools
import os
from pathlib import Path
import socket
import subprocess
import threading
import time
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq

from embodied.policy.groot.checkpoint import FineTunedCheckpoint


class PolicyError(RuntimeError):
    """Raised when the local model server cannot provide a valid response."""


class NumericSerializer:
    """Official msgpack-numpy wire format without object/pickle arrays."""

    @staticmethod
    def to_bytes(value: Any) -> bytes:
        default = functools.partial(NumericSerializer._safe_encode, chain=None)
        return msgpack.packb(value, default=default)

    @staticmethod
    def from_bytes(value: bytes) -> Any:
        object_hook = functools.partial(NumericSerializer._safe_decode, chain=None)
        return msgpack.unpackb(value, object_hook=object_hook, raw=False)

    @staticmethod
    def _safe_encode(value: Any, chain: Any = None) -> Any:
        if isinstance(value, np.ndarray) and value.dtype.kind == "O":
            raise TypeError("refusing to encode object-dtype ndarray")
        return mnp.encode(value, chain=chain)

    @staticmethod
    def _safe_decode(value: Any, chain: Any = None) -> Any:
        if isinstance(value, dict):
            ndarray = value.get(b"nd", value.get("nd"))
            kind = value.get(b"kind", value.get("kind"))
            if ndarray and kind in (b"O", "O"):
                raise ValueError("refusing to decode pickle-bearing ndarray")
        return mnp.decode(value, chain=chain)


class PolicyClient:
    def __init__(self, host: str, port: int, *, timeout_s: float) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_ms = max(1, round(float(timeout_s) * 1000.0))
        self._context = zmq.Context()
        self._closed = False
        self.socket: zmq.Socket
        self._replace_socket()

    def _replace_socket(self) -> None:
        previous = getattr(self, "socket", None)
        if previous is not None:
            previous.close(linger=0)
        self.socket = self._context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(f"tcp://{self.host}:{self.port}")

    def call(self, endpoint: str, data: dict[str, Any] | None = None) -> Any:
        request: dict[str, Any] = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        try:
            self.socket.send(NumericSerializer.to_bytes(request))
            response = NumericSerializer.from_bytes(self.socket.recv())
        except zmq.Again as exc:
            self._replace_socket()
            raise PolicyError(f"GR00T request timed out: {endpoint}") from exc
        except (TypeError, ValueError, msgpack.UnpackException, zmq.ZMQError) as exc:
            self._replace_socket()
            raise PolicyError(f"GR00T protocol failed: {exc}") from exc
        if isinstance(response, dict) and "error" in response:
            raise PolicyError(f"GR00T server error: {response['error']}")
        return response

    def ping(self) -> None:
        response = self.call("ping")
        if not isinstance(response, dict) or response.get("status") != "ok":
            raise PolicyError("GR00T ping returned an invalid response")

    def get_action(
        self,
        observation: dict[str, Any],
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        response = self.call(
            "get_action",
            {"observation": observation, "options": options},
        )
        if not isinstance(response, list) or len(response) != 2:
            raise PolicyError("GR00T get_action response must be [actions, info]")
        actions, info = response
        if not isinstance(actions, dict) or not isinstance(info, dict):
            raise PolicyError("GR00T get_action response types are invalid")
        return actions, info

    def kill_server(self) -> None:
        self.call("kill")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.socket.close(linger=0)
        self._context.term()

    def __enter__(self) -> PolicyClient:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _unused_loopback_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        return int(listener.getsockname()[1])


class ModelServer:
    def __init__(
        self,
        checkpoint: FineTunedCheckpoint,
        *,
        groot_root: Path,
        log_path: Path,
        startup_timeout_s: float = 180.0,
    ) -> None:
        self.checkpoint = checkpoint
        from embodied.core.dependencies import reject_reference_path
        self.groot_root = reject_reference_path(groot_root)
        self.log_path = log_path
        self.startup_timeout_s = float(startup_timeout_s)
        self.port = _unused_loopback_port()
        self.process: subprocess.Popen[bytes] | None = None
        self._log_stream: Any = None

    @property
    def python(self) -> Path:
        from embodied.core.dependencies import dependency_python, dependency_root
        return (dependency_python("groot") if self.groot_root == dependency_root("groot")
                else self.groot_root / ".venv/bin/python")

    def start(self) -> None:
        if not self.python.is_file():
            raise PolicyError(f"GR00T Python is missing: {self.python}; run ./run.sh components setup groot")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_stream = self.log_path.open("wb")
        command = [
            str(self.python),
            "-m",
            "gr00t.eval.run_gr00t_server",
            "--model-path",
            str(self.checkpoint.path),
            "--embodiment-tag",
            self.checkpoint.embodiment,
            "--device",
            "cuda:0",
            "--host",
            "127.0.0.1",
            "--port",
            str(self.port),
        ]
        environment = os.environ.copy()
        # gr00t is installed non-editably in this environment. Never import a checkout.
        environment.pop("PYTHONPATH", None)
        environment.pop("PYTHONHOME", None)
        self.process = subprocess.Popen(
            command,
            cwd=self.log_path.parent,
            env=environment,
            stdout=self._log_stream,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + self.startup_timeout_s
            with PolicyClient("127.0.0.1", self.port, timeout_s=0.5) as client:
                while time.monotonic() < deadline:
                    if self.process.poll() is not None:
                        raise PolicyError(
                            self._startup_failure("GR00T server exited")
                        )
                    try:
                        client.ping()
                        return
                    except PolicyError:
                        time.sleep(0.2)
            raise PolicyError(
                self._startup_failure("GR00T server startup timed out")
            )
        except BaseException:
            self.close()
            raise

    def _startup_failure(self, message: str) -> str:
        if self._log_stream is not None:
            self._log_stream.flush()
        try:
            tail = self.log_path.read_text(encoding="utf-8", errors="replace")[-4000:]
        except OSError:
            tail = ""
        return message if not tail else f"{message}:\n{tail}"

    def close(self) -> None:
        process = self.process
        self.process = None
        if process is not None and process.poll() is None:
            try:
                with PolicyClient("127.0.0.1", self.port, timeout_s=1.0) as client:
                    client.kill_server()
                process.wait(timeout=5.0)
            except (PolicyError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.0)
        if self._log_stream is not None:
            self._log_stream.close()
            self._log_stream = None

    def __enter__(self) -> ModelServer:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


@dataclass(frozen=True)
class InferenceResult:
    observation_ns: int
    response_ns: int
    actions: dict[str, np.ndarray] | None
    info: dict[str, Any] | None
    error: BaseException | None


class PolicyWorker:
    """One policy request at a time; a newer pending observation replaces the old."""

    def __init__(self, host: str, port: int, *, timeout_s: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self._condition = threading.Condition()
        self._pending: tuple[int, dict[str, Any], dict[str, Any] | None] | None = None
        self._results: deque[InferenceResult] = deque(maxlen=1)
        self._closing = False
        self._thread = threading.Thread(
            target=self._run,
            name="arx5-gr00t-policy",
            daemon=True,
        )
        self._thread.start()

    def submit(
        self,
        observation: dict[str, Any],
        observation_ns: int,
        options: dict[str, Any] | None = None,
    ) -> None:
        with self._condition:
            if self._closing:
                raise PolicyError("policy worker is closed")
            self._pending = (int(observation_ns), observation, options)
            self._condition.notify_all()

    def poll(self) -> InferenceResult | None:
        with self._condition:
            return None if not self._results else self._results.popleft()

    def _run(self) -> None:
        with PolicyClient(self.host, self.port, timeout_s=self.timeout_s) as client:
            while True:
                with self._condition:
                    while self._pending is None and not self._closing:
                        self._condition.wait()
                    if self._closing:
                        return
                    observation_ns, observation, options = self._pending
                    self._pending = None
                try:
                    actions, info = client.get_action(observation, options)
                    result = InferenceResult(
                        observation_ns,
                        time.monotonic_ns(),
                        actions,
                        info,
                        None,
                    )
                except BaseException as exc:
                    result = InferenceResult(
                        observation_ns,
                        time.monotonic_ns(),
                        None,
                        None,
                        exc,
                    )
                with self._condition:
                    self._results.append(result)

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._pending = None
            self._condition.notify_all()
        self._thread.join(self.timeout_s + 2.0)
        if self._thread.is_alive():
            raise PolicyError("policy worker did not stop")

    def __enter__(self) -> PolicyWorker:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
