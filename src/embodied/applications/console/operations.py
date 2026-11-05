"""Whitelisted long-running operations started from the web console.

The browser never sends a command line: it names a kind and parameters, and this
module maps that to a fixed ``./run.sh`` invocation. Only processes started here
can be stopped here.
"""
from __future__ import annotations

import asyncio
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from embodied.contracts.paths import PROJECT_ROOT
from embodied.contracts.runtime.control import ControlClient, live_runs
from embodied.core.project import port

RUNS = PROJECT_ROOT / "outputs/runtime/control"
LOG_DIR = PROJECT_ROOT / "outputs/logs/operations"
BOOTSTRAP_PYTHON = PROJECT_ROOT / ".venv/bin/python"
RUN_SH = PROJECT_ROOT / "run.sh"

KINDS = ("kernel", "camera", "sensor", "collect", "policy")
PROFILES = ("sim-manual", "real-manual", "real-quest", "real-dual-quest")
POLICY_KINDS = ("sft", "zeroshot")
SECONDS_RANGE = (1, 300)
TEXT_LIMIT = 500
LOG_TAIL_BYTES = 256 * 1024


class OperationError(Exception):
    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class Operation:
    kind: str
    argv: list[str]
    log_path: Path
    started_at: float
    run_id: str | None = None
    process: asyncio.subprocess.Process | None = None
    state: str = "starting"
    exit_code: int | None = None


def _text(value: object, label: str, *, required: bool = True) -> str:
    text = str(value or "").strip()
    if required and not text:
        raise OperationError(f"{label}不能为空")
    if len(text) > TEXT_LIMIT:
        raise OperationError(f"{label}超过 {TEXT_LIMIT} 字")
    if any(ord(character) < 32 for character in text):
        raise OperationError(f"{label}包含控制字符")
    return text


def _seconds(value: object, default: int) -> int:
    try:
        seconds = int(value if value is not None else default)
    except (TypeError, ValueError):
        raise OperationError("时长必须是整数秒") from None
    if not SECONDS_RANGE[0] <= seconds <= SECONDS_RANGE[1]:
        raise OperationError(f"时长必须在 {SECONDS_RANGE[0]}–{SECONDS_RANGE[1]} 秒之间")
    return seconds


def _enabled_cameras() -> list[str]:
    from embodied.core.project import load_config

    return [
        key
        for key, options in load_config().get("cameras", {}).items()
        if options.get("enabled", False)
    ]


def _collect_cameras(value: object) -> list[str]:
    enabled = _enabled_cameras()
    if value is None:
        if not enabled:
            raise OperationError("没有启用的相机")
        return enabled
    if not isinstance(value, list) or not value:
        raise OperationError("相机列表格式不正确")
    chosen = [str(item) for item in value]
    unknown = [item for item in chosen if item not in enabled]
    if unknown:
        raise OperationError(f"未启用的相机：{', '.join(unknown)}")
    return chosen


def _collect_output(value: object) -> str:
    text = _text(value, "输出目录", required=False) or "outputs/datasets/runtime"
    root = (PROJECT_ROOT / "outputs/datasets").resolve()
    path = Path(text)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    path = path.resolve()
    if root not in path.parents:
        raise OperationError("输出目录必须在 outputs/datasets 下")
    return str(path.relative_to(PROJECT_ROOT))


def _port_listening(number: int) -> bool:
    """Read the TCP listen table instead of connecting."""
    for table in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(table, encoding="ascii") as handle:
                next(handle, None)
                for line in handle:
                    fields = line.split()
                    if (
                        len(fields) > 3
                        and fields[3] == "0A"
                        and int(fields[1].split(":")[1], 16) == number
                    ):
                        return True
        except (OSError, ValueError, IndexError):
            continue
    return False


def _resolve_run(value: object) -> Path:
    if value:
        path = Path(str(value))
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        path = path.resolve()
        if RUNS.resolve() not in path.parents or not (path / "runtime.json").is_file():
            raise OperationError("运行目录必须是 outputs/runtime/control 下的有效内核")
        return path
    runs = live_runs(RUNS)
    if not runs:
        raise OperationError("没有运行中的内核")
    return runs[-1]


class OperationManager:
    """At most one process per kind; stop only what this manager started."""

    def __init__(self, *, host: str = "127.0.0.1", port: int = 8000) -> None:
        self.host = host
        self.port = port
        self._operations: dict[str, Operation] = {}
        self._lock = asyncio.Lock()

    # Read

    def snapshot(self) -> list[dict[str, object]]:
        return [self._describe(operation) for operation in self._operations.values()]

    def log_tail(self, kind: str, lines: int = 80) -> list[str]:
        operation = self._operations.get(kind)
        if operation is None:
            raise OperationError("该操作尚未启动", 404)
        try:
            size = operation.log_path.stat().st_size
            with operation.log_path.open("rb") as handle:
                handle.seek(max(0, size - LOG_TAIL_BYTES))
                data = handle.read()
        except OSError:
            return []
        return data.decode("utf-8", "replace").splitlines()[-lines:]

    # Write

    async def start(self, kind: str, payload: dict[str, object]) -> dict[str, object]:
        if kind not in KINDS:
            raise OperationError("未知操作", 404)
        async with self._lock:
            existing = self._operations.get(kind)
            if existing is not None and existing.state in ("starting", "running", "stopping"):
                raise OperationError(f"{kind} 正在运行，先停止它", 409)
            argv, run_id = self._build(kind, payload or {})
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            operation = Operation(
                kind=kind, argv=argv, run_id=run_id, started_at=time.time(),
                log_path=LOG_DIR / f"{kind}-{stamp}.log",
            )
            env = os.environ.copy()
            env["QUEST_BOOTSTRAP_PYTHON"] = str(BOOTSTRAP_PYTHON)
            try:
                with operation.log_path.open("ab") as log:
                    operation.process = await asyncio.create_subprocess_exec(
                        *argv, cwd=PROJECT_ROOT, env=env,
                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                    )
            except OSError as exc:
                raise OperationError(f"启动失败：{exc}", 500) from exc
            operation.state = "running"
            self._operations[kind] = operation
            asyncio.create_task(self._watch(operation))
            return self._describe(operation)

    async def stop(self, kind: str) -> dict[str, object]:
        operation = self._operations.get(kind)
        if operation is None or operation.process is None or operation.state not in (
            "starting", "running", "stopping"
        ):
            raise OperationError("该操作未在运行", 404)
        operation.state = "stopping"
        # Graceful stop: only the collector gets SIGINT, so it can close the
        # recorder (and its ffmpeg child) itself. Signalling the whole group
        # would kill ffmpeg mid-encode and mark the episode failed.
        await self._signal(operation, signal.SIGINT, group=False)
        try:
            await asyncio.wait_for(operation.process.wait(), timeout=15)
        except asyncio.TimeoutError:
            await self._signal(operation, signal.SIGTERM)
            try:
                await asyncio.wait_for(operation.process.wait(), timeout=5)
            except asyncio.TimeoutError:
                await self._signal(operation, signal.SIGKILL)
                await operation.process.wait()
        operation.state = "stopped" if operation.exit_code in (None, 0) else "failed"
        return self._describe(operation)

    async def shutdown(self) -> None:
        running = [
            operation for operation in self._operations.values()
            if operation.process is not None
            and operation.state in ("starting", "running", "stopping")
        ]
        for operation in running:
            await self._signal(operation, signal.SIGINT, group=False)
        for operation in running:
            try:
                await asyncio.wait_for(operation.process.wait(), timeout=10)
            except asyncio.TimeoutError:
                await self._signal(operation, signal.SIGKILL)

    # Internals

    async def _watch(self, operation: Operation) -> None:
        code = await operation.process.wait()
        operation.exit_code = code
        if operation.state != "stopping":
            operation.state = "stopped" if code == 0 else "failed"

    async def _signal(self, operation: Operation, number: int, *, group: bool = True) -> None:
        if operation.process is None or operation.process.returncode is not None:
            return
        try:
            if group:
                os.killpg(os.getpgid(operation.process.pid), number)
            else:
                operation.process.send_signal(number)
        except (ProcessLookupError, PermissionError):
            pass

    def _describe(self, operation: Operation) -> dict[str, object]:
        return {
            "kind": operation.kind,
            "state": operation.state,
            "pid": operation.process.pid if operation.process else None,
            "started_at": operation.started_at,
            "exit_code": operation.exit_code,
            "run_id": operation.run_id,
            "log": operation.log_path.name,
            "argv": operation.argv,
        }

    def _build(self, kind: str, payload: dict[str, object]) -> tuple[list[str], str | None]:
        if kind == "kernel":
            return self._kernel(payload)
        if kind == "camera":
            return self._camera(), None
        if kind == "sensor":
            return self._sensor()
        if kind == "collect":
            return self._collect(payload)
        return self._policy(payload)

    def _kernel(self, payload: dict[str, object]) -> tuple[list[str], str | None]:
        profile = str(payload.get("profile") or "sim-manual")
        if profile not in PROFILES:
            raise OperationError("不支持的运行配置")
        if live_runs(RUNS):
            raise OperationError("已有内核在运行，先停止它", 409)
        if profile.startswith("real"):
            if payload.get("confirm") is not True:
                raise OperationError("真机启动会使能电机，需要确认", 409)
            argv = ["bash", str(PROJECT_ROOT / "scripts/arm/kernel_real.sh"),
                    "--profile", profile, "--yes"]
        else:
            argv = ["bash", str(RUN_SH), "kernel", "profiles", "start", "--profile", profile]
        return argv, None

    def _camera(self) -> list[str]:
        number = port("camera_webrtc")
        collect = self._operations.get("collect")
        if collect is not None and collect.state in ("starting", "running", "stopping"):
            raise OperationError("采集中，启动视频会与采集争用相机", 409)
        if _port_listening(number):
            raise OperationError("相机服务已在运行", 409)
        return [
            "bash", str(RUN_SH), "camera", "rgbd",
            "--host", self.host, "--no-external",
            "--origin", f"http://{self.host}:{self.port}",
            "--origin", f"http://127.0.0.1:{self.port}",
        ]

    def _sensor(self) -> tuple[list[str], None]:
        """Shared camera service used by collection recording and previews."""
        from embodied.devices.sensors.client import socket_path

        path = str(socket_path())
        try:
            with open("/proc/net/unix", encoding="ascii") as handle:
                if any(line.rstrip().endswith(path) for line in handle):
                    raise OperationError("共享相机服务已在运行", 409)
        except OSError:
            pass
        return ["bash", str(RUN_SH), "camera-service"], None

    def _collect(self, payload: dict[str, object]) -> tuple[list[str], str]:
        if _port_listening(port("camera_webrtc")):
            raise OperationError("控制台视频服务正在使用相机，先停止视频再采集", 409)
        run = _resolve_run(payload.get("run"))
        task = _text(payload.get("task"), "任务描述")
        mode = str(payload.get("mode") or "fixed")
        if mode not in ("fixed", "manual"):
            raise OperationError("采集方式只能是固定时长或手动停止")
        seconds = _seconds(payload.get("seconds"), 30) if mode == "fixed" else 0
        rgb = str(payload.get("rgb") or "mp4")
        if rgb not in ("mp4", "png"):
            raise OperationError("图像格式只能是 mp4 或 png")
        cameras = _collect_cameras(payload.get("cameras"))
        output = _collect_output(payload.get("output"))
        argv = [
            "bash", str(RUN_SH), "collect", "run", "--run", str(run),
            "--task", task, "--duration", str(seconds),
            "--cameras", ",".join(cameras), "--rgb", rgb, "--output", output,
        ]
        return argv, run.name

    def _policy(self, payload: dict[str, object]) -> tuple[list[str], str]:
        if payload.get("confirm") is not True:
            raise OperationError("策略运行会使能机械臂，需要确认", 409)
        run = _resolve_run(payload.get("run"))
        prompt = _text(payload.get("prompt"), "任务描述")
        seconds = _seconds(payload.get("seconds"), 60)
        policy = str(payload.get("policy") or "sft")
        if policy not in POLICY_KINDS:
            raise OperationError("不支持的策略类型")
        try:
            owner = ControlClient(run).status().get("owner")
        except (OSError, EOFError, ValueError, KeyError, RuntimeError) as exc:
            raise OperationError(f"读取内核状态失败：{exc}", 503) from exc
        if owner:
            source = owner.get("source") or "其他客户端"
            raise OperationError(f"内核控制权被 {source} 占用，先关闭控制台页面", 409)
        argv = [
            "bash", str(RUN_SH), "policy", "kernel", "--run", str(run),
            "--prompt", prompt, "--seconds", str(seconds), "--policy", policy,
        ]
        return argv, run.name
