"""Registered policy models: what can be mounted, and whether it is answering.

The registry is a fixed file (`configs/models.toml`); the page only ever sends a
model id, never a command. Mount/unmount run the argv recorded here, so a browser
cannot turn this endpoint into a shell.
"""
from __future__ import annotations

import os
import subprocess
import time
import urllib.parse
import urllib.request
from pathlib import Path

from embodied.core.project import PROJECT_ROOT

try:  # 3.11+
    import tomllib
except ModuleNotFoundError:  # the 3.10 control runtime ships tomli
    import tomli as tomllib

REGISTRY_PATH = PROJECT_ROOT / "configs/models.toml"
LOG_DIR = PROJECT_ROOT / "outputs/logs/models"
STARTING_S = 15 * 60     # a first start compiles for minutes; keep "starting" that long
PROBE_TIMEOUT_S = 0.8


class ModelError(RuntimeError):
    """Registry or launch problem the page should show verbatim."""


def _argv(value, name: str) -> list[str]:
    if not isinstance(value, list) or not value or not all(isinstance(part, str) for part in value):
        raise ModelError(f"模型登记表里的 {name} 必须是字符串数组")
    return list(value)


def load_models(path: Path | None = None) -> list[dict]:
    """Parse and validate the registry; one bad entry must not hide the others."""
    path = Path(path or REGISTRY_PATH)
    if not path.is_file():
        return []
    data = tomllib.loads(path.read_text())
    models = []
    for entry in data.get("models", []):
        if not isinstance(entry, dict):
            raise ModelError("模型登记表的每条必须是表")
        model_id = entry.get("id")
        if not isinstance(model_id, str) or not model_id.strip():
            raise ModelError("模型缺少 id")
        url = str(entry.get("url") or "")
        if not url.startswith(("ws://", "http://")):
            raise ModelError(f"{model_id} 的 url 必须是 ws:// 或 http://")
        models.append({
            "id": model_id,
            "name": str(entry.get("name") or model_id),
            "prompt": str(entry.get("prompt") or ""),
            "url": url,
            "vram": str(entry.get("vram") or ""),
            "note": str(entry.get("note") or ""),
            # Local models record their serving pid here; two of them cannot
            # share one port, so the pid decides who owns it.
            "local_dir": str(entry.get("local_dir") or "") or None,
            "mountable": bool(entry.get("mount_ready", True)),
            "mount": _argv(entry["mount"], "mount") if entry.get("mount") else None,
            "unmount": _argv(entry["unmount"], "unmount") if entry.get("unmount") else None,
        })
    return models


def health_url(url: str) -> str:
    """ws://host:port -> http://host:port/healthz, the server's own probe."""
    parts = urllib.parse.urlsplit(url)
    host, port = parts.hostname or "", parts.port
    if not host or port is None:
        raise ModelError(f"无法从地址解析主机和端口：{url}")
    return f"http://{host}:{port}/healthz"


def probe(url: str, timeout: float = PROBE_TIMEOUT_S) -> bool:
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(health_url(url), timeout=timeout) as reply:
            return 200 <= reply.status < 300
    except (OSError, ValueError):
        return False


# Mount time per model id; only this process records it, so a restart of the
# console forgets a start that is still warming up (the probe then decides).
_mounted: dict[str, float] = {}


def _pid(entry: dict) -> int | None:
    local_dir = entry.get("local_dir")
    if not local_dir:
        return None
    try:
        return int((Path(local_dir) / "serve.pid").read_text().strip())
    except (OSError, ValueError):
        return None


def _alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        os.kill(pid, 0)          # signal 0 only asks whether the process exists
        return True
    except (ProcessLookupError, PermissionError):
        return False


def _serving(entry: dict) -> bool:
    """A local model is serving only when its own pid is alive."""
    return bool(entry.get("local_dir")) and _alive(_pid(entry))


def _owner_of(url: str, entries: list[dict], exclude: str | None = None) -> dict | None:
    for entry in entries:
        if entry["id"] != exclude and entry["url"] == url and _serving(entry):
            return entry
    return None


def _command(argv: list[str], model_id: str) -> list[str]:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log = str(LOG_DIR / f"{model_id}.log")
    return [part.replace("{log}", log).replace("{logdir}", str(LOG_DIR)) for part in argv]


def mount(model_id: str, models: list[dict] | None = None) -> dict:
    entries = load_models() if models is None else models
    entry = _find(model_id, entries)
    if not entry["mount"]:
        raise ModelError(f"{model_id} 没有登记挂载命令")
    if not entry["mountable"]:
        raise ModelError(entry["note"] or f"{model_id} 的挂载命令尚未核对")
    # One port, one local model: another local model already holding it must be
    # unmounted first, whether it is answering yet or still loading.
    owner = _owner_of(entry["url"], entries, exclude=model_id)
    if owner is not None:
        raise ModelError(f"先卸载 {owner['name']}，本地只能同时运行一个模型")
    if probe(entry["url"]):
        if entry.get("local_dir") and not _serving(entry):
            raise ModelError("端口已被其他进程占用，先停止它再挂载")
        _mounted[model_id] = time.monotonic()
        return {"id": model_id, "state": "online"}
    proc = subprocess.Popen(          # the command detaches itself; nothing to wait for
        _command(entry["mount"], model_id), cwd=str(PROJECT_ROOT),
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    _mounted[model_id] = time.monotonic()
    return {"id": model_id, "state": "starting", "pid": proc.pid}


def unmount(model_id: str, models: list[dict] | None = None) -> dict:
    entry = _find(model_id, models)
    if not entry["unmount"]:
        raise ModelError(f"{model_id} 没有登记卸载命令")
    try:
        result = subprocess.run(_command(entry["unmount"], model_id), cwd=str(PROJECT_ROOT),
                                capture_output=True, text=True, timeout=20)
    except subprocess.TimeoutExpired as exc:
        raise ModelError("卸载命令超时") from exc
    _mounted.pop(model_id, None)
    output = (result.stdout + result.stderr).strip()
    return {"id": model_id, "state": "offline", "exit_code": result.returncode,
            "output": output[-400:]}


def _find(model_id: str, models: list[dict] | None = None) -> dict:
    for entry in models if models is not None else load_models():
        if entry["id"] == model_id:
            return entry
    raise ModelError(f"未登记的模型：{model_id}")


def describe(models: list[dict] | None = None) -> list[dict]:
    """Registry plus live state; the probe and the local pid decide online."""
    entries = load_models() if models is None else models
    now = time.monotonic()
    healthy: dict[str, bool] = {}
    result = []
    for entry in entries:
        url = entry["url"]
        if url not in healthy:
            healthy[url] = probe(url)
        serving = _serving(entry)
        # A local model that is not the one serving this port reports offline
        # even though the port answers: the name on the card would be wrong.
        online = healthy[url] and (serving or not entry.get("local_dir"))
        occupied_by = None
        if healthy[url] and entry.get("local_dir") and not serving:
            owner = _owner_of(url, entries, exclude=entry["id"])
            occupied_by = owner["name"] if owner else "另一个进程"
        started = _mounted.get(entry["id"])
        if online:
            state = "online"
        elif started is not None and now - started < STARTING_S and not occupied_by:
            state = "starting"
        else:
            state = "offline"
        result.append({**{key: value for key, value in entry.items()
                          if key not in ("mount", "unmount", "local_dir")},
                       "state": state, "occupied_by": occupied_by})
    return result
