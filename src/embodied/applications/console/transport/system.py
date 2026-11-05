from __future__ import annotations

import asyncio
import hmac
import json
import os
import subprocess
import time
from pathlib import Path

from fastapi import APIRouter, HTTPException, Query

from embodied.applications.console.core.settings import BackendSettings
from embodied.applications.console.protocol import CommandCatalog
from embodied.contracts.runtime.control import ControlClient, live_runs
from embodied.core.project import load_config, port
from embodied.devices.robots.arx5.paths import PROJECT_ROOT

RUNS = PROJECT_ROOT / "outputs/runtime/control"


QUEST_TTL_S = 3.0
QUEST_ACTIONS = {
    # One USB2CAN adapter per arm: bring up can0/can1 in adapter order.
    "can-up": ["can-up", "--all"],
    "quest-prepare": ["quest-transport", "prepare"],
}
_quest_cache: dict[str, object] = {"at": 0.0, "value": None}


def can_up(interface="can0") -> bool:
    try:
        return (Path(f"/sys/class/net/{interface}/operstate").read_text().strip() == "up")
    except OSError:
        return False


def can_state(interfaces=("can0",)):
    """Every CAN interface the kernel uses (two for a dual-arm robot)."""
    names = [name for name in interfaces if name] or ["can0"]
    states = {name: can_up(name) for name in names}
    return {"interface": names[0], "interfaces": names, "states": states, "up": all(states.values())}


def command_env() -> dict[str, str]:
    return {**os.environ, "QUEST_BOOTSTRAP_PYTHON": str(PROJECT_ROOT / ".venv/bin/python")}


def quest_snapshot() -> dict[str, object]:
    """USB / ADB-reverse / client state from the existing read-only probe (cached)."""
    now = time.monotonic()
    cached = _quest_cache.get("value")
    if isinstance(cached, dict) and now - float(_quest_cache["at"]) < QUEST_TTL_S:
        return cached
    state: dict[str, object] = {}
    try:
        result = subprocess.run(
            [str(PROJECT_ROOT / "run.sh"), "quest-transport", "status", "--json"],
            cwd=PROJECT_ROOT, capture_output=True, text=True, timeout=12, env=command_env(),
        )
        text = result.stdout.strip().splitlines()
        state = json.loads(text[-1]) if text else {}
    except (OSError, subprocess.TimeoutExpired, ValueError, IndexError) as exc:
        state = {"error": str(exc)}
    value = {
        "receiver": bool(state.get("receiver_listening")),
        "usb": state.get("usb_status"),
        "serial": state.get("usb_serial"),
        "reverse": bool(state.get("reverse_configured")),
        "client": bool(state.get("client_running")),
        "peers": state.get("peers") or [],
        "connected": bool(state.get("usb_connected")),
        "error": state.get("error"),
    }
    _quest_cache.update(at=now, value=value)
    return value


def run_action(name: str) -> dict[str, object]:
    argv = QUEST_ACTIONS.get(name)
    if argv is None:
        raise KeyError(name)
    try:
        result = subprocess.run(
            [str(PROJECT_ROOT / "run.sh"), *argv], cwd=PROJECT_ROOT,
            capture_output=True, text=True, timeout=25, env=command_env(),
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "output": "命令超时"}
    except OSError as exc:
        return {"ok": False, "output": str(exc)}
    output = (result.stdout + result.stderr).strip()
    _quest_cache["value"] = None
    return {"ok": result.returncode == 0, "output": output[-800:]}


JOINTS_PER_ARM = 6


def arm_views(q, dq, described=None) -> list[dict[str, object]]:
    """Split the kernel joint vector into arms.

    The kernel reports one arm per six joints; a dual-arm backend yields two
    entries without any change here. `described` is the kernel's `robot.arms`
    list, which names each side (left/right) so the page can match an arm to a
    VR controller instead of guessing.
    """
    joints = [float(value) for value in (q or []) if isinstance(value, (int, float))]
    velocities = [float(value) for value in (dq or []) if isinstance(value, (int, float))]
    count = len(joints) // JOINTS_PER_ARM if joints and len(joints) % JOINTS_PER_ARM == 0 else 1
    count = max(count, 1)
    sides = {arm.get("id"): arm.get("side") for arm in described or () if isinstance(arm, dict)}
    arms = []
    for index in range(count):
        start = index * JOINTS_PER_ARM
        identifier = f"arm{index + 1}"
        label = "机械臂" if count == 1 else f"机械臂 {index + 1}"
        if count > 1 and sides.get(identifier) in ("left", "right"):
            label = "左臂" if sides[identifier] == "left" else "右臂"
        arms.append({
            "id": identifier,
            "label": label,
            "side": sides.get(identifier),
            "joints": joints[start:start + JOINTS_PER_ARM],
            "velocities": velocities[start:start + JOINTS_PER_ARM],
        })
    return arms


def camera_inventory() -> list[dict[str, object]]:
    """Configured cameras, regardless of whether the camera service is running."""
    cameras = load_config().get("cameras", {})
    return [
        {
            "id": camera_id,
            "label": settings.get("label") or camera_id,
            "model": settings.get("model"),
            "enabled": settings.get("enabled", True) is not False,
        }
        for camera_id, settings in cameras.items()
        if isinstance(settings, dict)
    ]


def port_listening(number: int) -> bool:
    """Check the listen table instead of connecting, so polling never logs failed handshakes."""
    for table in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(table, encoding="ascii") as handle:
                next(handle, None)
                for line in handle:
                    fields = line.split()
                    if len(fields) > 3 and fields[3] == "0A" and int(fields[1].split(":")[1], 16) == number:
                        return True
        except (OSError, ValueError, IndexError):
            continue
    return False


def camera_service_listening() -> bool:
    return port_listening(port("camera_webrtc"))


def quest_usb_connected() -> bool:
    """Oculus Meta Quest enumerates as USB vendor 2833."""
    for path in Path("/sys/bus/usb/devices").glob("*/idVendor"):
        try:
            if path.read_text().strip().lower() == "2833":
                return True
        except OSError:
            continue
    return False


def teleop_status(directory: Path | None) -> dict[str, object]:
    """Launch identity and VR input availability, read from the run directory."""
    runtime: dict[str, object] = {}
    workbench: dict[str, object] = {}
    if directory is not None:
        for name, target in (("runtime.json", runtime), ("workbench.json", workbench)):
            try:
                loaded = json.loads((directory / name).read_text())
            except (OSError, ValueError):
                continue
            if isinstance(loaded, dict):
                target.update(loaded)
    native = workbench.get("native") if isinstance(workbench.get("native"), dict) else {}
    return {
        "backend": runtime.get("backend"),
        "input": runtime.get("input"),
        "source": workbench.get("source"),
        "native": native.get("device"),
        "quest_usb": quest_usb_connected(),
        "xr_service": port_listening(port("xr_service")),
    }


def shared_camera_listening() -> bool:
    """Shared camera service (Unix socket) presence, read without connecting."""
    from embodied.devices.sensors.client import socket_path

    path = str(socket_path())
    try:
        with open("/proc/net/unix", encoding="ascii") as handle:
            return any(line.rstrip().endswith(path) for line in handle)
    except OSError:
        return False


def read_kernel_status(run: Path | None) -> dict[str, object]:
    """Read-only kernel snapshot; never acquires, heartbeats, or releases a lease."""
    cameras = camera_inventory()
    service = camera_service_listening()
    shared = shared_camera_listening()

    def with_capabilities(payload: dict[str, object], teleop=None, can=None) -> dict[str, object]:
        payload["cameras"] = cameras
        payload["camera_service"] = service
        payload["shared_camera"] = shared
        payload["can"] = can_state() if can is None else can
        payload["teleop"] = teleop_status(None) if teleop is None else teleop
        return payload

    try:
        if run is not None:
            directory = Path(run)
        else:
            runs = live_runs(RUNS)
            if not runs:
                return with_capabilities({"connected": False, "reason": "没有运行中的内核"})
            directory = runs[-1]
        client = ControlClient(directory)
        status = client.status()
    except (OSError, EOFError, ValueError, KeyError, RuntimeError) as exc:
        return with_capabilities({"connected": False, "reason": str(exc)})
    teleop = teleop_status(Path(directory))
    native = (status.get("ingresses") or {}).get("native") or {}
    teleop.update({
        "ready": bool(native.get("ready")),
        "armed": bool(native.get("armed")),
        "permit": bool(native.get("permit")),
        "reason": native.get("reason"),
    })
    try:
        robot = client.describe().get("robot", {})
    except (OSError, EOFError, ValueError, KeyError, RuntimeError):
        robot = {}
    owner = status.get("owner") or None
    interfaces = [arm.get("can_interface") for arm in robot.get("arms") or ()
                  if isinstance(arm, dict) and arm.get("can_interface")]
    return with_capabilities({
        "connected": True,
        "runtime_id": status["runtime_id"],
        "model_id": status["model_id"],
        "phase": status["phase"],
        "stopped": status["stopped"],
        "owner": None if not owner else {"source": owner.get("source"), "purpose": owner.get("purpose")},
        "lease_remaining_s": status["lease_remaining_s"],
        "q": status["q"],
        "dq": status["dq"],
        "arms": arm_views(status["q"], status["dq"], robot.get("arms")),
        "pose": status["pose"],
        "gripper_width": status["gripper_width"],
        "gripper_widths": status.get("gripper_widths"),
        "gripper_effort": status["gripper_effort"],
        "feedback_age_ms": (time.monotonic_ns() - status["feedback_monotonic_ns"]) / 1e6,
        "limits": {
            "joint_min": robot.get("joint_min"),
            "joint_max": robot.get("joint_max"),
            "gripper_max_width": robot.get("gripper_max_width"),
        },
        "operations": [
            {"operation_id": op["operation_id"], "status": op["status"]}
            for op in status["operations"]
        ],
    }, teleop, can_state(interfaces))


def create_system_router(catalog: CommandCatalog, settings: BackendSettings) -> APIRouter:
    router = APIRouter(prefix="/api/v1/system", tags=["system"])

    @router.get("/health")
    async def health() -> dict[str, object]:
        return {"status": "ok", "api_version": 1}

    @router.get("/capabilities")
    async def capabilities() -> dict[str, object]:
        return {"api_version": 1, "features": catalog.describe()}

    @router.get("/quest")
    async def quest() -> dict[str, object]:
        return await asyncio.to_thread(quest_snapshot)

    @router.post("/actions/{name}")
    async def action(name: str, token: str | None = Query(default=None)) -> dict[str, object]:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")
        if name not in QUEST_ACTIONS:
            raise HTTPException(status_code=404, detail="unknown action")
        return await asyncio.to_thread(run_action, name)

    @router.get("/status")
    async def status(token: str | None = Query(default=None)) -> dict[str, object]:
        if settings.token is not None and (
            token is None or not hmac.compare_digest(token, settings.token)
        ):
            raise HTTPException(status_code=401, detail="invalid pairing token")
        return await asyncio.to_thread(read_kernel_status, settings.run)

    return router
