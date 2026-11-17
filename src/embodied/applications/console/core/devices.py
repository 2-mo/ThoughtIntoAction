"""Console side of the device setup: discovery, kernel profile compilation.

The validated document itself lives in `embodied.core.device_setup` so config
loading stays application-free.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

from embodied.core.device_setup import (  # noqa: F401  (re-exported for the console)
    DEVICES_PATH, GENERATED_DIR, ROBOT_ID, CAMERA_ID, VENDORS, SetupError, camera_entries, load_setup,
    save_setup, validate_setup,
)
from embodied.core.project import PROJECT_ROOT

_DISCOVERY = {"at": 0.0, "value": None}
DISCOVERY_TTL_S = 5.0


# --------------------------------------------------------------------------- compile


def compile_setup(document: dict) -> dict:
    """Write the kernel-facing robot profile for the selected vendor and arms.

    The ARX profile loader is unchanged; the generated JSON simply carries the
    CAN interfaces chosen on the page. Vendors register their own generator in
    `VENDORS[...]` when their control code lands.
    """
    vendor = document["robots"][0]["vendor"]
    spec = VENDORS[vendor]
    count = len(document["robots"])
    profile = spec["robot_profile"].get(count)
    if profile is None:
        raise SetupError(f"{spec['label']} 不支持 {count} 台机械臂", 409)
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    arms = sorted(document["robots"], key=lambda arm: arm["side"] != "left")
    if spec.get("connection", "can") == "ethernet":
        robot = _ethernet_robot(vendor, arms[0], profile)
        robot_name = f"{vendor}-{arms[0]['id']}.json"
    elif count == 1:
        robot = {"profile": profile, "can_interface": arms[0]["can_interface"]}
        robot_name = f"{profile}.json"
    else:
        robot = {"profile": profile, "version": 1,
                 "arms": [{"id": arm["side"], "can_interface": arm["can_interface"]} for arm in arms]}
        robot_name = f"{profile}.json"
    robot_path = GENERATED_DIR / robot_name
    robot_path.write_text(json.dumps(robot, ensure_ascii=False, indent=2) + "\n")
    names = spec["workbench_profile"].get(count)
    if not names:
        raise SetupError(f"{spec['label']} 不支持 {count} 台机械臂", 409)
    workbench = {}
    for mode in ("collect", "policy"):
        base = json.loads((PROJECT_ROOT / "configs/workbench" / f"{names[mode]}.json").read_text())
        base["config"] = _portable(robot_path)
        # The VR profile carries vendor grip/yaw settings; ARX is the default.
        if spec.get("connection", "can") == "ethernet" and base.get("source") == "quest":
            vr = json.loads((PROJECT_ROOT / ETHERNET_VR.get(vendor, "configs/inputs/quest.json")).read_text())
            # No external owner: this kernel publishes and consumes its own XR topics.
            vr.setdefault("controller", "right")
            vr.setdefault("input_prefix", None)
            vr_path = GENERATED_DIR / f"{vendor}-{mode}-vr.json"
            vr_path.write_text(json.dumps(vr, ensure_ascii=False, indent=2) + "\n")
            base["vr_config"] = _portable(vr_path)
            base["native"] = {"device": "quest.owned", "routing": "quest.owned",
                              "ingress": "quest.owned", "channel": "native"}
        path = GENERATED_DIR / f"{names[mode]}.json"
        path.write_text(json.dumps(base, ensure_ascii=False, indent=2) + "\n")
        workbench[mode] = str(path)
    return {"robot_profile": _portable(robot_path), "workbench": workbench,
            "workbench_profile": _portable(Path(workbench["collect"])),
            "workbench_name": workbench["collect"]}


ETHERNET_TEMPLATES = {
    # The controller connection shape is vendor-specific; the host picks the
    # deployment, because the tool frame it expects differs per arm.
    "realman": {
        "192.168.1.18": PROJECT_ROOT / "configs/robots/realman/rm65-18.json",
        "192.168.1.19": PROJECT_ROOT / "configs/robots/realman/rm65-19.json",
    },
}
ETHERNET_DEFAULT_TEMPLATE = {"realman": PROJECT_ROOT / "configs/robots/realman/rm65-19.json"}
ETHERNET_MODELS = {
    "realman": PROJECT_ROOT / "assets/robots/realman/runtime/rm65.xml",
}
ETHERNET_VR = {"realman": "configs/inputs/quest-realman.json"}


def _ethernet_robot(vendor: str, arm: dict, profile: str) -> dict:
    """An address-based arm: write its connection file and the profile that points at it."""
    template = ETHERNET_TEMPLATES.get(vendor, {}).get(arm.get("host"))
    if template is None:
        template = ETHERNET_DEFAULT_TEMPLATE.get(vendor)
    model = ETHERNET_MODELS.get(vendor)
    if template is None or not template.is_file():
        raise SetupError(f"{VENDORS[vendor]['label']} 缺少控制器连接模板", 409)
    if model is None or not model.is_file():
        raise SetupError(f"{VENDORS[vendor]['label']} 缺少机器人模型文件", 409)
    connection = json.loads(template.read_text())
    connection["host"] = arm["host"]
    connection_path = GENERATED_DIR / f"{vendor}-{arm['id']}-connection.json"
    connection_path.write_text(json.dumps(connection, ensure_ascii=False, indent=2) + "\n")
    return {
        "profile": profile,
        "name": f"{VENDORS[vendor]['label']} {VENDORS[vendor]['models'][arm['model']]['label']} {arm['host']}",
        "connection_path": str(connection_path),
        "control_hz": connection["control_hz"],
        "model_path": str(model),
    }


def _portable(path: Path) -> str:
    """Repo-relative when possible, absolute for generated files outside it."""
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def pending_restarts(document: dict, *, kernel_running: bool, camera_running: bool) -> list[dict]:
    """What the operator must restart for the saved setup to take effect."""
    items = []
    if camera_running:
        items.append({"kind": "sensor", "label": "相机服务", "reason": "相机清单或序列号已变化"})
    if kernel_running:
        items.append({"kind": "kernel", "label": "内核", "reason": "机械臂或 CAN 配置已变化"})
    return items


# --------------------------------------------------------------------------- discovery


def _realsense_devices() -> list[dict]:
    try:
        import pyrealsense2 as rs
    except Exception as exc:  # SDK missing on this host
        return [{"error": f"RealSense SDK 不可用：{exc}"}]
    found = []
    try:
        context = rs.context()
        for device in context.query_devices():
            name = device.get_info(rs.camera_info.name)
            found.append({
                "kind": "realsense",
                "name": name,
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": _firmware(device, rs),
                "model": "D405" if "D405" in name else "D435i" if "D435" in name else name,
            })
    except Exception as exc:
        return [{"error": f"枚举 RealSense 失败：{exc}"}]
    return found


def _can_adapters() -> tuple[list[dict], list[dict]]:
    adapters = []
    links = sorted(Path("/dev/serial/by-id").glob("usb-ARX_USB2CAN_*-if00")) if Path("/dev/serial/by-id").is_dir() else []
    for index, link in enumerate(links):
        try:
            device = os.path.realpath(link)
        except OSError:
            continue
        serial = link.name.split("usb-ARX_USB2CAN_")[-1].split("-if00")[0]
        adapters.append({"kind": "usb2can", "serial": serial, "device": device,
                         "interface": f"can{index}", "symlink": str(link)})
    interfaces = []
    for path in sorted(Path("/sys/class/net").glob("can*")):
        try:
            state = (path / "operstate").read_text().strip()
        except OSError:
            continue
        interfaces.append({"name": path.name, "state": state})
    return adapters, interfaces


def _firmware(device, rs):
    try:
        return device.get_info(rs.camera_info.firmware_version)
    except Exception:
        return None


PREVIEW_TTL_S = 3.0
_preview_cache: dict[str, tuple[float, str]] = {}


def previews(serials: list[str], *, refresh: bool = False) -> dict:
    """One JPEG per camera, so the page can bind a role to the picture it sees.

    A running camera service already encodes previews, so its frames are reused.
    Otherwise the device is opened for a single frame — this only works while no
    service or recording owns it, and the error says so.
    """
    images, errors, now = {}, {}, time.monotonic()
    for serial in serials:
        cached = _preview_cache.get(serial)
        if not refresh and cached and now - cached[0] < PREVIEW_TTL_S:
            images[serial] = cached[1]
            continue
        try:
            image = _service_preview(serial)
            if image is None:
                image = _device_preview(serial)
            _preview_cache[serial] = (now, image)
            images[serial] = image
        except Exception as exc:  # a camera that is busy or unplugged is not fatal
            errors[serial] = f"{type(exc).__name__}: {exc}"
    return {"images": images, "errors": errors, "service": _camera_service_running()}


def _camera_service_running() -> bool:
    from embodied.devices.sensors.client import socket_path

    try:
        return socket_path().exists()
    except OSError:
        return False


def _service_preview(serial: str) -> str | None:
    from embodied.core.project import load_config
    from embodied.core.session_protocol import request
    from embodied.devices.sensors.client import socket_path

    if not _camera_service_running():
        return None
    camera_id = next((key for key, options in (load_config().get("cameras") or {}).items()
                      if isinstance(options, dict) and options.get("serial") == serial), None)
    if camera_id is None:
        return None
    state = request(socket_path(), "monitor", timeout=1, camera_id=camera_id)
    return (state.get("preview") or {}).get("jpeg_base64")


def _device_preview(serial: str) -> str:
    """A one-shot frame straight from the device; cheap and short-lived."""
    import base64

    import cv2
    import numpy as np
    import pyrealsense2 as rs

    from embodied.contracts.device_lease import DeviceLease

    # Resolving an unknown serial blocks inside the SDK, so refuse it up front:
    # a stale placeholder in the config must not hang the request.
    connected = {device.get_info(rs.camera_info.serial_number)
                 for device in rs.context().query_devices()}
    if serial not in connected:
        raise RuntimeError("未检测到这台相机（序列号不在当前设备列表）")
    lease = DeviceLease("camera:" + serial)
    try:
        configuration = rs.config()
        configuration.enable_device(serial)
        configuration.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline = rs.pipeline()
        pipeline.start(configuration)
        try:
            color = None
            for _ in range(3):
                color = pipeline.wait_for_frames(2000).get_color_frame()
                if color:
                    break
            if not color:
                raise RuntimeError("没有取到画面")
            image = np.asanyarray(color.get_data())
        finally:
            pipeline.stop()
    finally:
        lease.close()
    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok:
        raise RuntimeError("画面编码失败")
    return base64.b64encode(buffer.tobytes()).decode("ascii")


def discover(force: bool = False) -> dict:
    now = time.monotonic()
    cached = _DISCOVERY["value"]
    if cached is not None and not force and now - _DISCOVERY["at"] < DISCOVERY_TTL_S:
        return cached
    adapters, interfaces = _can_adapters()
    value = {
        "cameras": _realsense_devices(),
        "can_adapters": adapters,
        "can_interfaces": interfaces,
        "vendors": {name: {"label": spec["label"], "implemented": spec["implemented"],
                           "connection": spec.get("connection", "can"),
                           "models": {model: {"label": model_spec["label"], "arm_counts": list(model_spec["arm_counts"])}
                                      for model, model_spec in spec["models"].items()}}
                    for name, spec in VENDORS.items()},
        "generated_at": now,
    }
    _DISCOVERY["value"], _DISCOVERY["at"] = value, now
    return value
