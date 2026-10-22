"""Registered ports and sockets: who listens, what is free, what conflicts. Read-only.

Sources: [ports], [viewer_service].port and [telemetry].endpoint in configs/project.toml
(overridable in configs/local.toml), the platform's Unix sockets, and live kernel runs
under outputs/runtime/control/. Nothing is stopped; the table names the command that owns each port.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
from urllib.parse import urlsplit

from embodied.core.project import PROJECT_ROOT, load_config, resolve_path

# name: (protocol, purpose, owning command)
DESCRIPTIONS = {
    "monitor": ("tcp", "18081 监看页与 MuJoCo 工作台", "./run.sh web control start"),
    "telemetry": ("tcp", "旧机械臂服务状态发布（ZMQ）", "./run.sh arm service"),
    "console": ("tcp", "apps/console 网页与后端", "./run.sh web console"),
    "console_backend": ("tcp", "控制台后端单独运行", "./run.sh web backend"),
    "vision": ("tcp", "手势识别 WebSocket", "./run.sh web vision"),
    "camera_webrtc": ("tcp", "相机 WebRTC 信令", "./run.sh camera rgbd"),
    "collection_camera": ("tcp", "采集相机信令", "./run.sh collect groot"),
    "frontend_dev": ("tcp", "Vite 开发服务器", "./run.sh web preview"),
    "phone_udp": ("udp", "iPhone 位姿", "./run.sh phone run --run <RUN>"),
    "xr_service": ("tcp", "XRoboToolkit PC Service（Quest 经 adb reverse）", "./run.sh quest service"),
    "site": ("tcp", "宣传网站本地预览", "./run.sh web site"),
}
SOCKETS = {  # config section: (purpose, owning command)
    "camera_service": ("共享相机服务", "./run.sh camera service"),
    "arm_service": ("旧机械臂服务", "./run.sh arm service"),
    "runtime": ("旧遥操会话", "./run.sh quest teleop"),
    "viewer_service": ("监看页控制", "./run.sh web control start"),
}
PROJECT_MARKERS = (str(PROJECT_ROOT), "embodied.", "legacy.", "embodied.control", "rollio-", "RoboticsService", "XRoboToolkit")


def registry(config):
    entries = {name: int(value) for name, value in config.get("ports", {}).items()}
    entries["monitor"] = int(config["viewer_service"]["port"])
    entries["telemetry"] = urlsplit(config["telemetry"]["endpoint"]).port
    return entries


def listeners():
    """{(protocol, port): [pid, ...]} and {socket path: [pid, ...]} from ss."""
    def parse(args):
        result = subprocess.run(["ss", "-H", *args], capture_output=True, text=True)
        return result.stdout.splitlines() if result.returncode == 0 else []
    network, unix = {}, {}
    for line in parse(["-ltnup"]):
        fields = line.split()
        if len(fields) >= 5:
            local = fields[4]
            protocol = fields[0]
            port = local.rsplit(":", 1)[-1]
            if port.isdigit():
                network.setdefault((protocol, int(port)), []).extend(map(int, re.findall(r"pid=(\d+)", line)))
    for line in parse(["-lxp"]):
        fields = line.split()
        if len(fields) >= 5 and fields[4].startswith("/"):
            unix.setdefault(fields[4], []).extend(map(int, re.findall(r"pid=(\d+)", line)))
    return network, unix


def command_line(pid):
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def owner(pids):
    if not pids:
        return "空闲", ""
    commands = [command_line(pid) for pid in sorted(set(pids))]
    text = "; ".join(f"{pid} {cmd[:70]}" for pid, cmd in zip(sorted(set(pids)), commands))
    ours = any(any(marker in cmd for marker in PROJECT_MARKERS) for cmd in commands)
    return ("本项目" if ours else "其他程序"), text


def live_runs():
    runs = []
    for path in sorted((PROJECT_ROOT / "outputs/runtime/control").glob("*/runtime.json")):
        try:
            identity = json.loads(path.read_text())
            os.kill(identity["supervisor_pid"], 0)
        except (OSError, ValueError, KeyError):
            continue
        runs.append((path.parent, identity))
    return runs


def rows(config):
    network, unix = listeners()
    entries = registry(config)
    result = []
    by_port = {}
    for name, port in entries.items():
        by_port.setdefault(port, []).append(name)
    for name, port in sorted(entries.items(), key=lambda kv: kv[1]):
        protocol, purpose, command = DESCRIPTIONS.get(name, ("tcp", "（未说明）", ""))
        pids = network.get((protocol, port), [])
        state, process = owner(pids)
        if len(by_port[port]) > 1:
            state = "登记冲突：" + "、".join(by_port[port])
        result.append({"name": name, "address": f"{protocol} {port}", "state": state, "process": process,
                       "purpose": purpose, "command": command})
    for section, (purpose, command) in SOCKETS.items():
        value = config.get(section, {}).get("socket") or ("outputs/runtime/viewer.sock" if section == "viewer_service" else None)
        if value:
            path = str(resolve_path(value))
            state, process = owner(unix.get(path, []))
            result.append({"name": section, "address": "unix " + os.path.relpath(path, PROJECT_ROOT), "state": state,
                           "process": process, "purpose": purpose, "command": command})
    for directory, identity in live_runs():
        sockets = [s for s in ("commands.sock", "observer.sock") if str(directory / s) in unix]
        result.append({"name": "kernel", "address": "unix " + os.path.relpath(directory, PROJECT_ROOT),
                       "state": "本项目", "process": f"{identity['supervisor_pid']} {identity.get('backend', '?')} / "
                       f"{identity.get('input', '?')} · {', '.join(sockets) or '无 socket'}",
                       "purpose": "运行中的 arx 内核", "command": "./run.sh kernel start"})
    return result


def conflicts(config):
    """Short descriptions of registered ports held by other programs or registered twice."""
    return [f"{row['name']} {row['address']} {row['state']}" for row in rows(config)
            if row["state"].startswith(("其他程序", "登记冲突"))]


def summary(config):
    table = rows(config)
    used = [row for row in table if row["state"] == "本项目"]
    problems = [row for row in table if row["state"].startswith(("其他程序", "登记冲突"))]
    text = f"{len(used)} 个在用 · {len(table)} 个登记"
    return (text + " · 冲突：" + "、".join(f"{r['name']} {r['address']}" for r in problems)) if problems else text, bool(problems)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(argv)
    table = rows(load_config())
    if args.json:
        print(json.dumps(table, ensure_ascii=False, indent=2))
        return 0
    conflicts = [row for row in table if row["state"].startswith(("其他程序", "登记冲突"))]
    try:
        from rich.console import Console
        from rich.table import Table
    except ImportError:
        for row in table:
            print(f"{row['name']:18} {row['address']:34} {row['state']:8} {row['purpose']}  {row['process']}")
    else:
        view = Table(title="端口与 socket（configs/project.toml \\[ports]）", expand=True)
        for column in ("名称", "地址", "状态", "进程", "用途", "启动命令"):
            view.add_column(column, overflow="fold")
        styles = {"空闲": "dim", "本项目": "green", "其他程序": "bold red"}
        for row in table:
            style = next((value for key, value in styles.items() if row["state"].startswith(key)), "bold red")
            view.add_row(row["name"], row["address"], f"[{style}]{row['state']}[/]", row["process"], row["purpose"], row["command"])
        Console().print(view)
    if conflicts:
        print("冲突：" + "；".join(f"{row['name']} ({row['address']}) {row['state']}" for row in conflicts)
              + "。修改 configs/local.toml 的 [ports] 或先停止占用的程序。")
    return 1 if conflicts else 0


if __name__ == "__main__":
    raise SystemExit(main())
