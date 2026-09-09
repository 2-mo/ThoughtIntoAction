#!/usr/bin/env python3
"""Configure USB transport and report observed XR sockets using Rich."""
import argparse
import ipaddress
import subprocess
import time

from rich.console import Console
from rich.panel import Panel
from rich.text import Text

console = Console()


def command(*args):
    result = subprocess.run(args, text=True, capture_output=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or " ".join(args) + " failed")
    return result.stdout


def usb_device():
    devices = [line.split()[0] for line in command("adb", "devices", "-l").splitlines()
               if len(line.split()) >= 2 and line.split()[1] == "device" and "usb:" in line]
    if len(devices) != 1:
        raise RuntimeError("需要且只能连接一台已授权的 USB Quest；请检查数据线和头显内的 USB 调试授权。")
    return devices[0]


def peers_from_ss(output):
    peers = []
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 5 and parts[0] == "ESTAB" and parts[3].endswith(":63901"):
            host = parts[4].rsplit(":", 1)[0].strip("[]")
            address = ipaddress.ip_address(host)
            peers.append(address.ipv4_mapped if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped else address)
    return peers


def inspect():
    serial = usb_device()
    reverse = command("adb", "-s", serial, "reverse", "--list")
    configured = any(line.split()[-2:] == ["tcp:63901", "tcp:63901"] for line in reverse.splitlines())
    peers = peers_from_ss(command("ss", "-Htn"))
    local = any(peer.is_loopback for peer in peers)
    remote = any(not peer.is_loopback for peer in peers)
    ready = configured and local and not remote
    state = "USB 数据连接已确认" if ready else "仍有网络连接（非 USB）" if remote else "等待 Quest 通过 USB 连接"
    return ready, state, serial, configured, peers


def show(state):
    ready, label, serial, configured, peers = state
    content = Text()
    content.append(label + "\n", style="bold green" if ready else "bold yellow")
    content.append(f"默认方式：USB    设备：{serial}\n")
    content.append(f"ADB 转发：{'已配置' if configured else '未配置'}    XR 端口：63901\n")
    content.append("当前连接：" + (", ".join(map(str, peers)) or "无") + "\n")
    if not ready:
        content.append("Quest 客户端：服务器地址设为 127.0.0.1，点击 Connect。\n", style="yellow")
        content.append("这是 TCP 连接检查，不代表追踪数据或机械臂已就绪。")
    console.print(Panel(content, title="遥操连接", border_style="green" if ready else "yellow"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", default="prepare", choices=["prepare", "wait", "status"])
    args = parser.parse_args(argv)
    try:
        if args.action == "prepare":
            serial = usb_device()
            # Idempotent; don't remove a working mapping and interrupt its stream.
            command("adb", "-s", serial, "reverse", "tcp:63901", "tcp:63901")
        state = inspect()
        if args.action == "wait" and not state[0]:
            show(state)
            console.print("等待 USB 数据连接，最多 15 秒…")
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and not state[0]:
                time.sleep(.5)
                state = inspect()
        show(state)
        if args.action == "wait" and not state[0]:
            console.print("[yellow]未确认 USB 数据连接，遥操未启动/仍保持暂停；连接后按 w 重试。[/yellow]")
            return 1
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
        console.print(Panel(Text(str(exc)), title="USB 连接不可用", border_style="red"))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
