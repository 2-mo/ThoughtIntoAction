#!/usr/bin/env python3
"""Unified terminal controls; device ownership remains in existing services."""
from contextlib import closing
import os
import select
import sys
import termios
import time
import tty

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

import tmux_workspace as workspace
from robot_simulation.control import command as viewer_command, status as viewer_status


def overview():
    windows = workspace.panes()
    mode = workspace.selected_mode()
    rows = []
    for role in ("arm", "teleop", "web"):
        pane = windows.get(role)
        state = "窗口已退出" if not pane or pane[1] else {
            "waiting": "等待设备", "starting": "启动中", "console": "控制台运行中",
            "arm-console": "控制台运行中", "details": "待机 · 等待启动"}.get(pane[2], "启动中")
        hint = {"arm": "A 启动 / 重连", "teleop": "T 启动 / 恢复", "web": "W 开网页"}[role]
        color = "yellow"
        try:
            if mode == 'simulation' and role in ('arm', 'teleop'):
                rows.append((workspace.ROLES[role], Text('仿真模式 · 真机不跟随', style='dim'),
                             Text('T 遥操待机 / S 暂停保持' if role == 'teleop' else 'F 切回真机')))
                continue
            if role == "arm":
                info = workspace.arm_info()
                if info is not None:
                    stamp = (info.get("state") or {}).get("host_monotonic_ns", 0)
                    fresh = 0 <= (time.monotonic_ns() - stamp) / 1e9 < .5
                    mode = {"damping": "阻尼待机", "hold": "位置保持", "control": "控制中", "stopping": "停车中"}.get(info.get("mode"), "已连接")
                    state, color = (mode, "green") if fresh else ("反馈已过期", "yellow")
                elif pane and not pane[1]:
                    service = (workspace.tmux("show-options", "-pqv", "-t", pane[0], "@questarm_service_state").stdout.strip()
                               if pane[2] == "arm-console" else "")
                    state = {"waiting": "等待通电 / 连接", "exited": "服务已退出 · 可重连",
                             "starting": "连接中"}.get(service, state)
                    if pane[2] == "console":
                        state = "服务已退出 · 可重连" if workspace.old_arm_console_exited(pane[0]) else "启动中 / 可能等待 sudo"
            elif role == "teleop":
                if workspace.teleop_running():
                    state, color = "会话已连接", "green"
                    hint = "S 停止；追踪状态见网页"
            else:
                info = viewer_status()
                if info:
                    state, color, hint = workspace.MODES[info.get('mode', 'real')], "green", info["url"]
                    if info.get('simulation'):
                        hint += ' · ' + info['simulation']['message']
            if pane and pane[1] and color == "green":
                hint += " · 外部服务"
        except (OSError, ValueError, RuntimeError) as exc:
            state, color, hint = "状态读取异常", "red", str(exc)
        rows.append((workspace.ROLES[role], Text(state, style=color), Text(hint)))
    return rows


def arm_key(key):
    if workspace.selected_mode() == 'simulation':
        action = {'h':'sim-home', 'p':'sim-pregrasp', 's':'sim-pause'}.get(key)
        if action is None:
            raise RuntimeError('仿真模式不操作真机阻尼；S 暂停保持。')
        return viewer_command(action)
    pane = workspace.panes().get("arm")
    if not pane or pane[1] or pane[2] != "arm-console":
        raise RuntimeError("机械臂控制台未就绪，请先按 A 连接；按 1 可查看详情。")
    info = workspace.arm_info()
    if info is None:
        raise RuntimeError("机械臂尚未连接，请先按 A。")
    workspace.tmux("send-keys", "-t", pane[0], key)
    return "已发送到机械臂控制台；执行结果见窗口 1。"


def operate(key):
    if key in "123":
        role = {"1": "arm", "2": "teleop", "3": "web"}[key]
        pane = workspace.panes().get(role)
        if pane:
            workspace.tmux("select-window", "-t", pane[0])
        return "Ctrl+B 再按 4 返回总控。"
    if key == "q":
        workspace.tmux("detach-client", "-s", workspace.SESSION)
        return "已离开工作台，服务继续运行。"
    if key == "i":
        return "状态已刷新。"
    with closing(workspace.DeviceLease("tmux-workspace:" + str(workspace.ROOT))):
        if key in ('m', 'f'):
            return workspace.switch_mode('simulation' if key == 'm' else 'real')
        if key in ('u', 'o'):
            return viewer_command('sim-resume' if key == 'u' else 'sim-pause')
        if key == "z":
            workspace.shutdown_services()
            workspace.tmux("kill-session", "-t", workspace.SESSION)
            return "全部服务已关闭。"
        if key == "r":
            workspace.restart_all()
            return "旧服务已退出，新服务正在启动；设备未就绪时保持等待。"
        if key in ("a", "t"):
            return workspace.start_role("arm" if key == "a" else "teleop")
        if workspace.selected_mode() == 'simulation' and key in ('s', ' ', 'x'):
            return viewer_command('sim-pause')
        if key in ("s", "x"):
            workspace.stop_pane("teleop")
            if key == "x":
                workspace.stop_pane("arm")
            return ("已退出工作台管理的控制台；外部服务、网页及相机保留。" if key == "x"
                    else "工作台遥操控制台已退出；按 T 可重新启动，外部会话保持原状。")
        if key in ("w", "e"):
            return viewer_command("start" if key == "w" else "stop")
        if key in ("h", "p", "d", " "):
            return arm_key("s" if key == " " else key)
    return "请使用下方快捷键，无需回车。"


def render(rows, message, failed=False):
    table = Table(expand=True, box=None, padding=(0, 1))
    table.add_column("服务", no_wrap=True, style="bold")
    table.add_column("状态", no_wrap=True)
    table.add_column("操作 / 地址", overflow="fold")
    for row in rows:
        table.add_row(*row)
    menu = Table.grid(padding=(0, 2), expand=True)
    for row in [
        ("M 切到仿真", "F 切到真机（待机）", "同一个网页地址"),
        ("U 仿真遥操待机", "O 暂停仿真", "右摇杆下拨 1 秒启用"),
        ("A 机械臂启动 / 重连", "T 遥操启动 / 恢复", "S 停止遥操"),
        ("H 回零", "P 预抓取", "空格 停止并保持"),
        ("D 阻尼（先支撑机械臂）", "W 开网页", "E 关网页"),
        ("1 机械臂详情", "2 遥操详情", "3 网页 / 相机控制"),
        ("I 刷新状态", "X 停止控制服务", "Q 离开，保持运行"),
        ("R 完整重启服务", "Z 全部关闭并退出", "含遥操、机械臂、网页、相机"),
    ]:
        menu.add_row(*(Text(item, style="cyan") for item in row))
    return Group(
        Panel(table, title="QuestArm · 统一工作台 · " + workspace.MODES[workspace.selected_mode()], border_style="cyan"),
        Panel(menu, title="直接按键 · 无需回车", border_style="dim"),
        Text(message, style="red" if failed else "yellow", overflow="fold"),
        Text("Ctrl+B 再按 4 返回总控。切仿真时退出真机遥操并保持位置；切回后需重新启用。", style="dim"),
    )


def main():
    pane = os.environ["TMUX_PANE"]
    workspace.tmux("set-option", "-w", "-t", pane, "remain-on-exit", "on")
    workspace.tmux("set-option", "-w", "-t", pane, "automatic-rename", "off")
    workspace.tmux("set-option", "-p", "-t", pane, "@questarm_role", "home")
    workspace.tmux("set-option", "-p", "-t", pane, "@questarm_phase", "console")
    console = Console()
    previous = termios.tcgetattr(sys.stdin.fileno())
    message, failed = "等待设备也可留在这里；需要输入 sudo 密码时按 1。", False
    try:
        tty.setcbreak(sys.stdin.fileno())
        with Live(console=console, screen=True, auto_refresh=False) as live:
            rows, updated = [], 0
            while True:
                if time.monotonic() - updated >= 2:
                    rows, updated = overview(), time.monotonic()
                live.update(render(rows, message, failed), refresh=True)
                if not select.select([sys.stdin], [], [], .2)[0]:
                    continue
                key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()
                if not key:
                    return
                if key in ("\r", "\n"):
                    continue
                # ANSI function/arrow keys must never be interpreted as A/D actions.
                if key == "\x1b":
                    while select.select([sys.stdin], [], [], .05)[0]:
                        os.read(sys.stdin.fileno(), 1024)
                    continue
                live.update(render(rows, "正在处理，请等待服务完成操作…"), refresh=True)
                try:
                    message, failed = operate(key), False
                except (OSError, ValueError, RuntimeError) as exc:
                    message, failed = str(exc), True
                updated = 0
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, previous)


if __name__ == "__main__":
    main()
