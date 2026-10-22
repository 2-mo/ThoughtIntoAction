#!/usr/bin/env python3
"""Persistent mode-aware status windows; all actions go through the workbench."""
import sys
import time

import tmux_workspace as workspace
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from embodied.core.terminal_keys import terminal_keys, read_key
from embodied.applications.monitor.control import status as viewer_status


def render(role, info, mode, message):
    menu = ("4 返回总控  ·  T 遥操待机  ·  S / 空格 暂停保持  ·  H 回零  ·  P 预抓取\n"
            "M 仿真  ·  F 真机  ·  W 开网页  ·  Q 离开工作台，保持运行")
    table = Table.grid(padding=(0, 3))
    table.add_column(style="dim")
    table.add_column()
    table.add_row("当前模式", workspace.MODES[mode])
    if mode == 'simulation':
        sim = (info or {}).get('simulation') or {}
        table.add_row("MuJoCo", sim.get('message', '等待网页仿真启动；按 W 启动'))
        table.add_row("Quest 跟踪", '有效' if sim.get('tracking') else '等待头显连接 / 有效跟踪')
        table.add_row("遥操控制", '已启用' if sim.get('control_enabled') else '待机 / 暂停')
        if role == 'arm':
            table.add_row("控制对象", 'MuJoCo 虚拟机械臂；真机不跟随')
        else:
            table.add_row("右手（上游）", 'A 长按 150 ms 切换模式；A+B 回 Home；Grip 跟随；Trigger 夹爪')
            table.add_row("左手（采集）", 'X 录制 / 保存；Y 预抓取，长按作废；摇杆管理任务；左 Grip 取消预设')
    else:
        table.add_row("控制台", '已切回真机，等待显式启动')
        table.add_row("下一步", '按 A 连接机械臂，按 T 启动遥操；启动进度见总控')
        menu = "4 返回总控  ·  A 连接机械臂  ·  T 启动遥操  ·  M 切仿真  ·  Q 离开"
    if info:
        table.add_row("网页", info.get('url', '—'))
    return Group(
        Panel(Text(menu, style='cyan'), title='快捷键 · 无需回车', border_style='dim'),
        Panel(table, title=f'{workspace.ROLES[role]}控制台', border_style='cyan'),
        Text(message, style='yellow'),
    )


def forward_key(key):
    if key == '\x03':
        key = 's'
    if key not in ('4', 't', 's', ' ', 'h', 'p', 'm', 'f', 'a', 'w', 'q'):
        return '请使用顶部快捷键。'
    home = workspace.panes().get('home')
    if not home or home[1]:
        workspace.ensure_home()
        return '总控已恢复，请再次按所需快捷键。'
    target = home[0]
    if key == '4':
        workspace.tmux('select-window', '-t', target)
        return ''
    # Dispatch to the lifecycle owner, so it can stop this window without a
    # self-wait or competing workspace lease when returning to physical control.
    workspace.tmux('send-keys', '-t', target, '-l', key)
    if key in ('m', 'f', 'a') or workspace.selected_mode() == 'real':
        workspace.tmux('select-window', '-t', target)
    return '已发送至总控；按 4 查看执行结果。'


def main(role):
    workspace.phase('details')
    message, info, updated = '', None, 0
    with terminal_keys(), Live(console=Console(), screen=True, auto_refresh=False) as live:
        while True:
            workspace.check_cancelled()
            if time.monotonic() - updated >= 1:
                try:
                    info = viewer_status()
                except (OSError, ValueError, RuntimeError) as exc:
                    info, message = None, str(exc)
                updated = time.monotonic()
                live.update(render(role, info, workspace.selected_mode(), message), refresh=True)
            key = read_key(.2)
            if key == 'eof':
                return
            if key is not None:
                try:
                    message = forward_key(key)
                except (OSError, ValueError, RuntimeError) as exc:
                    message = str(exc)
                updated = 0


if __name__ == '__main__':
    main(sys.argv[1])
