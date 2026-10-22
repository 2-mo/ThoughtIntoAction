"""Read-only command navigation, with optional Rich rendering for terminals."""

from __future__ import annotations

import os
import sys

from .catalog import GROUPS


QUICK_START = (
    ("./run.sh", "打开统一工作台；无 tmux 时显示常用入口"),
    ("./run.sh examples", "学习 Robot API · movej / movel / 采集示例"),
    ("./run.sh web site", "本地预览宣传网站"),
    ("./run.sh sim start", "Quest 遥操 · MuJoCo 仿真"),
    ("./run.sh real start", "真机日常工作台 · 当前旧链路"),
    ("./run.sh kernel doctor", "检查内核环境，不连接设备"),
)
TASK_GROUPS = (
    ("bench / sim / real", "工作台与运行模式"),
    ("kernel / quest / phone / motion", "控制内核、遥操与动作"),
    ("arm / camera / calib", "设备与标定"),
    ("collect / data / policy", "采集、数据与策略"),
    ("web / task", "网页与任务程序"),
    ("setup / check", "安装与诊断"),
)
HELP_ROUTES = (
    ("./run.sh help data", "查看一组（也可用 ./run.sh data）"),
    ("./run.sh data export --help", "查看具体参数"),
    ("./run.sh search 相机", "按关键词找命令"),
    ("./run.sh help --all", "展开完整目录"),
)


def _plain_rows(rows) -> list[str]:
    # Command names are mostly ASCII; keep Chinese search examples aligned too.
    from unicodedata import east_asian_width

    def width(value):
        return sum(2 if east_asian_width(char) in "WF" else 1 for char in value)

    padding = max((width(command) for command, _ in rows), default=0) + 2
    return [f"  {command}{' ' * (padding - width(command))}{description}" for command, description in rows]


def overview_text() -> str:
    lines = ["知行有灵 · Thought into Action · 命令导航", "用法：./run.sh <功能> <命令> [参数]"]
    for title, rows in (("常用起点", QUICK_START), ("按任务找功能组", TASK_GROUPS), ("逐层查看", HELP_ROUTES)):
        lines.extend(["", title, *_plain_rows(rows)])
    return "\n".join(lines)


def _command_rows(group: str):
    return [(f"./run.sh {group} {command}", description)
            for command, (description, _) in GROUPS[group][1].items()]


def catalog_text(group: str | None = None) -> str:
    lines = []
    for name in ([group] if group else GROUPS):
        lines.extend([f"{name} · {GROUPS[name][0]}", *_plain_rows(_command_rows(name)), ""])
    lines.append(f"查看参数：./run.sh {group or '<功能>'} <命令> --help")
    if not group:
        lines.append("更多组件：./run.sh components list；原有命令名仍可直接使用。")
    return "\n".join(lines)


def search_matches(query: str):
    terms = query.casefold().split()
    return [(f"./run.sh {group} {command}", description)
            for group, (title, commands) in GROUPS.items()
            for command, (description, _) in commands.items()
            if all(term in f"{group} {title} {command} {description}".casefold() for term in terms)]


def _console():
    # Bootstrap help must remain usable before the project environment exists.
    if not sys.stdout.isatty() or os.environ.get("TERM") == "dumb":
        return None
    try:
        from rich.console import Console
    except ImportError:
        return None
    return Console(highlight=False, markup=False)


def _rich_rows(console, rows):
    from rich.console import Group
    from rich.padding import Padding
    from rich.table import Table
    from rich.text import Text

    if console.width < 70:
        # Keep commands copyable instead of squeezing both columns on narrow panes.
        return Group(*(Group(Text(command, style="cyan"), Padding(Text(description), (0, 0, 0, 2)))
                       for command, description in rows))
    table = Table.grid(padding=(0, 2), expand=True)
    table.add_column(style="cyan", no_wrap=True)
    table.add_column(ratio=1)
    for command, description in rows:
        table.add_row(Text(command), Text(description))
    return table


def print_help(group: str | None = None, *, full: bool = False) -> None:
    console = _console()
    if console is None:
        print(catalog_text(group) if group or full else overview_text())
        return

    from rich.console import Group
    from rich.panel import Panel
    from rich.text import Text

    if not group and not full:
        contents = [Text("用法：./run.sh <功能> <命令> [参数]")]
        for title, rows in (("常用起点", QUICK_START), ("按任务找功能组", TASK_GROUPS), ("逐层查看", HELP_ROUTES)):
            contents.extend([Text(""), Text(title, style="bold"), _rich_rows(console, rows)])
        console.print(Panel(Group(*contents), title=Text("知行有灵 · Thought into Action · 命令导航", style="bold cyan"),
                            title_align="left", border_style="bright_black", expand=False))
        return

    for name in ([group] if group else GROUPS):
        console.print(Panel(Group(Text(GROUPS[name][0]), Text(""), _rich_rows(console, _command_rows(name))),
                            title=Text(name, style="bold cyan"), title_align="left", border_style="bright_black"))
    console.print(Text(f"查看参数：./run.sh {group or '<功能>'} <命令> --help", style="dim"))
    if not group:
        console.print(Text("更多组件：./run.sh components list；原有命令名仍可直接使用。", style="dim"))


def print_search(query: str) -> None:
    matches = search_matches(query)
    if not matches:
        print(f"没有匹配“{query}”的命令。试试：./run.sh search 相机，或 ./run.sh help。")
        return
    heading = f"匹配“{query}”的命令（{len(matches)}）"
    console = _console()
    if console is None:
        print("\n".join([heading, "", *_plain_rows(matches)]))
        return
    from rich.text import Text

    console.print(Text(heading, style="bold"))
    console.print(_rich_rows(console, matches))
