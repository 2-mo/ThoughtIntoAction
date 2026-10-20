#!/usr/bin/env python3
"""Interactive lifecycle owner for a teleop child; no device SDK imports."""
from pathlib import Path
from collections import deque
import signal
import subprocess
import sys
import time
import threading

from rich.console import Console as RichConsole, Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.layout import Layout
from rich.live import Live
from embodied.core.terminal_keys import terminal_keys, read_key

rich_console = RichConsole()
print = rich_console.print

ROOT = Path(__file__).resolve().parents[2]


class TerminalView:
    """Pinned Rich shortcuts, with existing command/Enter semantics below the logs."""
    def __init__(self, owner, terminal):
        self.owner, self.terminal = owner, terminal
        self.lines = deque(maxlen=1000)
        self.partial = ''
        self.command = ''
        self.lock = threading.RLock()
        self.live = Live(console=terminal, screen=True, refresh_per_second=5,
                         get_renderable=self.render, redirect_stdout=False, redirect_stderr=False)

    def write(self, text):
        with self.lock:
            rows = (self.partial + text).split('\n')
            self.lines.extend(rows[:-1])
            self.partial = rows[-1]
        return len(text)

    def flush(self):
        pass

    def render(self):
        layout = Layout()
        layout.split_column(Layout(name='keys', size=5), Layout(name='state', size=3),
                            Layout(name='logs'), Layout(name='input', size=3))
        layout['keys'].update(Panel(Group(
            Text('W/1 启动/恢复  U/3 暂停  S/2 停止并休眠', style='cyan'),
            Text('C 方向对齐  CR 复用方向  P 状态  ? 帮助  Q 退出', style='cyan'),
            Text('输入命令后回车；Ctrl+U 清空输入', style='dim')),
            title='快捷键 · Quest 遥操', border_style='cyan'))
        child = self.owner.child
        code = None if child is None else child.poll()
        state = '尚未启动' if child is None else '进程运行中' if code is None else f'已退出 · 退出码 {code}'
        layout['state'].update(Panel(Text(state), title='遥操状态', border_style='dim'))
        with self.lock:
            lines = [*self.lines, *([self.partial] if self.partial else [])]
        count = max(1, self.terminal.size.height - 13)
        layout['logs'].update(Panel(Group(*(Text(line, no_wrap=True, overflow='ellipsis') for line in lines[-count:])),
                                    title='运行日志', border_style='dim'))
        layout['input'].update(Panel(Text('遥操> ' + self.command + '▏'), border_style='cyan'))
        return layout

    def read_command(self):
        self.command = ''
        with terminal_keys():
            while True:
                key = read_key()
                if key in ('\r', '\n'):
                    command = self.command
                    self.write('遥操> ' + command + '\n')
                    self.command = ''
                    return command
                if key in ('eof', '\x04'):
                    raise EOFError
                if key == '\x03':
                    raise KeyboardInterrupt
                if key == '\x15':
                    self.command = ''
                elif key in ('\x7f', '\b'):
                    self.command = self.command[:-1]
                elif key and len(key) == 1 and key.isprintable():
                    self.command = (self.command + key)[:80]


def notice(message, level="info"):
    label, color = {"info": ("提示", "cyan"), "success": ("完成", "green"),
                    "waiting": ("等待", "yellow"), "error": ("异常", "red")}[level]
    line = Text(f"{label}  ", style="bold " + color)
    line.append(str(message), style="default")
    rich_console.print(line)


class Console:
    def __init__(self, runner, arguments, log_dir):
        self.runner, self.arguments = str(runner), arguments
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"teleop-console-{time.time_ns()}.log"
        self.log = self.log_path.open("ab", buffering=0)
        self.child = None
        self.ui = None

    def show_help(self):
        if self.ui is not None:
            notice('快捷键固定在顶部；W 恢复，U 暂停保留进程，S 停止并休眠，Q 退出且保持头显电源状态。')
            return
        commands = Table.grid(padding=(0, 2))
        commands.add_column(style="bold cyan", no_wrap=True)
        commands.add_column()
        for key, description in [
            ("w / 1", "模拟佩戴并启动 / 恢复遥操"),
            ("u / 3", "取消模拟佩戴，暂停控制并保留进程"),
            ("s / 2", "停止遥操并请求头显休眠"),
            ("c / cr", "方向对齐 / 复用上次方向（Quest 未重定位）"),
            ("p", "查看 USB 连接与进程状态"),
            ("?", "显示操作说明"),
            ("q", "退出遥操控制台，头显电源状态保持"),
        ]:
            commands.add_row(key, description)
        rich_console.print(Panel(commands, title="Quest · 遥操控制台", subtitle="输入命令后回车",
                                 border_style="cyan", expand=False))

    def show_process(self):
        code = None if self.child is None else self.child.poll()
        label, color = (("尚未启动", "yellow") if self.child is None else
                        ("进程运行中", "green") if code is None else
                        (f"已退出 · 退出码 {code}", "red" if code else "yellow"))
        rows = Table.grid(padding=(0, 2))
        rows.add_column(style="dim", no_wrap=True)
        rows.add_column(overflow="fold")
        rows.add_row("遥操进程", Text(label, style="bold " + color))
        rows.add_row("追踪 / 控制", "请在网页确认手柄数据及控制模式；进程存活不代表追踪正常")
        rows.add_row("完整日志", Text(str(self.log_path)))
        rich_console.print(Panel(rows, title="运行状态", border_style=color, expand=False))

    def stop(self):
        if self.child is None or self.child.poll() is not None:
            return True
        self.child.send_signal(signal.SIGINT)
        try:
            self.child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            notice("遥操尚未退出，未切换头显电源。请查看日志并处理当前进程。", "waiting")
            return False
        return True

    def power(self, command):
        try:
            result = subprocess.run([self.runner, "quest-wake", command],
                                    stdout=self.log, stderr=self.log, timeout=20)
            if result.returncode == 0:
                return True
        except (OSError, subprocess.TimeoutExpired) as exc:
            notice(f"头显操作失败：{exc}", "error")
        notice(f"头显操作未成功，请查看 {self.log_path}", "error")
        return False

    def transport(self, action):
        try:
            options = {'stdout': subprocess.PIPE, 'stderr': subprocess.STDOUT, 'text': True} if self.ui else {}
            result = subprocess.run([self.runner, "quest-transport", action], timeout=35, **options)
            if self.ui and result.stdout:
                rich_console.print(Text(result.stdout.rstrip()))
            return result.returncode == 0
        except (OSError, subprocess.TimeoutExpired) as exc:
            notice(f"USB 检查失败：{exc}", "error")
            return False

    def wake(self):
        if self.child is not None and self.child.poll() is None:
            if self.session("pause") and self.transport("prepare") and self.power("start") and self.transport("wait") and self.session("resume"):
                notice("已模拟佩戴，等待新帧后自动恢复；松开 Grip/A/B/右摇杆按键后按正常方式遥操。", "success")
            return
        if self.transport("prepare") and self.power("start") and self.transport("wait"):
            self.child = subprocess.Popen([self.runner, *self.arguments], stdin=subprocess.DEVNULL,
                                          stdout=self.log, stderr=self.log, start_new_session=True)
            notice("已请求模拟佩戴，遥操正在启动；输入 p 查看状态。", "waiting")

    def session(self, command):
        try:
            result = subprocess.run([self.runner, "session", command],
                                    stdout=self.log, stderr=self.log, timeout=20)
            if result.returncode == 0:
                return True
        except (OSError, subprocess.TimeoutExpired) as exc:
            notice(f"遥操会话操作失败：{exc}", "error")
        notice(f"未完成 {command}；请检查采集会话和日志：{self.log_path}", "error")
        return False

    def unwear(self):
        if self.child is not None and self.child.poll() is None:
            if not self.session("pause"):
                return
        if self.power("unwear"):
            notice("已取消模拟佩戴，由 Quest 自行息屏；程序保留，机械臂控制已暂停。", "success")

    def sleep(self):
        if self.stop() and self.power("stop"):
            notice("遥操已停止，已发送取消模拟佩戴和休眠请求。", "success")

    def status(self):
        self.transport("status")
        self.show_process()

    def watch_child(self, stop):
        reported = None
        while not stop.wait(.2):
            child = self.child
            if child is None or child is reported:
                continue
            code = child.poll()
            if code is None:
                continue
            reported = child
            details = [Text(f"遥操进程已退出（退出码 {code}）")]
            if code:
                try:
                    with self.log_path.open("rb") as log:
                        log.seek(0, 2)
                        log.seek(max(0, log.tell() - 8192))
                        lines = log.read().decode("utf-8", errors="replace").splitlines()
                    errors = [line for line in lines if "Error:" in line or "Exception:" in line]
                    if errors:
                        details.append(Text(errors[-1][:600]))
                except OSError:
                    pass
            details.extend([Text(f"日志：{self.log_path}", style="dim"),
                            Text("处理后输入 w 重新启动，或 q 退出。", style="cyan")])
            rich_console.print(Panel(Group(*details), title="遥操异常" if code else "遥操已停止",
                                     border_style="red" if code else "yellow", expand=False))

    def calibrate(self, reuse=False):
        if self.child is None or self.child.poll() is not None:
            notice("请先输入 w 启动遥操，再在待机状态输入 c 标定。", "waiting")
            return
        # Calibration owns its interactive prompts; restore the dashboard afterwards.
        if self.ui:
            self.ui.live.stop()
        try:
            subprocess.run([self.runner, "quest-calibrate", *(["--reuse"] if reuse else [])], check=False)
        finally:
            if self.ui:
                self.ui.live.start(refresh=True)

    def run(self):
        global rich_console
        if not sys.stdin.isatty() or not rich_console.is_terminal:
            return self.run_commands()
        terminal = rich_console
        self.ui = TerminalView(self, terminal)
        rich_console = RichConsole(file=self.ui, width=max(20, terminal.size.width - 4),
                                   force_terminal=False, highlight=False)
        try:
            with self.ui.live:
                self.run_commands()
        finally:
            rich_console = terminal
            self.ui = None

    def run_commands(self):
        self.show_help()
        watcher_stop = threading.Event()
        watcher = threading.Thread(target=self.watch_child, args=(watcher_stop,), daemon=True)
        watcher.start()
        try:
            self.wake()
            self.show_process()
            while True:
                try:
                    command = (self.ui.read_command() if self.ui else input("遥操> ")).strip().lower()
                except (EOFError, KeyboardInterrupt):
                    command = "q"
                    rich_console.print()
                if command in ("w", "1"):
                    self.wake()
                elif command in ("s", "2"):
                    self.sleep()
                elif command in ("u", "3"):
                    self.unwear()
                elif command == "p":
                    self.status()
                elif command in ("?", "help"):
                    self.show_help()
                elif command in ("c", "cr"):
                    self.calibrate(reuse=command == "cr")
                elif command == "q":
                    if self.stop():
                        break
                else:
                    notice("输入 w / u / s / c / cr / p / q 后回车；? 查看操作说明。")
        finally:
            watcher_stop.set()
            watcher.join(timeout=1)
            self.stop()
            self.log.close()


if __name__ == "__main__":
    if any(arg in ("-h", "--help") for arg in sys.argv[1:]):
        print("Usage: quest-console TELEOP_ARGUMENTS; daily entry: ./启动遥操.sh")
        raise SystemExit(0)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    Console(ROOT / "run.sh", sys.argv[1:], ROOT / "outputs/logs").run()
