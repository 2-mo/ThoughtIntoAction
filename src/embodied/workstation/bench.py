"""Unified tmux workbench: start a kernel and run the everyday commands by single keys.

    ./run.sh            open it (./run.sh help lists every command)
    ./run.sh bench stop stop the kernel and web console it started, and close it

Windows: 1 内核 (the kernel and its prompts), 2 命令 (a shell; every action is typed here, so it can be
read, edited and rerun), 3 总控 (this dashboard), 4 网页 (the web console server, once opened).
The dashboard only types commands and sends Ctrl+C to the kernel window; it never owns a device.
"""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

from embodied.core.project import PROJECT_ROOT as ROOT

SESSION = os.environ.get('EMBODIED_BENCH_SESSION') or 'embodied-' + hashlib.sha256(str(ROOT).encode()).hexdigest()[:8]
WINDOWS = {'kernel': '内核', 'shell': '命令', 'home': '总控', 'web': '网页'}
SHELLS = {'bash', 'zsh', 'sh', 'fish'}
REAL = 'bash scripts/arm/kernel_real.sh'
KERNELS = [  # key, label, command typed into the kernel window, real arm
    ('1', '真机 · Quest 直控（上游操控）', REAL, True),
    ('2', '真机 · Quest + 网页 / 动作 / 标定 / 策略', REAL + ' --profile real-quest', True),
    ('3', '真机 · 无 Quest（网页 / 动作 / 标定 / 策略）', REAL + ' --profile real-manual', True),
    ('4', '仿真 · Quest', './run.sh quest-transport prepare && ./run.sh kernel profiles start --profile sim-quest', False),
    ('5', '仿真 · 无输入（网页 / 动作 / 策略）', './run.sh kernel profiles start --profile sim-manual', False),
]


# tmux

def tmux(*args, check=True):
    result = subprocess.run(['tmux', *args], text=True, capture_output=True)
    if check and result.returncode:
        raise RuntimeError(result.stderr.strip() or f'tmux {args[0]} failed')
    return result


def exists():
    return tmux('has-session', '-t', SESSION, check=False).returncode == 0


def window(role):
    """(pane id, current command) of a workbench window, or None."""
    rows = tmux('list-panes', '-s', '-t', SESSION, '-F', '#{pane_id}|#{@bench_role}|#{pane_current_command}|#{pane_dead}',
                check=False).stdout.splitlines()
    for pane, name, command, dead in (row.split('|', 3) for row in rows):
        if name == role:
            return pane, ('' if dead == '1' else command)
    return None


def idle(role):
    found = window(role)
    return found is not None and found[1] in SHELLS


def shell(banner):
    return 'bash -c ' + shlex.quote(f'echo {shlex.quote(banner)}; exec bash')


def new_window(role, command, first=False):
    if first:
        pane = tmux('new-session', '-d', '-P', '-F', '#{pane_id}', '-s', SESSION, '-n', WINDOWS[role], '-c', str(ROOT),
                    command).stdout.strip()
        tmux('set-option', '-t', SESSION, 'base-index', '1')
        tmux('set-option', '-t', SESSION, 'renumber-windows', 'on')
        tmux('move-window', '-r', '-t', SESSION, check=False)
    else:
        pane = tmux('new-window', '-d', '-P', '-F', '#{pane_id}', '-t', SESSION, '-n', WINDOWS[role], '-c', str(ROOT),
                    command).stdout.strip()
    tmux('set-option', '-p', '-t', pane, '@bench_role', role)
    tmux('set-option', '-w', '-t', pane, 'automatic-rename', 'off')
    return pane


def dashboard_command():
    python = shlex.quote(sys.executable)
    return (f'env PYTHONPATH={shlex.quote(str(ROOT / "src"))} EMBODIED_BENCH_SESSION={shlex.quote(SESSION)} '
            f'{python} -B -m embodied.workstation.bench _dashboard')


def start(attach=True):
    if not shutil.which('tmux'):
        raise RuntimeError('请先安装 tmux：sudo apt install tmux')
    if not exists():
        new_window('kernel', shell('内核窗口：在总控（Ctrl+B 3）按 1-5 启动内核；内核的提示和日志显示在这里。'), first=True)
        new_window('shell', shell('命令窗口：总控的每个操作都会在这里输入实际命令，可修改后回车；也可以直接输入命令。'))
        new_window('home', dashboard_command())
    elif window('home') is None or not window('home')[1]:
        home = window('home')
        if home is None:
            new_window('home', dashboard_command())
        else:
            tmux('respawn-pane', '-k', '-t', home[0], '-c', str(ROOT), dashboard_command())
    tmux('select-window', '-t', window('home')[0])
    if attach:
        if not sys.stdin.isatty():
            print(f'工作台已在后台运行：tmux attach -t {SESSION}')
            return
        os.execvp('tmux', ['tmux', 'switch-client' if os.environ.get('TMUX') else 'attach-session', '-t', SESSION])


def stop(wait_s=30):
    if not exists():
        print('工作台未运行。')
        return
    run = current_run()
    for role in ('web', 'kernel'):
        found = window(role)
        if found and found[1] and found[1] not in SHELLS:
            tmux('send-keys', '-t', found[0], 'C-c')
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline and not all(idle(role) or window(role) is None for role in ('web', 'kernel')):
        time.sleep(.2)
    while run and current_run() and time.monotonic() < deadline:
        time.sleep(.2)
    if run and current_run():
        raise RuntimeError('内核仍在退出，工作台保留；查看窗口 1。')
    tmux('kill-session', '-t', SESSION)
    print('工作台已关闭' + ('，内核已退出（真机进入 SDK 阻尼）。' if run else '。'))


# State

def current_run(root=None):
    """(directory, identity) of the newest ready kernel run of this boot with a live supervisor, or None."""
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    for path in sorted(Path(root or ROOT / 'control/run').glob('*/runtime.json'), reverse=True):
        try:
            identity = json.loads(path.read_text())
            os.kill(identity['supervisor_pid'], 0)
        except (OSError, ValueError, KeyError):
            continue
        if identity.get('boot_id') == boot and (path.parent / 'runtime.ready').exists():
            return path.parent, identity
    return None


def latest_episode(root=None):
    episodes = sorted(Path(root or ROOT / 'datasets/runtime').glob('*/episode.json'))
    return episodes[-1].parent if episodes else None


def relative(path):
    try:
        return str(Path(path).relative_to(ROOT))
    except ValueError:
        return str(path)


def actions(run, identity, episode=None, now=None):
    """[(key, label, command or None, run immediately, needs the command gateway)] for a running kernel."""
    r = relative(run)
    gateway = (Path(run) / 'commands.sock').exists()
    cameras = '--cameras wrist_rgb,exterior_rgb' if identity.get('backend') == 'real' else '--camera synthetic'
    stamp = (now or datetime.now()).strftime('%Y%m%d-%H%M%S')
    rows = [
        ('v', 'MuJoCo 视图', f'./run.sh kernel viewer --run {r}', True, False),
        ('o', '遥测一帧', f'./run.sh kernel observe --run {r} --count 1', True, False),
        ('c', '采集 20 s（改任务名后回车）', f'./run.sh collect run --run {r} --task 验收 {cameras} --duration 20', False, False),
        ('k', '检查最新 episode', f'./run.sh data runtime validate {relative(episode)}' if episode else None, True, False),
        ('e', '导出 LeRobot（确认后回车）', f'./run.sh data export --source datasets/runtime --output datasets/lerobot/{stamp}', False, False),
        ('w', '网页控制台 http://127.0.0.1:8000', 'web', True, True),
        ('m', '动作：状态（再输入 preview / movej 等）', f'./run.sh motion run --run {r} status', True, True),
        ('h', '手眼标定 · 腕部相机（标定板在桌面）', f'./run.sh calib handeye capture wrist_rgb --auto --run {r}', False, True),
        ('j', '手眼标定 · 固定相机（标定板在夹爪）', f'./run.sh calib handeye capture exterior_rgb --auto --run {r}', False, True),
        ('g', 'π0.5 策略（填写任务后回车）', f'./run.sh policy kernel --run {r} --policy sft --prompt ""', False, True),
    ]
    return [(key, label, command if gateway or not needs else None, enter, needs) for key, label, command, enter, needs in rows]


def kernel_summary(run, identity):
    """Short lines describing the running kernel, read without a lease."""
    mode = '真机' if identity.get('backend') == 'real' else '仿真'
    source = '多来源（命令网关）' if identity.get('managed') else '上游直控（无命令网关）'
    lines = [f'{mode} · {source} · {relative(run)}']
    if (Path(run) / 'commands.sock').exists():
        try:
            from embodied.contracts.runtime.control import ControlClient
            status = ControlClient(run).status()
        except (OSError, ValueError, RuntimeError, EOFError) as exc:
            lines.append(f'状态读取失败：{exc}')
        else:
            import math
            owner = (status['owner'] or {}).get('source', '无')
            q = ' '.join(f'{math.degrees(v):6.1f}' for v in status['q'])
            lines += [f'控制权 {owner} · 阶段 {status["phase"]}' + (' · 已 stop 锁存' if status['stopped'] else ''),
                      f'关节 ° {q} · 夹爪 {1000 * status["gripper_width"]:.1f} mm']
    else:
        lines.append('关节状态见 MuJoCo 视图（V）或遥测（O）')
    return lines


# Actions

def type_command(command, enter=True):
    target = window('shell')
    if target is None:
        raise RuntimeError('命令窗口不存在；关闭后重新打开工作台。')
    if target[1] not in SHELLS:
        raise RuntimeError(f'命令窗口还在运行 {target[1] or "上一条命令"}；Ctrl+B 2 查看，结束后再试。')
    tmux('send-keys', '-t', target[0], 'C-u')
    tmux('send-keys', '-t', target[0], '-l', command)
    if enter:
        tmux('send-keys', '-t', target[0], 'Enter')
    tmux('select-window', '-t', target[0])
    return '已在命令窗口执行。' if enter else '命令已输入命令窗口，检查或修改后按回车。'


def start_kernel(command):
    if current_run():
        raise RuntimeError('已有内核在运行；先按 S 停止（或在其原终端退出）。')
    target = window('kernel')
    if target is None or target[1] not in SHELLS:
        raise RuntimeError('内核窗口不空闲；Ctrl+B 1 查看。')
    tmux('send-keys', '-t', target[0], 'C-u')
    tmux('send-keys', '-t', target[0], '-l', command)
    tmux('send-keys', '-t', target[0], 'Enter')
    tmux('select-window', '-t', target[0])
    return '内核窗口正在启动；按提示确认，就绪后 Ctrl+B 3 回总控。'


def open_web(run):
    found = window('web')
    command = f'./run.sh web console --run {relative(run)}'
    if found and found[1] and found[1] not in SHELLS:
        tmux('select-window', '-t', found[0])
        return '网页控制台已在运行：http://127.0.0.1:8000'
    if found:
        tmux('send-keys', '-t', found[0], '-l', command)
        tmux('send-keys', '-t', found[0], 'Enter')
    else:
        new_window('web', shell('网页控制台窗口：Ctrl+C 停止。'))
        tmux('send-keys', '-t', window('web')[0], '-l', command)
        tmux('send-keys', '-t', window('web')[0], 'Enter')
    return '网页控制台启动中：http://127.0.0.1:8000，页面上点“连接”。'


def stop_kernel():
    target = window('kernel')
    if target is None or target[1] in SHELLS or not target[1]:
        raise RuntimeError('内核不是从本工作台启动的；请在它的原终端按 Ctrl+C。')
    tmux('send-keys', '-t', target[0], 'C-c')
    return '已向内核发送 Ctrl+C；真机进入 SDK 阻尼，请扶住机械臂。'


# Dashboard

def dashboard():
    import select
    import termios
    import tty
    from rich.console import Console, Group
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text

    message, failed, pending_stop = '直接按键，无需回车。', False, 0.

    def view():
        found = current_run()
        status = Table.grid(padding=(0, 1))
        menu = Table.grid(padding=(0, 2))
        if found is None:
            status.add_row(Text('没有运行中的内核', style='yellow'))
            for key, label, _, real in KERNELS:
                menu.add_row(Text(key, style='bold cyan'), Text(label + ('（先扶好机械臂）' if real else '')))
        else:
            run, identity = found
            for line in kernel_summary(run, identity):
                status.add_row(Text(line, style='green'))
            for key, label, command, _, needs in actions(run, identity, latest_episode()):
                usable = command is not None
                note = '' if usable else ('（需要多来源模式 2 / 3 / 5）' if needs else '（还没有 episode）')
                menu.add_row(Text(key.upper(), style='bold cyan' if usable else 'dim'), Text(label + note, style=None if usable else 'dim'))
            menu.add_row(Text('S', style='bold red'), Text('停止内核（按两次；真机进入阻尼，先扶住）'))
        try:
            from embodied.core.project import load_config
            from embodied.workstation.ports import summary
            text, problem = summary(load_config())
            status.add_row(Text('端口：' + text, style='red' if problem else 'dim'))
        except (OSError, ValueError, KeyError):
            pass
        menu.add_row(Text('P', style='bold cyan'), Text('端口占用'))
        menu.add_row(Text('Q', style='bold cyan'), Text('离开（全部继续运行）  Z 关闭工作台'))
        return Group(Panel(status, title='具身平台 · 工作台', border_style='cyan'),
                     Panel(menu, title='按键', border_style='dim'),
                     Text(message, style='red' if failed else 'yellow'),
                     Text('Ctrl+B 1 内核 · 2 命令 · 3 总控 · 4 网页；Ctrl+B D 离开。', style='dim'))

    def operate(key):
        nonlocal pending_stop
        found = current_run()
        if key == 'q':
            tmux('detach-client', '-s', SESSION)
            return '已离开，全部继续运行；./run.sh 回到工作台。'
        if key == 'z':
            if found:
                raise RuntimeError('先按 S 停止内核，再按 Z 关闭工作台。')
            tmux('kill-session', '-t', SESSION)
            return ''
        if key == 'p':
            return type_command('./run.sh check ports')
        if found is None:
            for item_key, _, command, _ in KERNELS:
                if key == item_key:
                    return start_kernel(command)
            return '先按 1-5 启动内核。'
        run, identity = found
        if key == 's':
            if time.monotonic() - pending_stop < 4:
                pending_stop = 0.
                return stop_kernel()
            pending_stop = time.monotonic()
            return '再按一次 S 停止内核' + ('；真机将进入 SDK 阻尼，先扶住机械臂。' if identity.get('backend') == 'real' else '。')
        for item_key, label, command, enter, needs in actions(run, identity, latest_episode()):
            if key == item_key:
                if command is None:
                    raise RuntimeError(label + '：' + ('需要多来源模式（2 / 3 / 5）的内核。' if needs else '还没有采集的 episode。'))
                return open_web(run) if command == 'web' else type_command(command, enter)
        return '这个键没有对应操作。'

    console = Console()
    previous = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        with Live(console=console, screen=True, auto_refresh=False) as live:
            shown = 0.
            while True:
                if time.monotonic() - shown >= 1:
                    live.update(view(), refresh=True)
                    shown = time.monotonic()
                if not select.select([sys.stdin], [], [], .2)[0]:
                    continue
                key = os.read(sys.stdin.fileno(), 1).decode(errors='ignore').lower()
                if not key:
                    return
                if key == '\x1b':  # arrow / function keys are not actions
                    while select.select([sys.stdin], [], [], .05)[0]:
                        os.read(sys.stdin.fileno(), 1024)
                    continue
                if key in '\r\n ':
                    continue
                try:
                    message, failed = operate(key), False
                except (OSError, ValueError, RuntimeError) as exc:
                    message, failed = str(exc), True
                shown = 0.
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, previous)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', nargs='?', default='start', choices=('start', 'stop', 'status', '_dashboard'))
    parser.add_argument('--detach', action='store_true', help='start: create without attaching')
    args = parser.parse_args(argv)
    if args.action == '_dashboard':
        dashboard()
    elif args.action == 'stop':
        stop()
    elif args.action == 'status':
        found = current_run()
        print(f'工作台：{"运行中" if exists() else "未运行"}（tmux 会话 {SESSION}）')
        print('\n'.join(kernel_summary(*found)) if found else '没有运行中的内核。')
    else:
        start(attach=not args.detach)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        raise SystemExit(f'工作台：{exc}')
