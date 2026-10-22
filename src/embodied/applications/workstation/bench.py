"""Unified tmux workbench: start a kernel and run the everyday commands by single keys.

    ./run.sh            open it (./run.sh help shows task navigation)
    ./run.sh help --all list every command
    ./run.sh bench stop stop the kernel and web console it started, and close it

Windows: 1 总控, 2 命令, 3 内核, 4 网页, 5 VR, 6 相机, 7 宣传.
Web services start with the workbench; device windows stay visible while waiting for setup.
The dashboard only types commands and sends Ctrl+C to the kernel window; it never owns a device.
"""
import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

from embodied.core.project import PROJECT_ROOT as ROOT
from embodied.applications.workstation.status import Status, status_text

DEFAULT_SESSION = 'emb'
LEGACY_SESSION = 'embodied-' + hashlib.sha256(str(ROOT).encode()).hexdigest()[:8]
SESSION = os.environ.get('EMBODIED_BENCH_SESSION') or DEFAULT_SESSION
WINDOWS = {'home': '总控', 'shell': '命令', 'kernel': '内核', 'web': '网页',
           'vr': 'VR', 'camera': '相机', 'site': '宣传'}
WINDOW_HINT = 'Ctrl+B → 1 总控 · 2 命令 · 3 内核 · 4 网页 · 5 VR · 6 相机 · 7 宣传'
UI_REVISION = '5'
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
    return bool(shutil.which('tmux')) and tmux('has-session', '-t', '=' + SESSION, check=False).returncode == 0


def check_session_owner():
    owner = tmux('show-options', '-qv', '-t', f'={SESSION}:', '@bench_root').stdout.strip()
    if owner != str(ROOT):
        raise RuntimeError(f'tmux 会话 {SESSION} 已被其他工作区使用；请用 '
                           'EMBODIED_BENCH_SESSION=其他名称 ./run.sh 启动。')


def prepare_session():
    """Adopt this checkout's old session without replacing its windows or processes."""
    if exists():
        check_session_owner()
    elif SESSION == DEFAULT_SESSION and tmux('has-session', '-t', '=' + LEGACY_SESSION, check=False).returncode == 0:
        tmux('rename-session', '-t', '=' + LEGACY_SESSION, SESSION)
        tmux('set-option', '-t', SESSION, '@bench_root', str(ROOT))
        home = window('home')
        if home:
            # Its environment still names the old session; refresh the dashboard.
            tmux('set-option', '-p', '-t', home[0], '@bench_ui_revision', 'renamed')


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
    index = list(WINDOWS).index(role) + 1
    if first:
        pane = tmux('new-session', '-d', '-P', '-F', '#{pane_id}', '-s', SESSION, '-n', WINDOWS[role], '-c', str(ROOT),
                    command).stdout.strip()
        tmux('set-option', '-t', SESSION, 'base-index', '1')
        tmux('set-option', '-t', SESSION, 'renumber-windows', 'off')
        tmux('set-option', '-t', SESSION, '@bench_root', str(ROOT))
        current = tmux('display-message', '-p', '-t', pane, '#{window_index}').stdout.strip()
        if current != str(index):
            tmux('move-window', '-s', pane, '-t', f'{SESSION}:{index}')
    else:
        pane = tmux('new-window', '-d', '-P', '-F', '#{pane_id}', '-t', f'{SESSION}:{index}', '-n', WINDOWS[role], '-c', str(ROOT),
                    command).stdout.strip()
    tmux('set-option', '-p', '-t', pane, '@bench_role', role)
    tmux('set-option', '-w', '-t', pane, 'automatic-rename', 'off')
    tmux('set-option', '-w', '-t', pane, 'remain-on-exit', 'on')
    tmux('set-option', '-p', '-t', pane, '@bench_ui_revision', UI_REVISION)
    if role == 'kernel':
        tmux('set-option', '-p', '-t', pane, '@bench_activity', 'waiting')
    return pane


def dashboard_command():
    python = shlex.quote(sys.executable)
    return (f'env PYTHONPATH={shlex.quote(str(ROOT / "src"))} EMBODIED_BENCH_SESSION={shlex.quote(SESSION)} '
            f'{python} -B -m embodied.applications.workstation.bench _dashboard')


def panel_command(role):
    return (f'env PYTHONPATH={shlex.quote(str(ROOT / "src"))} EMBODIED_BENCH_SESSION={shlex.quote(SESSION)} '
            f'{shlex.quote(sys.executable)} -B -m embodied.applications.workstation.bench_panels {role}'
            + (' --serve' if role in ('web', 'site') else ''))


def activity(role):
    found = window(role)
    return (tmux('display-message', '-p', '-t', found[0], '#{@bench_activity}', check=False).stdout.strip()
            if found else '')


def descendant_pid(pid, ancestor):
    """Linux process ancestry, without signalling or attaching to a device."""
    seen = set()
    while pid > 1 and pid not in seen:
        if pid == ancestor:
            return True
        seen.add(pid)
        try:
            stat = Path(f'/proc/{pid}/stat').read_text()
            pid = int(stat.rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            return False
    return False


def owned_run():
    """Only the kernel launched below this session's kernel pane belongs to it."""
    found, pane = current_run(), window('kernel')
    if found and pane:
        value = tmux('display-message', '-p', '-t', pane[0], '#{pane_pid}', check=False).stdout.strip()
        if value.isdigit() and descendant_pid(found[1]['supervisor_pid'], int(value)):
            return found
    return None


def kernel_busy():
    found = window('kernel')
    state = activity('kernel')
    return bool(found and found[1] and (state == 'kernel' or (found[1] not in SHELLS and state != 'waiting')))


def refresh_panel(pane, command):
    """Let a waiting/web panel clean up its own children before loading new code."""
    tmux('send-keys', '-t', pane, 'C-c')
    deadline = time.monotonic() + 8
    while tmux('display-message', '-p', '-t', pane, '#{pane_dead}').stdout.strip() != '1':
        if time.monotonic() >= deadline:
            raise RuntimeError('界面仍在退出，保留原进程；稍后再次执行 ./run.sh。')
        time.sleep(.1)
    tmux('respawn-pane', '-t', pane, '-c', str(ROOT), command)


def start(attach=True):
    if not shutil.which('tmux'):
        print(overview_text('未安装 tmux。安装后可使用多窗口工作台：sudo apt install tmux'))
        return
    if importlib.util.find_spec('rich') is None:
        print(overview_text('缺少终端界面依赖。请先创建项目环境：uv sync --locked'))
        return
    prepare_session()
    running = exists()
    if running:
        tmux('set-option', '-t', SESSION, 'renumber-windows', 'off')
    for index, role in enumerate(WINDOWS, 1):
        command = (dashboard_command() if role == 'home' else
                   shell('命令窗口：总控操作会输入实际命令；可修改后回车，也可直接输入命令。')
                   if role == 'shell' else panel_command(role))
        found = window(role) if running else None
        # Keep fixed shortcuts even when a window was closed in an older session.
        destination = f'{SESSION}:{index}'
        occupant = (tmux('list-windows', '-t', SESSION, '-F', '#{window_index}|#{window_id}', check=False)
                    .stdout.splitlines() if running else [])
        occupied = next((row.split('|')[1] for row in occupant if row.split('|')[0] == str(index)), None)
        source = tmux('display-message', '-p', '-t', found[0], '#{window_id}').stdout.strip() if found else None
        if occupied and occupied != source:
            end = max(int(row.split('|')[0]) for row in occupant) + 1
            tmux('move-window', '-s', occupied, '-t', f'{SESSION}:{end}')
        if found:
            if source != occupied:
                tmux('move-window', '-s', source, '-t', destination)
            revision = tmux('display-message', '-p', '-t', found[0], '#{@bench_ui_revision}').stdout.strip()
            original = tmux('display-message', '-p', '-t', found[0], '#{pane_start_command}').stdout
            tmux('set-option', '-w', '-t', found[0], 'remain-on-exit', 'on')
            upgrade = (revision != UI_REVISION and role == 'home'
                       and 'embodied.applications.workstation.bench _dashboard' in original)
            panel_upgrade = revision != UI_REVISION and (
                f'embodied.applications.workstation.bench_panels {role}' in original
                and role in ('web', 'site', 'vr', 'camera'))
            if not found[1] or upgrade:
                tmux('respawn-pane', '-k', '-t', found[0], '-c', str(ROOT), command)
                if role == 'kernel':
                    tmux('set-option', '-p', '-t', found[0], '@bench_activity', 'waiting')
                tmux('set-option', '-p', '-t', found[0], '@bench_ui_revision', UI_REVISION)
            elif panel_upgrade:
                refresh_panel(found[0], command)
                tmux('set-option', '-p', '-t', found[0], '@bench_ui_revision', UI_REVISION)
        else:
            new_window(role, command, first=not running)
            running = True
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
    check_session_owner()
    run = owned_run()
    visible = current_run()
    if visible and not run:
        panes = tmux('list-panes', '-s', '-t', SESSION, '-F', '#{pane_id}|#{pane_pid}').stdout.splitlines()
        for row in panes:
            pane, pid = row.split('|')
            if pid.isdigit() and descendant_pid(visible[1]['supervisor_pid'], int(pid)):
                raise RuntimeError(f'内核在本工作台的其他窗口（{pane}）运行；请先在原窗口安全退出，工作台保留。')
    command_pane = window('shell')
    if command_pane and command_pane[1] and command_pane[1] not in SHELLS:
        pid = tmux('display-message', '-p', '-t', command_pane[0], '#{pane_pid}').stdout.strip()
        # Running `bench stop` from the command window is itself a foreground
        # Python process. Allow that caller, while preserving unrelated tasks.
        if not pid.isdigit() or not descendant_pid(os.getpid(), int(pid)):
            raise RuntimeError('命令窗口还有任务运行；请先在窗口 2 结束任务，工作台保留。')
    stopping_kernel = bool(run) or kernel_busy()
    if stopping_kernel:
        tmux('send-keys', '-t', window('kernel')[0], 'C-c')
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline and (owned_run() or (stopping_kernel and kernel_busy())):
        time.sleep(.2)
    if owned_run() or (stopping_kernel and kernel_busy()):
        raise RuntimeError('内核仍在退出，工作台保留；查看窗口 3。')
    for role in ('web', 'site', 'vr', 'camera', 'kernel'):
        found = window(role)
        if found and found[1] and found[1] not in SHELLS:
            tmux('send-keys', '-t', found[0], 'C-c')
    deadline = time.monotonic() + min(wait_s, 5)
    while time.monotonic() < deadline and any(
            (found := window(role)) and found[1] and found[1] not in SHELLS for role in ('web', 'site')):
        time.sleep(.2)
    tmux('kill-session', '-t', SESSION)
    print('工作台已关闭' + ('，内核已退出（真机进入 SDK 阻尼）。' if run else '。'))


# State

def current_run(root=None):
    """(directory, identity) of the newest ready kernel run of this boot with a live supervisor, or None."""
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    for path in sorted(Path(root or ROOT / 'outputs/runtime/control').glob('*/runtime.json'), reverse=True):
        try:
            identity = json.loads(path.read_text())
            os.kill(identity['supervisor_pid'], 0)
        except (OSError, ValueError, KeyError):
            continue
        if identity.get('boot_id') == boot and (path.parent / 'runtime.ready').exists():
            return path.parent, identity
    return None


def latest_episode(root=None):
    episodes = sorted(Path(root or ROOT / 'outputs/datasets/runtime').glob('*/episode.json'))
    return episodes[-1].parent if episodes else None


def relative(path):
    try:
        return str(Path(path).relative_to(ROOT))
    except ValueError:
        return str(path)


def overview_text(reason=None):
    """Read-only fallback for pipes and machines without terminal UI dependencies."""
    found = current_run()
    lines = ['知行有灵 · Thought into Action · 工作台概览', '']
    if reason:
        lines += [reason, '']
    if found:
        run, identity = found
        mode = '真机' if identity.get('backend') == 'real' else '仿真'
        lines += [f'内核：{mode} · {relative(run)}', '', '可复制到终端的命令：']
        for _, label, command, _, _ in actions(run, identity, latest_episode()):
            if command is not None:
                if command == 'web':
                    command = f'./run.sh web monitor --kernel --run {shlex.quote(relative(run))} --no-camera --no-connections'
                lines.extend([f'  {label}', f'    {command}'])
    else:
        lines += [
            '内核：未运行。选择运行配置后再开始采集、标定或策略推理。',
            '',
            '下一步',
            '  ./run.sh kernel doctor                              检查环境',
            '  ./run.sh kernel profiles start --profile sim-manual  启动无输入仿真',
            '  ./run.sh kernel real                                启动真机（按提示确认）',
        ]
    lines += ['', '命令目录：./run.sh help    搜索：./run.sh search <关键词>',
              '这里只显示状态与命令；未启动新的服务。']
    return '\n'.join(lines)


def actions(run, identity, episode=None, now=None):
    """[(key, label, command or None, run immediately, needs the command gateway)] for a running kernel."""
    r = shlex.quote(relative(run))
    backend = identity['backend']
    gateway = (Path(run) / 'commands.sock').exists()
    cameras = '--cameras wrist_rgb,exterior_rgb' if identity.get('backend') == 'real' else '--camera synthetic'
    stamp = (now or datetime.now()).strftime('%Y%m%d-%H%M%S')
    rows = [
        ('v', 'MuJoCo 视图', f'./run.sh kernel viewer --run {r}', True, False),
        ('o', '遥测一帧', f'./run.sh kernel observe --run {r} --count 1', True, False),
        ('c', '采集 20 s（改任务名后回车）', f'./run.sh collect run --run {r} --task 验收 {cameras} --duration 20', False, False),
        ('k', '检查最新 episode', f'./run.sh data runtime validate {relative(episode)}' if episode else None, True, False),
        ('e', '导出 LeRobot（确认后回车）', f'./run.sh data export --source outputs/datasets/runtime --output outputs/datasets/lerobot/{stamp}', False, False),
        ('w', '打开监看与采集网页（地址与连接提示）', 'web', True, False),
        ('m', '读取状态与设备能力', f'./run.sh examples read_state --backend {backend} --run {r}', True, True),
        ('b', '测速：关节 1 往返 10°，保存报告（回车开始）',
         f'./run.sh examples benchmark --backend {backend} --run {r} --joint 1 --degrees 10 --repeats 1 --speed 0', False, True),
        ('x', 'MoveL：基座 Z +2 cm（回车开始）',
         f'./run.sh examples movel --backend {backend} --run {r} --offset 0 0 0.02 --speed 0 --tcp-speed 0.1', False, True),
        ('t', '连续路径：Z +2 cm → Y +2 cm（回车开始）',
         f'./run.sh examples move_path --backend {backend} --run {r} --speed 0 --tcp-speed 0.1', False, True),
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
        except (OSError, ValueError, RuntimeError, EOFError, KeyError) as exc:
            lines.append(Status(f'状态读取失败：{exc}', 'error'))
        else:
            import math
            owner = (status['owner'] or {}).get('source', '无')
            q = ' '.join(f'{math.degrees(v):6.1f}' for v in status['q'])
            lines += [Status(f'{mode}反馈已收到 · 控制权 {owner} · 阶段 {status["phase"]}'
                             + (' · 已停止（stop 锁存）' if status['stopped'] else ''),
                             'waiting' if status['stopped'] else 'ready'),
                      f'关节 ° {q} · 夹爪 {1000 * status["gripper_width"]:.1f} mm']
    else:
        lines.append(Status(f'{mode}内核已启动 · 反馈待确认（无命令网关）', 'waiting'))
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
    if target is None or kernel_busy():
        raise RuntimeError('内核窗口不空闲；Ctrl+B 3 查看。')
    # A waiting panel owns no device. Replace only that panel (or an idle shell),
    # then return to it after the explicitly selected kernel has finished exiting.
    script = f'trap ":" INT; {command}; result=$?; echo "内核已退出（状态 $result）。"; exec {panel_command("kernel")}'
    tmux('set-option', '-p', '-t', target[0], '@bench_activity', 'kernel')
    tmux('respawn-pane', '-k', '-t', target[0], '-c', str(ROOT), 'bash -c ' + shlex.quote(script))
    tmux('select-window', '-t', target[0])
    return '内核窗口正在启动；按提示确认，就绪后 Ctrl+B 1 回总控。'


def open_web(run=None):
    from embodied.applications.workstation.bench_panels import open_browser
    found = window('web')
    if found is None:
        new_window('web', panel_command('web'))
    elif not found[1] or found[1] in SHELLS:
        tmux('respawn-pane', '-k', '-t', found[0], '-c', str(ROOT), panel_command('web'))
    tmux('select-window', '-t', window('web')[0])
    return open_browser() + '；网页窗口保留服务状态与日志。'


def stop_kernel():
    target = window('kernel')
    if not owned_run() and not kernel_busy():
        raise RuntimeError('内核不是从本工作台启动的；请在它的原终端按 Ctrl+C。')
    tmux('send-keys', '-t', target[0], 'C-c')
    return '已向内核发送 Ctrl+C；真机进入 SDK 阻尼，请扶住机械臂。'


# Dashboard

def dashboard_view(found, episode, message, failed=False, show_help=False):
    """Build the screen separately from input, using literal text for all state."""
    from rich.console import Group
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text

    status = Table.grid(padding=(0, 1), expand=True)
    menu = Table.grid(padding=(0, 1), expand=True)
    menu.add_column(style='dim', width=6)
    menu.add_column(style='bold color(215)', width=2)
    menu.add_column(ratio=1)
    if found is None:
        status.add_row(status_text(Status('机械臂 · 等待启动 / 尚未收到反馈', 'waiting')))
        status.add_row(Text('真机测试按 3，仿真按 5；启动后 Ctrl+B → 1 返回总控。', style='dim'))
        status.add_row(Text('网页已随工作台启动；内核就绪后自动连接。', style='dim'))
        for real, section in ((False, '仿真'), (True, '真机')):
            first = True
            for key, label, _, is_real in KERNELS:
                if is_real == real:
                    menu.add_row(section if first else '', key, Text(label.split(' · ', 1)[1]))
                    first = False
        menu.add_row('', '', Text('真机启动前扶好机械臂，并按内核窗口提示确认。', style='yellow'))
        menu.add_row('网页', 'W', Text('查看监看与采集网页（无需先启动内核）'))
    else:
        run, identity = found
        for line in kernel_summary(run, identity):
            status.add_row(status_text(line))
        rows = actions(run, identity, episode)
        for section, keys in (('观察', 'wmvo'), ('测试', 'bxt'), ('数据', 'cke'), ('进阶', 'hjg')):
            first = True
            for key, label, command, _, needs in rows:
                if key not in keys:
                    continue
                usable = command is not None
                note = '' if usable else (' · 需命令网关' if needs else ' · 暂无 episode')
                menu.add_row(section if first else '', Text(key.upper(), style='bold color(215)' if usable else 'dim'),
                             Text(label + note, style=None if usable else 'dim'))
                first = False
        menu.add_row('停止', 'S', Text('连按两次停止内核；真机进入阻尼，请先扶住', style='yellow'))
    from embodied.applications.workstation.bench_panels import vr_rows
    vr, _ = vr_rows(found)
    label, value = vr[0]
    status.add_row(Text.assemble(f'{label} · ', status_text(value)))
    try:
        from embodied.core.project import load_config
        from embodied.applications.workstation.ports import summary
        text, problem = summary(load_config())
        status.add_row(Text('端口：' + text, style='red' if problem else 'dim'))
    except (OSError, ValueError, KeyError):
        pass
    if show_help:
        body = Text('\n'.join([
            '直接按键，无需回车；数字键选择配置，字母键执行对应操作。',
            'B 10°往返测速 · X XYZ直线 · T连续路径；运行目录和后端自动填写。',
            '动作测试、采集、导出、标定与策略先填到命令窗口，按回车开始。',
            '测试中在命令窗口 Ctrl+C 取消动作；Ctrl+B → 1 返回总控。',
            '/  搜索命令：输入关键词后回车；L 查看完整命令目录。',
            WINDOW_HINT,
            'VR、相机或机械臂未就绪时，窗口会继续显示状态与下一步提示。',
            'Q 离开但保持运行；S 连按两次停止内核；Z 关闭空闲工作台。',
            '禁用项会说明原因；? 关闭帮助。',
        ]))
    else:
        body = menu
    hints = Text('? 帮助   / 搜索   L 命令目录   P 端口   Q 离开   Z 关闭', style='dim')
    return Group(
        Panel(status, title='知行有灵 · Thought into Action · 工作台', border_style='#b7a1e8'),
        Panel(body, title='操作帮助' if show_help else '选择下一步', border_style='dim'),
        Text(('操作未完成 · ' if failed else '') + message, style='red' if failed else '#63c5f8'),
        hints,
        Text(WINDOW_HINT, style='dim'),
    )


def dashboard():
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print(overview_text())
        return
    import select
    import termios
    import tty
    from rich.console import Console
    from rich.live import Live

    message, failed, pending_stop, show_help = '直接按键开始；? 查看操作帮助。', False, 0., False

    def view():
        return dashboard_view(current_run(), latest_episode(), message, failed, show_help)

    def operate(key):
        nonlocal pending_stop, show_help
        if key == '?':
            show_help = not show_help
            return '再次按 ? 返回操作列表。' if show_help else '直接按键，无需回车。'
        if key == '/':
            return type_command('./run.sh search ', enter=False)
        if key == 'l':
            return type_command('./run.sh help --all')
        found = current_run()
        if key == 'q':
            tmux('detach-client', '-s', SESSION)
            return '已离开，全部继续运行；./run.sh 回到工作台。'
        if key == 'z':
            if owned_run() or kernel_busy():
                raise RuntimeError('先按 S 停止内核，再按 Z 关闭工作台。')
            stop()
            return ''
        if key == 'p':
            return type_command('./run.sh check ports')
        if key == 'w':
            return open_web()
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
        print('\n'.join(map(str, kernel_summary(*found))) if found else '没有运行中的内核。')
    else:
        start(attach=not args.detach)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        raise SystemExit(f'工作台：{exc}')
