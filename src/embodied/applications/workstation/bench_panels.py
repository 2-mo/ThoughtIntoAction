"""Persistent workbench windows; device panels only observe existing services.

    python -m embodied.applications.workstation.bench_panels kernel
    python -m embodied.applications.workstation.bench_panels web --serve

Only web/site with --serve may start a child, and neither service connects the arm
automatically. A noninteractive invocation always prints one read-only snapshot.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import ipaddress
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import time

from embodied.core.project import PROJECT_ROOT as ROOT, load_config, port, resolve_path
from embodied.contracts.robot import load_robot_profile
from embodied.applications.workstation.status import Status, status_text

TITLES = {'kernel': '机械臂 · 内核', 'vr': 'Quest · VR', 'camera': '共享相机',
          'web': '监看与采集', 'site': '项目宣传网站'}
SERVICE_COMMANDS = {
    'web': ['./run.sh', 'web', 'monitor', '--kernel', '--mode', 'real', '--no-camera', '--no-connections'],
    'site': ['./run.sh', 'web', 'site'],
}


def listening(number, host='127.0.0.1'):
    """Probe a local listener without starting or stopping its owner."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.settimeout(.2)
        return client.connect_ex((host, number)) == 0


def service_host(role):
    if role != 'web':
        return '127.0.0.1'
    configured = load_config().get('viewer_service', {}).get('host')
    connection = os.environ.get('SSH_CONNECTION', '').split()
    value = configured or (connection[2] if len(connection) == 4 else '127.0.0.1')
    address = ipaddress.ip_address(value)
    if address.version != 4 or address.is_unspecified:
        raise ValueError('viewer_service.host 必须是本机明确的 IPv4 地址')
    return str(address)


def service_command(role):
    command = list(SERVICE_COMMANDS[role])
    return command + ['--host', service_host(role)] if role == 'web' else command


def service_address(role):
    number = int(load_config()['viewer_service']['port']) if role == 'web' else port('site')
    return number, f'http://{service_host(role)}:{number}' + ('/apps/site/' if role == 'site' else '')


def open_browser():
    """Open the monitor on a local desktop; SSH clients use the displayed URL."""
    number, url = service_address('web')
    if not listening(number, service_host('web')):
        return f'网页尚未就绪，查看日志或按 R 重试；地址 {url}'
    if os.environ.get('SSH_CONNECTION') or not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        return f'请在浏览器打开 {url}'
    opener = shutil.which('xdg-open')
    if opener is None:
        return f'请在浏览器打开 {url}'
    try:
        subprocess.Popen([opener, url], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
    except OSError as exc:
        return f'浏览器未打开：{exc}；地址 {url}'
    return f'已请求浏览器打开 {url}'


def kernel_rows():
    from embodied.applications.workstation import bench

    found = bench.current_run()
    if found:
        run, identity = found
        mode = '真机' if identity.get('backend') == 'real' else '仿真'
        rows = [('内核', mode + '内核运行中'), ('运行目录', bench.relative(run))]
        rows.extend(('反馈' if i == 0 else '', line)
                    for i, line in enumerate(bench.kernel_summary(run, identity)[1:]))
        hints = ['当前内核保留运行；控制操作在总控窗口选择。']
        if mode == '仿真':
            hints.append('当前反馈来自仿真，不能据此判断真机是否通电。')
        return rows, hints
    config = load_config().get('robot', {})
    can_port = load_robot_profile()['robot'].get('can_interface')
    if not isinstance(can_port, str) or not can_port.strip():
        raise ValueError('configs/robots/arx5/robot.json 未配置 can_interface')
    device = resolve_path(config.get('can_device', '/dev/arx_r5'))
    visible = (Path('/sys/class/net') / can_port).exists() or device.is_char_device()
    return [('内核', Status('等待启动 / 真机尚未提供反馈', 'waiting')),
            ('USB-CAN', Status('适配器可见；等待机械臂反馈' if visible else '未发现配置中的 CAN 接口或适配器', 'waiting')),
            ('配置', f'{can_port} · {device}')], [
                '真机未通电或尚未连接时，本窗口保持等待。',
                '通电、连接后，在总控按 1 / 2 / 3 选择真机配置；按内核提示确认。',
                '无硬件也可在总控按 5 启动无输入仿真。',
            ]


def vr_rows(found):
    from embodied.applications.workstation import bench

    hints = ['启动头显并连接 USB，在头显内允许 USB 调试。',
             '检查连接：./run.sh quest usb status；准备连接：./run.sh quest usb prepare',
             '在总控选择带 Quest 的配置后，跟踪状态会自动更新。']
    if not found:
        return [('VR', Status('等待头显连接及 Quest 内核配置', 'waiting')), ('内核', '尚未运行')], hints
    run, identity = found
    native = identity.get('native') or {}
    quest = identity.get('input') in ('vr', 'quest') or 'quest' in str(native.get('device', '')).lower()
    rows = []
    if not quest:
        rows += [('VR', Status('当前内核配置未启用 Quest', 'disabled')),
                 ('运行目录', bench.relative(run)), ('VR 窗口', '保持待机；不会切换或重启现有内核')]
        return rows, ['需要遥操时，在总控选择带 Quest 的配置。',
                      '先按当前内核的退出提示处理设备，再切换配置。']
    if not (Path(run) / 'commands.sock').exists():
        rows += [('VR', Status('已配置 · 跟踪待确认（无命令网关）', 'waiting')),
                 ('跟踪状态', '请查看内核日志或遥测'), ('运行目录', bench.relative(run))]
        return rows, hints
    from embodied.contracts.runtime.control import ControlClient

    try:
        status = ControlClient(run).status()
    except (OSError, ValueError, RuntimeError, EOFError, KeyError) as exc:
        return [('VR', Status(f'状态读取失败：{exc}', 'error'))], hints
    ingress = status.get('ingresses', {}).get('native')
    if ingress is None:
        rows.append(('VR', Status('等待原生 Quest 通道状态', 'waiting')))
    else:
        rows += [('VR', Status('跟踪 / 通道就绪' if ingress.get('ready') else '等待有效跟踪',
                              'ready' if ingress.get('ready') else 'waiting')),
                 ('操作激活', Status('已激活' if ingress.get('armed') else '待机',
                                  'ready' if ingress.get('armed') else 'waiting')),
                 ('滤波许可', Status('有效' if ingress.get('permit') else '等待许可',
                                  'ready' if ingress.get('permit') else 'waiting'))]
        if ingress.get('reason'):
            rows.append(('等待原因', Status(str(ingress['reason']), 'waiting')))
    rows.append(('运行目录', bench.relative(run)))
    return rows, hints


def camera_rows():
    config = load_config()
    camera_socket = resolve_path(config.get('camera_service', {}).get('socket', 'outputs/runtime/camera.sock'))
    rows = []
    feeds = {}
    if camera_socket.exists():
        from embodied.core.session_protocol import request
        try:
            state = request(camera_socket, 'monitor', timeout=.3)
            feeds = state.get('cameras', {})
            rows.append(('共享服务', '在线（只读查询）'))
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            rows.append(('共享服务', f'尚未就绪：{exc}'))
    else:
        rows.append(('共享服务', '等待启动'))
    serials = set()
    for path in Path('/sys/bus/usb/devices').glob('*/serial'):
        try:
            serials.add(path.read_text().strip())
        except OSError:
            pass
    for name, options in config.get('cameras', {}).items():
        serial = str(options.get('serial', ''))
        feed = feeds.get(name, {}).get('camera', {})
        if not options.get('enabled', False):
            detail = '配置中未启用'
        elif feed.get('error'):
            detail = str(feed['error'])
        elif feed.get('metadata'):
            detail = '共享服务已报告相机信息'
        elif serial and serial in serials:
            detail = 'USB 设备可见；等待相机服务提供帧'
        elif serial:
            detail = '等待 USB 相机连接'
        else:
            detail = '未指定序列号；检查相机配置'
        rows.append((options.get('label', name), f'{detail} · {name}'))
    return rows, ['本窗口保留等待，不会打开相机设备。',
                  '启动共享服务：./run.sh camera service',
                  '采集或标定前，确认所需相机已经连接并配置。']


def snapshot(role):
    """Return (title, labeled rows, hints); never start a process or a device."""
    try:
        if role == 'kernel':
            rows, hints = kernel_rows()
        elif role == 'vr':
            from embodied.applications.workstation import bench
            rows, hints = vr_rows(bench.current_run())
        elif role == 'camera':
            rows, hints = camera_rows()
        else:
            number, url = service_address(role)
            rows = [('地址', url), ('端口', '已有监听；未确认服务身份' if listening(number, service_host(role)) else '尚未监听')]
            hints = ['启动命令：' + ' '.join(service_command(role))]
        return TITLES[role], rows, hints
    except (OSError, ValueError, RuntimeError, KeyError, EOFError) as exc:
        return TITLES[role], [('状态', Status(f'状态读取失败：{exc}', 'error'))], ['窗口保持打开；修复连接或配置后按 R 刷新。']


class Service:
    """Own only the web process group created by this panel, never an existing listener."""

    def __init__(self, role):
        if role not in SERVICE_COMMANDS:
            raise ValueError('Only web/site panels may start a service')
        self.role = role
        self.process = None
        self.log_path = None
        self.message = '尚未启动服务'
        self.external = False
        self.attempted = False
        self._child_reserved = False
        self.browser_message = None

    def open_when_ready(self):
        """One browser request after the monitor listens, avoiding a refused page."""
        if self.role == 'web' and self.browser_message is None:
            number, _ = service_address('web')
            if listening(number, service_host('web')):
                self.browser_message = open_browser()

    def exit_code(self):
        """Inspect a child without reaping it, reserving its PID until group cleanup.

        An exited child may remain visible in a waiting panel for hours. Keeping
        it waitable prevents that PID from being reused by an unrelated process
        group before this panel eventually retries or closes.
        """
        if self.process is None:
            return None
        try:
            result = os.waitid(os.P_PID, self.process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        except ChildProcessError:
            # If another owner unexpectedly reaped it, its old PID is no longer
            # safe to signal. Preserve the failure display without signalling it.
            self._child_reserved = False
            return self.process.returncode if self.process.returncode is not None else -1
        if result is None:
            return None
        return result.si_status if result.si_code == os.CLD_EXITED else -result.si_status

    def start(self):
        if self.process is not None and self.exit_code() is None:
            return
        if self.process is not None:
            self.stop()
        self.attempted = True
        self.external = False
        try:
            number, _ = service_address(self.role)
            if listening(number, service_host(self.role)):
                self.external = True
                self.message = '已有端口监听；保留原进程，不重复启动'
                return
            logs = ROOT / 'outputs/logs/bench'
            logs.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
            self.log_path = logs / f'{self.role}-{stamp}-{os.getpid()}.log'
            with self.log_path.open('ab', buffering=0) as output:
                self.process = subprocess.Popen(service_command(self.role), cwd=ROOT,
                                                stdin=subprocess.DEVNULL, stdout=output, stderr=output,
                                                start_new_session=True)
                self._child_reserved = True
            self.message = '正在启动；日志显示在下方'
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            self.message = f'服务未启动：{exc}；修复后按 R 重试'

    def snapshot(self):
        title, rows, hints = snapshot(self.role)
        state = self.message
        if self.process is not None:
            code = self.exit_code()
            state = '本工作台服务运行中' if code is None else f'服务已退出（{code}）；修复后按 R 重试'
        rows.append(('服务', state))
        if self.browser_message:
            rows.append(('浏览器', self.browser_message))
        if self.log_path:
            rows.append(('日志', str(self.log_path)))
        if self.external:
            hints.append('可以打开上述地址检查；退出本窗口不会停止已有服务。')
        elif self.role == 'web':
            hints.append('自动只读连接运行中的统一内核；无内核时保留页面等待。')
            hints.append('此页显示机械臂、相机与采集状态；不会使能机械臂或接管控制权。')
        return title, rows, hints

    def logs(self, limit=8):
        if not self.log_path:
            return []
        try:
            with self.log_path.open('rb') as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - 8192))
                return stream.read().decode(errors='replace').splitlines()[-limit:]
        except OSError as exc:
            return [f'日志暂不可读：{exc}']

    def stop(self):
        child = self.process
        if child is None:
            return
        self.exit_code()
        if not self._child_reserved:
            self.process = None
            self.message = '服务进程已被回收；未向旧 PID 发送信号'
            return
        # Scripts may spawn descendants (frontend setup, uvicorn); terminate the
        # dedicated group even when its original shell has already exited.
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 3
        while self.exit_code() is None and time.monotonic() < deadline:
            time.sleep(.05)
        if not self._child_reserved:
            self.process = None
            self.message = '服务进程已被回收；未向旧 PID 发送信号'
            return
        # The leader is still waitable here, so its PID/PGID cannot have been
        # reused even when it exited before its descendants. Reap only last.
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=3)
        self.process = None
        self._child_reserved = False
        self.message = '本窗口启动的服务已结束'


def render(state, service=None):
    from rich.console import Group
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text

    title, rows, hints = state
    table = Table.grid(padding=(0, 2), expand=True)
    table.add_column(style='dim', no_wrap=True)
    table.add_column(ratio=1)
    for label, value in rows:
        table.add_row(Text(label), status_text(value))
    states = [value for _, value in rows if isinstance(value, Status)]
    primary = next((value for value in states if value.level == 'error'), states[0] if states else None)
    sections = [Panel(table, title=title, border_style=primary.style if primary else '#b7a1e8'),
                Panel(Group(*(Text(line) for line in hints)), title='下一步', border_style='dim')]
    if service is not None:
        logs = service.logs()
        if logs:
            sections.append(Panel(Group(*(Text(line, overflow='fold') for line in logs)),
                                  title='最近日志', border_style='dim'))
    sections.append(Text('Ctrl+B → 1 返回总控 · Ctrl+B → D 离开工作台 · R ' +
                         ('重试 / 刷新' if service is not None else '刷新') +
                         (' · W 打开浏览器' if service is not None and service.role == 'web' else ''), style='dim'))
    return Group(*sections)


def mark_waiting(role):
    if role != 'kernel' or not os.environ.get('TMUX_PANE'):
        return
    try:
        from embodied.applications.workstation import bench
        bench.tmux('set-option', '-p', '-t', os.environ['TMUX_PANE'], '@bench_activity', 'waiting', check=False)
    except (OSError, RuntimeError):
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=TITLES)
    parser.add_argument('--serve', action='store_true', help='web/site: start once; press R to retry after a failure')
    args = parser.parse_args(argv)
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        title, rows, hints = snapshot(args.role)
        print('\n'.join([title, *(f'{label}：{value}' for label, value in rows), '', *hints]))
        return 0
    from rich.console import Console
    from rich.live import Live
    from embodied.core.terminal_keys import read_key, terminal_keys

    mark_waiting(args.role)
    service = Service(args.role) if args.serve and args.role in SERVICE_COMMANDS else None
    stopping = False

    def stop_requested(*_):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, stop_requested) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        if service is not None:
            service.start()
        with terminal_keys(), Live(console=Console(), screen=True, auto_refresh=False) as live:
            refreshed = 0.
            while not stopping:
                if time.monotonic() - refreshed >= 1:
                    if service is not None:
                        service.open_when_ready()
                    live.update(render(service.snapshot() if service else snapshot(args.role), service), refresh=True)
                    refreshed = time.monotonic()
                key = read_key(.2)
                if key in ('eof', '\x03', '\x04'):
                    break
                if key == 'r':
                    if service is not None:
                        service.start()
                    refreshed = 0.
                if key == 'w' and args.role == 'web' and service is not None:
                    service.browser_message = open_browser()
                    refreshed = 0.
    finally:
        if service is not None:
            service.stop()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
