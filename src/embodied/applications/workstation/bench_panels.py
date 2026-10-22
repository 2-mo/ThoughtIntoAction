"""Persistent workbench windows with shared camera and web service startup.

    python -m embodied.applications.workstation.bench_panels kernel
    python -m embodied.applications.workstation.bench_panels web --serve

Interactive --serve starts web/site children, ensures the independent camera
service, or prepares the Quest USB transport. Robot keys attach to an existing
kernel; no panel opens a hardware driver or enables motors.
Noninteractive invocations stay read-only.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import threading
import time

from embodied.core.project import PROJECT_ROOT as ROOT, load_config, port, resolve_path
from embodied.contracts.robot import load_robot_profile
from embodied.applications.workstation.status import Status, status_text

TITLES = {'kernel': 'robot · 机械臂', 'vr': 'Quest · VR', 'camera': '共享相机',
          'web': '监看与采集', 'site': '项目宣传网站', 'phone': '手机 · RobotTeleop'}
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
        rows = [('robot', mode + '控制运行中'), ('运行目录', bench.relative(run))]
        rows.extend(('反馈' if i == 0 else '', line)
                    for i, line in enumerate(bench.kernel_summary(run, identity)[1:]))
        hints = ['P  预抓取位       H  回零位       空格 / S  停止本面板动作并保持',
                 'D  真机阻尼（扶住机械臂，再按 D 确认；退出内核）',
                 '按键直接执行；运动中不排队。速度使用后端每轴上限，实际峰值由路径及加速度 / jerk 决定。',
                 'Ctrl+B z 展开原始日志；再次按下恢复面板。',
                 '内核日志：' + bench.relative(run / 'backend.log')]
        if mode == '仿真':
            hints.append('当前反馈来自仿真，不能据此判断真机是否通电。')
        return rows, hints
    config = load_config().get('robot', {})
    can_port = load_robot_profile()['robot'].get('can_interface')
    if not isinstance(can_port, str) or not can_port.strip():
        raise ValueError('configs/robots/arx5/robot.json 未配置 can_interface')
    device = resolve_path(config.get('can_device', '/dev/arx_r5'))
    visible = (Path('/sys/class/net') / can_port).exists() or device.is_char_device()
    state = bench.activity('kernel') if os.environ.get('EMBODIED_BENCH_SESSION') else ''
    if state == 'failed':
        status = Status('启动失败；Ctrl+B → 3 查看原始错误', 'error')
        hints = ['启动错误已保留在 robot 窗口；修复后在总控按 3 重新启动。']
    elif state == 'kernel':
        status = Status('等待启动确认 / 正在启动；Ctrl+B → 3 查看', 'waiting')
        hints = ['robot 启动流程已准备；在窗口 3 按唯一提示确认后，才会使能电机。']
    else:
        status = Status('等待启动；接通 USB-CAN 不会自动使能电机', 'waiting')
        hints = ['真机未通电或尚未连接时，本窗口保持等待。',
                 '重新执行 ./run.sh 会直接打开 robot 的启动确认；也可在总控按 3 启动。']
    hints.append('无硬件也可在总控按 5 启动无输入仿真。')
    return [('robot', status),
            ('USB-CAN', Status('适配器可见；等待机械臂反馈' if visible else '未发现配置中的 CAN 接口或适配器', 'waiting')),
            ('配置', f'{can_port} · {device}')], hints


def transport_command(arguments, *, timeout, stop=None, stdout=subprocess.PIPE, stderr=subprocess.PIPE):
    """Cancel only this CLI and its ADB children; the shared XR service is detached."""
    with subprocess.Popen(arguments, stdout=stdout, stderr=stderr, text=True, start_new_session=True) as child:
        deadline = time.monotonic() + timeout
        try:
            while True:
                if stop is not None and stop.is_set():
                    raise InterruptedError('窗口已关闭')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise subprocess.TimeoutExpired(arguments, timeout)
                try:
                    output, error = child.communicate(timeout=min(.2, remaining))
                    return subprocess.CompletedProcess(arguments, child.returncode, output, error)
                except subprocess.TimeoutExpired:
                    continue
        finally:
            if child.returncode is None:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.communicate()


def vr_transport_status(stop=None):
    """Read transport evidence through the existing CLI, without opening any SDK."""
    try:
        result = transport_command([str(ROOT / 'run.sh'), 'quest', 'usb', 'status', '--json'], timeout=3, stop=stop)
        state = json.loads(result.stdout)
        if not isinstance(state, dict):
            raise ValueError('连接状态不是对象')
        return state
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        return {'error': str(exc)}


def prepare_vr(stop=None):
    """Prepare only the shared PC receiver and USB tunnel; never attach to robot."""
    logs = ROOT / 'outputs/logs/bench'
    path = logs / 'quest-transport.log'
    try:
        logs.mkdir(parents=True, exist_ok=True)
        with path.open('a') as output:
            result = transport_command([str(ROOT / 'run.sh'), 'quest', 'usb', 'prepare', '--launch-client'],
                                       stdout=output, stderr=output, timeout=25, stop=stop)
        if result.returncode:
            return Status(f'等待连接条件就绪；详情：{path.relative_to(ROOT)}', 'waiting')
        return Status('USB 转发与客户端已准备；等待头显连接。', 'ready')
    except (OSError, subprocess.SubprocessError) as exc:
        return Status(f'连接准备失败：{exc}；详情：{path.relative_to(ROOT)}', 'error')


def request_vr_wearing(enabled, stop=None):
    """Send only the headset wear request; keep client and robot lifecycles separate."""
    state = vr_transport_status(stop)
    if state.get('usb_status') != 'ready':
        reason = {'unauthorized': 'USB 未授权', 'missing': 'USB 未连接',
                  'offline': 'USB 已离线', 'multiple': '连接了多台 USB 设备'}.get(
                      state.get('usb_status'), state.get('error') or 'USB 状态未确认')
        return Status(f'{reason}，未发送佩戴请求；模拟佩戴不能代替 USB 授权', 'error')
    if not state.get('boot_completed'):
        return Status('头显仍在开机，未发送佩戴请求', 'waiting')
    label = '模拟佩戴' if enabled else '取消模拟佩戴'
    try:
        result = transport_command([str(ROOT / 'run.sh'), 'quest', 'wake', 'wear' if enabled else 'unwear'],
                                   timeout=10, stop=stop)
        if result.returncode:
            return Status(label + '失败：' + (result.stderr or result.stdout or 'ADB 未确认执行成功').strip(), 'error')
        return Status('已发送' + label + '请求；实际佩戴状态未读回', 'waiting')
    except (OSError, subprocess.SubprocessError) as exc:
        return Status(label + f'失败：{exc}', 'error')


class VRConnection:
    """One background job keeps slow ADB calls out of terminal input handling."""

    def __init__(self, *, auto_prepare=False):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='quest-transport')
        self.stop = threading.Event()
        self.future = None
        self.preparing = False
        self.wearing = False
        self.pending_wear = None
        self.wear_message = Status('未发送请求 · 实际佩戴状态未读回', 'disabled')
        self.auto_prepare = auto_prepare
        self.retry = False
        self.attempt = None
        self.state = {'checking': True}
        self.message = None
        self.checked_at = 0.
        self.retry_at = 0.
        self.backoff = 2.

    def snapshot(self):
        if self.future is not None:
            if not self.future.done():
                return self.state
            result = self.future.result()
            self.future = None
            if self.wearing:
                self.wear_message = result
                self.wearing = False
                self.checked_at = 0.
            elif self.preparing:
                self.message = result
                self.state = {'checking': True}
                self.checked_at = 0.
                self.preparing = False
                self.retry_at = time.monotonic() + self.backoff
                self.backoff = min(self.backoff * 2, 30.)
            else:
                self.state = result
                self.checked_at = time.monotonic()
        state = self.state
        need_prepare = (not state.get('receiver_listening') or
                        (state.get('usb_status') == 'ready' and state.get('boot_completed') and
                         (not state.get('reverse_configured') or not state.get('client_running'))))
        attempt = (state.get('receiver_listening'), state.get('usb_status'), state.get('usb_serial'),
                   state.get('boot_completed'), state.get('client_running'))
        known = not state.get('checking') and not state.get('error')
        if known and (not need_prepare or attempt != self.attempt):
            self.message = None
        if known and not need_prepare:
            self.attempt = None
            self.backoff = 2.
        if self.pending_wear is not None:
            enabled, self.pending_wear = self.pending_wear, None
            self.wearing = True
            self.wear_message = Status('正在发送' + ('模拟佩戴' if enabled else '取消模拟佩戴') + '请求…', 'waiting')
            self.future = self.executor.submit(request_vr_wearing, enabled, self.stop)
        elif self.retry or (self.auto_prepare and known and need_prepare and
                          (attempt != self.attempt or time.monotonic() >= self.retry_at)):
            self.retry = False
            self.attempt = attempt
            self.preparing = True
            self.message = Status('正在准备主机接收与 USB 转发…', 'waiting')
            self.future = self.executor.submit(prepare_vr, self.stop)
        elif time.monotonic() - self.checked_at >= 1:
            self.future = self.executor.submit(vr_transport_status, self.stop)
        return self.state

    def close(self):
        self.stop.set()
        self.executor.shutdown(wait=False, cancel_futures=True)

    def set_wearing(self, enabled):
        self.pending_wear = enabled


def vr_rows(found, transport=None):
    state = vr_transport_status() if transport is None else transport
    labels = {'ready': 'USB 已识别并授权', 'missing': 'USB 未检测到 Quest',
              'unauthorized': '等待头显内允许 USB 调试', 'offline': 'ADB 设备离线，等待重新连接',
              'multiple': '发现多个 USB 设备，请只连接一台 Quest'}
    usb = state.get('usb_status', 'missing')
    connected = state.get('usb_connected', False)
    if state.get('checking'):
        summary = Status('正在检查 USB 与头显连接…', 'waiting')
    elif state.get('error'):
        summary = Status('连接检查失败：' + state['error'], 'error')
    elif usb != 'ready':
        summary = Status(labels.get(usb, usb), 'waiting')
    elif not state.get('boot_completed'):
        summary = Status('USB 已授权，等待 Quest 开机完成', 'waiting')
    elif not state.get('receiver_listening'):
        summary = Status('主机 XR 接收服务未运行', 'waiting')
    elif not state.get('reverse_configured'):
        summary = Status('USB 已识别，等待建立转发', 'waiting')
    elif not state.get('client_running'):
        summary = Status('USB 转发就绪，等待客户端启动', 'waiting')
    elif not connected:
        summary = Status('客户端正在自动连接' if state.get('client_package') == 'com.thoughtintoaction.quest'
                         else 'USB 转发就绪，等待头显 Connect', 'waiting')
    else:
        summary = Status('USB 通信已连接；跟踪状态见下方', 'waiting')
    control, hints = vr_control_rows(found)
    if state.get('checking'):
        return [('VR', summary), *control], ['正在后台检查连接，终端按键仍可使用。', *hints]
    client_label = ('待 USB 连接 / 授权后检查' if usb != 'ready' else '运行中' if state.get('client_running')
                    else '等待 Quest 开机' if not state.get('boot_completed') else '尚未运行')
    if state.get('client_package'):
        client_label += ' · ' + ('自动连接版' if state['client_package'] == 'com.thoughtintoaction.quest' else '官方版')
    rows = [('VR', summary),
            ('主机接收', Status('在线 · TCP 63901' if state.get('receiver_listening') else '尚未监听 TCP 63901',
                              'ready' if state.get('receiver_listening') else 'waiting')),
            ('USB 设备', Status(labels.get(usb, usb), 'ready' if usb == 'ready' else 'waiting')),
            ('USB 转发', Status('已配置' if state.get('reverse_configured') else '尚未配置',
                              'ready' if state.get('reverse_configured') else 'waiting')),
            ('客户端', Status(client_label,
                            'ready' if state.get('client_running') else 'waiting')),
            ('头显连接', Status('USB TCP 已连接' if connected else '网络 TCP 连接，尚未确认 USB' if state.get('peers') else '未连接',
                              'ready' if connected else 'waiting'))]
    if not state.get('error') and connected and control[0][1].level == 'ready':
        rows[0] = ('VR', Status('USB 已连接 · 跟踪就绪', 'ready'))
    rows.extend(control)
    client_hint = ('自动连接版客户端：USB 地址 127.0.0.1，连接失败会自动重试。'
                   if state.get('client_package') == 'com.thoughtintoaction.quest' else
                   '官方旧版客户端需在头显填 127.0.0.1 并 Connect；免点击需要安装自动连接版 APK。')
    return rows, ['W 模拟佩戴 · U 取消模拟佩戴；需要 USB 授权，不切换机器人控制模式。', client_hint,
                  '可先开工作台再开头显；授权并开机完成后自动准备转发、打开客户端。',
                  '首次请在头显允许 USB 调试并勾选“始终允许”；按 R 重试。',
                  'TCP 已连接不等于追踪有效，也不会自动取得机械臂控制权。', *hints]


def vr_control_rows(found):
    from embodied.applications.workstation import bench

    hints = ['在总控选择带 Quest 的配置后，跟踪状态会自动更新。']
    if not found:
        return [('遥操接入', Status('robot 尚未运行', 'disabled'))], hints
    run, identity = found
    native = identity.get('native') or {}
    quest = identity.get('input') in ('vr', 'quest') or 'quest' in str(native.get('device', '')).lower()
    rows = []
    if not quest:
        rows += [('遥操接入', Status('当前 robot 为手动模式，未接入 Quest', 'disabled')),
                 ('运行目录', bench.relative(run)), ('VR 窗口', '保持待机；不会切换或重启现有内核')]
        return rows, ['需要遥操时，在总控选择带 Quest 的配置。',
                      '先按当前内核的退出提示处理设备，再切换配置。']
    if not (Path(run) / 'commands.sock').exists():
        rows += [('遥操接入', Status('已配置 · 跟踪待确认（无命令网关）', 'waiting')),
                 ('跟踪状态', '请查看内核日志或遥测'), ('运行目录', bench.relative(run))]
        return rows, hints
    from embodied.contracts.runtime.control import ControlClient

    try:
        status = ControlClient(run).status()
    except (OSError, ValueError, RuntimeError, EOFError, KeyError) as exc:
        return [('遥操接入', Status(f'状态读取失败：{exc}', 'error'))], hints
    ingress = status.get('ingresses', {}).get('native')
    if ingress is None:
        rows.append(('遥操接入', Status('等待原生 Quest 通道状态', 'waiting')))
    else:
        rows += [('遥操接入', Status('跟踪 / 通道就绪' if ingress.get('ready') else '等待有效跟踪',
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
            rows.append(('共享服务', Status('在线（只读查询）', 'ready')))
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            rows.append(('共享服务', Status(f'尚未就绪：{exc}', 'error')))
    else:
        rows.append(('共享服务', Status('等待启动', 'waiting')))
    detected = []
    for path in Path('/sys/bus/usb/devices').glob('*/product'):
        try:
            product = path.read_text().strip()
            if 'realsense' in product.lower():
                detected.append(product)
        except OSError:
            pass
    for name, options in config.get('cameras', {}).items():
        serial = str(options.get('serial', ''))
        feed = feeds.get(name, {}).get('camera', {})
        age = feed.get('age_s')
        valid_age = type(age) in (int, float) and math.isfinite(age) and age >= 0
        if feed.get('enabled') is False or (not feed and not options.get('enabled', False)):
            detail, level = '相机已关闭' if feed else '配置中未启用', 'disabled'
        elif feed.get('error'):
            detail, level = str(feed['error']), 'error'
        elif feed.get('enabled') is True and valid_age and age < .5:
            detail, level = f'画面在线 · 帧龄 {age * 1000:.0f} ms', 'ready'
        elif feed:
            stale = valid_age and age >= .5
            detail = f'画面已过期 · 帧龄 {age:.2f} s' if stale else '等待相机服务提供有效帧'
            level = 'error' if stale else 'waiting'
        elif serial and detected:
            detail, level = 'RealSense USB 可见；等待相机服务确认', 'waiting'
        elif serial:
            detail, level = '等待 USB 相机连接', 'waiting'
        else:
            detail, level = '未指定序列号；检查相机配置', 'error'
        rows.append((options.get('label', name), Status(f'{detail} · {name}', level)))
    if detected:
        rows.append(('USB 识别', '；'.join(detected)))
    return rows, ['工作台会启动共享相机服务；按 R 重试，设备断线后服务自动重连。',
                  '关闭或刷新本窗口不会停止共享服务，也不会中断采集。',
                  '每台相机由共享服务按 SDK 序列号确认；USB 序列号不用于匹配配置。',
                  '独立启动共享服务：./run.sh camera service']


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
        elif role == 'phone':
            from embodied.applications.workstation.phone_panel import rows as phone_rows
            rows, hints = phone_rows()
        else:
            number, url = service_address(role)
            rows = [('地址', url), ('端口', '已有监听；未确认服务身份' if listening(number, service_host(role)) else '尚未监听')]
            hints = ['启动命令：' + ' '.join(service_command(role))]
        return TITLES[role], rows, hints
    except (OSError, ValueError, RuntimeError, KeyError, EOFError) as exc:
        return TITLES[role], [('状态', Status(f'状态读取失败：{exc}', 'error'))], ['窗口保持打开；修复连接或配置后按 R 刷新。']


class Service:
    """Own web children; only request startup of the independent shared camera service."""

    def __init__(self, role):
        if role not in (*SERVICE_COMMANDS, 'camera', 'phone'):
            raise ValueError('Only web/site/camera/phone panels may start a service')
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
        if self.role == 'phone':
            from embodied.devices.inputs.phone_service import ensure_service, LOG
            self.attempted = True
            self.log_path = LOG
            try:
                ensure_service()
                self.message = '手机接收已就绪 · 默认仅监看'
            except (OSError, ValueError, RuntimeError) as exc:
                self.message = Status(str(exc), 'error')
            return
        if self.role == 'camera':
            from embodied.devices.sensors.client import ensure_service
            self.attempted = True
            try:
                logs = load_config().get('paths', {}).get('log_dir', 'outputs/logs')
                self.log_path = resolve_path(logs) / 'camera-service.log'
                ensure_service()
                self.message = '共享服务已连接；相机是否出帧见上方状态'
            except (OSError, ValueError, RuntimeError, KeyError) as exc:
                self.message = Status(f'共享服务未启动：{exc}；修复后按 R 重试', 'error')
            return
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
        if self.role in ('camera', 'phone'):
            return  # The shared service may be feeding a recorder after this pane closes.
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
    active = any(value.level == 'active' for value in states)
    body = Group(Text('  手机遥操已开启 · S / 空格停止  ', style='bold black on bright_cyan'), table) if active else table
    sections = [Panel(body, title=title, border_style=('red' if primary and primary.level == 'error' else
                    'bright_cyan' if active else primary.style if primary else '#b7a1e8')),
                Panel(Group(*(Text(line) for line in hints)), title='下一步', border_style='dim')]
    if service is not None:
        logs = service.logs()
        if logs:
            sections.append(Panel(Group(*(Text(line, overflow='fold') for line in logs)),
                                  title='最近日志', border_style='dim'))
    sections.append(Text('Ctrl+B → 1 返回总控 · Ctrl+B → D 离开工作台 · R ' +
                         ('连接 / 刷新' if title == TITLES['vr'] else '重试 / 刷新' if service is not None else '刷新') +
                         (' · W 模拟佩戴 · U 取消模拟佩戴' if title == TITLES['vr'] else '') +
                         (' · P 预抓取 · H 回零 · 空格停止 · D 阻尼' if title == TITLES['kernel'] else '') +
                         (' · T 手机遥操 · S / 空格停止' if title == TITLES['phone'] else '') +
                         (' · W 打开浏览器' if service is not None and service.role == 'web' else ''), style='dim'))
    return Group(*sections)


def mark_waiting(role):
    if role != 'kernel' or not os.environ.get('TMUX_PANE'):
        return
    try:
        from embodied.applications.workstation import bench
        pane = os.environ['TMUX_PANE']
        state = bench.tmux('display-message', '-p', '-t', pane, '#{@bench_activity}', check=False).stdout.strip()
        if state != 'failed':
            bench.tmux('set-option', '-p', '-t', pane, '@bench_activity', 'waiting', check=False)
    except (OSError, RuntimeError):
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=TITLES)
    parser.add_argument('--serve', action='store_true', help='Prepare web/site/camera or VR transport; press R to retry')
    args = parser.parse_args(argv)
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        title, rows, hints = snapshot(args.role)
        print('\n'.join([title, *(f'{label}：{value}' for label, value in rows), '', *hints]))
        return 0
    from rich.console import Console
    from rich.live import Live
    from embodied.core.terminal_keys import read_key, terminal_keys

    mark_waiting(args.role)
    service = Service(args.role) if args.serve and args.role in (*SERVICE_COMMANDS, 'camera', 'phone') else None
    stopping = False
    vr = VRConnection(auto_prepare=args.serve) if args.role == 'vr' else None
    from embodied.applications.workstation.robot_panel import RobotControls
    robot = RobotControls() if args.role == 'kernel' else None
    focused_run = None

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
                if time.monotonic() - refreshed >= (.1 if robot is not None else 1):
                    if service is not None:
                        service.open_when_ready()
                    if vr is not None:
                        transport = vr.snapshot()
                        from embodied.applications.workstation import bench
                        rows, hints = vr_rows(bench.current_run(), transport)
                        rows.append(('佩戴请求', vr.wear_message))
                        if vr.message is not None:
                            rows.append(('连接准备', vr.message))
                        state = (TITLES['vr'], rows, hints)
                    else:
                        state = service.snapshot() if service else snapshot(args.role)
                    if robot is not None:
                        from embodied.applications.workstation import bench
                        found = bench.current_run()
                        state[1].extend(robot.rows(found))
                        if found and found[0] != focused_run:
                            if bench.focus_robot_controls(os.environ.get('TMUX_PANE')):
                                focused_run = found[0]
                        elif found is None:
                            focused_run = None
                    live.update(render(state, service), refresh=True)
                    refreshed = time.monotonic()
                key = read_key(.05 if robot is not None else .2)
                if key in ('eof', '\x03', '\x04'):
                    break
                if key == 'r':
                    if service is not None:
                        service.start()
                    elif vr is not None and args.serve:
                        vr.retry = not vr.preparing
                    refreshed = 0.
                if key == 'w' and args.role == 'web' and service is not None:
                    service.browser_message = open_browser()
                    refreshed = 0.
                if vr is not None and key in ('w', 'u'):
                    vr.set_wearing(key == 'w')
                    refreshed = 0.
                if robot is not None and key is not None:
                    from embodied.applications.workstation import bench
                    robot.press(key, bench.current_run())
                    refreshed = 0.
                if args.role == 'phone' and key in ('t', 's', ' '):
                    from embodied.applications.workstation.phone_panel import press
                    try:
                        message = press(key)
                    except (OSError, ValueError, RuntimeError) as exc:
                        message = Status(str(exc), 'error')
                    if service is not None:
                        service.message = message
                    refreshed = 0.
    finally:
        if robot is not None:
            robot.close()
        if vr is not None:
            vr.close()
        if service is not None:
            service.stop()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
