#!/usr/bin/env python3
"""Configure USB transport and report observed XR sockets using Rich."""
import argparse
from contextlib import closing
import ipaddress
import json
import subprocess
import time

from embodied.contracts.device_lease import DeviceLease
from embodied.core.project import PROJECT_ROOT, LOG_DIR

from rich.console import Console
from rich.panel import Panel
from rich.text import Text

console = Console()
CLIENT_PACKAGE = 'com.xrobotoolkit.client.quest'
AUTO_CLIENT_PACKAGE = 'com.thoughtintoaction.quest'


def command(*args):
    result = subprocess.run(args, text=True, capture_output=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or " ".join(args) + " failed")
    return result.stdout


def usb_devices_from_adb(output):
    devices = []
    for line in output.splitlines():
        fields = line.split()
        if len(fields) < 2:
            continue
        serial, state = fields[:2]
        if fields[1:3] == ['no', 'permissions']:
            state = 'unauthorized'
        usb = any(field.startswith('usb:') for field in fields[2:])
        # Unauthorized USB devices may lack transport details. Keep them as
        # candidates, but require explicit USB evidence for an authorized one.
        pending = state in ('unauthorized', 'offline') and ':' not in serial and not serial.endswith('._tcp')
        if usb or pending:
            devices.append({'serial': serial, 'state': state})
    return devices


def usb_selection(devices):
    authorized = [device['serial'] for device in devices if device['state'] == 'device']
    if len(authorized) == 1:
        return 'ready', authorized[0]
    if len(authorized) > 1 or len(devices) > 1:
        return 'multiple', None
    if devices:
        return ('unauthorized' if devices[0]['state'] == 'unauthorized' else 'offline'), None
    return 'missing', None


def require_usb(status, serial):
    if status == 'ready':
        return serial
    messages = {
        'missing': '未检测到已授权的 USB Quest；请检查数据线，并在头显内允许 USB 调试。',
        'unauthorized': 'USB 设备尚未授权；请在头显内允许 USB 调试。',
        'offline': 'USB 设备处于 ADB offline 状态；请重新连接数据线。',
        'multiple': '检测到多个 USB 设备；请只保留一台已授权的 USB Quest。',
    }
    raise RuntimeError(messages[status])


def usb_device():
    devices = usb_devices_from_adb(command('adb', 'devices', '-l'))
    return require_usb(*usb_selection(devices))


def xr_listening():
    return bool(command('ss', '-Hltn', 'sport = :63901').strip())


def ensure_xr_service():
    """The ADB tunnel needs the PC receiver; reuse it without interrupting clients."""
    if xr_listening():
        return
    with closing(DeviceLease('xr-service-launch')):
        if xr_listening():
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_path = LOG_DIR / 'xr-service.log'
        console.print('正在启动主机 XR 接收服务…')
        with log_path.open('ab', buffering=0) as log:
            result = subprocess.run(['bash', str(PROJECT_ROOT / 'scripts/quest/start_service.sh')],
                                    stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                    start_new_session=True, timeout=10)
        if result.returncode:
            raise RuntimeError(f'XR 接收服务启动失败，请查看 {log_path}')
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if xr_listening():
                console.print('主机 XR 接收服务已就绪：TCP 63901')
                return
            time.sleep(.2)
        raise RuntimeError(f'主机 XR 接收端口 63901 未就绪，请查看 {log_path}')


def peers_from_ss(output):
    peers = []
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 5 and parts[0] == "ESTAB" and parts[3].endswith(":63901"):
            host = parts[4].rsplit(":", 1)[0].strip("[]")
            address = ipaddress.ip_address(host)
            peers.append(address.ipv4_mapped if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped else address)
    return peers


def client_state(serial):
    adb = ('adb', '-s', serial, 'shell')
    state = {'boot_completed': command(*adb, 'getprop', 'sys.boot_completed').strip() == '1',
             'client_running': False, 'client_package': CLIENT_PACKAGE}
    if not state['boot_completed']:
        return state
    processes = command(*adb, 'ps', '-A', '-o', 'NAME').split()
    for package in (AUTO_CLIENT_PACKAGE, CLIENT_PACKAGE):
        if package in processes:
            return {**state, 'client_running': True, 'client_package': package}
    packages = command(*adb, 'pm', 'list', 'packages', AUTO_CLIENT_PACKAGE).split()
    if 'package:' + AUTO_CLIENT_PACKAGE in packages:
        state['client_package'] = AUTO_CLIENT_PACKAGE
    return state


def launch_client(serial):
    state = client_state(serial)
    if not state['boot_completed']:
        raise RuntimeError('Quest 正在开机；系统就绪后工作台会继续准备连接。')
    if state['client_running']:
        return  # Never relaunch or interrupt an already running client.
    activity = state['client_package'] + '/com.unity3d.player.UnityPlayerActivity'
    output = command('adb', '-s', serial, 'shell', 'am', 'start', '-n', activity)
    if 'Error' in output or 'Exception' in output:
        raise RuntimeError('Quest 客户端未启动：' + output.strip())
    console.print('已请求打开 Quest 客户端；连接结果以 TCP 状态为准。')


def status_snapshot():
    """Independent read-only probes; missing USB must not hide receiver/network state."""
    state = {'receiver_listening': False, 'usb_devices': [], 'usb_serial': None,
             'usb_status': 'missing', 'reverse_configured': False, 'peers': [],
             'usb_connected': False, 'boot_completed': False, 'client_running': False,
             'client_package': None, 'error': None}
    errors = []
    failures = (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired)
    try:
        state['receiver_listening'] = xr_listening()
    except failures as exc:
        errors.append(f'主机接收端查询失败：{exc}')
    try:
        state['usb_devices'] = usb_devices_from_adb(command('adb', 'devices', '-l'))
        state['usb_status'], state['usb_serial'] = usb_selection(state['usb_devices'])
    except failures as exc:
        errors.append(f'USB 设备查询失败：{exc}')
    if state['usb_serial'] is not None:
        try:
            reverse = command('adb', '-s', state['usb_serial'], 'reverse', '--list')
            state['reverse_configured'] = any(line.split()[-2:] == ['tcp:63901', 'tcp:63901']
                                              for line in reverse.splitlines())
        except failures as exc:
            errors.append(f'ADB 转发查询失败：{exc}')
        try:
            state.update(client_state(state['usb_serial']))
        except failures as exc:
            errors.append(f'Quest 客户端查询失败：{exc}')
    peers = []
    try:
        peers = peers_from_ss(command('ss', '-Htn'))
        state['peers'] = list(map(str, peers))
    except failures as exc:
        errors.append(f'XR 连接查询失败：{exc}')
    local = any(peer.is_loopback for peer in peers)
    remote = any(not peer.is_loopback for peer in peers)
    state['usb_connected'] = state['reverse_configured'] and local and not remote
    state['error'] = '；'.join(errors) or None
    return state


def inspect():
    state = status_snapshot()
    if state['error']:
        raise RuntimeError(state['error'])
    serial = require_usb(state['usb_status'], state['usb_serial'])
    peers = list(map(ipaddress.ip_address, state['peers']))
    remote = any(not peer.is_loopback for peer in peers)
    ready = state['usb_connected']
    label = 'USB 数据连接已确认' if ready else '仍有网络连接（非 USB）' if remote else '等待 Quest 通过 USB 连接'
    return ready, label, serial, state['reverse_configured'], peers


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
    parser.add_argument('--json', action='store_true', help='status: print independent receiver/USB/TCP state as JSON')
    parser.add_argument('--launch-client', action='store_true', help='prepare: open Quest client after Android finishes booting')
    args = parser.parse_args(argv)
    if args.launch_client and args.action != 'prepare':
        parser.error('--launch-client 仅用于 prepare')
    if args.json:
        if args.action != 'status':
            parser.error('--json 仅用于 status')
        state = status_snapshot()
        print(json.dumps(state, ensure_ascii=False))
        return int(state['error'] is not None)
    try:
        if args.action == "prepare":
            ensure_xr_service()
            serial = usb_device()
            # Idempotent; don't remove a working mapping and interrupt its stream.
            command("adb", "-s", serial, "reverse", "tcp:63901", "tcp:63901")
            if args.launch_client:
                launch_client(serial)
        state = inspect()
        if not state[0] and not xr_listening():
            console.print('[yellow]主机 XR 接收服务未运行；按 W 重试会自动启动。[/yellow]')
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
