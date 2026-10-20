"""User-operated viewer lifecycle. No arm commands or camera device access."""
import argparse
from contextlib import closing
import subprocess
import time
import sys
from robot_core.project import PROJECT_ROOT, load_config, resolve_path
from robot_core.session_protocol import request
from embodied_contracts.device_lease import DeviceLease
from .mode import MODES, selected_mode


def socket_path():
    return resolve_path(load_config().get('viewer_service', {}).get('socket', 'outputs/runtime/viewer.sock'))


def status():
    try:
        return request(socket_path(), 'status', timeout=.5)
    except (FileNotFoundError, ConnectionRefusedError):
        return None


def command(action, *, mode=None):
    mode = selected_mode() if mode is None else mode
    if mode not in MODES:
        raise ValueError('无效网页模式')
    if action.startswith('camera-'):
        from robot_core.camera_control import set_enabled
        result = set_enabled(action == 'camera-on')
        return '相机已' + ('开启' if result['camera']['enabled'] else '关闭')
    with closing(DeviceLease('viewer-launch:' + str(socket_path()))):
        info = status()
        if action in ('sim-resume', 'sim-pause', 'sim-home', 'sim-pregrasp', 'sim-record', 'sim-save', 'demo-restart', 'demo-stop'):
            if not info or info.get('mode') != 'simulation':
                raise RuntimeError('请先在总控按 M 切到仿真')
            request(socket_path(), action, timeout=6)
            return {'sim-home':'仿真正在回零', 'sim-pregrasp':'仿真正在前往预抓取位', 'sim-record':'正在记录 MuJoCo 原始状态', 'sim-save':'MuJoCo 原始状态已保存'}.get(action, '仿真遥操已允许：松开全部按键后右手 A 长按切换模式' if action in ('sim-resume', 'demo-restart') else '仿真已暂停，保持姿态')
        switching = action == 'start' and info is not None and info.get('mode', 'real') != mode
        if switching and info.get('mode_protocol') == 2:
            request(socket_path(), 'select-mode', mode=mode, timeout=10)
            info = status()
            switching = False
        if action == 'stop' or switching:
            if info:
                request(socket_path(), 'stop', timeout=1)
                deadline = time.monotonic() + 25
                while status() is not None and time.monotonic() < deadline:
                    time.sleep(.1)
                if status() is not None:
                    raise RuntimeError('网页仍在关闭，请稍后查看状态')
            info = None
            if action == 'stop':
                return '网页及其仿真已关闭；相机、机械臂服务保持各自状态'
        if action == 'start' and info is None:
            folder = resolve_path(load_config()['paths']['log_dir'])
            folder.mkdir(parents=True, exist_ok=True)
            with (folder / 'viewer-service.log').open('ab') as log:
                child = subprocess.Popen([str(PROJECT_ROOT / 'run.sh'), 'view-arm', '--camera' if load_config().get('viewer_service', {}).get('camera', True) else '--no-camera', '--port', str(load_config().get('viewer_service', {}).get('port', 18081)), '--managed', '--mode', mode],
                    stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic() + 15
            while info is None and time.monotonic() < deadline and child.poll() is None:
                time.sleep(.1)
                info = status()
            if info is None:
                if child.poll() is None:
                    child.terminate()
                    child.wait(timeout=5)
                raise RuntimeError('网页启动失败，请查看 outputs/logs/viewer-service.log')
        return '网页未运行（W 启动）' if info is None else f"{MODES[info.get('mode', 'real')]} · {info['url']} · PID {info['pid']}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', choices=['start', 'stop', 'status', 'camera-on', 'camera-off', 'sim-resume', 'sim-pause', 'sim-home', 'sim-pregrasp', 'sim-record', 'sim-save', 'demo-restart', 'demo-stop'])
    args = parser.parse_args()
    if args.action:
        try:
            print(command(args.action), flush=True)
        except (OSError, ValueError, RuntimeError) as exc:
            print(str(exc), file=sys.stderr)
            raise SystemExit(1)
        return
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    console = Console()
    menu = Table.grid(padding=(0, 2))
    menu.add_column(style='bold cyan')
    menu.add_column()
    for key, detail in [('W / E', '启动 / 关闭网页'), ('C / X', '开启 / 关闭腕部相机（双相机可在网页分别操作）'),
                        ('U / O', '仿真遥操待机 / 暂停保持'),
                        ('I', '查看网页状态；总控 M 仿真 / F 真机'), ('Q', '退出本控制台，服务继续运行')]:
        menu.add_row(key, detail)
    console.print(Panel(menu, title='网页 · 相机控制', subtitle='输入命令后回车', border_style='cyan', expand=False))
    console.print(Panel(Text(command('status')), title='运行状态', border_style='green', expand=False))
    choices = {'w':'start', 'e':'stop', 'i':'status', 'c':'camera-on', 'x':'camera-off', 'u':'sim-resume', 'o':'sim-pause'}
    while True:
        try:
            key = console.input('[bold cyan]网页控制 > [/]').strip().lower()
            if key == 'q':
                return
            if key in choices:
                try:
                    console.print(Panel(Text(command(choices[key])), title='操作结果', border_style='cyan', expand=False))
                except (OSError, ValueError, RuntimeError) as exc:
                    console.print(Panel(Text(str(exc)), title='操作未完成', border_style='yellow', expand=False))
        except (EOFError, KeyboardInterrupt):
            return


if __name__ == '__main__':
    main()
