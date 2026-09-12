"""User-operated viewer lifecycle. No arm commands or camera device access."""
import argparse
from contextlib import closing
import subprocess
import time
import sys
from robot_core.project import PROJECT_ROOT, load_config, resolve_path
from robot_core.session_protocol import request
from robot_core.device_lease import DeviceLease


def socket_path():
    return resolve_path(load_config().get('viewer_service', {}).get('socket', 'outputs/runtime/viewer.sock'))


def status():
    try:
        return request(socket_path(), 'status', timeout=.5)
    except (FileNotFoundError, ConnectionRefusedError):
        return None


def command(action):
    if action.startswith('camera-'):
        from robot_core.camera_control import set_enabled
        result = set_enabled(action == 'camera-on')
        return '相机已' + ('开启' if result['camera']['enabled'] else '关闭')
    with closing(DeviceLease('viewer-launch:' + str(socket_path()))):
        info = status()
        if action == 'stop':
            if info:
                request(socket_path(), 'stop', timeout=1)
                deadline = time.monotonic() + 10
                while status() is not None and time.monotonic() < deadline:
                    time.sleep(.1)
                if status() is not None:
                    raise RuntimeError('网页仍在关闭，请稍后查看状态')
            return '网页监视器已关闭；相机、机械臂服务保持各自状态'
        if action == 'start' and info is None:
            folder = resolve_path(load_config()['paths']['log_dir'])
            folder.mkdir(parents=True, exist_ok=True)
            with (folder / 'viewer-service.log').open('ab') as log:
                child = subprocess.Popen([str(PROJECT_ROOT / 'run.sh'), 'view-arm', '--camera' if load_config().get('viewer_service', {}).get('camera', True) else '--no-camera', '--port', str(load_config().get('viewer_service', {}).get('port', 18081)), '--managed'],
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
        return '网页未运行（W 启动）' if info is None else f"网页运行中：{info['url']} · PID {info['pid']}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', choices=['start', 'stop', 'status', 'camera-on', 'camera-off'])
    args = parser.parse_args()
    if args.action:
        try:
            print(command(args.action), flush=True)
        except (OSError, ValueError, RuntimeError) as exc:
            print(str(exc), file=sys.stderr)
            raise SystemExit(1)
        return
    print('网页 / 相机控制：W 启动网页 · E 停止网页 · I 状态 · C 开相机 · X 关相机 · Q 退出终端（服务保持）')
    print(command('status'))
    choices = {'w':'start', 'e':'stop', 'i':'status', 'c':'camera-on', 'x':'camera-off'}
    while True:
        try:
            key = input('网页控制 > ').strip().lower()
            if key == 'q':
                return
            if key in choices:
                try:
                    print(command(choices[key]), flush=True)
                except (OSError, ValueError, RuntimeError) as exc:
                    print(str(exc), flush=True)
        except (EOFError, KeyboardInterrupt):
            return


if __name__ == '__main__':
    main()
