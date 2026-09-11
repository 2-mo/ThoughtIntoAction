#!/usr/bin/env python3
"""Control the native PC-service discovery setting; never reconnect USB or touch CAN."""
import argparse
import configparser
import os
from pathlib import Path
import tempfile
import time


def config_path():
    default = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'QuestArmTeleop/xr-discovery.ini'
    return Path(os.environ.get('QUEST_XR_DISCOVERY_CONFIG', default)).expanduser().resolve()


def read(path):
    value = configparser.ConfigParser()
    value.optionxform = str
    value.read(path, encoding='utf-8')
    return value


def write_enabled(path, enabled):
    config = read(path)
    if not config.has_section('TCP'):
        config.add_section('TCP')
    # QSettings keys are case-sensitive; preserve the native spelling.
    config.optionxform = str
    for key in list(config['TCP']):
        if key.lower() == 'broadcastenabled':
            config.remove_option('TCP', key)
    config.set('TCP', 'BroadcastEnabled', 'true' if enabled else 'false')
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as stream:
            name = Path(stream.name)
            config.write(stream)
        name.replace(path)
    finally:
        if name is not None:
            name.unlink(missing_ok=True)


def observed(path):
    try:
        status = read(str(path) + '.status')
        pid = status.getint('Service', 'Pid')
        age = time.time() - status.getint('Service', 'CheckedAtMs') / 1000
        if status.getint('Service', 'Protocol') != 1 or not 0 <= age < 12:
            return None
        if Path(f'/proc/{pid}/exe').resolve().name != 'RoboticsServiceProcess':
            return None
        return status.getboolean('TCP', 'BroadcastEnabled')
    except (OSError, ValueError, configparser.Error):
        return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', default='status', choices=['on', 'off', 'status'])
    args = parser.parse_args(argv)
    path = config_path()
    try:
        if args.action != 'status':
            enabled = args.action == 'on'
            write_enabled(path, enabled)
            deadline = time.monotonic() + 7
            while time.monotonic() < deadline:
                if observed(path) is enabled:
                    print('头显发现广播已' + ('开启' if enabled else '关闭') + '；USB 追踪连接保持。')
                    return 0
                time.sleep(.1)
            print(f'配置已保存至 {path}，但尚未收到 XR 服务生效确认。')
            print('需要安装带广播开关的原生模块，并重启一次 XR 服务；未自动重启当前服务。')
            return 1
        desired = read(path).getboolean('TCP', 'BroadcastEnabled', fallback=False)
        actual = observed(path)
        print('广播配置：' + ('开启' if desired else '关闭（默认）'))
        print('服务实际状态：' + ('尚未确认，服务未运行或未加载新模块' if actual is None
                                 else '开启' if actual else '关闭'))
        print(f'配置文件：{path}')
        return 0
    except (OSError, ValueError, configparser.Error) as exc:
        print(f'广播设置失败：{exc}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
