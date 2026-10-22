"""Explicit workbench restarts using the existing owners and shutdown paths."""
from contextlib import closing
import shlex
import time

from embodied.applications.workstation import bench


def _restart_interfaces():
    refreshed, preserved = [], []
    for role in ('web', 'site', 'vr', 'camera', 'phone', 'home'):
        found = bench.window(role)
        if found is None:
            continue
        pane, running = found
        original = bench.tmux('display-message', '-p', '-t', pane, '#{pane_start_command}').stdout.strip()
        try:
            args = shlex.split(original)
        except ValueError:
            args = []
        module = ('embodied.applications.workstation.bench' if role == 'home'
                  else 'embodied.applications.workstation.bench_panels')
        expected = ['-m', module, '_dashboard' if role == 'home' else role]
        is_panel = any(args[index:index + 3] == expected for index in range(len(args)))
        if not is_panel:
            preserved.append(bench.WINDOWS[role])
            continue
        command = bench.dashboard_command() if role == 'home' else bench.panel_command(role)
        if running:
            bench.refresh_panel(pane, command)
        else:
            bench.tmux('respawn-pane', '-t', pane, '-c', str(bench.ROOT), command)
        bench.tmux('set-option', '-p', '-t', pane, '@bench_ui_revision', bench.UI_REVISION)
        refreshed.append(bench.WINDOWS[role])
    home = bench.window('home')
    if home:
        bench.tmux('select-window', '-t', home[0])
    message = '已重启界面：' + ('、'.join(refreshed) or '无可重启面板')
    if preserved:
        message += '；保留其它任务：' + '、'.join(preserved)
    return message + '。robot、命令任务和共享相机服务继续运行；外部网页服务保持原状。'


def _saved_robot_command(pane):
    saved = bench.tmux('show-options', '-pqv', '-t', pane, '@bench_kernel_command').stdout.strip()
    for _, _, command, real in bench.KERNELS:
        if saved == command or (real and saved == command + ' --yes'):
            return command + (' --yes' if real else '')
    raise RuntimeError('未记录可恢复的 robot 启动配置；请先停止 robot，再从总控选择配置启动。')


def _check_robot_activity(run, identity):
    from embodied.applications.monitor.kernel_view import collection_snapshot
    from embodied.core.project import load_config, resolve_path
    roots = [bench.ROOT / 'outputs/datasets/runtime',
             resolve_path(load_config()['paths']['dataset_dir']) / 'mujoco']
    capture = collection_snapshot(identity, roots)
    if (capture.get('recording') or {}).get('live'):
        raise RuntimeError('正在采集或结束写盘；请先在采集终端结束任务，再重启 robot。')
    if identity.get('managed'):
        from embodied.contracts.runtime.control import ControlClient
        status = ControlClient(run).status()
        owner = status.get('owner') or {}
        if (owner and owner.get('ingress', 'rpc') == 'rpc') or any(
                operation.get('status') == 'running' for operation in status.get('operations', [])):
            raise RuntimeError('有控制任务持有 robot；请先在原终端结束动作或策略，再重启。')


def _restart_robot(*, confirmed, wait_s=30):
    if not confirmed:
        raise RuntimeError('重启 robot 需要确认；真机退出会进入阻尼，请先支撑机械臂。')
    owned = bench.owned_run()
    if owned is None:
        raise RuntimeError('robot 未运行或不是本工作台启动的；请在原终端处理，未发送停止信号。')
    pane = bench.window('kernel')
    command = _saved_robot_command(pane[0])
    run, identity = owned
    from embodied.devices.sensors.control import check_legacy_session, recording_guard
    # Keep cooperating collectors excluded throughout shutdown and restart submission.
    try:
        guard = recording_guard()
    except RuntimeError as exc:
        raise RuntimeError('正在使用共享相机采集；请先结束采集，再重启 robot。') from exc
    with closing(guard):
        check_legacy_session()
        _check_robot_activity(run, identity)
        if bench.owned_run() != owned:
            raise RuntimeError('robot 运行实例已变化，未停止；请刷新总控后重试。')
        bench.stop_kernel()
        deadline = time.monotonic() + wait_s
        while bench.owned_run() or bench.kernel_busy():
            if time.monotonic() >= deadline:
                raise RuntimeError('robot 仍在退出，保留现场；未强制停止，也未启动新实例。')
            time.sleep(.1)
        bench.start_kernel(command)
    return '已沿用原配置重新启动 robot；Ctrl+B → 3 查看启动与反馈。'


def restart(target, *, robot_confirmed=False):
    """Restart only the selected workbench component, never an inferred process."""
    if target not in ('interfaces', 'robot', 'camera'):
        raise ValueError('未知重启目标：' + str(target))
    if not bench.exists():
        raise RuntimeError('工作台未运行；请先执行 ./run.sh。')
    bench.check_session_owner()
    if target == 'interfaces':
        return _restart_interfaces()
    if target == 'robot':
        return _restart_robot(confirmed=robot_confirmed)
    from embodied.devices.sensors.lifecycle import stop_service
    from embodied.devices.sensors.client import ensure_service
    stop_service()
    ensure_service()
    return '共享相机服务已重启；图像是否恢复请查看总控和相机窗口。'
