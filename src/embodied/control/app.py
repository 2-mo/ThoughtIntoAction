"""Launch the existing VR pipeline with an explicit simulation or native SDK backend."""

import argparse
import json
import os
from pathlib import Path
import platform
import signal
import sys
import time
import uuid
from embodied.control.runtime import Children
from embodied.contracts.registry import Registry
from embodied.control.settings import ROOT, load_settings


def validate_run(backend: str, source: str, headless: bool, managed=False):
    if backend not in ('sim', 'real'):
        raise ValueError('backend must be sim or real')
    if backend == 'real' and source != 'vr' and not (managed and source == 'idle'):
        raise ValueError('Real hardware accepts VR input only; synthetic demo is simulation only')
    if not headless and not os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
        raise ValueError("No DISPLAY is available; use --headless or run in the desktop session")


def validate_completion(backend_status, report):
    if backend_status != 0:
        raise RuntimeError(f"Backend did not exit cleanly: {backend_status}")
    if report is None:
        raise RuntimeError("Backend verification report is missing")
    if report.get("error"):
        raise RuntimeError(report["error"])
    if not report.get('shutdown_completed'):
        raise RuntimeError('Robot driver shutdown did not complete')


def doctor(config_path=None, require_vr=False, vr_config_path=None):
    settings = load_settings(config_path, vr_config_path)
    driver = Registry().create('robot', settings.robot.simulation_plugin, runtime='control')
    checks = {"python": sys.version.split()[0], "platform": platform.platform(),
              "control_hz": settings.robot.control_hz, "robot": settings.robot.name}
    errors = []
    try:
        checks['robot'] = driver.preflight(settings.robot)
        errors.extend(checks['robot'].get('errors', []))
    except Exception as exc:
        errors.append(str(exc))
    if require_vr:
        try:
            device = Registry().load('vr', 'arx.quest', runtime='control')
            device.validate(settings)
            checks['input'] = {'ready': True, 'profile': settings.vr_config_path}
        except Exception as exc:
            errors.append(str(exc))
    checks['errors'] = errors
    print(json.dumps(checks, ensure_ascii=False, indent=2))
    return int(bool(errors))


def native_components(registry, device, routing, settings, backend):
    spec = registry.spec('vr', device)
    if backend == 'real' and spec.simulation_only:
        raise ValueError('Synthetic input is simulation only')
    driver = registry.load('vr', device, runtime='control')
    driver.validate(settings)
    return driver, registry.load('routing', routing, runtime='control')


def print_ready(device_plugin, routing_plugin, gripper_max_width):
    """Describe the selected input, including native plugins used by managed profiles."""
    if device_plugin == 'arx.quest':
        print('robot · 已就绪，等待 Quest 控制器输入。', flush=True)
        trigger = '夹爪' if gripper_max_width > 0 else '未启用（无夹爪）'
        print(f'右手柄：长按 A 150 ms 切换模式 · Grip 跟随 · Trigger {trigger} · A+B 回 Home。', flush=True)
        if routing_plugin == 'arx.workbench':
            print('左手柄：数据采集。', flush=True)
    elif device_plugin == 'arx.demo':
        print('robot · 仿真演示已就绪，约 4.5 秒后开始自动运动。', flush=True)
    elif device_plugin:
        print(f'robot · 已就绪，输入插件：{device_plugin}。', flush=True)
    else:
        print('robot · 已就绪，等待网页或命令控制。', flush=True)


def component_exit_message(name, code):
    if name == 'viewer':
        state = '正常关闭' if code == 0 else '异常退出'
        return f'桌面视图已退出（{state}，退出码 {code}）；robot 仍在运行。详情：日志目录中的 viewer.log。'
    if name == 'observer':
        return f'监看反馈服务已退出（退出码 {code}）；监看反馈已中断，robot 仍在运行。详情：日志目录中的 observer.log。'
    return f'输入组件 {name} 已退出（退出码 {code}）；相关输入将退出控制。详情：日志目录中的 {name}.log。'


def run(args):
    validate_run(args.backend, args.input, args.headless, getattr(args, "managed", False))
    record_tuning = args.record_tuning
    settings = load_settings(args.config, args.vr_config)
    from embodied.contracts.registry import Registry
    registry = Registry()
    selected_robot = args.robot_plugin or (settings.robot.hardware_plugin if args.backend == 'real' else settings.robot.simulation_plugin)
    if args.backend == 'real':
        driver = registry.create('robot', selected_robot, runtime='control')
        checks = driver.preflight(settings.robot)
        if not checks['ready']:
            raise ValueError(f'robot 启动检查未通过：{checks}')
        record_tuning = True
    native_device = native_routing = None
    device_plugin = args.input_plugin
    device_plugin = device_plugin or {'vr': 'arx.quest', 'demo': 'arx.demo'}.get(args.input)
    if device_plugin:
        native_device, native_routing = native_components(registry, device_plugin, args.routing_plugin, settings, args.backend)
        if args.managed:
            registry.load('ingress', args.ingress_plugin, runtime='control')
            if not args.ingress_channel.isidentifier() or args.ingress_channel == 'rpc':
                raise ValueError('Native ingress channel must be an identifier other than rpc')
    prefix = "robot-" + uuid.uuid4().hex[:12]
    directory = ROOT / "outputs/runtime/control" / (time.strftime("%Y%m%d-%H%M%S-") + prefix)
    directory.mkdir(parents=True)
    telemetry = getattr(args, 'telemetry', False)
    configs = native_routing.configure(directory, prefix, settings) if native_routing else {}
    native_commands = {}
    native = None
    if native_device:
        native_commands = native_routing.commands(configs, prefix)
        device_commands = native_device.commands(configs, prefix)
        if set(native_commands) & set(device_commands):
            raise ValueError('Native component process names must be unique')
        native_commands.update(device_commands)
        if not native_commands or len(native_commands) > 64 or set(native_commands) & {'backend', 'observer', 'viewer'}:
            raise ValueError('Invalid native component process names/count')
        native = {'device': device_plugin, 'routing': args.routing_plugin, 'ingress': args.ingress_plugin,
                  'channel': args.ingress_channel, 'dependencies': list(native_commands)}
    (directory / 'runtime.json').write_text(json.dumps({
        'runtime_id': prefix, 'backend': args.backend, 'input': args.input,
        'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'supervisor_pid': os.getpid(), 'started_monotonic_ns': time.monotonic_ns(),
        'telemetry': telemetry, 'managed': args.managed, 'native': native,
        'plugins': {'robot': selected_robot,
                    'controller': args.controller_plugin, 'planner': args.planner_plugin}}, indent=2) + '\n')
    children = Children(directory)
    common = [sys.executable, "-m", "embodied.control"]
    config_flag = ["--config", str(Path(args.config).resolve())] if args.config else []
    if args.vr_config:
        config_flag += ["--vr-config", str(Path(args.vr_config).resolve())]
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    failure = None
    dependency_health = None
    retired = set()
    mode = '真机' if args.backend == 'real' else '仿真'
    print(f'robot · 正在启动{mode} · {settings.robot.name} · {settings.robot.control_hz} Hz', flush=True)
    print(f'日志目录：{directory}', flush=True)
    if args.backend == 'real':
        if args.managed:
            behavior = '使能电机，先保持当前位置和夹爪，随后接受所选输入控制'
        else:
            behavior = '使能电机，回 Home 并打开夹爪'
        print(f'真机行为：{behavior}。Ctrl+C 退出后进入阻尼，退出前请支撑机械臂。', flush=True)
    try:
        if telemetry:
            children.start("observer", [sys.executable, '-m', 'embodied.control.observer',
                           '--directory', str(directory)], cwd=ROOT)
            deadline = time.monotonic() + 10
            while not (directory / 'observer.ready').exists():
                if children.failed() or time.monotonic() > deadline:
                    raise RuntimeError('监看反馈服务启动失败；详情见日志目录中的 observer.log')
                if stopped:
                    return 130
                time.sleep(.025)
        children.start("backend", common + ["_worker", "--backend", args.backend, "--prefix", prefix,
                       "--directory", str(directory)] + config_flag
                       + (["--record-tuning"] if record_tuning else [])
                       + (["--telemetry"] if telemetry else [])
                       + (["--managed"] if args.managed else [])
                       + (["--robot-plugin", args.robot_plugin] if args.robot_plugin else [])
                       + ["--planner-plugin", args.planner_plugin]
                       + (["--controller-plugin", args.controller_plugin] if args.controller_plugin else []), cwd=ROOT)
        deadline = time.monotonic() + 15
        while not (directory / "backend.ready").exists():
            failure = children.failed()
            if failure or time.monotonic() > deadline:
                detail = ''
                try:
                    cause = json.loads((directory / 'report.json').read_text()).get('error')
                    if cause:
                        detail = f': {cause}'
                except (OSError, ValueError):
                    pass
                reason = f'{failure[0]} 退出，退出码 {failure[1]}' if failure else '等待反馈超时'
                raise RuntimeError(f'robot 启动失败（{reason}）{detail}；详情见日志目录中的 backend.log')
            if stopped:
                return 130
            time.sleep(.025)
        if native:
            native_routing.start(children, configs, prefix, lambda: stopped)
            for name, command in native_device.commands(configs, prefix).items():
                children.start(name, command, cwd=ROOT)
            if args.managed:
                dependency_health = native_routing.health(prefix, native['dependencies'])
                dependency_health.publish(children.processes)
                (directory / 'native-processes.json').write_text(json.dumps(
                    {name: children.processes[name].pid for name in native_commands}, indent=2))
        if not args.headless:
            children.start("viewer", common + ["_viewer", "--prefix", prefix] + config_flag, cwd=ROOT)
        (directory / 'runtime.ready').write_text('ready\n')
        print_ready(device_plugin, args.routing_plugin, settings.robot.gripper_max_width)
        started = time.monotonic()
        while not stopped and (args.duration <= 0 or time.monotonic() - started < args.duration):
            if dependency_health:
                dependency_health.publish(children.processes)
            failure = next(((name, process.returncode) for name, process in children.processes.items()
                            if name not in retired and process.poll() is not None), None)
            if failure:
                if args.managed and failure == ("backend", 0):
                    report = json.loads((directory / "report.json").read_text())
                    if report.get("managed_stop") and not report.get("error"):
                        break
                if failure[0] in ('viewer', 'observer') or failure[0] in native_commands:
                    print(component_exit_message(*failure), flush=True)
                    retired.add(failure[0])
                    failure = None
                    continue
                raise RuntimeError(f'控制进程 {failure[0]} 已退出（退出码 {failure[1]}）；详情见日志目录中的 {failure[0]}.log')
            time.sleep(.05)
    finally:
        active_error = sys.exc_info()[0] is not None
        children.stop()
        if dependency_health:
            dependency_health.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        report_path = directory / "report.json"
        report = None
        if report_path.exists():
            report = json.loads(report_path.read_text())
            if args.report:
                destination = Path(args.report)
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(json.dumps(report, indent=2))
            print(f"运行结束 · 实测 {report['achieved_hz']:.2f} Hz · {report['ticks']} 个控制周期 · "
                  f"{report['pose_commands']} 次位姿指令；报告：日志目录中的 report.json", flush=True)
            if report.get('tuning_recording'):
                recording = report['tuning_recording']
                complete = '完整' if recording['complete'] else '不完整'
                print(f"控制诊断记录 · 已写入 {recording['written']}/{recording['submitted']} · "
                      f"丢弃 {recording['dropped']} · {complete}；文件：{Path(recording['path']).name}", flush=True)
        if not active_error and (directory / "backend.ready").exists():
            validate_completion(children.processes["backend"].returncode, report)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Modular robot control runtime")
    sub = parser.add_subparsers(dest="command", required=True)
    launch = sub.add_parser("run")
    launch.add_argument("--backend", choices=("sim", "real"), default="sim")
    launch.add_argument("--input", choices=("vr", "demo", "idle"), default="vr")
    launch.add_argument("--managed", action="store_true", help="Enable plugin command gateway; starts in feedback hold")
    launch.add_argument("--robot-plugin")
    launch.add_argument("--input-plugin", "--vr-plugin", dest="input_plugin")
    launch.add_argument("--routing-plugin", default="arx.native")
    launch.add_argument("--ingress-plugin", default="arx.native")
    launch.add_argument("--ingress-channel", default="native")
    launch.add_argument("--controller-plugin")
    launch.add_argument("--planner-plugin", default="motion.default")
    launch.add_argument("--headless", action="store_true")
    launch.add_argument("--duration", type=float, default=0)
    launch.add_argument("--report")
    launch.add_argument("--config", help="robot profile (default: configs/robots/arx5/robot.json)")
    launch.add_argument("--vr-config", help="VR profile (default: configs/inputs/quest.json)")
    launch.add_argument("--record-tuning", action="store_true", help="record feedback/targets/gains (always enabled for real)")
    launch.add_argument("--telemetry", action="store_true", help="Publish RobotTickV3 and start the independent read-only observer")
    check = sub.add_parser("doctor")
    check.add_argument("--vr", action="store_true")
    check.add_argument("--config")
    check.add_argument("--vr-config")
    snap = sub.add_parser("snapshot")
    snap.add_argument("--output", default=str(ROOT / "outputs/preview.png"))
    snap.add_argument("--config")
    for hidden in ("_worker", "_viewer"):
        child = sub.add_parser(hidden)
        child.add_argument("--prefix", required=True)
        if hidden in ("_worker", "_viewer"):
            child.add_argument("--config")
            child.add_argument("--vr-config")
        if hidden == "_worker":
            child.add_argument("--backend", required=True, choices=("sim", "real"))
            child.add_argument("--record-tuning", action="store_true")
            child.add_argument("--telemetry", action="store_true")
            child.add_argument("--managed", action="store_true")
            child.add_argument("--robot-plugin")
            child.add_argument("--controller-plugin")
            child.add_argument("--planner-plugin", default="motion.default")
        if hidden == "_worker":
            child.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "doctor":
            return doctor(args.config, args.vr, args.vr_config)
        if args.command == "run":
            return run(args)
        if args.command == "snapshot":
            os.environ.setdefault("MUJOCO_GL", "egl")
            profile = load_settings(args.config).robot
            viewer = Registry().create('viewer', profile.viewer_plugin, runtime='control')
            print(viewer.snapshot(Path(args.output), args.config))
        elif args.command == "_worker":
            from embodied.control.worker import run as worker
            worker(args.backend, args.prefix, args.config, args.directory, record_tuning=args.record_tuning, vr_config_path=args.vr_config, telemetry=args.telemetry, managed=args.managed, robot_plugin=args.robot_plugin, planner_plugin=args.planner_plugin, controller_plugin=args.controller_plugin)
        elif args.command == "_viewer":
            profile = load_settings(args.config).robot
            viewer = Registry().create('viewer', profile.viewer_plugin, runtime='control')
            viewer.run(args.prefix, args.config)
        return 0
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
