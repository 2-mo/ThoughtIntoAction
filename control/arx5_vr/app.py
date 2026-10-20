"""Launch the existing VR pipeline with an explicit simulation or native SDK backend."""

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import uuid
from .runtime import Children
from .settings import ROOT, load_settings


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
    if report.get('backend') == 'real':
        if not report.get('stop_requested', report.get('damping_requested')):
            raise RuntimeError('Robot stop request did not complete')
    elif not report.get("park_completed"):
        raise RuntimeError("Parking did not complete")


def doctor(config_path=None, require_vr=False, vr_config_path=None):
    settings = load_settings(config_path, vr_config_path)
    checks = {"python": sys.version.split()[0], "platform": platform.platform(), "control_hz": 250}
    errors = []
    hardware_package = 'arx5-interface'
    for package in ("numpy", "scipy", "mujoco", "iceoryx2", hardware_package):
        try:
            checks[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            checks[package] = "missing"
            if package != hardware_package:
                errors.append(f"Missing {package}")
    if checks.get("iceoryx2") != "0.9.1":
        errors.append("iceoryx2 must match native binaries: 0.9.1")
    for binary in ("rollio-device-xrobot", "rollio-teleop-router"):
        path = ROOT / "native/bin" / binary
        try:
            result = subprocess.run([str(path), "--version"], capture_output=True, text=True, timeout=5)
            checks[binary] = result.stdout.strip()
            if result.returncode:
                errors.append(result.stderr.strip())
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(str(exc))
    checks["vr_sdk_exists"] = Path(settings.sdk_library).is_file()
    if require_vr and not checks["vr_sdk_exists"]:
        errors.append(f"XRoboToolkit SDK missing: {settings.sdk_library}")
    try:
        from .model import X5Model
        model = X5Model(settings.robot)
        checks["model"] = {"nq": model.model.nq, "tcp": model.tcp_link,
                           "calibration_status": model.calibration_status,
                           "urdf": str(model.urdf_path)}
    except Exception as exc:
        errors.append(f"Model cannot load: {exc}")
    checks["errors"] = errors
    print(json.dumps(checks, ensure_ascii=False, indent=2))
    return int(bool(errors))


def run(args):
    validate_run(args.backend, args.input, args.headless, getattr(args, "managed", False))
    record_tuning = args.record_tuning
    settings = load_settings(args.config, args.vr_config)
    if args.backend == 'real':
        from .real import preflight
        from .model import X5Model
        checks = preflight(settings.robot)
        if not checks['ready']:
            raise ValueError(f'Real preflight failed: {checks}')
        X5Model(settings.robot)  # Reject mismatched models before starting children.
        record_tuning = True
    native_device = native_routing = None
    device_plugin = args.input_plugin
    if args.managed:
        sys.path.insert(0, str(ROOT.parent / "src" if (ROOT.parent / "src/embodied/contracts").is_dir() else ROOT / "shared"))
        from embodied.contracts.registry import Registry
        registry = Registry(ROOT / "config/components.json")
        # Old --input presets remain shortcuts; profiles pass explicit connections.
        device_plugin = device_plugin or {'vr': 'arx.quest', 'demo': 'arx.demo'}.get(args.input)
        if device_plugin:
            from adapters.composition import native_components
            native_device, native_routing = native_components(registry, device_plugin, args.routing_plugin, settings, args.backend)
            registry.load('ingress', args.ingress_plugin, runtime='control')
            if not args.ingress_channel.isidentifier() or args.ingress_channel == 'rpc':
                raise ValueError('Native ingress channel must be an identifier other than rpc')
    elif args.input_plugin:
        raise ValueError('Input plugin selection requires --managed')
    elif args.input == 'vr' and not Path(settings.sdk_library).is_file():
        raise ValueError(f"XRoboToolkit PC Service SDK not found: {settings.sdk_library}")
    prefix = "arx5-" + uuid.uuid4().hex[:12]
    directory = ROOT / "run" / (time.strftime("%Y%m%d-%H%M%S-") + prefix)
    directory.mkdir(parents=True)
    telemetry = getattr(args, 'telemetry', False)
    from .native_config import write_native_configs
    configs = write_native_configs(directory, prefix, settings.robot.gripper_max_width, settings.sdk_library,
                                   settings.vr.base_yaw_deg, settings.vr.grip_threshold, settings.vr.position_scale)
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
        'plugins': {'robot': args.robot_plugin or 'arx.' + args.backend,
                    'controller': args.controller_plugin, 'planner': args.planner_plugin}}, indent=2) + '\n')
    children = Children(directory)
    common = [sys.executable, "-m", "arx5_vr"]
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
    print(f"{args.backend.upper()} | X5 {settings.robot.variant} | 250 Hz | input={args.input}")
    print(f"Runtime logs: {directory}", flush=True)
    if args.backend == 'real':
        stop_action = 'SDK damping'
        print(f'Real startup connects and moves to Home {list(settings.robot.park_q)} rad; '
              f'A+B returns Home; Ctrl+C requests {stop_action} without a home move.', flush=True)
    try:
        if telemetry:
            children.start("observer", [sys.executable, '-m', 'adapters.observer',
                           '--directory', str(directory)], cwd=ROOT)
            deadline = time.monotonic() + 10
            while not (directory / 'observer.ready').exists():
                if children.failed() or time.monotonic() > deadline:
                    raise RuntimeError('Observer startup failed; see observer.log')
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
                raise RuntimeError(f"Backend startup failed ({failure}){detail}; see {directory / 'backend.log'}")
            if stopped:
                return 130
            time.sleep(.025)
        if args.managed:
            if native:
                native_routing.start(children, configs, prefix, lambda: stopped)
                for name, command in native_device.commands(configs, prefix).items():
                    children.start(name, command, cwd=ROOT)
            if native:
                from adapters.native_health import DependencyPublisher
                dependency_health = DependencyPublisher(prefix, native['dependencies'])
                dependency_health.publish(children.processes)
                (directory / 'native-processes.json').write_text(json.dumps(
                    {name: children.processes[name].pid for name in native_commands}, indent=2))
        else:
            router = str(ROOT / "native/bin/rollio-teleop-router")
            for route in (name for name in ('arm', 'gripper') if name in configs and args.input != 'idle'):
                children.start(route, [router, "run", "--config", str(configs[route])], cwd=ROOT)
            if args.input != 'idle':
                children.start('coordinator', [router, 'coordinate', '--config', str(configs['coordinator'])], cwd=ROOT)
            if args.input == 'vr':
                children.start('input', [str(ROOT / 'native/bin/rollio-device-xrobot'), 'run', '--config', str(configs['xrobot'])], cwd=ROOT)
            elif args.input == 'demo':
                children.start('input', common + ['_demo', '--prefix', prefix], cwd=ROOT)
        if not args.headless:
            children.start("viewer", common + ["_viewer", "--prefix", prefix] + config_flag, cwd=ROOT)
        (directory / 'runtime.ready').write_text('ready\n')
        trigger = 'gripper' if settings.robot.gripper_max_width > 0 else 'unused (no tool)'
        if args.routing_plugin == 'arx.workbench':
            print(f'Workbench: right controller as upstream (A: hold 150 ms to change mode | Grip: clutch | Trigger: {trigger} | A+B: Home); left controller: collection.')
        else:
            print(f"A: hold 150 ms to change mode | Grip: clutch | Trigger: {trigger} | A+B: Home")
        if args.input == "demo":
            print("Demo: automatic input starts following after about 4.5 s; never used on real hardware.")
        elif args.input == "vr":
            print("Waiting for XRoboToolkit right-controller frames. Start the headset application and PC Service.")
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
                    message = 'dependent ingress will withdraw control' if failure[0] in native_commands else 'control continues'
                    print(f'Component {failure[0]} exited ({failure[1]}); {message}.', flush=True)
                    retired.add(failure[0])
                    failure = None
                    continue
                raise RuntimeError(f"Process {failure[0]} exited ({failure[1]}); see {directory / (failure[0] + '.log')}")
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
            print(f"Measured {report['achieved_hz']:.2f} Hz; {report['ticks']} ticks; "
                  f"pose commands {report['pose_commands']}; report: {report_path}")
            if report.get('tuning_recording'):
                recording = report['tuning_recording']
                print(f"Tuning records: {recording['written']}/{recording['submitted']}; "
                      f"dropped={recording['dropped']}; complete={recording['complete']}; {recording['path']}")
        if not active_error and (directory / "backend.ready").exists():
            validate_completion(children.processes["backend"].returncode, report)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="ARX X5 VR control and MuJoCo, 250 Hz")
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
    launch.add_argument("--planner-plugin", default="arx.basic")
    launch.add_argument("--headless", action="store_true")
    launch.add_argument("--duration", type=float, default=0)
    launch.add_argument("--report")
    launch.add_argument("--config", help="robot profile (default: config/robot.json)")
    launch.add_argument("--vr-config", help="VR profile (default: config/vr.json)")
    launch.add_argument("--record-tuning", action="store_true", help="record feedback/targets/gains (always enabled for real)")
    launch.add_argument("--telemetry", action="store_true", help="Publish RobotTickV1 and start the independent read-only observer")
    check = sub.add_parser("doctor")
    check.add_argument("--vr", action="store_true")
    check.add_argument("--config")
    check.add_argument("--vr-config")
    snap = sub.add_parser("snapshot")
    snap.add_argument("--output", default=str(ROOT / "preview.png"))
    snap.add_argument("--config")
    for hidden in ("_worker", "_viewer", "_demo"):
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
            child.add_argument("--planner-plugin", default="arx.basic")
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
            from .viewer import snapshot
            print(snapshot(Path(args.output), args.config))
        elif args.command == "_worker":
            from .worker import run as worker
            worker(args.backend, args.prefix, args.config, args.directory, record_tuning=args.record_tuning, vr_config_path=args.vr_config, telemetry=args.telemetry, managed=args.managed, robot_plugin=args.robot_plugin, planner_plugin=args.planner_plugin, controller_plugin=args.controller_plugin)
        elif args.command == "_viewer":
            from .viewer import run as viewer
            viewer(args.prefix, args.config)
        else:
            from .demo import run as demo
            demo(args.prefix)
        return 0
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
