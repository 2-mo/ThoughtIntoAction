"""Standalone, read-only 3D arm monitor. No input source or arm backend is opened."""

import argparse
from ipaddress import IPv4Address
import signal
import os
from pathlib import Path
import threading

from embodied.devices.robots.arx5.pinocchio_model import DEFAULT_URDF, X5PinocchioModel
from embodied.core.telemetry import StateSubscriber, endpoint
from embodied.contracts.registry import Registry
from embodied.core.project import load_config, resolve_path


def main(argv=None):
    parser = argparse.ArgumentParser(prog="view-arm", description=__doc__)
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--eef", default="link6")
    parser.add_argument("--endpoint", help="Local state publisher endpoint; defaults to project telemetry config")
    parser.add_argument("--port", type=int, default=load_config()["viewer_service"]["port"], help="Browser HTTP port (default 18081; 0 chooses a free port)")
    parser.add_argument('--host', type=IPv4Address, default=IPv4Address('127.0.0.1'),
                        help='Explicit IPv4 address to bind (default: 127.0.0.1; wildcard addresses are not accepted)')
    parser.add_argument("--viewer", default="web", help="Display plugin; independent of controllers")
    parser.add_argument("--camera", action=argparse.BooleanOptionalAction, default=False, help="Ensure independent shared RGB service is running")
    parser.add_argument("--connections", action=argparse.BooleanOptionalAction, default=True, help="Read-only arm-service and USB RTT probes")
    parser.add_argument("--managed", action="store_true", help="Expose terminal start/stop control")
    parser.add_argument('--kernel', action='store_true', help='Observe existing unified kernels; never start or control a robot')
    parser.add_argument('--run', type=Path, help='Pin the read-only kernel monitor to outputs/runtime/control/<RUN>')
    from embodied.applications.monitor.mode import MODES, selected_mode
    parser.add_argument('--mode', choices=MODES, default=selected_mode())
    args = parser.parse_args(argv)
    if args.host.is_unspecified:
        parser.error('--host must be an explicit IPv4 address, not 0.0.0.0')
    args.host = str(args.host)
    if args.viewer != 'web' and args.host != '127.0.0.1':
        parser.error('--host is supported by the web viewer only')
    if args.kernel or args.run is not None:
        if args.viewer != 'web' or args.managed:
            parser.error('--kernel/--run supports the web viewer without --managed')
        from embodied.applications.monitor.kernel_view import serve
        return serve(args)
    address = endpoint(args.endpoint)
    robot = X5PinocchioModel(args.urdf, args.eef)
    scene = Registry().create("viewer", args.viewer, robot=robot, port=args.port,
                              **({'host': args.host} if args.viewer == 'web' else {}))
    previous = signal.getsignal(signal.SIGTERM)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    from embodied.applications.monitor.connections import LegacyGripperFeedback
    legacy_gripper = LegacyGripperFeedback(resolve_path(load_config()["arm_service"]["socket"]))
    subscriber = None
    control_server = None
    from embodied.applications.monitor.arx_preview import Simulation
    from embodied.core.project import PROJECT_ROOT
    simulation = Simulation()
    simulation_model = None
    mode_lock = threading.RLock()
    current_mode = 'real'
    def select_mode(mode):
        nonlocal current_mode, simulation_model
        if mode not in MODES:
            raise ValueError('无效工作台模式')
        with mode_lock:
            if mode == current_mode:
                return
            if mode == 'simulation':
                if simulation_model is None:
                    simulation_model = X5PinocchioModel(PROJECT_ROOT / 'assets/robots/x5/runtime/x5_2025.urdf', 'tcp_link')
                scene.set_model(simulation_model, simulation)
                simulation.start(scene)
            else:
                simulation.command('pause')
                scene.set_model(robot)
            current_mode = mode
    quit_event = threading.Event()
    try:
        select_mode(args.mode)
        if args.managed:
            from embodied.core.local_server import LocalServer
            from embodied.applications.monitor.control import socket_path
            def dispatch(payload):
                command = payload.get("command")
                if command == "stop":
                    quit_event.set()
                elif command == 'select-mode':
                    select_mode(payload.get('mode'))
                elif command in ('sim-home', 'sim-pregrasp', 'sim-record', 'sim-save'):
                    if current_mode != 'simulation' or simulation.directory is None:
                        raise ValueError('MuJoCo 尚未就绪')
                    if command == 'sim-record':
                        simulation.actions.start_recording()
                    elif command == 'sim-save':
                        simulation.actions.save()
                    else:
                        simulation.actions.move(home=command == 'sim-home')
                elif command in ('sim-resume', 'sim-pause', 'demo-restart', 'demo-stop'):
                    if current_mode != 'simulation':
                        raise ValueError('当前不是仿真模式')
                    simulation.command('resume' if command in ('sim-resume', 'demo-restart') else 'pause')
                elif command != "status":
                    raise ValueError("Only viewer status/stop are supported")
                return {"url": scene.url, "pid": os.getpid(), "mode": current_mode,
                        "mode_protocol": 2, "simulation": simulation.status() if current_mode == 'simulation' else None}
            control_server = LocalServer(socket_path(), dispatch)
        if args.connections and hasattr(scene, "connection_probe"):
            from embodied.applications.monitor.connections import ConnectionProbe
            scene.connection_probe = ConnectionProbe(resolve_path(load_config()["arm_service"]["socket"]))
        if args.camera:
            from embodied.devices.sensors.client import ensure_service
            try:
                ensure_service()
            except (OSError, RuntimeError) as exc:
                print(f"Camera service: {exc}", flush=True)
        subscriber = StateSubscriber(address)
        print(f"Watching robot state: {address}; waiting for arm-service", flush=True)
        while not quit_event.is_set():
            try:
                packet = subscriber.receive()
                if packet is not None and current_mode == 'real':
                    packet["gripper"] = legacy_gripper.read(packet)
                    packet["actual_torque"] = legacy_gripper.read_torque(packet)
                    packet["gripper_actual_torque"] = legacy_gripper.read_gripper_torque(packet)
                    with mode_lock:
                        if current_mode == 'real':
                            scene.accept(packet)
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                scene.set_stream_error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        simulation.close()
        if subscriber is not None:
            subscriber.close()
        if getattr(scene, "connection_probe", None):
            scene.connection_probe.close()
        scene.close()
        if control_server:
            control_server.close()
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    main()
