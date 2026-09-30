"""One 250 Hz robot process; rendering and XR communication run elsewhere."""

import json
import fcntl
import os
from dataclasses import asdict
from pathlib import Path
import signal
import time
import numpy as np
from .ipc import RobotBridge
from .runtime import RateStats, PeriodicClock
from .settings import load_settings
from .types import ControlCommand, Mode
from .recording import TuningRecorder, tuning_row, write_json, write_parameter_snapshot


def check_feedback(state, now):
    if state.fault:
        raise RuntimeError(state.fault)
    if not np.isfinite(state.received_at) or not 0 <= now - state.received_at < .1:
        raise RuntimeError("Robot feedback exceeded 100 ms or has an invalid timestamp")
    if not all(np.isfinite(value).all() for value in (state.q, state.dq, state.pose, state.gripper_width)):
        raise RuntimeError("Robot feedback contains non-finite values")


def run(backend_name: str, prefix: str, config_path: str | None, directory: Path, *, record_tuning=False, vr_config_path=None, telemetry=False, managed=False, robot_plugin=None, planner_plugin="arx.basic", controller_plugin=None):
    if backend_name not in ('sim', 'real'):
        raise ValueError('backend must be sim or real')
    settings = load_settings(config_path, vr_config_path)
    record_tuning = record_tuning or backend_name == 'real'
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    backend = bridge = initial = lock = None
    device_lease = None
    gateway = None
    ingresses = {}
    controller_options = {}
    plugin_factory = None
    if managed or robot_plugin or controller_plugin:
        import sys
        control_root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(control_root.parent / "src" if (control_root.parent / "src/component_api").is_dir() else control_root / "shared"))
        from component_api.registry import Registry
        registry = Registry(Path(__file__).resolve().parents[1] / "config/components.json")
        selected = robot_plugin or "arx." + backend_name
        plugin_spec = registry.spec("robot", selected, allow_hardware=backend_name == "real")
        if plugin_spec.hardware != (backend_name == "real"):
            raise ValueError("Robot plugin/backend hardware mismatch")
        plugin_factory = registry.load("robot", selected, runtime="control")
        if controller_plugin:
            controller_options["controller_factory"] = registry.load("controller", controller_plugin, runtime="control")
    clock, stats = PeriodicClock(250), RateStats(250)
    width_min = width_max = None
    max_motion = max_joint_delta = 0.0
    pose_commands = grip_commands = 0
    mode_changes = []
    previous_mode = None
    started = time.monotonic()
    error = None
    parked = False
    stop_requested = False
    park_error = None
    recorder = None
    ticks = None
    feedback_read_ns = 0
    recording_error = None
    parameter_path = directory / 'parameters.json'
    backend_parameter_path = directory / 'backend-parameters.json'

    def record(state, command, tick, phase, completed, step_error=None):
        nonlocal recording_error
        if recorder is not None and recording_error is None:
            try:
                recorder.submit(tuning_row(recorder.submitted, tick, phase, state, command,
                                           backend, completed, step_error))
            except Exception as exc:
                # Diagnostics must not interrupt motion or controlled parking.
                recording_error = f'{type(exc).__name__}: {exc}'

    def context():
        if gateway is None:
            return None
        engine = gateway.engine
        return {'source': engine.owner['source'] if engine.owner else 'idle', 'epoch': engine.epoch,
                'source_sequence': engine.source_sequence, 'phase': engine.phase,
                'operation_id': engine.operation['operation_id'] if engine.operation else ''}

    def step_and_record(state, command, tick, phase):
        try:
            if backend_name == 'real':
                backend.step(command, tick, cancel_requested=lambda: stopped,
                             feedback_validator=lambda state: check_feedback(state, time.monotonic()))
            else:
                backend.step(command, tick)
        except Exception as exc:
            if ticks is not None:
                ticks.publish(state, command, backend, tick, feedback_read_ns, phase, False, str(exc), context=context())
            record(state, command, tick, phase, False, str(exc))
            raise
        if ticks is not None:
            ticks.publish(state, command, backend, tick, feedback_read_ns, phase, True, context=context())
        record(state, command, tick, phase, True)

    def check_and_record_feedback(state, tick, phase):
        try:
            check_feedback(state, tick)
        except Exception as exc:
            record(state, None, tick, phase, False, str(exc))
            raise

    try:
        directory.mkdir(parents=True, exist_ok=True)
        write_parameter_snapshot(directory, settings, backend_name, config_path)
        if record_tuning:
            recorder = TuningRecorder(directory / 'tuning.jsonl')
        if telemetry:
            from adapters.ticks import TickPublisher
            identity_file = directory / 'runtime.json'
            source = json.loads(identity_file.read_text())['input'] if identity_file.exists() else 'quest'
            ticks = TickPublisher(prefix, 'quest' if source == 'vr' else source)
        if backend_name == 'real':
            from .real import RealBackend as HardwareBackend, preflight
            lock_key = f'bench-{settings.robot.can_interface}'
            checks = preflight(settings.robot)
            if not checks['ready']:
                raise RuntimeError(f'Real preflight failed: {checks}')
            from adapters.device_lease import DeviceLease
            device_lease = DeviceLease(settings.robot.can_interface)
            # Share ownership with the existing bench; never connect two local
            # controllers to the same interface from these entry points.
            lock = (directory.parent / f'.{lock_key}.lock').open('a+')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError('Robot interface is already owned by a VR/bench process') from exc
            lock.seek(0)
            lock.truncate()
            interface = settings.robot.can_interface
            lock.write(json.dumps({'pid': os.getpid(), 'interface': interface,
                                   'directory': str(directory), 'entry': 'vr'}) + '\n')
            lock.flush()
            backend = (plugin_factory or HardwareBackend)(settings.robot, cancel_requested=lambda: stopped, **controller_options)
        else:
            from .sim import MujocoBackend
            backend = (plugin_factory or MujocoBackend)(settings.robot, **controller_options)
        if hasattr(backend, 'runtime_parameters'):
            write_json(backend_parameter_path, backend.runtime_parameters())
        if managed:
            from adapters.gateway import Gateway
            native = json.loads((directory / 'runtime.json').read_text()).get('native')
            if native:
                ingresses[native['channel']] = registry.create('ingress', native['ingress'], runtime='control',
                    prefix=prefix, dependencies=native['dependencies'], gripper=settings.robot.gripper_max_width > 0)
            gateway = Gateway(directory, prefix, settings.robot,
                              registry.load("planner", planner_plugin, runtime="control"), ingresses)
        bridge = RobotBridge(prefix, commands=not managed)
        now = time.monotonic()
        state = backend.read_state(now)
        check_and_record_feedback(state, now, 'startup')
        initial = state
        prior_q = initial.q.copy()
        width_min = width_max = initial.gripper_width
        clock = PeriodicClock(250)
        while not stopped:
            tick = time.monotonic()
            state = backend.read_state(tick)
            feedback_read_ns = time.monotonic_ns()
            check_and_record_feedback(state, tick, 'control')
            if bridge is not None:
                bridge.publish_state(state, tick)
            if previous_mode != (state.mode, state.gripper_mode):
                previous_mode = state.mode, state.gripper_mode
                mode_changes.append({"time_s": tick - started, "arm": state.mode.value,
                                     "gripper": (state.gripper_mode or state.mode).value})
                print(f"mode arm={state.mode.value} gripper={(state.gripper_mode or state.mode).value}", flush=True)
            command = bridge.poll_commands(tick) if not managed else None
            if gateway is not None:
                command = gateway.tick(state, tick)
                if gateway.engine.stopped:
                    stopped = True
                    break
            if backend_name == 'real' and stopped:
                break
            pose_commands += int(command.pose_target is not None)
            grip_commands += int(command.gripper_target is not None)
            step_and_record(state, command, tick, 'control')
            width_min, width_max = min(width_min, state.gripper_width), max(width_max, state.gripper_width)
            max_motion = max(max_motion, float(np.linalg.norm(state.pose[:3] - initial.pose[:3])))
            max_joint_delta = max(max_joint_delta, float(np.max(np.abs(state.q - prior_q))))
            prior_q = state.q.copy()
            stats.observe(tick, time.monotonic() - tick)
            if stats.ticks == 1:
                (directory / "backend.ready").write_text("ready\n")
            clock.wait()
    except InterruptedError as exc:
        if not stopped:
            error = str(exc)
    except Exception as exc:
        error = str(exc)
    finally:
        # Real exit never introduces a new Home trajectory. A+B owns that
        # action. A signal/fault latches targets, then releases the SDK in damping.
        if backend_name == 'real' and backend is not None:
            try:
                backend.request_stop()
                stop_requested = True
            except Exception as exc:
                error = error or f'Stop request failed: {exc}'
        if backend_name == 'sim' and backend is not None and initial is not None:
            try:
                clock = PeriodicClock(250)
                state = backend.read_state(time.monotonic())
                home_duration = max(1., float(np.max(np.abs(state.q - settings.robot.park_q))) + .5)
                end = time.monotonic() + max(4., home_duration + 1.)
                while time.monotonic() < end:
                    now = time.monotonic()
                    state = backend.read_state(now)
                    feedback_read_ns = time.monotonic_ns()
                    check_and_record_feedback(state, now, 'shutdown-parking')
                    step_and_record(state, ControlCommand(Mode.DISABLED, gripper_mode=Mode.DISABLED),
                                    now, 'shutdown-parking')
                    state = backend.read_state(now)
                    check_and_record_feedback(state, now, 'shutdown-parking-feedback')
                    if bridge is not None:
                        bridge.publish_state(state, now)
                    clock.wait()
                park_error = float(np.max(np.abs(state.q - settings.robot.park_q)))
                if park_error > .05 or abs(state.gripper_width - settings.robot.gripper_max_width) > .005:
                    raise RuntimeError(f"Parking did not reach target: joint error {park_error:.4f} rad")
                parked = True
            except Exception as exc:
                error = error or f"Parking failed: {exc}"
                print(f"Parking failed: {exc}", flush=True)
        for resource in (gateway, *([] if gateway else ingresses.values()), backend, bridge):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    error = error or f"Cleanup failed: {exc}"
        if lock is not None:
            lock.close()
        if device_lease is not None:
            device_lease.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        recording_status = recorder.close() if recorder is not None else None
        if recording_status is not None:
            if recording_error is not None:
                recording_status.update(error=recording_error, complete=False)
            if not recording_status['complete']:
                error = error or f'Tuning recording incomplete: {recording_status}'
        if ticks is not None:
            ticks.finish(error)
            ticks.close()
    report = stats.report() | {
        "backend": backend_name, "variant": settings.robot.variant,
        "pose_commands": pose_commands, "gripper_commands": grip_commands,
        "max_cartesian_motion_m": max_motion, "max_measured_joint_step_rad": max_joint_delta,
        "gripper_min_m": width_min, "gripper_max_m": width_max,
        "mode_changes": mode_changes, "park_completed": parked,
        "shutdown_policy": 'sdk-damping' if backend_name == 'real' else 'home-and-hold',
        "stop_requested": stop_requested,
        "managed_stop": bool(gateway and gateway.engine.stopped),
        "damping_requested": stop_requested,
        "park_joint_error_rad": park_error, "error": error,
        "ik_rejections": getattr(getattr(backend, 'controller', None), 'ik_rejections', 0),
        "ik_partial_solutions": getattr(getattr(backend, 'controller', None), 'ik_partial_solutions', 0),
        "effective_robot_config": asdict(settings.robot),
        "parameters_path": str(parameter_path),
        "backend_parameters_path": str(backend_parameter_path) if backend_parameter_path.is_file() else None,
        "tuning_recording": recording_status,
        "tick_telemetry": ticks.status() if ticks is not None else None,
    }
    (directory / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    if error:
        raise RuntimeError(error)
