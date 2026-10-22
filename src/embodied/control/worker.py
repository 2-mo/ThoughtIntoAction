"""One profile-paced robot process; rendering and XR communication run elsewhere."""

import json
from contextlib import ExitStack
import os
from dataclasses import asdict
from pathlib import Path
import signal
import time
import numpy as np
from embodied.control.ipc import RobotBridge
from embodied.control.runtime import RateStats, PeriodicClock
from embodied.control.settings import load_settings
from embodied.contracts.control import ControlCommand, Mode
from embodied.control.recording import TuningRecorder, tuning_row, write_json, write_parameter_snapshot


def check_feedback(state, now):
    if state.fault:
        raise RuntimeError(state.fault)
    if not np.isfinite(state.received_at) or not 0 <= now - state.received_at < .1:
        raise RuntimeError("Robot feedback exceeded 100 ms or has an invalid timestamp")
    if not all(np.isfinite(value).all() for value in (state.q, state.dq, state.pose, state.gripper_width)):
        raise RuntimeError("Robot feedback contains non-finite values")


def run(backend_name: str, prefix: str, config_path: str | None, directory: Path, *, record_tuning=False, vr_config_path=None, telemetry=False, managed=False, robot_plugin=None, planner_plugin="motion.default", controller_plugin=None):
    if backend_name not in ('sim', 'real'):
        raise ValueError('backend must be sim or real')
    settings = load_settings(config_path, vr_config_path)
    record_tuning = record_tuning or backend_name == 'real'
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    backend = bridge = initial = lock = None
    resources = ExitStack()
    gateway = None
    ingresses = {}
    controller_options = {}
    from embodied.contracts.registry import Registry
    registry = Registry()
    selected = robot_plugin or (settings.robot.hardware_plugin if backend_name == "real" else settings.robot.simulation_plugin)
    spec = registry.spec("robot", selected, allow_hardware=backend_name == "real")
    if spec.hardware != (backend_name == "real"):
        raise ValueError("Robot plugin/backend hardware mismatch")
    driver = registry.create("robot", selected, runtime="control")
    if controller_plugin:
        controller_options["controller_factory"] = registry.load("controller", controller_plugin, runtime="control")
    clock, stats = PeriodicClock(settings.robot.control_hz), RateStats(settings.robot.control_hz)
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
    shutdown_policy = None
    shutdown_completed = False
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
        nonlocal feedback_read_ns
        feedback_read_ns = time.monotonic_ns()
        try:
            check_feedback(state, tick)
            count = len(settings.robot.park_q)
            if np.shape(state.q) != (count,) or np.shape(state.dq) != (count,):
                raise RuntimeError(f"Robot feedback must contain {count} joints")
        except Exception as exc:
            record(state, None, tick, phase, False, str(exc))
            raise

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        directory.mkdir(parents=True, exist_ok=True)
        write_parameter_snapshot(directory, settings, backend_name, config_path, driver=driver)
        if record_tuning:
            recorder = TuningRecorder(directory / 'tuning.jsonl')
        if telemetry:
            from embodied.control.ticks import TickPublisher
            identity_file = directory / 'runtime.json'
            source = json.loads(identity_file.read_text())['input'] if identity_file.exists() else 'quest'
            ticks = TickPublisher(prefix, 'quest' if source == 'vr' else source, joint_count=len(settings.robot.park_q))
        checks = driver.preflight(settings.robot)
        if not checks['ready']:
            raise RuntimeError(f'Robot preflight failed: {checks}')
        from embodied.contracts.device_lease import DeviceLease
        for resource in driver.resources(settings.robot):
            lease = DeviceLease(resource)
            resources.callback(lease.close)
        backend = driver.open(settings.robot, cancel_requested=lambda: stopped, **controller_options)
        backend_parameters = backend.runtime_parameters() if hasattr(backend, 'runtime_parameters') else None
        if backend_parameters is not None:
            write_json(backend_parameter_path, backend_parameters)
        from embodied.control.motion_limits import MotionLimits
        capabilities = driver.capabilities(settings.robot, backend_parameters)
        motion_limits = MotionLimits.load(settings.robot, capabilities=capabilities, backend=backend_name)
        native_step_limits = motion_limits.step_limits()
        if managed:
            from embodied.control.gateway import Gateway
            native = json.loads((directory / 'runtime.json').read_text()).get('native')
            if native:
                ingresses[native['channel']] = registry.create('ingress', native['ingress'], runtime='control',
                    prefix=prefix, dependencies=native['dependencies'], gripper=settings.robot.gripper_max_width > 0)
            gateway = Gateway(directory, prefix, settings.robot,
                              registry.load("planner", planner_plugin, runtime="control"), ingresses,
                              motion_limits=motion_limits, capabilities=capabilities,
                              simulation=backend if backend_name == 'sim' and all(callable(getattr(backend, name, None))
                                  for name in ('simulation_snapshot', 'simulation_restore', 'simulation_reset')) else None)
        bridge = RobotBridge(prefix, commands=not managed)
        now = time.monotonic()
        state = backend.read_state(now)
        check_and_record_feedback(state, now, 'startup')
        initial = state
        prior_q = initial.q.copy()
        width_min = width_max = initial.gripper_width
        clock = PeriodicClock(settings.robot.control_hz)
        while not stopped:
            tick = time.monotonic()
            state = backend.read_state(tick)
            feedback_read_ns = time.monotonic_ns()
            check_and_record_feedback(state, tick, 'control')
            if previous_mode != (state.mode, state.gripper_mode):
                previous_mode = state.mode, state.gripper_mode
                mode_changes.append({"time_s": tick - started, "arm": state.mode.value,
                                     "gripper": (state.gripper_mode or state.mode).value})
                print(f"mode arm={state.mode.value} gripper={(state.gripper_mode or state.mode).value}", flush=True)
            command = bridge.poll_commands(tick) if not managed else None
            if command is not None:
                command.joint_step_limit = native_step_limits
            if gateway is not None:
                command = gateway.tick(state, tick)
                if state is not gateway.feedback:
                    state = gateway.feedback
                    check_and_record_feedback(state, time.monotonic(), 'simulation-state-change')
                if gateway.engine.stopped:
                    stopped = True
                    break
            if bridge is not None:
                bridge.publish_state(state, time.monotonic())
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
        if backend is not None:
            try:
                result = driver.shutdown(settings.robot, backend, initial=initial,
                    publish=lambda state, now: bridge.publish_state(state, now) if bridge else None,
                    step=step_and_record, check=check_and_record_feedback)
                parked = result.get('park_completed', False)
                park_error = result.get('park_joint_error_rad')
                stop_requested = result.get('stop_requested', False)
                shutdown_policy = result['policy']
                shutdown_completed = True
            except Exception as exc:
                error = error or f"Device shutdown failed: {exc}"
        for resource in (gateway, *([] if gateway else ingresses.values()), backend, bridge):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    error = error or f"Cleanup failed: {exc}"
        resources.close()
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
        "backend": backend_name, "robot": settings.robot.name,
        "pose_commands": pose_commands, "gripper_commands": grip_commands,
        "max_cartesian_motion_m": max_motion, "max_measured_joint_step_rad": max_joint_delta,
        "gripper_min_m": width_min, "gripper_max_m": width_max,
        "mode_changes": mode_changes, "park_completed": parked,
        "shutdown_policy": shutdown_policy, "shutdown_completed": shutdown_completed,
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
