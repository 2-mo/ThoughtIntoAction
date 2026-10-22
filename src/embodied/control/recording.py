"""Effective parameter snapshots and bounded, asynchronous tuning telemetry."""

from dataclasses import asdict
from datetime import datetime, timezone
from enum import Enum
import hashlib
import inspect
import importlib.metadata
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time

import numpy as np

from embodied.control.settings import ROOT


def json_value(value):
    """Detach arrays; represent unavailable/non-finite diagnostics as JSON null."""
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(json_value(value), ensure_ascii=False, indent=2,
                                   allow_nan=False) + '\n', encoding='utf-8')


def _version(package):
    try:
        return importlib.metadata.version(package)
    except importlib.metadata.PackageNotFoundError:
        return None


def write_parameter_snapshot(directory, settings, backend_name, config_path=None, *, driver=None):
    """Save resolved settings/model/source identity before hardware construction."""
    directory = Path(directory)
    if driver is None:
        from embodied.contracts.registry import Registry
        selected = settings.robot.hardware_plugin if backend_name == 'real' else settings.robot.simulation_plugin
        driver = Registry().create('robot', selected, runtime='control')
    config = Path(settings.robot_config_path)
    native_configs = {path.name: path.read_text() for path in sorted(directory.glob('*.toml'))}
    input_config = settings.vr if native_configs else None
    sdk_library = Path(input_config.sdk_library) if input_config else None
    implementation = Path(inspect.getfile(type(driver))).parent
    artifacts = [*sorted((ROOT / 'src/embodied/control').rglob('*.py')),
                 *sorted(implementation.rglob('*.py')), ROOT / 'configs/plugins.json',
                 ROOT / 'configs/control/motion.json']
    shared = ROOT / 'src'
    shared_artifacts = [shared / 'embodied/__init__.py', *[path for module in ('contracts', 'motion')
                        for path in (shared / 'embodied' / module).rglob('*')
                        if path.is_file() and path.suffix in ('.py', '.json')]]
    snapshot = {
        'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'backend': backend_name, 'python': sys.version, 'config_path': str(config.resolve()),
        'effective_robot_config': asdict(settings.robot),
        'effective_input_config': asdict(input_config) if input_config else None,
        'input_config_path': settings.vr_config_path if input_config else None,
        'sdk_library': str(sdk_library) if sdk_library else None,
        'sdk_library_sha256': hashlib.sha256(sdk_library.read_bytes()).hexdigest() if sdk_library and sdk_library.is_file() else None,
        'packages': {name: _version(name) for name in
                     ('numpy', 'scipy', 'ruckig', 'iceoryx2')},
        'source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in artifacts if path.is_file()},
        'shared_source_sha256': {str(path.relative_to(shared)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in shared_artifacts},
        'native_configs': native_configs,
        'planning_limits_config': json.loads((ROOT / 'configs/control/motion.json').read_text()),
        **driver.snapshot(settings.robot),
        'recording': {'sample_rate_hz': settings.robot.control_hz, 'queue_capacity': 2048,
                      'units': {'q': 'rad', 'dq': 'rad/s', 'pose': 'm + quaternion xyzw',
                                'gripper': 'm', 'torque': 'N m', 'time': 's'},
                      'timing': 'feedback is sampled before step; command and applied gains belong to that step',
                      'missing': 'null means unavailable or non-finite; see IK status and error',
                      'runtime_parameters_file': 'backend-parameters.json',
                      'hardware_readback': False},
    }
    path = directory / 'parameters.json'
    write_json(path, snapshot)
    return path


class TuningRecorder:
    """One bounded queue; producer never waits for disk. Rows transfer ownership."""

    def __init__(self, path, capacity=2048):
        if capacity < 1:
            raise ValueError('recording capacity must be positive')
        self.path = Path(path)
        self._queue = queue.Queue(maxsize=capacity)
        # Fail before backend construction if output cannot be opened.
        self._file = self.path.open('x', encoding='utf-8')
        self._stop = threading.Event()
        self._accepting = True
        self.submitted = self.written = self.dropped = 0
        self.error = None
        self._thread = threading.Thread(target=self._run, name='tuning-writer', daemon=True)
        self._thread.start()

    def submit(self, row):
        if not self._accepting:
            raise RuntimeError('tuning recorder is closed')
        self.submitted += 1
        if self.error is not None:
            self.dropped += 1
            return
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def _write_row(self, row):
        self._file.write(json.dumps(row, ensure_ascii=False, allow_nan=False,
                                    separators=(',', ':')) + '\n')

    def _run(self):
        flushed_at = time.monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    row = self._queue.get(timeout=.05)
                except queue.Empty:
                    row = None
                if row is not None:
                    self._write_row(row)
                    self.written += 1
                now = time.monotonic()
                if now - flushed_at >= 1.:
                    self._file.flush()
                    flushed_at = now
            self._file.flush()
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'
        finally:
            try:
                self._file.close()
            except Exception as exc:
                self.error = self.error or f'{type(exc).__name__}: {exc}'

    def status(self):
        closed = not self._thread.is_alive()
        return {'path': str(self.path), 'submitted': self.submitted, 'written': self.written,
                'dropped': self.dropped, 'unwritten': self.submitted - self.written,
                'queue_capacity': self._queue.maxsize, 'closed': closed, 'error': self.error,
                'complete': closed and not self.error and self.submitted == self.written}

    def close(self, timeout=2.):
        self._accepting = False
        self._stop.set()
        self._thread.join(timeout)
        return self.status()


def tuning_row(sequence, tick, phase, state, command, backend, step_completed, error=None):
    """Record pre-step feedback and the request made during this tick."""
    control = backend.controller
    row = {
        'schema_version': 1, 'sequence': sequence, 'monotonic_s': tick, 'phase': phase,
        'feedback_received_at_s': state.received_at, 'feedback_age_s': tick - state.received_at,
        'q': state.q, 'dq': state.dq, 'pose': state.pose, 'gripper_width': state.gripper_width,
        'feedback_mode': state.mode, 'feedback_gripper_mode': state.gripper_mode or state.mode,
        'feedback_fault': state.fault,
        'command_mode': command.mode if command else None,
        'command_gripper_mode': (command.gripper_mode or command.mode) if command else None,
        'command_pose': command.pose_target if command else None,
        'command_gripper_target': command.gripper_target if command else None,
        'command_hold': command.hold if command else None,
        'command_engaged': command.engaged if command else None,
        'command_reason': command.reason if command else None,
        'joint_goal': getattr(control, '_joint_goal', None), 'controller_mode': control.mode,
        'controller_gripper_mode': control.gripper_mode,
        'ik_converged': getattr(control, 'ik_converged', None), 'ik_residual': getattr(control, 'ik_residual', None),
        'ik_rejections': getattr(control, 'ik_rejections', None), 'ik_partial_solutions': getattr(control, 'ik_partial_solutions', None),
        'step_completed': step_completed, 'error': error,
    }
    row.update(backend.telemetry())
    return json_value(row)
