"""Observe an existing unified kernel; never acquire control or create a runtime."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import threading
import time

from embodied.contracts.runtime import RuntimeReader
from embodied.core.project import PROJECT_ROOT, load_config, resolve_path
from embodied.core.telemetry import SCHEMA
from embodied.applications.monitor.quality import MotionQuality

from embodied.applications.monitor.capture_status import capture_snapshot
from embodied.applications.monitor.model import RuntimeModel
from embodied.data.episode.contract import runtime_contract


def discover_run(directory=None):
    """Return a ready live run of this boot, optionally pinned to one directory."""
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    paths = ([Path(directory) / 'runtime.json'] if directory else
             sorted((PROJECT_ROOT / 'outputs/runtime/control').glob('*/runtime.json'), reverse=True))
    for path in paths:
        try:
            identity = json.loads(path.read_text())
            if identity.get('boot_id') != boot or not (path.parent / 'runtime.ready').exists():
                continue
            if identity.get('backend') not in ('real', 'sim'):
                continue
            os.kill(identity['supervisor_pid'], 0)
            return path.parent, identity
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return None


def open_diagnostics(descriptor):
    """Optional receive-only driver observer; never construct a control backend."""
    if descriptor.get('backend') != 'real':
        return None
    plugin = (descriptor.get('plugins') or {}).get('robot')
    if plugin is None:
        return None
    from embodied.contracts.registry import Registry
    factory = Registry().load('robot', plugin, runtime='control')
    observe = getattr(factory, 'open_diagnostics', None)
    return observe(descriptor) if observe is not None else None


def kernel_packet(robot, tick, identity, robot_config=None, data_contract=None):
    """Keep measured feedback, submitted targets and SDK output distinct."""
    feedback = tick['feedback']
    command, output = tick.get('command') or {}, tick.get('sdk_output') or {}
    gripper = data_contract.get('gripper') if data_contract else None
    width = gripper['range'][1] if gripper else (robot_config or {}).get('gripper_max_width')
    if type(width) not in (int, float) or not math.isfinite(width) or width <= 0:
        width = None
    has_gripper = data_contract is None or gripper is not None
    return {
        'schema': SCHEMA, 'robot': robot.metadata(), 'stream_id': tick['runtime_id'],
        'sequence': tick['sequence'] + 1,
        'source': {'kind': 'hardware' if identity['backend'] == 'real' else 'virtual',
                   'controller': '统一内核 · ' + str(tick.get('source') or '保持'),
                   'input': identity.get('input', 'unknown')},
        'actual': feedback['q'], 'target': command.get('q', feedback['q']),
        'actual_vel': feedback.get('dq'), 'target_vel': command.get('dq'),
        'sample_monotonic_ns': tick['timing']['feedback_received_ns'],
        'emitted_monotonic_ns': time.monotonic_ns(), 'paths': {},
        'gripper': {'actual': feedback.get('gripper_width'), 'target': command.get('gripper_width'), 'max': width} if has_gripper else None,
        'actual_torque': None, 'gripper_actual_torque': None,
        'command_details': {
            'submission': {'pos': command.get('q'), 'gripper_pos': command.get('gripper_width')},
            'sdk_cmd_pos': output.get('q'), 'sdk_cmd_vel': output.get('dq'),
            'sdk_cmd_torque': output.get('torque'), 'kp': command.get('kp'), 'kd': command.get('kd'),
            'gripper_sdk_cmd_pos': output.get('gripper_width'),
            'gripper_kp': command.get('gripper_kp'), 'gripper_kd': command.get('gripper_kd'),
            'gripper_mode': command.get('gripper_mode'),
        },
    }


def collection_snapshot(identity, roots):
    """Use verified heartbeats for activity and finalized manifests for completion."""
    result = {'collection': {'state': '待机', 'task': '未发现当前内核的采集记录'},
              'collection_count': 0, 'recording': None,
              'collection_hint': '在总控按 C，或运行 ./run.sh collect run --run <RUN> --task <任务>。'}
    if not identity:
        result['collection'].update(state='OFFLINE', task='等待统一内核')
        result['collection_count'] = None
        return result
    records = []
    for root in roots:
        for path in Path(root).glob('*/episode.json'):
            try:
                manifest = json.loads(path.read_text())
                runtime = manifest.get('runtime') or {}
                if (runtime.get('runtime_id'), runtime.get('boot_id')) != (identity['runtime_id'], identity['boot_id']):
                    continue
                if manifest.get('schema_version') != 2:
                    continue
                records.append((manifest.get('started_monotonic_ns', 0), path, manifest))
            except (OSError, ValueError, TypeError):
                continue
    result['collection_count'] = sum(row[2].get('lifecycle') == 'complete' for row in records)
    if not records:
        return result
    live = []
    for record in records:
        heartbeat = capture_snapshot(record[1].parent, identity, record[2])
        if heartbeat and (heartbeat.get('recording') or {}).get('live'):
            live.append((*record, heartbeat))
    # A later, short recording can finish while an earlier collector is still active.
    # Prefer verified live captures; never promote an old recording manifest alone.
    if live:
        _, path, latest, heartbeat = max(live, key=lambda row: row[0])
    else:
        _, path, latest = max(records, key=lambda row: row[0])
        heartbeat = capture_snapshot(path.parent, identity, latest)
    lifecycle = latest.get('lifecycle')
    result['collection']['task'] = str(latest.get('task') or '未命名任务')
    # Historic writer format reserves `frame` for wrist_rgb, regardless of the anchor.
    counts = latest.get('counts') or {}
    cameras = latest.get('cameras') or ({'wrist_rgb': latest['camera']} if latest.get('camera') else {})
    result['episode_metadata'] = {'id': latest.get('id', path.parent.name), 'path': str(path),
                                  'lifecycle': lifecycle, 'counts': latest.get('counts') if lifecycle != 'recording' else None,
                                  'camera_counts': {key: counts.get('frame:' + key,
                                      counts.get('frame') if key == 'wrist_rgb' else None) for key in cameras}}
    if lifecycle == 'recording':
        result['collection']['state'] = '录制状态待确认'
        result['collection_hint'] = '发现未提交的录制元数据；无法据此确认采集仍在运行。请查看采集终端，实时计数未提供。'
    elif lifecycle == 'failed':
        result['collection']['state'] = 'ERROR'
        result['collection']['error'] = latest.get('error') or '上一条采集未完整保存'
        result['collection_hint'] = '上一条采集失败；查看采集终端及 episode.json。'
    else:
        result['collection_hint'] = '上一条已完整写盘；这不代表任务成功。下一条仍在总控 C / collect run 启动。'
    if heartbeat:
        result.update(heartbeat)
        if len(live) > 1:
            result['collection_hint'] += f' 当前有 {len(live)} 个采集进程；显示最近启动的一条。'
    elif lifecycle == 'complete':
        result['collection']['state'] = 'COMPLETE'
    return result


class KernelMonitor:
    """A latest-only observer whose lifetime is independent from the robot."""

    def __init__(self, directory=None, *, reader_factory=RuntimeReader, finder=discover_run, episode_roots=None, model_factory=RuntimeModel,
                 diagnostics_factory=open_diagnostics):
        self.pinned = directory
        self.reader_factory, self.finder = reader_factory, finder
        self.roots = episode_roots if episode_roots is not None else [PROJECT_ROOT / 'outputs/datasets/runtime',
            resolve_path(load_config()['paths']['dataset_dir']) / 'mujoco']
        self.reader = self.directory = self.identity = self.latest = None
        self.diagnostics_factory = diagnostics_factory
        self.diagnostics = None
        self.diagnostics_error = None
        self.robot_config = {}
        self.data_contract = None
        self.model_factory = model_factory
        self.model_runtime_id = None
        self.motion_quality = MotionQuality()
        self.scene = self.thread = None
        self.error = '等待运行中的统一内核；监看不会启动或控制机械臂。'
        self.closed = threading.Event()
        self.lock = threading.RLock()
        self.discovery_at = self.collection_at = 0.
        self.display_at = 0.
        self.collection = collection_snapshot(None, [])

    def _disconnect(self):
        if self.diagnostics is not None:
            self.diagnostics.close()
            self.diagnostics = None
        self.diagnostics_error = None
        if self.reader is not None:
            self.reader.close()
            self.reader = None

    def step(self):
        """One bounded observer read, also usable with an offline fake reader."""
        now = time.monotonic()
        if now >= self.discovery_at:
            self.discovery_at = now + 1
            found = self.finder(self.pinned)
            candidate = found[0] if found else None
            if candidate != self.directory:
                self._disconnect()
                with self.lock:
                    self.directory, self.identity = (found if found else (None, None))
                    self.latest = None
                    self.motion_quality.reset()
                    self.model_runtime_id = None
                    if self.scene:
                        self.scene.set_model(None)
                    self.collection_at = self.display_at = 0
            if candidate is None:
                self.error = '等待运行中的统一内核；监看不会启动或控制机械臂。'
        if self.directory is None:
            if self.scene:
                self.scene.set_stream_error(self.error)
            return
        try:
            if self.reader is None:
                self.reader = self.reader_factory(self.directory, 'latest', timeout=.5)
                self.robot_config = self.reader.descriptor.get('parameters', {}).get('effective_robot_config', {})
                self.data_contract = runtime_contract(self.reader.descriptor)
                try:
                    self.diagnostics = self.diagnostics_factory(self.reader.descriptor)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    # Diagnostics failure must not interrupt kinematic telemetry.
                    self.diagnostics_error = str(exc)
                if self.scene and self.model_runtime_id != self.identity['runtime_id']:
                    self.scene.set_model(self.model_factory(self.reader.descriptor))
                    self.model_runtime_id = self.identity['runtime_id']
            tick = self.reader.read()
            if tick.get('terminal') or tick.get('error'):
                raise EOFError(tick.get('error') or '内核已退出；等待下一次运行')
            if tick.get('feedback'):
                if self.scene and time.monotonic() >= self.display_at:
                    packet = kernel_packet(self.scene.robot, tick, self.identity, self.robot_config, self.data_contract)
                    if self.diagnostics is not None:
                        packet['motor_diagnostics'] = self.diagnostics.snapshot()
                    elif self.diagnostics_error:
                        packet['motor_diagnostics'] = {'motors': [], 'error': self.diagnostics_error}
                    self.scene.accept(packet)
                    self.display_at = time.monotonic() + 1 / 30
                with self.lock:
                    self.latest, self.error = tick, None
                    self.motion_quality.accept(tick)
        except (OSError, ValueError, RuntimeError, EOFError, KeyError, TypeError) as exc:
            self._disconnect()
            with self.lock:
                self.latest, self.error = None, f'等待内核反馈：{exc}'
                self.motion_quality.reset()
            if self.scene:
                self.scene.set_stream_error(self.error)

    def monitor(self):
        with self.lock:
            tick, identity, directory, error = self.latest, self.identity, self.directory, self.error
            if time.monotonic() >= self.collection_at:
                self.collection = collection_snapshot(identity, self.roots)
                self.collection_at = time.monotonic() + .25
            payload = deepcopy(self.collection)
            payload['motion_quality'] = self.motion_quality.snapshot()
        stamp = (tick or {}).get('timing', {}).get('feedback_received_ns')
        age = max(0., (time.monotonic_ns() - stamp) / 1e9) if stamp else None
        payload.update(runtime_monitor=True, runtime_id=(identity or {}).get('runtime_id'),
                       runtime_directory=str(directory) if directory else None,
                       state_age_s=age, input_age_s=None, input_error='当前内核未提供手柄监看帧',
                       monitor_message=error, connections={})
        payload['collection']['control_enabled'] = False
        if directory:
            try:
                inp = json.loads((directory / 'workbench-input.json').read_text())
                input_stamp = inp.get('sample_monotonic_ns')
                if input_stamp:
                    payload.update(input=inp, input_age_s=max(0., (time.monotonic_ns() - input_stamp) / 1e9), input_error=None)
                    payload['collection']['control_enabled'] = bool(inp.get('control_enabled'))
            except (OSError, ValueError, TypeError):
                pass
        return payload

    def start(self, scene):
        self.scene = scene
        def observe():
            while not self.closed.is_set():
                self.step()
                # Drain every observer message; throttling socket reads would
                # accumulate old ticks in the transport even in latest mode.
                # Only scene/FK work is limited to 30 Hz in step().
                if self.reader is None:
                    self.closed.wait(.5)
        self.thread = threading.Thread(target=observe, name='kernel-monitor-observer', daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        if self.thread:
            self.thread.join(timeout=2)
        self._disconnect()


def serve(args):
    """Foreground HTTP monitor; owns only its observer socket and scene."""
    from embodied.applications.monitor.scene import Scene
    monitor = KernelMonitor(args.run)
    scene = Scene(None, port=args.port, host=args.host, runtime_monitor=monitor)
    import signal
    previous = signal.getsignal(signal.SIGTERM)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        monitor.start(scene)
        print('统一内核只读监看：自动等待反馈；采集由总控 C / collect run 启动。', flush=True)
        while True:
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.close()
        scene.close()
        signal.signal(signal.SIGTERM, previous)
