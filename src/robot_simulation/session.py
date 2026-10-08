"""Simulation-only episode buttons and preset operations outside the servo loop."""
import threading
import time

from episode_io.runtime_episode import RuntimeEpisodeWriter
from quest_teleop.collection import Press, StickAction
from robot_core.presets import DEFAULT_GRASP_READY_JOINTS
from robot_core.project import load_config, resolve_path
from runtime_client import RuntimeReader
from runtime_client.control import ControlSession


class SimulationSession:
    def __init__(self, simulation):
        self.simulation = simulation
        self.a, self.b, self.r3 = Press(), Press(long_s=1.), Press(long_s=1.)
        self.stick = StickAction()
        self.task_active = False
        self.count = 0
        self.writer = None
        self.reader_thread = None
        self.record_stop = threading.Event()
        self.cancel = threading.Event()
        self.worker = None
        self.last = None
        self.error = None
        self.phase = None
        self.stamp = None
        self.was_enabled = False

    def snapshot(self):
        writer = self.writer
        return {'collection': {'state': 'ERROR' if self.error else 'POSITIONING' if self.phase else
                               'RECORDING' if writer else 'READY' if self.task_active else 'IDLE',
                               'task': 'MuJoCo 仿真 · 原始状态', 'error': self.error},
                'collection_count': self.count,
                'recording': {'id': writer.path.name, 'task': 'MuJoCo 仿真 · 原始状态',
                              'duration_s': (time.monotonic_ns()-writer.manifest['started_monotonic_ns'])/1e9,
                              'counts': dict(writer.counts)} if writer else None}

    def start_recording(self):
        if self.writer or self.reader_thread and self.reader_thread.is_alive():
            return
        reader = RuntimeReader(self.simulation.directory, 'full')
        root = resolve_path(load_config()['paths']['dataset_dir']) / 'mujoco'
        try:
            writer = RuntimeEpisodeWriter(root, reader.descriptor, 'MuJoCo Quest 遥操仿真')
        except BaseException:
            reader.close()
            raise
        self.writer = writer
        self.record_stop.clear()
        def capture():
            error = None
            try:
                while not self.record_stop.is_set():
                    tick = reader.read()
                    writer.add_tick(tick)
                    if tick['terminal'] or tick['error']:
                        raise RuntimeError(tick['error'] or '仿真服务结束')
            except (OSError, ValueError, RuntimeError, EOFError) as exc:
                error = str(exc)
            finally:
                manifest = writer.close('operator-stop', error)
                reader.close()
                self.last = writer.path
                if manifest['lifecycle'] == 'complete':
                    self.count += 1
                else:
                    self.error = error or manifest.get('error') or '仿真记录未完整保存'
                self.writer = None
        self.reader_thread = threading.Thread(target=capture, name='mujoco-recording', daemon=True)
        self.reader_thread.start()

    def save(self):
        self.record_stop.set()
        if self.reader_thread:
            self.reader_thread.join(timeout=4)
            if self.reader_thread.is_alive():
                raise RuntimeError('仿真数据仍在保存，未切换控制目标')

    def stop(self):
        self.cancel.set()
        self.save()
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=3)
            if self.worker.is_alive():
                raise RuntimeError('仿真动作仍在取消')

    def move(self, home=False, record_after=False):
        if self.worker and self.worker.is_alive():
            return
        self.save()
        self.simulation.command('pause')
        self.cancel.clear()
        self.phase = 'home' if home else 'pregrasp'
        def execute():
            try:
                with ControlSession(self.simulation.directory, source='simulation-preset') as session:
                    plan = session.preview('home' if home else 'movej',
                                           q=list(DEFAULT_GRASP_READY_JOINTS), speed=.5,
                                           gripper_width=session.status()['gripper_width'] if home else .082)
                    operation = session.execute(plan)['operation_id']
                    while not self.cancel.wait(.03):
                        status = session.status()
                        item = next(item for item in status['operations'] if item['operation_id'] == operation)
                        if item['status'] != 'running':
                            if item['status'] != 'completed':
                                raise RuntimeError(item.get('reason') or item['status'])
                            break
                    else:
                        session.command('cancel')
                    if self.cancel.is_set():
                        return
                self.simulation.command('resume', preserve_actions=True)
                # Restore the prior explicit enable only after a successful preset with released controls.
                with self.simulation.lock:
                    self.simulation.restore_enabled = True
                    self.simulation.revision += 1
                    self.simulation._request()
                deadline = time.monotonic()+2
                while time.monotonic() < deadline and not self.cancel.wait(.02):
                    if self.simulation.session and self.simulation.input.get('revision') == self.simulation.revision:
                        if record_after:
                            self.start_recording()
                        break
            except (OSError, ValueError, RuntimeError, EOFError) as exc:
                self.error = str(exc)
            finally:
                self.phase = None
        self.worker = threading.Thread(target=execute, name='mujoco-preset', daemon=True)
        self.worker.start()

    def update(self, inp, enabled):
        if inp.get('device_timestamp_ns') == self.stamp:
            return
        self.stamp = inp.get('device_timestamp_ns')
        hand = inp.get('hands', {}).get('right') or {}
        buttons = hand.get('buttons', {})
        grip = hand.get('grip', 0)
        if self.phase:
            if grip > .1 or not inp.get('tracking'):
                self.cancel.set()
            return
        if self.was_enabled and not enabled and self.writer:
            self.save()
        self.was_enabled = enabled
        now = time.monotonic()
        a = self.a.update(bool(buttons.get('A')), now, enabled=enabled and self.task_active and grip < .1)
        b = self.b.update(bool(buttons.get('B')), now, enabled=enabled and grip < .1)
        end = self.r3.update(bool(buttons.get('R3')), now, enabled=enabled)
        direction = self.stick.update(hand.get('axes', [0,0]), now, enabled=enabled and grip < .1)
        try:
            if direction == 'start_task':
                self.task_active, self.error = True, None
            elif direction == 'pause_task' or end == 'long':
                self.save()
                self.task_active = False
                if end == 'long': self.count = 0
            elif direction == 'discard_previous' and self.last and not self.writer:
                self.last.rename(self.last.with_name(self.last.name + '.discarded'))
                self.last = None
                self.count = max(0, self.count-1)
            elif a == 'short':
                was_recording = self.writer is not None
                self.move(record_after=not was_recording)
            elif b:
                self.move(home=b == 'long')
        except (OSError, ValueError, RuntimeError) as exc:
            self.error = str(exc)
