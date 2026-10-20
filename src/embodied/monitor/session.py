"""Simulation-only episode buttons and preset operations outside the servo loop.

The right controller belongs to the arx-televr coordinator (A hold: mode, A+B: Home,
Grip: follow, Trigger: gripper). Collection reads only the left controller.
"""
import threading
import time

from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
from embodied.inputs.gestures import Press, StickAction
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
from embodied.core.project import load_config, resolve_path
from embodied.contracts.runtime import RuntimeReader
from embodied.contracts.runtime.control import ControlSession

# Shown by the monitor; the legacy real-arm page keeps its own layout.
BUTTON_LAYOUT = {
    'left': {'stick': 'L3', 'holds': ['stick', 'Y'],
             'keys': {'X': '录制 / 保存', 'Y': '预抓取 / 长按作废'}},
    'right': {'stick': None, 'holds': [],
              'keys': {'A': '长按切换模式', 'B': 'A+B 回 Home'}},
    'help': ['左手 X 预抓取后开始 / 保存后返位 · Y 预抓取 / 长按 1 秒作废当前 · 左 Grip 取消预设',
             '左摇杆：左开始任务 · 右暂停 · 上作废上一条 · 按住 1 秒结束清零'],
}


class SimulationSession:
    layout = BUTTON_LAYOUT

    def __init__(self, simulation):
        self.simulation = simulation
        self.x, self.y, self.l3 = Press(), Press(long_s=1.), Press(long_s=1.)
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

    def feedback(self):
        now = time.monotonic()
        def hold(press):
            elapsed = max(0., now - press.started) if press.started is not None else 0.
            return {'progress': min(1., elapsed / press.long_s), 'fired': press.fired and press.started is not None}
        return {'Y': hold(self.y), 'L3': hold(self.l3)}

    def discard_current(self):
        if not self.writer:
            return
        self.save()
        if self.last:
            self.last.rename(self.last.with_name(self.last.name + '.discarded'))
            self.last = None
            self.count = max(0, self.count - 1)

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
                # Resuming re-arms input only after the operator releases every control.
                self.simulation.command('resume', preserve_actions=True)
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
        hands = inp.get('hands', {})
        hand = hands.get('left') or {}
        buttons = hand.get('buttons', {})
        grip = hand.get('grip', 0)
        if self.phase:
            # Left Grip cancels; right Grip also means the operator is taking over.
            if grip > .1 or (hands.get('right') or {}).get('grip', 0) > .1 or not inp.get('tracking'):
                self.cancel.set()
            return
        if self.was_enabled and not enabled and self.writer:
            self.save()
        self.was_enabled = enabled
        now = time.monotonic()
        x = self.x.update(bool(buttons.get('X')), now, enabled=enabled and self.task_active and grip < .1)
        y = self.y.update(bool(buttons.get('Y')), now, enabled=enabled and grip < .1)
        end = self.l3.update(bool(buttons.get('L3')), now, enabled=enabled)
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
            elif x == 'short':
                was_recording = self.writer is not None
                self.move(record_after=not was_recording)
            elif y == 'short':
                self.move()
            elif y == 'long':
                self.discard_current()
        except (OSError, ValueError, RuntimeError) as exc:
            self.error = str(exc)
