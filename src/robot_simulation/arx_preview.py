"""Isolated arx-televr/MuJoCo web preview. This entry has no hardware backend option."""
import argparse
import json
import os
import signal
import subprocess
import threading
import time

from robot_core.model import RobotModel
from robot_core.project import PROJECT_ROOT, LOG_DIR, load_config
from robot_core.telemetry import SCHEMA
from runtime_client import RuntimeReader
from .scene import Scene


def preview_packet(robot, tick):
    feedback = tick['feedback']
    command = tick.get('command') or {}
    return {
        'schema': SCHEMA, 'robot': robot.metadata(), 'stream_id': tick['runtime_id'],
        'sequence': tick['sequence'] + 1, 'source': {'kind': 'virtual', 'controller': 'arx-televr / MuJoCo', 'input': 'Quest 遥操'},
        'actual': feedback['q'], 'target': command.get('q', feedback['q']),
        'actual_vel': feedback['dq'], 'target_vel': command.get('dq'),
        'sample_monotonic_ns': tick['timing']['feedback_received_ns'],
        'emitted_monotonic_ns': time.monotonic_ns(), 'paths': {},
        'gripper': {'actual': feedback['gripper_width'], 'target': command.get('gripper_width'), 'max': .082},
        # Native RobotTick does not expose measured torque; never label feedforward as feedback.
        'actual_torque': None, 'gripper_actual_torque': None,
        'command_details': {'sdk_cmd_pos': command.get('q'), 'sdk_cmd_vel': command.get('dq'),
                            'kp': command.get('kp'), 'kd': command.get('kd')},
    }


class Simulation:
    """A persistent MuJoCo runtime. Deactivation releases input ownership, not the model."""
    def __init__(self):
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.active = False
        self.revision = 0
        self.restore_enabled = False
        from .session import SimulationSession
        self.actions = SimulationSession(self)
        self.directory = None
        self.session = None
        self.latest = None
        self.input = {}
        self.scene = None
        self.thread = None
        self.connection_process = None
        self.state = {'state': 'waiting', 'message': '仿真未启动', 'control_hz': 250}

    def status(self):
        with self.lock:
            return dict(self.state)

    def _request(self):
        if self.directory:
            temporary = self.directory / 'workbench-request.tmp'
            temporary.write_text(json.dumps({'revision': self.revision, 'active': self.active, 'restore_enabled': self.restore_enabled}))
            temporary.replace(self.directory / 'workbench-request.json')

    def command(self, action, *, preserve_actions=False):
        if action not in ('resume', 'pause', 'restart', 'stop'):
            raise ValueError('只支持启用遥操或暂停保持')
        if not preserve_actions:
            self.actions.stop()
        with self.lock:
            self.restore_enabled = False
            self.active = action in ('resume', 'restart')
            self.revision += 1
            self._request()
            if self.session:
                self.session.close()
                self.session = None
            self.state.update(message='待机：右摇杆下拨 1 秒启用' if self.active else '仿真已暂停 · 保持姿态')

    def start(self, scene):
        self.scene = scene
        if self.thread is None:
            self.thread = threading.Thread(target=self.run, name='mujoco-quest', daemon=True)
            self.thread.start()
        self.command('resume')

    def monitor(self):
        with self.lock:
            inp = dict(self.input)
            stamp = inp.get('sample_monotonic_ns')
            age = max(0., (time.monotonic_ns() - stamp) / 1e9) if stamp else None
            enabled = self.active and inp.get('control_enabled', False) and age is not None and age < .2
            tick = self.latest
            target = None
            if tick and (tick.get('target') or {}).get('pose'):
                import numpy as np
                from scipy.spatial.transform import Rotation
                pose = tick['target']['pose']
                matrix = np.eye(4)
                matrix[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
                matrix[:3, 3] = pose[:3]
                target = matrix.flatten(order='F').tolist()
            collection = self.actions.snapshot()
            collection['collection']['control_enabled'] = enabled
            return {**collection, 'state_age_s': age, 'input_age_s': age,
                    'input': {'sample_monotonic_ns': stamp, 'device_timestamp_ns': inp.get('device_timestamp_ns'),
                              'hands': inp.get('hands', {})},
                    'button_feedback': {'down': {'progress': inp.get('down_progress', 0)}, 'Y': ''},
                    'loop_rates': {'arm_service': {'configured_hz': 250, 'observed_hz': None}},
                    'orientation_preview': {'input_ready': bool(inp.get('tracking')), 'native_basis': True,
                         'basis': [[1,0,0],[0,1,0],[0,0,1]],
                         'active': enabled and (inp.get('hands', {}).get('right') or {}).get('grip', 0) >= .7,
                         'target': target},
                    'simulation': self.status()}

    def run(self):
        from runtime_client.control import ControlSession
        process = reader = None
        try:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            with (LOG_DIR / 'arx-web-simulation.log').open('ab', buffering=0) as log:
                # Prepare only the headset connection; never initializes CAN.
                if load_config().get('viewer_service', {}).get('prepare_quest', True):
                    self.connection_process = subprocess.Popen([str(PROJECT_ROOT / 'run.sh'), 'quest-transport', 'prepare'],
                                         stdin=subprocess.DEVNULL, stdout=log, stderr=log)
                process = subprocess.Popen([str(PROJECT_ROOT / 'run.sh'), 'control', 'start',
                    '--backend', 'sim', '--input', 'quest', '--managed', '--input-plugin', 'arx.workbench',
                    '--routing-plugin', 'arx.workbench', '--ingress-plugin', 'arx.workbench', '--headless'],
                    stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic() + 20
            while not self.closed.is_set() and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('MuJoCo 启动失败，见 arx-web-simulation.log')
                for path in (PROJECT_ROOT / 'control/run').glob('*/runtime.json'):
                    try:
                        identity = json.loads(path.read_text())
                    except (OSError, ValueError):
                        continue
                    if identity.get('supervisor_pid') == process.pid and (path.parent / 'runtime.ready').exists():
                        if identity.get('backend') != 'sim' or identity.get('input') != 'vr':
                            raise RuntimeError('仿真运行身份不匹配')
                        with self.lock:
                            self.directory = path.parent
                            self._request()
                        break
                if self.directory:
                    break
                self.closed.wait(.05)
            if self.closed.is_set():
                return
            if self.directory is None:
                raise RuntimeError('等待 MuJoCo 就绪超时')
            reader = RuntimeReader(self.directory, 'latest')
            last_display = 0
            while not self.closed.is_set():
                tick = reader.read()
                if tick.get('error') or tick['terminal']:
                    raise RuntimeError(tick.get('error') or '仿真运行已结束')
                if tick.get('feedback') and time.monotonic() - last_display >= 1/30:
                    with self.lock:
                        self.latest = tick
                        try:
                            self.input = json.loads((self.directory / 'workbench-input.json').read_text())
                        except FileNotFoundError:
                            pass
                        # A lost tracking lease cannot reactivate a held Grip on reconnect.
                        if self.session and self.session.error:
                            self.session.close()
                            self.session = None
                            self.revision += 1
                            self._request()
                        if self.active and self.session is None and self.input.get('revision') == self.revision:
                            self.session = ControlSession(self.directory, source='quest', ingress='native', purpose='teleop')
                            self.session.__enter__()
                        inp = self.input
                        message = ('仿真已暂停 · 保持姿态' if not self.active else
                                   '等待 Quest 连接 / 有效跟踪' if not inp.get('tracking') else
                                   '控制已启用 · Grip 跟随 / Trigger 夹爪' if inp.get('control_enabled') else
                                   '待机：松开按键，右摇杆回中后下拨 1 秒启用')
                        self.state.update(state='running' if self.active else 'paused', message=message,
                                          runtime_id=tick['runtime_id'], directory=str(self.directory),
                                          tracking=bool(inp.get('tracking')), control_enabled=bool(self.active and inp.get('control_enabled')))
                    with self.scene._lock:
                        if self.scene.simulation is self:
                            self.scene.accept(preview_packet(self.scene.robot, tick))
                    self.actions.update(self.input, bool(self.active and self.input.get('control_enabled')))
                    last_display = time.monotonic()
        except (OSError, RuntimeError, ValueError, EOFError, KeyError) as exc:
            with self.lock:
                self.state.update(state='error', message=str(exc))
        finally:
            with self.lock:
                self.actions.record_stop.set()
                if self.session:
                    self.session.close()
                    self.session = None
            if reader:
                reader.close()
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)

    def close(self):
        self.command('pause')
        self.closed.set()
        if self.thread:
            self.thread.join(timeout=22)


def main(argv=None):
    from .cli import main as viewer_main
    viewer_main(['--mode', 'simulation', *(argv or [])])


if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
