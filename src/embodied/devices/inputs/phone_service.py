"""One phone receiver shared by the workbench and web monitor.

The receiver starts in observation mode. Only an explicit enable request leases
an already running kernel; UI refreshes never acquire or transfer robot control.
"""
from collections import deque
from contextlib import closing
import json
import os
import signal
import subprocess
import threading
import time

from embodied.contracts.device_lease import DeviceLease
from embodied.contracts.runtime.control import ControlClient, live_runs
from embodied.core.local_server import LocalServer
from embodied.core.project import PROJECT_ROOT, port
from embodied.core.session_protocol import request

SOCKET = PROJECT_ROOT / 'outputs/runtime/phone.sock'
LOG = PROJECT_ROOT / 'outputs/logs/phone-service.log'
LATCH = PROJECT_ROOT / 'outputs/runtime/phone-stop.json'
CONTROL_PROTOCOL = 'tia.phone.control.v1'


def addresses():
    """Show local interface addresses; discovery never scans other devices."""
    try:
        rows = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show', 'scope', 'global'],
                                                  text=True, timeout=2))
        rows.sort(key=lambda row: (not row['ifname'].startswith('wl'), row['ifname']))
        return [value['local'] for row in rows if not row['ifname'].startswith(('docker', 'br-', 'veth'))
                for value in row.get('addr_info', []) if value.get('family') == 'inet']
    except (OSError, ValueError, subprocess.SubprocessError):
        return []


def monitor():
    try:
        return request(SOCKET, 'monitor', timeout=.3)
    except (OSError, ValueError, RuntimeError) as exc:
        return {'available': False, 'connected': False, 'enabled': False, 'error': str(exc),
                'port': port('phone_udp'), 'addresses': []}


def command(action, **fields):
    return request(SOCKET, action, timeout=3, **fields)


def ensure_service():
    if monitor()['available']:
        return
    with closing(DeviceLease('phone-launch:' + str(SOCKET))):
        if monitor()['available']:
            return
        LOG.parent.mkdir(parents=True, exist_ok=True)
        with LOG.open('ab', buffering=0) as output:
            child = subprocess.Popen([str(PROJECT_ROOT / '.venv/bin/python'), '-B', '-m',
                'embodied.devices.inputs.phone_service'], cwd=PROJECT_ROOT,
                env={**os.environ, 'PYTHONPATH': str(PROJECT_ROOT / 'src')},
                stdin=subprocess.DEVNULL, stdout=output, stderr=output, start_new_session=True)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if monitor()['available']:
                return
            if child.poll() is not None:
                break
            time.sleep(.1)
        raise RuntimeError(f'手机接收服务未启动，请查看 {LOG}')


class PhoneService:
    def __init__(self, controller=None, *, latch_path=None):
        if controller is None:
            from embodied.devices.inputs.phone import PhoneController
            controller = PhoneController(bind_host='0.0.0.0', port=port('phone_udp'), ghost=True)
        self.controller = controller
        self.lock = threading.RLock()
        self.samples = deque(maxlen=1000)
        self.error = None
        self.run = None
        self.addresses = addresses()
        self.closed = threading.Event()
        self.server = None
        self.messages = deque(maxlen=16)
        self.controller.source.on_message = self.messages.append
        self.controller.reset_request = self.reset_from_phone
        self.latch_path = latch_path
        if latch_path is not None and latch_path.exists():
            self.controller.estop = bool(json.loads(latch_path.read_text())['latched'])
        self.saved_latch = self.controller.estop
        self.request_session = None
        self.request_sequence = -1
        self.last_reply = None
        self.reset_ack = None

    def save_latch(self):
        if self.saved_latch == self.controller.estop:
            return
        if self.latch_path is not None:
            self.latch_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.latch_path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'latched': self.controller.estop}) + '\n')
            temporary.replace(self.latch_path)
        self.saved_latch = self.controller.estop

    def recover(self, packet):
        state = self.snapshot()
        if packet.session_id != state['session_id'] or not state['connected']:
            raise ValueError('手机连接已变化，请重新连接后恢复')
        if state['clutch'] or state['tracking'] != 'normal' or self.controller.last_input.estop_requested:
            raise ValueError('松开 Clutch，等待追踪正常后恢复')
        if (packet.client_time_ns < self.controller.stop_input_time_ns
                or abs(packet.client_time_ns - self.controller.last_input.timestamp_ns) > 500_000_000):
            raise ValueError('恢复请求已过期，请再次点恢复')
        self.controller.disable()
        self.controller.estop = self.controller.zero_requested = False
        try:
            self.save_latch()
        except OSError:
            self.controller.estop = True
            raise
        self.run = self.error = None

    def reset_from_phone(self, packet):
        try:
            if self.reset_ack == packet and not self.controller.estop and not self.controller.zero_requested:
                return True  # Lost ACK retry must not release a subsequently acquired owner.
            self.recover(packet)
            self.reset_ack = packet
            return True
        except (OSError, ValueError, RuntimeError) as exc:
            self.error = str(exc)
            return False

    def phone_message(self, payload):
        """Small session-bound command channel beside binary pose packets.

        Sequence order and bounded client timestamps reject delayed starts.
        Retries of the latest request return its reply without acquiring twice.
        """
        if len(payload) > 1024:
            return
        try:
            data = json.loads(payload)
            value = self.controller.last_input
            if (not isinstance(data, dict) or data.get('protocol') != CONTROL_PROTOCOL
                    or type(data.get('session_id')) is not int
                    or value is None or data.get('session_id') != value.session_id):
                return
            sequence = data.get('id')
            stamp = data.get('client_time_ns')
            if (type(sequence) is not int or not 0 <= sequence < 2**32
                    or type(stamp) is not int or not 0 <= stamp < 2**64):
                return
            if self.request_session != value.session_id:
                self.request_session, self.request_sequence, self.last_reply = value.session_id, -1, None
            if sequence == self.request_sequence and self.last_reply is not None:
                self.controller.transport.send(self.last_reply)
                return
            if sequence < self.request_sequence:
                return
            self.request_sequence = sequence
            error = None
            try:
                if abs(stamp - value.timestamp_ns) > 500_000_000:
                    raise ValueError('操作已过期，请重试')
                if data.get('action') not in ('monitor', 'enable', 'disable', 'home', 'pregrasp', 'align'):
                    raise ValueError('不支持的手机操作')
                if data['action'] != 'monitor':
                    self.dispatch({'command': data['action'], 'runtime_id': data.get('runtime_id')})
            except (OSError, ValueError, RuntimeError) as exc:
                error = str(exc)
            runs = live_runs(PROJECT_ROOT / 'outputs/runtime/control')
            runtime_id = ControlClient(runs[-1]).identity['runtime_id'] if runs else None
            if self.controller.calibrated_runtime_id not in (None, runtime_id):
                self.controller.clear_calibration()
            state = self.snapshot()
            response = {'protocol': CONTROL_PROTOCOL, 'id': sequence, 'session_id': value.session_id,
                        'ok': error is None, 'message': (error or self.error or '')[:240],
                        'kernel_runtime_id': runtime_id,
                        **{key: state[key] for key in ('connected', 'enabled', 'active', 'estop', 'tracking',
                            'position_scale', 'rotation_scale', 'target_limited', 'position_lead_m', 'preset', 'aligned')}}
            client, robot = self.controller.client, self.controller.last_robot
            response['robot_gripper'] = (robot['gripper_width'] /
                client.parameters['effective_robot_config']['gripper_max_width'] if client is not None and robot else None)
            self.last_reply = json.dumps(response, ensure_ascii=False, allow_nan=False).encode()
            self.controller.transport.send(self.last_reply)
        except (OSError, ValueError, RuntimeError):
            return

    def snapshot(self):
        now = time.monotonic()
        controller = self.controller
        value = controller.last_input
        age = now - controller.last_received if controller.last_received else None
        fresh = age is not None and 0 <= age < .5
        recent = [stamp for stamp in self.samples if now - stamp < 1.]
        fps = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 else 0.
        return {'available': True, 'connected': fresh, 'enabled': not controller.ghost,
                'active': bool(fresh and controller.active), 'error': self.error,
                'addresses': self.addresses, 'port': port('phone_udp'),
                'peer': controller.transport.last_peer, 'age_s': age, 'fps': fps,
                'sequence': value.sequence if value else None,
                'session_id': value.session_id if value else None,
                'tracking': value.tracking_quality.value if fresh and value else 'unavailable',
                'clutch': value.clutch_active if fresh and value else False,
                'gripper': value.gripper if fresh and value else None,
                'position': list(value.pose.position.as_tuple()) if fresh and value else None,
                'phone_pose': ({'frame': 'phone_reference', 'quaternion_xyzw': list(value.pose.orientation.as_tuple())}
                               if fresh and value and value.tracking_quality.value == 'normal' else None),
                'preset': dict(controller.preset) if controller.preset else None,
                'aligned': controller.policy.rotation_alignment is not None,
                'orientation_preview': ({'frame': 'base_tcp',
                    'input_quaternion': list(controller.policy.mapped_orientation.as_tuple()),
                    'target_quaternion': controller.last_target['pose'][3:]}
                    if fresh and controller.active and controller.last_target
                    and controller.policy.mapped_orientation is not None else None),
                'estop': controller.estop, 'mapping': controller.mapping_name,
                'position_scale': controller.mapping.position_scale,
                'rotation_scale': controller.mapping.rotation_scale,
                'target_limited': bool(controller.active and controller.policy.limited),
                'position_lead_m': controller.policy.position_lead_m if controller.active else 0.,
                'runtime_id': self.run, 'received': controller.input_count}

    def dispatch(self, data):
        with self.lock:
            action = data.get('command')
            if action == 'monitor':
                return self.snapshot()
            if action == 'enable':
                state = self.snapshot()
                if not state['connected'] or state['tracking'] != 'normal':
                    raise ValueError('先连接手机并等待 ARKit 正常追踪')
                if state['clutch']:
                    raise ValueError('先松开手机 Clutch，再启用遥操')
                runs = live_runs(PROJECT_ROOT / 'outputs/runtime/control')
                if not runs:
                    raise ValueError('先在总控启动 robot')
                client = ControlClient(runs[-1])
                if data.get('runtime_id') != client.identity['runtime_id']:
                    raise ValueError('robot 运行实例已变化，请刷新后重新启用')
                self.controller.enable(runs[-1])
                self.run = client.identity['runtime_id']
                self.error = None
            elif action == 'disable':
                self.controller.disable()
                self.run = None
                self.error = None
            elif action == 'align':
                # Calibration observes the phone even in monitor mode. It never
                # acquires a robot lease and cannot start a robot or a preset.
                client = self.controller.client
                if client is None:
                    runs = live_runs(PROJECT_ROOT / 'outputs/runtime/control')
                    if not runs:
                        raise ValueError('请先启动 robot 取得当前夹爪姿态')
                    client = ControlClient(runs[-1])
                runtime_id = client.identity['runtime_id']
                if data.get('runtime_id') != runtime_id:
                    raise ValueError('robot 运行实例已变化，请刷新后重新对齐')
                self.controller.request_alignment(client.status(), runtime_id)
                self.error = None
            elif action in ('home', 'pregrasp'):
                if not self.run or data.get('runtime_id') != self.run:
                    raise ValueError('robot 运行实例已变化，请重新开始手机遥操')
                self.controller.request_preset(action)
                self.error = None
            else:
                raise ValueError('Unknown phone service action')
            return self.snapshot()

    def step(self):
        with self.lock:
            previous = self.controller.input_count
            try:
                self.controller.latest()
                if not self.controller.ghost and time.monotonic() - self.controller.last_received > .5:
                    self.controller.disable()
                    self.run = None
                    self.error = '手机断流，已停止遥操；连接恢复后点开始'
            except (OSError, ValueError, RuntimeError, EOFError) as exc:
                self.controller.disable()
                self.run = None
                self.error = str(exc)
            if self.controller.input_count != previous:
                self.samples.append(time.monotonic())
            self.save_latch()
            while self.messages:
                self.phone_message(self.messages.popleft())

    def run_forever(self):
        self.controller.start()
        try:
            self.server = LocalServer(SOCKET, self.dispatch)
            print(f'Phone receiver ready: UDP {port("phone_udp")} · observation only', flush=True)
            while not self.closed.is_set():
                self.step()
                # Yield the lock to monitor / stop requests even without UDP input.
                self.closed.wait(.001)
        finally:
            if self.server is not None:
                self.server.close()
            self.controller.stop()


def main():
    service = PhoneService(latch_path=LATCH)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: service.closed.set())
    service.run_forever()


if __name__ == '__main__':
    main()
