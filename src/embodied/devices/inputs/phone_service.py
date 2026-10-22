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
    def __init__(self, controller=None):
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
                'estop': controller.estop, 'mapping': controller.mapping_name,
                'position_scale': controller.mapping.position_scale,
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
            else:
                raise ValueError('Unknown phone service action')
            return self.snapshot()

    def step(self):
        with self.lock:
            previous = self.controller.input_count
            try:
                self.controller.latest()
            except (OSError, ValueError, RuntimeError, EOFError) as exc:
                self.controller.disable()
                self.run = None
                self.error = str(exc)
            if self.controller.input_count != previous:
                self.samples.append(time.monotonic())

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
    service = PhoneService()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: service.closed.set())
    service.run_forever()


if __name__ == '__main__':
    main()
