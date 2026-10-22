"""Receive-only X5 CAN diagnostics. No transmit API and no SDK/controller ownership.

X5 motor IDs/types follow pinned arx5-sdk config 8c797d5. EC current scaling
matches the SDK hardware decoder. DM load bits are torque, not amperes.
See docs/guides/motor-feedback.md for wire layouts, units and verification evidence.
"""
from collections import deque
import socket
import struct
import threading
import time
import uuid


def decode_feedback(can_id, data, motor_types):
    if can_id & ~0x7ff or len(data) != 8:
        return None
    if can_id == 0:
        motor_id, status = data[0] & 15, data[0] >> 4
        if motor_types.get(motor_id) != 'DM_J4310':
            return None
        limit = 18.
        temperature, rotor = float(data[6]), float(data[7])
        fault = status if status >= 8 else 0 if status in (0, 1) else None
        protocol = 'dm_mit'
    else:
        motor_id = can_id
        if motor_types.get(motor_id) != 'EC_A4310' or data[0] >> 5 != 1:
            return None
        status = fault = data[0] & 31
        limit = 30.
        temperature, rotor = (data[6] - 50) / 2., None
        protocol = 'ec_ack1'
    raw = ((data[4] & 15) << 8) | data[5]
    return {'motor_id': motor_id, 'protocol': protocol, 'current_raw': raw if protocol == 'ec_ack1' else None,
            'current_a': raw * (2 * limit) / 4095 - limit if protocol == 'ec_ack1' else None,
            'load_raw': raw, 'load_value': raw * (2 * limit) / 4095 - limit,
            'load_unit': 'A' if protocol == 'ec_ack1' else 'N*m',
            'temperature_c': temperature, 'rotor_temperature_c': rotor,
            'bus_voltage_v': None, 'status_code': status, 'fault_code': fault,
            'raw_feedback_hex': data.hex()}


class MotorFeedback:
    def __init__(self, interface, joint_ids, joint_types, gripper_id, gripper_type):
        self.ids = [*joint_ids, gripper_id]
        types = [*joint_types, gripper_type]
        self.types = dict(zip(self.ids, types))
        self.stream_id = uuid.uuid4().hex
        self.interface = interface
        self.lock = threading.Lock()
        self.latest = {}
        self.error = None
        self.events = deque(maxlen=32)
        self.event_sequence = 0
        self.statuses = {}
        self.stop_event = threading.Event()
        self.sock = None
        self.thread = None
        try:
            self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            filters = [struct.pack('=II', i, 0xC00007FF) for i in {0, *self.ids}]
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b''.join(filters))
            self.sock.settimeout(.05)
            self.sock.bind((interface,))
            self.thread = threading.Thread(target=self._receive, name='motor-diagnostics', daemon=True)
            self.thread.start()
        except OSError as exc:
            self.error = str(exc)
            if self.sock is not None: self.sock.close()
            self.sock = None

    def _status(self, motor_id, now):
        row = self.latest.get(motor_id)
        fresh = row is not None and 0 <= now - row['host_received_monotonic_ns'] <= 250_000_000
        fault = row['fault_code'] if fresh else None
        return ('error' if self.error else 'live' if fresh else 'stale' if row else 'missing',
                fault, row['status_code'] if fresh else None)

    def _events(self, now):
        for motor_id in self.ids:
            status = self._status(motor_id, now)
            if self.statuses.get(motor_id) != status:
                self.statuses[motor_id] = status
                self.event_sequence += 1
                self.events.append({'sequence': self.event_sequence, 'host_monotonic_ns': now,
                    'motor_id': motor_id, 'feedback_status': status[0], 'fault_code': status[1],
                    'status_code': status[2], 'valid': status[0] == 'live' and status[1] == 0})

    def _receive(self):
        try:
            while not self.stop_event.is_set():
                try:
                    raw, _, flags, _ = self.sock.recvmsg(16)
                except socket.timeout:
                    with self.lock: self._events(time.monotonic_ns())
                    continue
                # MSG_DONTROUTE marks locally generated CAN traffic (other send sockets).
                if flags & socket.MSG_DONTROUTE or len(raw) != 16:
                    continue
                can_id, length, data = struct.unpack('=IB3x8s', raw)
                row = decode_feedback(can_id, data[:length], self.types)
                if row is not None:
                    now = time.monotonic_ns()
                    with self.lock:
                        self.latest[row['motor_id']] = {**row, 'host_received_monotonic_ns': now}
                        self._events(now)
        except OSError as exc:
            if not self.stop_event.is_set():
                with self.lock:
                    self.error = str(exc)
                    self._events(time.monotonic_ns())

    def snapshot(self):
        now = time.monotonic_ns()
        with self.lock:
            self._events(now)
            rows = []
            for motor_id in self.ids:
                row = self.latest.get(motor_id)
                status, fault, code = self._status(motor_id, now)
                fresh = status == 'live'
                rows.append({'motor_id': motor_id, 'motor_type': self.types[motor_id],
                    'feedback_status': status, 'valid': fresh and fault == 0,
                    'age_s': (now-row['host_received_monotonic_ns'])/1e9 if row else None,
                    **{key: row[key] if row and fresh else None for key in
                       ('current_a', 'current_raw', 'load_raw', 'load_value', 'load_unit', 'temperature_c', 'rotor_temperature_c',
                        'status_code', 'fault_code', 'raw_feedback_hex', 'host_received_monotonic_ns')},
                    'bus_voltage_v': None})
            return {'host_monotonic_ns': now, 'source': 'socketcan_rx', 'stream_id': self.stream_id, 'interface': self.interface,
                    'joint_order': self.ids[:-1], 'gripper_id': self.ids[-1],
                    'motors': rows, 'error': self.error, 'events': list(self.events)}

    def close(self):
        self.stop_event.set()
        if self.thread is not None: self.thread.join(timeout=1)
        if self.sock is not None: self.sock.close()
