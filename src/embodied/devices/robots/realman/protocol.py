"""Documented RealMan JSON/TCP framing; no enable or configuration commands."""
import json
import socket
import threading
import time


READS = {
    'get_arm_software_info': 'arm_software_info',
    'get_current_arm_state': 'current_arm_state',
    'get_current_tool_frame': 'current_tool_frame',
    'get_current_work_frame': 'current_work_frame',
}


class Client:
    def __init__(self, host, port=8080, timeout_s=.3, *, allow_motion=False):
        self.socket = socket.create_connection((host, port), timeout_s)
        self.socket.settimeout(timeout_s)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.timeout = timeout_s
        self.allow_motion = allow_motion
        self.buffer = b''
        self.lock = threading.RLock()

    def send(self, message):
        command = message.get('command')
        if command not in READS and not (self.allow_motion and command in
                                        {'movep_canfd', 'set_arm_stop'}):
            raise PermissionError(f'Command not permitted: {command}')
        with self.lock:
            self.socket.sendall((json.dumps(message, allow_nan=False, separators=(',', ':')) + '\r\n').encode())

    def query(self, command):
        if command not in READS:
            raise ValueError('Only documented read commands are queryable')
        with self.lock:
            self.send({'command': command})
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                while b'\n' not in self.buffer:
                    self.socket.settimeout(max(.001, deadline - time.monotonic()))
                    chunk = self.socket.recv(8192)
                    if not chunk:
                        raise ConnectionError('RealMan closed the TCP connection')
                    self.buffer += chunk
                    if len(self.buffer) > 1024 * 1024:
                        raise ValueError('Oversized RealMan JSON frame')
                line, self.buffer = self.buffer.split(b'\n', 1)
                reply = json.loads(line)
                if reply.get('state', reply.get('command')) == READS[command]:
                    return reply
                # Motion acknowledgements and asynchronous notifications are not feedback.
            raise TimeoutError(f'No matching reply to {command}')

    def close(self):
        self.socket.close()
