"""Local raw frames over PyZMQ multipart messages (JSON metadata + BGR buffer)."""
import json
import os
from pathlib import Path
import threading

import zmq

from embodied_contracts.device_lease import DeviceLease


def frame_path(control_path):
    return Path(str(control_path) + '.frames')


class FrameServer:
    """Read latest frames on demand; sockets stay on their owning thread."""

    def __init__(self, control_path, feeds):
        self.path = frame_path(control_path)
        self.feeds = feeds
        self.lease = DeviceLease('camera-frames:' + str(self.path.resolve()))
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, name='camera-frames', daemon=True)
        self.thread.start()
        self.ready.wait()
        if self.error:
            self.close()
            raise self.error

    def run(self):
        try:
            with zmq.Context.instance().socket(zmq.REP) as socket:
                socket.setsockopt(zmq.LINGER, 0)
                socket.setsockopt(zmq.SNDTIMEO, 1000)
                socket.setsockopt(zmq.MAXMSGSIZE, 16384)
                socket.bind('ipc://' + str(self.path))
                os.chmod(self.path, 0o600)
                self.ready.set()
                while not self.stop.is_set():
                    if not socket.poll(100):
                        continue
                    try:
                        camera_id = socket.recv_json()['camera_id']
                        feed = self.feeds[camera_id]
                        with feed.data_lock:
                            frame = feed.frame
                        metadata = None if frame is None else {k: v for k, v in frame.items() if k != 'image'}
                        data = b'' if frame is None else memoryview(frame['image']).cast('B')
                        header = {'ok': True, 'frame': metadata}
                    except (KeyError, TypeError, ValueError) as exc:
                        header, data = {'ok': False, 'error': str(exc)}, b''
                    socket.send_multipart([json.dumps(header, allow_nan=False).encode(), data], copy=False)
        except Exception as exc:
            self.error = exc
        finally:
            self.ready.set()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)
        self.path.unlink(missing_ok=True)
        self.lease.close()


class FrameClient:
    def __init__(self, control_path):
        self.socket = zmq.Context.instance().socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDTIMEO, 1000)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)
        self.socket.setsockopt(zmq.MAXMSGSIZE, 32 * 1024 * 1024)
        self.socket.connect('ipc://' + str(frame_path(control_path)))

    def read(self, camera_id):
        try:
            self.socket.send_json({'camera_id': camera_id})
            header, pixels = self.socket.recv_multipart(copy=False)
        except zmq.ZMQError as exc:
            self.close()
            raise RuntimeError('Camera frame transport unavailable') from exc
        response = json.loads(header.bytes)
        if not response['ok']:
            raise RuntimeError(response['error'])
        return response['frame'], pixels.buffer

    def close(self):
        self.socket.close(linger=0)
