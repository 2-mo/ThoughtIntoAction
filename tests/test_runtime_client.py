"""Exercise the real socket framing and loss contracts without control dependencies."""
import json
from pathlib import Path
import socket
import struct
import tempfile
import threading
import unittest

from embodied.contracts.runtime.client import RuntimeReader, read_message


class RuntimeClientTests(unittest.TestCase):
    def server(self, rows, **overrides):
        directory = tempfile.TemporaryDirectory(prefix='runtime-client-')
        self.addCleanup(directory.cleanup)
        path = Path(directory.name)
        identity = {'runtime_id': 'test-runtime',
                    'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
        (path / 'runtime.json').write_text(json.dumps(identity))
        server = socket.socket(socket.AF_UNIX)
        server.bind(str(path / 'observer.sock'))
        server.listen(1)
        self.addCleanup(server.close)

        def serve():
            sock, _ = server.accept()
            with sock:
                request = read_message(sock)
                descriptor = {'type': 'descriptor', 'protocol_version': 1, 'observer_gaps': 0,
                              'mode': request['mode'], **identity, **overrides}
                try:
                    for value in [descriptor, *rows]:
                        encoded = json.dumps(value).encode()
                        sock.sendall(struct.pack('!I', len(encoded)) + encoded)
                except OSError:
                    pass
        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 2)
        return path

    def tick(self, sequence, **changes):
        return {'type': 'tick', 'version': 1, 'runtime_id': 'test-runtime', 'sequence': sequence,
                'terminal': False, 'producer_dropped': 0, 'observer_gaps': 0, **changes}

    def test_terminal_is_explicit_and_eof_is_not_success(self):
        with RuntimeReader(self.server([self.tick(50), self.tick(51, terminal=True)]), 'full') as reader:
            self.assertEqual(reader.read()['sequence'], 50)
            self.assertTrue(reader.read()['terminal'])
            with self.assertRaisesRegex(EOFError, 'already received'):
                reader.read()
        with RuntimeReader(self.server([self.tick(50)]), 'full') as reader:
            reader.read()
            with self.assertRaises(EOFError):
                reader.read()
            self.assertFalse(reader.terminal)

    def test_full_rejects_missing_ticks_or_reported_loss(self):
        for second in [self.tick(52), self.tick(51, observer_gaps=1), self.tick(51, producer_dropped=1)]:
            with self.subTest(second=second), RuntimeReader(self.server([self.tick(50), second]), 'full') as reader:
                reader.read()
                with self.assertRaises(ValueError):
                    reader.read()

    def test_latest_may_skip_but_cannot_repeat(self):
        with RuntimeReader(self.server([self.tick(5), self.tick(12), self.tick(12)])) as reader:
            reader.read()
            self.assertEqual(reader.read()['sequence'], 12)
            with self.assertRaisesRegex(ValueError, 'Repeated'):
                reader.read()

    def test_identity_version_and_boot_are_checked(self):
        for changes in [{'runtime_id': 'different'}, {'protocol_version': 2}, {'boot_id': 'old-boot'}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                RuntimeReader(self.server([], **changes))
        for message in [[], self.tick(1, version=99), self.tick(1, runtime_id='other'),
                        self.tick(True), self.tick(1, terminal='false')]:
            with self.subTest(message=message), RuntimeReader(self.server([message])) as reader:
                with self.assertRaises(ValueError):
                    reader.read()

    def test_invalid_frame_length_fails_before_allocation(self):
        left, right = socket.socketpair()
        with left, right:
            left.sendall(struct.pack('!I', 0xffffffff))
            with self.assertRaisesRegex(ValueError, 'length'):
                read_message(right)


if __name__ == '__main__':
    unittest.main()
