"""Terminal key input for Rich consoles; no device/control dependencies."""
from contextlib import contextmanager
import os
import select
import sys
import termios


@contextmanager
def terminal_keys():
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    settings = termios.tcgetattr(fd)
    settings[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
    settings[6][termios.VMIN] = 1
    settings[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    try:
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def read_key(timeout=.1):
    fd = sys.stdin.fileno()
    if not select.select([fd], [], [], timeout)[0]:
        return None
    value = os.read(fd, 1)
    if not value:
        return 'eof'
    if value == b'\x1b':
        while len(value) < 24 and select.select([fd], [], [], .02)[0]:
            value += os.read(fd, 1)
            if len(value) >= 3 and (value[-1:] == b'~' or value[-1:].isalpha()):
                break
        return {b'\x1b[A': 'up', b'\x1bOA': 'up', b'\x1b[B': 'down', b'\x1bOB': 'down',
                b'\x1b[F': 'end', b'\x1bOF': 'end', b'\x1b[4~': 'end', b'\x1b[6~': 'end'}.get(value, 'escape')
    return value.decode(errors='ignore').lower()
