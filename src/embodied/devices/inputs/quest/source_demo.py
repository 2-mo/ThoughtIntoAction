"""Simulation input using the upstream demo, starting from managed feedback hold."""
from dataclasses import replace
import signal
import time
from embodied.devices.inputs.quest.demo import frame_at
from embodied.control.ipc import XRPublisher
from embodied.control.runtime import PeriodicClock


def run(prefix):
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    publisher = XRPublisher(prefix)
    clock = PeriodicClock(90)
    started, sequence = time.monotonic(), 0
    try:
        while not stopped:
            sequence += 1
            elapsed = time.monotonic() - started
            frame = frame_at(elapsed, sequence)
            # Managed hold is already enabled. The first A enters following;
            # the unmanaged demo's second A would toggle back to FreeDrive.
            if 4 <= elapsed % 18 < 4.25:
                frame = replace(frame, primary=False)
            publisher.publish(frame)
            clock.wait()
    finally:
        publisher.close()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    run(parser.parse_args().prefix)
