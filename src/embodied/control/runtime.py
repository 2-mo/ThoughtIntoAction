"""Process lifetime and measured 250 Hz scheduling; no robot-specific code."""

from collections import deque
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np


class RateStats:
    def __init__(self, hz: int = 250):
        self.hz = hz
        self.ticks = 0
        self.first = self.last = None
        self.intervals = deque(maxlen=20000)
        self.durations = deque(maxlen=20000)
        self.missed = 0
        self.max_duration = 0.0

    def observe(self, started: float, duration: float):
        if self.first is None:
            self.first = started
        if self.last is not None:
            interval = started - self.last
            self.intervals.append(interval)
            if interval > 1.2 / self.hz:
                self.missed += 1
        self.last = started
        self.ticks += 1
        self.durations.append(duration)
        self.max_duration = max(self.max_duration, duration)

    def report(self) -> dict:
        elapsed = (self.last - self.first) if self.ticks > 1 else 0.0
        return {
            "target_hz": self.hz,
            "ticks": self.ticks,
            "elapsed_s": elapsed,
            "achieved_hz": (self.ticks - 1) / elapsed if elapsed > 0 else 0.0,
            "missed_deadlines": self.missed,
            "deadline_threshold_ms": 1200 / self.hz,
            "max_tick_ms": self.max_duration * 1000,
            "p95_tick_ms": float(np.percentile(self.durations, 95) * 1000) if self.durations else 0.0,
            "p95_interval_ms": float(np.percentile(self.intervals, 95) * 1000) if self.intervals else 0.0,
        }


class PeriodicClock:
    def __init__(self, hz: int = 250):
        self.period = 1.0 / hz
        self.deadline = time.monotonic()

    def wait(self):
        self.deadline += self.period
        now = time.monotonic()
        if now > self.deadline + self.period:
            self.deadline = now
        remaining = self.deadline - now
        if remaining > 0:
            time.sleep(remaining)


class Children:
    def __init__(self, log_directory: Path):
        self.directory = Path(log_directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.processes = {}
        self.logs = []

    def start(self, name: str, command: list[str], *, cwd: Path | None = None):
        stream = (self.directory / f"{name}.log").open("w")
        self.logs.append(stream)
        env = os.environ.copy()
        env.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", PYTHONUNBUFFERED="1")
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        self.processes[name] = process
        return process

    def failed(self):
        for name, process in self.processes.items():
            status = process.poll()
            if status is not None:
                return name, status
        return None

    def terminate(self, names: list[str], timeout: float = 5.0):
        selected = [self.processes[name] for name in names if name in self.processes]
        for process in selected:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        for process in selected:
            if process.poll() is None:
                try:
                    process.wait(timeout=max(0.01, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()

    def stop(self):
        # Do not broadcast control/events: existing Rollio sessions are unrelated.
        self.terminate([name for name in self.processes if name not in ("backend", "viewer", "observer")], 3.0)
        # Four-second parking plus the SDK's two-second destructor damping.
        self.terminate(["backend"], 10.0)
        self.terminate(["observer"], 3.0)
        self.terminate(["viewer"], 3.0)
        for stream in self.logs:
            stream.close()
