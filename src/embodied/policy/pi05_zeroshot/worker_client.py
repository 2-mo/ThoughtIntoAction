"""Persistent policy subprocess with deadlines and explicit per-observation responses."""
import hashlib
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[4]


class PolicyWorker:
    def __init__(self, stats, *, python=None, script=None, extra_args=(), startup_timeout=180):
        self.stats = Path(stats).resolve()
        self.digest = hashlib.sha256(self.stats.read_bytes()).hexdigest()
        self.sequence = 0
        self.messages = queue.Queue()
        python = python or os.environ.get('PI05_PYTHON', str(ROOT / 'outputs/envs/pi05-zero-shot/bin/python'))
        script = script or ROOT/'src/embodied/policy/pi05_zeroshot/policy_worker.py'
        self.process = subprocess.Popen([str(python), str(script), '--stats', str(self.stats), *map(str,extra_args)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        try:
            ready = self.receive(startup_timeout)
            if not ready.get('ready') or ready.get('stats_sha256') != self.digest:
                raise ValueError('Policy/statistics startup mismatch')
        except BaseException:
            self.close()
            raise

    def _read(self):
        try:
            for line in self.process.stdout:
                if line.startswith('@pi05 '):
                    self.messages.put(json.loads(line[6:]))
                else:
                    print(line, end='', flush=True)
        except Exception as exc:
            self.messages.put({'error': str(exc)})
        finally:
            self.messages.put({'error': 'Policy worker exited'})

    def receive(self, timeout, check=lambda: None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check()
            try:
                result = self.messages.get(timeout=min(.05, max(.001, deadline-time.monotonic())))
            except queue.Empty:
                continue
            if 'error' in result:
                raise RuntimeError(result['error'])
            return result
        raise TimeoutError('Policy response timed out')

    def predict(self, output, prompt, check=lambda: None):
        self.sequence += 1
        self.process.stdin.write(json.dumps({'output': str(output.resolve()), 'prompt': prompt,
                                            'request_id': self.sequence}) + '\n')
        self.process.stdin.flush()
        value = self.receive(5, check)
        if value.get('request_id') != self.sequence or value['prediction']['stats_sha256'] != self.digest:
            raise ValueError('Mismatched policy response')
        return value['prediction']

    def close(self):
        try:
            self.process.stdin.close()
        except (OSError, ValueError):
            pass
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.reader.join(timeout=2)
        self.process.stdout.close()
