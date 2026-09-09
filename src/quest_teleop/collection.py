"""Debounced physical presses and non-blocking episode operations."""
import threading
import time


class Press:
    def __init__(self, long_s=None, debounce_s=.03):
        self.long_s, self.debounce_s = long_s, debounce_s
        self.raw = self.stable = False
        self.changed = 0.
        self.started = None
        self.fired = False
        self.armed = False
        self.allowed = True

    def update(self, pressed, now, enabled=True):
        if not pressed and self.started is None:
            self.armed = True
        if pressed != self.raw:
            self.raw, self.changed = pressed, now
            if pressed:
                self.allowed = enabled
        if self.raw != self.stable and now - self.changed >= self.debounce_s:
            self.stable = self.raw
            if self.stable:
                if self.armed and self.allowed:
                    self.started, self.fired = self.changed, False
            elif self.started is not None:
                duration = self.changed - self.started
                self.started = None
                if not self.fired:
                    return "long" if self.long_s is not None and duration >= self.long_s else "short"
        if self.stable and self.raw and self.started is not None and not self.fired:
            if self.long_s is not None and now - self.started >= self.long_s:
                self.fired = True
                return "long"
        return None


class StickAction:
    """Trigger on directional entry once; neutral rearms, without a hold timer."""
    def __init__(self):
        self.armed = False

    def update(self, axes, now, enabled=True):
        import math
        x, y = axes
        if not all(math.isfinite(v) for v in (x, y)):
            self.armed = False
            return None
        if abs(x) < .25 and abs(y) < .25:
            self.armed = True
            return None
        if not enabled:
            self.armed = False
            return None
        direction = ("start_task" if x < 0 else "pause_task") if abs(x) >= .8 and abs(y) < .35 else None
        if y >= .8 and abs(x) < .35:
            direction = "discard_previous"
        if self.armed and (abs(x) >= .8 or abs(y) >= .8):
            self.armed = False
            return direction
        return None


class EpisodeActions:
    def __init__(self, session, task):
        self.session, self.task = session, task
        self.lock = threading.Lock()
        self.busy = False
        self.home = False
        self.error = None
        self.closed = False
        self.generation = 0
        self.thread = None
        self.saving_episode = None
        self.discard_after_save = False

    def submit(self, action):
        with self.lock:
            if self.closed:
                return False
            if self.busy:
                if action == "discard_previous" and self.saving_episode is not None:
                    self.discard_after_save = True
                    return True
                return False
            session = self.session()
            writer = session.recording if session is not None else None
            self.saving_episode = writer.id if action in ("stop_episode", "pause_task", "end_task") and writer is not None else None
            self.discard_after_save = False
            self.busy, self.error = True, None
            generation = self.generation
            self.thread = threading.Thread(target=self._perform, args=(action, generation), daemon=True)
            self.thread.start()
            return True

    def _perform(self, action, generation):
        try:
            session = self.session()
            if session is not None:
                session.dispatch({"command": "collection." + action, "task": self.task})
            elif action != "home":
                raise RuntimeError("采集会话未启用；请使用 --enable-session")
            with self.lock:
                discard_id = self.saving_episode if self.discard_after_save else None
                self.saving_episode = None
                if discard_id is None:
                    self.home = action == "home" and generation == self.generation and not self.closed
                    self.busy = self.home
                    return
            # Keep BUSY until the exact episode just saved has been marked discarded.
            session.dispatch({"command": "collection.discard_previous", "episode_id": discard_id})
            with self.lock:
                self.busy = False
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.busy = False
                self.saving_episode = None
            print(f"COLLECTION {action} failed: {exc}", flush=True)

    def take_home(self):
        with self.lock:
            if not self.home:
                return False
            self.home = self.busy = False
            return True

    def cancel_home(self):
        with self.lock:
            self.generation += 1
            if self.home:
                self.home = self.busy = False

    @property
    def task_active(self):
        session = self.session()
        return session is not None and session.task_active

    @property
    def recording(self):
        session = self.session()
        return session is not None and session.recording is not None

    def status(self):
        with self.lock:
            session = self.session()
            return {"state": "HOME_WAIT" if self.home else "BUSY" if self.busy else
                    "RECORDING" if session is not None and session.recording is not None else "READY" if self.task_active else "PAUSED" if session is not None and session.task_paused else "IDLE",
                    "error": self.error, "task": self.task}

    def close(self):
        with self.lock:
            self.closed = True
            self.generation += 1
            self.home = False
            thread = self.thread
        if thread is not None:
            thread.join(timeout=12)
