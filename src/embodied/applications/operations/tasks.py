"""Readable task stages over Robot; no device SDK or motion controller lives here.

Each movement waits for the existing kernel operation. A failed stage stops the
task, even if its exception is caught by the caller. Events are written outside
the realtime loop, under ``<run>/tasks/<task_id>/``.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import json
import logging
import math
from pathlib import Path
import time
from uuid import uuid4

from embodied.core.storage import save_json
from .robot import _vector


class TaskFailed(RuntimeError):
    """A task stage failed; later stages cannot execute."""
    def __init__(self, message, *, result=None):
        super().__init__(message)
        self.result = result


class GraspUnconfirmed(TaskFailed):
    """Closing the gripper did not provide evidence of a successful grasp."""


@dataclass(frozen=True)
class Observation:
    """Detector output: base-frame TCP pose (UR or XYZW), identity and evidence."""
    pose: list[float]
    target_id: str
    evidence: dict = field(default_factory=dict)


@dataclass(frozen=True)
class GraspCheck:
    """Explicit task evidence; a gripper width command alone is not confirmation."""
    confirmed: bool
    reason: str
    evidence: dict = field(default_factory=dict)

    @classmethod
    def contact(cls, *, stop_reason, width, maximum_width):
        """Reuse the cube task's contact-stop + plausible measured-width criterion.

        The caller supplies actual device/task feedback. This helper neither
        sends a force command nor infers contact from position alone.
        """
        if (isinstance(maximum_width, bool) or not math.isfinite(maximum_width)
                or maximum_width <= 0):
            raise ValueError("maximum_width must be finite and positive")
        if isinstance(width, bool) or not math.isfinite(width) or width < 0:
            raise ValueError("Measured width must be finite and non-negative")
        confirmed = stop_reason == "contact" and width <= maximum_width
        return cls(confirmed, "contact confirmed" if confirmed else "contact not confirmed",
                   {"stop_reason": stop_reason, "width_m": width, "maximum_width_m": maximum_width})


class Task:
    """Use ``with robot.task('pick and place') as task: ...``.

    ``on_event(event)`` receives JSON-compatible stage events. ``result`` is a
    snapshot of execution status, not an assertion that a scene goal succeeded.
    Motion limits have exactly the same meaning as Robot.movel's limits.
    """
    def __init__(self, robot, name, *, on_event=None):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Task name must be a non-empty string")
        if on_event is not None and not callable(on_event):
            raise TypeError("on_event must be callable")
        self.robot, self.on_event = robot, on_event
        self._data = {"schema_version": 1, "task_id": uuid4().hex, "task_name": name,
                      "status": "pending", "started_unix_ns": None, "started_monotonic_ns": None,
                      "finished_unix_ns": None, "finished_monotonic_ns": None,
                      "phases": [], "error": None}
        self.directory = None

    @property
    def result(self):
        return deepcopy(self._data)

    def __enter__(self):
        if self._data["status"] != "pending":
            raise RuntimeError("Task contexts cannot be reused")
        self.robot._connected()
        self.directory = Path(self.robot.run) / "tasks" / self._data["task_id"]
        self.directory.mkdir(parents=True, exist_ok=False)
        self._data.update(status="running", started_unix_ns=time.time_ns(),
                          started_monotonic_ns=time.monotonic_ns())
        self._save()
        return self

    def __exit__(self, exc_type, exc, traceback):
        if exc is not None:
            self._fail(exc)
        elif self._data["status"] == "running":
            self._data["status"] = "completed"
        self._data.update(finished_unix_ns=time.time_ns(), finished_monotonic_ns=time.monotonic_ns())
        self._save()
        if exc is None and self._data["status"] == "failed":
            raise TaskFailed(self._data["error"])
        return False

    def _save(self):
        save_json(self.directory / "result.json", self._data)

    def _fail(self, error):
        self._data["status"] = "failed"
        if self._data["error"] is None:
            self._data["error"] = f"{type(error).__name__}: {error}"
        self.robot._cancel_on_error()

    def _event(self, index, phase, status, started, *, result=None, error=None, notify=True):
        now = time.monotonic_ns()
        event = {"schema_version": 1, "task_id": self._data["task_id"],
                 "task_name": self._data["task_name"], "phase_index": index, "phase": phase,
                 "status": status, "time_unix_ns": time.time_ns(), "host_monotonic_ns": now,
                 "elapsed_s": (now - started) / 1e9,
                 "operation_id": result.get("operation_id") if result else None,
                 "result": result, "error": error}
        encoded = json.dumps(event, ensure_ascii=False, allow_nan=False)
        with (self.directory / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
            stream.flush()
        if index == len(self._data["phases"]):
            self._data["phases"].append(event)
        else:
            self._data["phases"][index] = event
        self._save()
        if notify and self.on_event is not None:
            try:
                self.on_event(deepcopy(event))
            except Exception:
                logging.getLogger(__name__).exception("Task event callback failed; the event is saved on disk")

    def _step(self, phase, action):
        if self._data["status"] != "running":
            raise TaskFailed(f"Task is {self._data['status']}; no further stages can run")
        index, started = len(self._data["phases"]), time.monotonic_ns()
        try:
            self._event(index, phase, "started", started)
            result = action()
            self._event(index, phase, "completed", started, result=result)
            return result
        except BaseException as exc:
            self._fail(exc)
            self._event(index, phase, "failed", started, result=getattr(exc, "result", None),
                        error=f"{type(exc).__name__}: {exc}")
            raise

    def _move(self, phase, pose=None, **options):
        def move():
            motion = self.robot.movel(pose, wait=False, **options)
            result = motion.wait(timeout=options.get("timeout"))
            return {"operation_id": motion.operation_id, "motion": result}
        return self._step(phase, move)

    def _target(self, pose, xyz):
        return self.robot._pose_target(pose, xyz, None, "base")

    @staticmethod
    def _offset(pose, offset):
        delta = _vector(offset, 3, "offset")
        return [p + d for p, d in zip(pose[:3], delta)] + list(pose[3:])

    def approach(self, pose=None, *, xyz=None, offset=(0., 0., .05), **motion):
        """Move to target + base-frame offset, keeping the requested orientation."""
        target = self._target(pose, xyz)
        return self._move("approach", self._offset(target, offset), **motion)

    def retreat(self, offset=(0., 0., .05), *, frame="base", **motion):
        """Move away from the measured current pose using Robot's offset semantics."""
        return self._move("retreat", offset=offset, frame=frame, **motion)

    def _observe(self, observe, target_id=None):
        def detect():
            observation = observe(self.robot)
            if not isinstance(observation, Observation):
                raise TypeError("observe(robot) must return Observation")
            if not observation.target_id:
                raise ValueError("Observation requires a target_id")
            if target_id is not None and observation.target_id != target_id:
                raise TaskFailed("The target changed after approach")
            return {"pose": self._target(observation.pose, None),
                    "target_id": observation.target_id, "evidence": observation.evidence}
        return self._step("observe", detect)

    def _gripper(self, phase, width):
        def move():
            actual = self.robot.gripper.open() if width is None else self.robot.gripper.move(width)
            return {"requested_width_m": width, "actual_width_m": actual}
        return self._step(phase, move)

    def pick(self, pose=None, *, xyz=None, width, verify_grasp, observe=None,
             approach_offset=(0., 0., .05), open_width=None, **motion):
        """Approach, optionally re-observe, descend, close, verify, then lift.

        Supply either pose/xyz or ``observe(robot) -> Observation``. With an
        observer, detection runs before and after approach and must retain the
        same target_id. ``verify_grasp(robot) -> GraspCheck`` is mandatory;
        closing to ``width`` only establishes gripper position, never contact.
        """
        if not callable(verify_grasp):
            raise TypeError("verify_grasp must be callable")
        if observe is not None and (not callable(observe) or pose is not None or xyz is not None):
            raise ValueError("Use either observe or pose/xyz, not both")
        self.robot.gripper._validate_width(width)
        if open_width is not None:
            self.robot.gripper._validate_width(open_width)
        offset = _vector(approach_offset, 3, "approach_offset")
        first = self._observe(observe) if observe is not None else None
        target = first["pose"] if first else self._target(pose, xyz)
        self._gripper("open", open_width)
        self.approach(target, offset=offset, **motion)
        if observe is not None:
            target = self._observe(observe, first["target_id"])["pose"]
            # Re-align above the refined target before descending vertically.
            self.approach(target, offset=offset, **motion)
        self._move("descend", target, **motion)
        self._gripper("grasp", width)

        def verify():
            check = verify_grasp(self.robot)
            if not isinstance(check, GraspCheck) or type(check.confirmed) is not bool:
                raise TypeError("verify_grasp(robot) must return GraspCheck with a boolean confirmed")
            result = {"confirmed": check.confirmed, "reason": check.reason, "evidence": check.evidence}
            if not check.confirmed:
                raise GraspUnconfirmed(check.reason, result=result)
            return result
        self._step("verify_grasp", verify)
        return self._move("lift", self._offset(target, offset), **motion)

    def place(self, pose=None, *, xyz=None, approach_offset=(0., 0., .05),
              open_width=None, **motion):
        """Approach the release pose, descend, open, then retreat above it."""
        if open_width is not None:
            self.robot.gripper._validate_width(open_width)
        target = self._target(pose, xyz)
        offset = _vector(approach_offset, 3, "approach_offset")
        self.approach(target, offset=offset, **motion)
        self._move("place", target, **motion)
        self._gripper("release", open_width)
        return self._move("retreat", self._offset(target, offset), **motion)
