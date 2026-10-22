"""Simple robot operations over the existing kernel; SI units and XYZW poses.

No device SDK is imported here. Motion planning and limits remain in the kernel.
Read-only access acquires no lease; commands share one lazily acquired session.
"""
from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass
import math
from pathlib import Path
import tempfile
import time

from embodied.contracts.runtime.control import ControlClient, ControlSession


@dataclass
class Motion:
    """Started motion. Default calls already waited; wait=False returns it running."""
    robot: Robot
    plan: dict
    operation_id: str
    result: dict | None = None

    @property
    def metrics(self):
        return self.plan["metrics"]

    def wait(self, timeout=None):
        if self.result is not None:
            return self.result
        timeout = _positive(timeout, "timeout") if timeout is not None else self.plan["times"][-1] + 8
        session = self.robot._acquire()
        try:
            self.result = session.wait(self.operation_id, timeout=timeout)
        except BaseException:
            self.robot._cancel_on_error()
            raise
        return self.result


def _positive(value, name):
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _vector(value, size, name):
    result = list(value)
    if len(result) != size or any(isinstance(v, bool) or not math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return [float(v) for v in result]


class Robot:
    """Use ``Robot.sim()`` or ``Robot.real(run)`` with the same motion interface.

    Construction performs no I/O. An explicit run is never started or stopped.
    Motion calls block until feedback settles unless wait=False is requested.
    """
    def __init__(self, run=None, *, backend=None, viewer=False, config=None, speed=None, acceleration=None, jerk=None):
        if backend not in (None, "sim", "real"):
            raise ValueError("backend must be 'sim' or 'real'")
        if backend == "real" and run is None:
            raise ValueError("Real hardware requires an explicit existing runtime directory; use Robot.real(run)")
        if type(viewer) is not bool:
            raise ValueError("viewer must be a boolean")
        if viewer and (run is not None or backend == "real"):
            raise ValueError("viewer=True is only supported when creating a new simulation")
        if config is not None and (run is not None or backend == 'real'):
            raise ValueError('config is only supported when creating a new simulation')
        self._config = config
        self._expected_backend = backend or ("sim" if run is None else None)
        self._viewer = viewer
        self._requested_run = Path(run) if run is not None else None
        self.run = self._requested_run
        self.defaults = {k: v for k, v in dict(speed=speed, acceleration=acceleration, jerk=jerk).items() if v is not None}
        self._client = self._session = self._stack = None
        self.last_motion = None
        self.gripper = Gripper(self)

    @classmethod
    def sim(cls, *, run=None, viewer=False, **options):
        """Create an idle MuJoCo kernel, or attach to an explicit existing simulation.

        viewer=True opens a read-only desktop window for a newly created kernel.
        Closing that window does not stop the script; leaving the context does.
        """
        return cls(run, backend="sim", viewer=viewer, **options)

    @classmethod
    def real(cls, run, **options):
        """Attach to an explicit real runtime, without starting or enabling hardware."""
        return cls(run, backend="real", **options)

    @classmethod
    def connect(cls, run, **options):
        """Connect to one explicit existing simulation or hardware run."""
        if run is None:
            raise ValueError("connect requires an explicit runtime directory")
        return cls(run, **options)

    def __enter__(self):
        if self._stack is not None:
            raise RuntimeError("Robot is already connected")
        stack = ExitStack()
        try:
            if self._requested_run is None:
                from embodied.applications.workstation.control import sim_kernel
                from embodied.core.project import PROJECT_ROOT
                root = PROJECT_ROOT / "outputs/robot/runs"
                root.mkdir(parents=True, exist_ok=True)
                output = Path(tempfile.mkdtemp(prefix="sim-", dir=root))
                options = {'viewer': self._viewer}
                if self._config is not None:
                    options['config'] = self._config
                self.run = stack.enter_context(sim_kernel(output / "runtime.log", **options))
                deadline = time.monotonic() + 10
                while True:
                    try:
                        self._client = ControlClient(self.run)
                        self._check_backend()
                        self._client.status()
                        if not (self.run / "observer.sock").exists():
                            raise FileNotFoundError("observer.sock")
                        break
                    except (OSError, ValueError, RuntimeError, EOFError):
                        if time.monotonic() >= deadline:
                            raise RuntimeError(f"Simulation did not become ready; see {output / 'runtime.log'}")
                        time.sleep(.05)
            else:
                self._client = ControlClient(self.run)
                self._check_backend()
                self._client.status()
            self._stack = stack
            return self
        except BaseException:
            self._client = None
            stack.close()
            raise

    def _check_backend(self):
        actual = self._client.identity.get("backend")
        if actual not in ("sim", "real"):
            raise ValueError(f"Runtime has an unknown backend: {actual!r}")
        if self._expected_backend is not None and actual != self._expected_backend:
            raise ValueError(f"Expected {self._expected_backend!r} backend, found {actual!r}: {self.run}")

    def _connected(self):
        if self._client is None or self._stack is None:
            raise RuntimeError("Use Robot inside a with block")
        return self._client

    def _acquire(self):
        self._connected()
        if self._session is None:
            session = ControlSession(self.run, source="robot-api")
            session.__enter__()
            self._session = session
        return self._session

    def _cancel_on_error(self):
        if self._session is not None:
            try:
                self._session.command("cancel")
            except (OSError, ValueError, RuntimeError, EOFError):
                pass  # Lease expiry remains enforced by the worker.

    def close(self):
        try:
            if self._session is not None:
                self._session.close()
        finally:
            self._session = self._client = None
            stack, self._stack = self._stack, None
            if stack is not None:
                stack.close()

    def __exit__(self, *exc):
        self.close()

    @property
    def state(self):
        return self._connected().status()

    @property
    def backend(self):
        """Verified runtime backend: 'sim' or 'real'."""
        return self._connected().identity["backend"]

    @property
    def simulation(self):
        """Optional scene snapshot/restore and repeatable tasks for a simulation."""
        from .simulation import Simulation
        return Simulation(self)

    @property
    def q(self):
        """Measured joint positions in radians, returned as a fresh list."""
        return list(self.state["q"])

    @property
    def pose(self):
        """Measured base-frame TCP [x, y, z, qx, qy, qz, qw], metres and XYZW."""
        return list(self.state["pose"])

    @property
    def xyz(self):
        """Measured base-frame TCP position in metres, returned as a fresh list."""
        return self.pose[:3]

    def describe(self):
        return self._connected().describe()

    def task(self, name, *, on_event=None):
        """Compose observed task stages; events are saved under this run's tasks/."""
        from .tasks import Task
        return Task(self, name, on_event=on_event)

    def _limits(self, options):
        return {**self.defaults, **{k: v for k, v in options.items() if v is not None}}

    def plan_movej(self, q, *, speed=None, acceleration=None, jerk=None):
        return self._connected().preview("movej", q=_vector(q, len(self.q), "q"),
            **self._limits(dict(speed=speed, acceleration=acceleration, jerk=jerk)))

    def _pose_target(self, pose, xyz, offset, frame):
        if sum(value is not None for value in (pose, xyz, offset)) != 1:
            raise ValueError("Provide exactly one of pose, xyz or offset")
        if frame not in ("base", "tool") or offset is None and frame != "base":
            raise ValueError("Absolute pose/xyz use frame='base'; offsets support 'base' or 'tool'")
        if pose is not None:
            from embodied.motion.contracts import Pose
            values = list(pose)
            if len(values) not in (6, 7):
                raise ValueError("pose must be XYZ + rotation vector (6) or XYZ + XYZW quaternion (7)")
            values = _vector(values, len(values), "pose")
            target = (Pose.from_rotvec(values[:3], values[3:]) if len(values) == 6
                      else Pose(values[:3], values[3:]))
            return target.position_xyz_m.tolist() + target.orientation_xyzw.tolist()
        if xyz is not None:
            return _vector(xyz, 3, "xyz") + self.pose[3:]
        target = self.pose
        delta = _vector(offset, 3, "offset")
        if frame == "tool":
            from embodied.motion.contracts import rotate_vector_xyzw
            delta = rotate_vector_xyzw(target[3:], delta).tolist()
        target[:3] = [p + d for p, d in zip(target[:3], delta)]
        return target

    def plan_movel(self, pose=None, *, xyz=None, offset=None, frame="base", speed=None, acceleration=None, jerk=None,
                   tcp_speed=None, tcp_acceleration=None, tcp_jerk=None,
                   angular_speed=None, angular_acceleration=None, angular_jerk=None):
        return self._connected().preview("movel", pose=self._pose_target(pose, xyz, offset, frame),
            **self._limits(dict(speed=speed, acceleration=acceleration, jerk=jerk,
                tcp_speed=tcp_speed, tcp_acceleration=tcp_acceleration, tcp_jerk=tcp_jerk,
                angular_speed=angular_speed, angular_acceleration=angular_acceleration, angular_jerk=angular_jerk)))

    def plan_move_path(self, waypoints, *, blend_radius=.01, **limits):
        """Preview one continuous Cartesian path. XYZ inherits the preceding orientation.

        The current pose is the implicit start; intermediate corners have the
        requested radius in metres, with zero radius at both endpoints.
        """
        points, orientation = [], self.pose[3:]
        for waypoint in waypoints:
            values = list(waypoint)
            if len(values) == 3:
                pose = _vector(values, 3, 'waypoint') + orientation
            else:
                pose = self._pose_target(values, None, None, 'base')
            points.append(pose)
            orientation = pose[3:]
        return self._connected().preview('move_path', waypoints=points, blend_radius=blend_radius,
                                          **self._limits(limits))

    def move_path(self, waypoints, *, blend_radius=.01, wait=True, timeout=None, **limits):
        """Execute a blended path as one operation, settling only at the final pose."""
        self._acquire()
        return self.execute(self.plan_move_path(waypoints, blend_radius=blend_radius, **limits),
                            wait=wait, timeout=timeout)

    def execute(self, plan, *, wait=True, timeout=None):
        if timeout is not None:
            _positive(timeout, "timeout")
        session = self._acquire()
        try:
            operation = session.execute(plan)
        except BaseException:
            # The worker may have accepted a request whose reply was lost.
            self._cancel_on_error()
            raise
        motion = Motion(self, plan, operation["operation_id"])
        self.last_motion = motion
        if wait:
            motion.wait(timeout)
        return motion

    def movej(self, q, *, wait=True, timeout=None, **limits):
        """Plan and move to absolute joint positions, in radians."""
        self._acquire()
        return self.execute(self.plan_movej(q, **limits), wait=wait, timeout=timeout)

    def movel(self, pose=None, *, xyz=None, offset=None, frame="base", wait=True, timeout=None, **limits):
        """Move along a TCP line. XYZ preserves orientation; pose accepts UR or XYZW.

        Positions are metres, rotation vectors radians. speed/acceleration/jerk
        retain joint units. tcp_speed/tcp_acceleration/tcp_jerk use m/s, m/s²,
        m/s³; angular_speed/angular_acceleration/angular_jerk use rad/s, rad/s²,
        rad/s³. Both TCP and joint limits apply; preview reports the limiting bound.
        """
        self._acquire()
        return self.execute(self.plan_movel(pose, xyz=xyz, offset=offset, frame=frame, **limits), wait=wait, timeout=timeout)

    def home(self, *, release_gripper=False, wait=True, timeout=None, **limits):
        """Return to park_q; release_gripper confirms opening before arm motion.

        With release_gripper and wait=False, advance the returned sequence with
        step(fresh_status), or call its wait(); it does not create a worker.
        """
        if timeout is not None:
            _positive(timeout, 'timeout')
        session = self._acquire()
        if release_gripper:
            from embodied.control.homing import ReleaseThenHome
            self.last_motion = ReleaseThenHome(session, limits=self._limits(limits))
            if wait:
                self.last_motion.wait(timeout)
            return self.last_motion
        return self.execute(session.preview("home", **self._limits(limits)), wait=wait, timeout=timeout)

    def jog(self, joint, delta, *, wait=True, timeout=None, **limits):
        """Move one zero-based joint by delta radians, using a planned trajectory."""
        q = self.q
        if type(joint) is not int or not 0 <= joint < len(q):
            raise ValueError(f"joint must be a zero-based index 0..{len(q) - 1}")
        if isinstance(delta, bool) or not math.isfinite(delta):
            raise ValueError("delta must be finite radians")
        session = self._acquire()
        q[joint] += delta
        return self.execute(session.preview("jog", q=q, **self._limits(limits)), wait=wait, timeout=timeout)

    def wait(self, timeout=None):
        if self.last_motion is None:
            raise RuntimeError("No motion has been started")
        return self.last_motion.wait(timeout)

    def hold(self):
        """Cancel any operation and capture current feedback as the hold target."""
        return self._acquire().command("hold")

    def cancel(self):
        return self._acquire().command("cancel")

    def teach(self):
        """Snapshot a return point; this does not enable free drive."""
        state = self.state
        point = {"model_id": state["model_id"], "q": list(state["q"]),
                 "pose": list(state["pose"]), "gripper_width": state["gripper_width"]}
        if "feedback_monotonic_ns" in state:
            point["feedback_monotonic_ns"] = state["feedback_monotonic_ns"]
        return point

    def return_to(self, point, *, wait=True, timeout=None, **limits):
        if point["model_id"] != self.describe()["model_id"]:
            raise ValueError("Teach point belongs to a different robot model")
        session = self._acquire()
        plan = session.preview("movej", q=point["q"], gripper_width=point["gripper_width"], **self._limits(limits))
        return self.execute(plan, wait=wait, timeout=timeout)

    def record(self, duration, *, output="outputs/datasets/runtime", task="robot trace"):
        """Blocking read-only trace capture; no camera or motion is started."""
        _positive(duration, "duration")
        self._connected()
        from embodied.contracts.runtime import RuntimeReader
        from embodied.data.capture.capture import capture
        with RuntimeReader(self.run, mode="full") as reader:
            path, manifest = capture(reader, Path(output), task=task, duration=duration)
        if manifest["lifecycle"] != "complete" or manifest.get("error"):
            raise RuntimeError(f"Recording failed: {manifest.get('error')}; see {path / 'episode.json'}")
        return path


class Gripper:
    """Positioning and capability-checked contact grasping; effort units are explicit."""
    def __init__(self, robot):
        self.robot = robot

    @property
    def width(self):
        return self.robot.state["gripper_width"]

    def _validate_width(self, width):
        robot = self.robot.describe()['robot']
        capability = robot.get('gripper', {'present': robot['gripper_max_width'] > 0})
        if not capability.get('present'):
            raise ValueError('The connected robot has no gripper')
        maximum = robot['gripper_max_width']
        if isinstance(width, bool) or not math.isfinite(width) or not 0 <= width <= maximum:
            raise ValueError(f"Gripper width must be in 0..{maximum} metres")

    def move(self, width, *, timeout=2., tolerance=.003):
        timeout, tolerance = _positive(timeout, "timeout"), _positive(tolerance, "tolerance")
        self._validate_width(width)
        session = self.robot._acquire()
        deadline, settled_since = time.monotonic() + timeout, None
        try:
            while time.monotonic() < deadline:
                session.command("gripper", gripper_width=width, ttl_ms=200)
                actual = self.width
                now = time.monotonic()
                if abs(actual - width) <= tolerance:
                    settled_since = now if settled_since is None else settled_since
                    if now - settled_since >= .2:
                        return actual
                else:
                    settled_since = None
                time.sleep(.04)
            raise TimeoutError("Gripper did not reach the requested width; contact is not verified")
        except BaseException:
            self.robot._cancel_on_error()
            raise

    def open(self, **options):
        return self.move(self.robot.describe()["robot"]["gripper_max_width"], **options)

    def close(self, **options):
        return self.move(0., **options)

    def grasp(self, *, effort, minimum_width=0., timeout=5., contact_time=.2):
        """Close until contact, travel floor or timeout; return explicit stop_reason.

        effort uses describe().robot.gripper.effort_unit (ARX: motor_Nm; the
        simulator: N per jaw). It is never silently interpreted as fingertip
        force. Contact holds the same closing effort during subsequent arm moves;
        a new gripper position, cancel, release or lost lease clears that effort.
        """
        self._validate_width(minimum_width)
        capability = self.robot.describe()['robot'].get('gripper', {})
        if not capability.get('grasp'):
            raise ValueError('The connected device does not support contact grasping')
        effort, timeout = _positive(effort, 'effort'), _positive(timeout, 'timeout')
        contact_time = _positive(contact_time, 'contact_time')
        if effort > capability['effort_max'] or not contact_time < timeout <= 60:
            raise ValueError('Effort exceeds capability or contact_time < timeout <= 60 is not satisfied')
        session = self.robot._acquire()
        try:
            operation = session.command('grasp', effort=effort, minimum_width=minimum_width,
                                        timeout=timeout, contact_time=contact_time)
            completed = session.wait(operation['operation_id'], timeout=timeout + 1.)
            return {'operation_id': operation['operation_id'], **completed['result']}
        except BaseException:
            self.robot._cancel_on_error()
            raise
