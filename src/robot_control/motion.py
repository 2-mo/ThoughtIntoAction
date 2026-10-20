"""Execute bounded joint trajectories through an injected arm, without owning devices."""

import time

import numpy as np

from robot_control.trajectory import trajectory


def move_and_settle(arm, robot, target, settings, *, bounds=None, clock=time.monotonic, sleep=time.sleep):
    """No automatic return-to-home on success, error or cancellation."""
    def checked_snapshot():
        state = arm.snapshot()
        actual = robot.joints(state["q"])
        if bounds is not None:
            box = np.asarray(bounds)
            point = robot.fk(actual)[:3, 3]
            if np.any(point < box[:, 0]) or np.any(point > box[:, 1]):
                raise RuntimeError("Measured EEF left the configured workspace")
        return state
    start_q = checked_snapshot()["q"]
    times, qs = trajectory(robot, start_q, target, **settings)
    if bounds is not None:
        box = np.asarray(bounds)
        points = np.asarray([robot.fk(q)[:3, 3] for q in qs])
        if np.any(points < box[:, 0]) or np.any(points > box[:, 1]):
            raise ValueError("Trajectory from measured start leaves the configured workspace")
    began = clock()
    for deadline, q in zip(times, qs):
        sleep(max(0., began + deadline - clock()))
        if clock() - (began + deadline) > 0.15:
            raise RuntimeError("Motion loop missed its deadline; aborting")
        actual = robot.joints(checked_snapshot()["q"])
        if np.max(np.abs(actual - q)) > 0.12:
            raise RuntimeError("Joint tracking error exceeds 0.12 rad")
        arm.command(q)
    deadline, stable_since = clock() + 6, None
    while clock() < deadline:
        state = checked_snapshot()
        error = np.max(np.abs(np.asarray(state["q"]) - target))
        if error > 0.12:
            raise RuntimeError("Joint tracking error while settling")
        arm.command(target)
        if error < 0.015 and np.max(np.abs(state["dq"])) < 0.02:
            stable_since = clock() if stable_since is None else stable_since
            if clock() - stable_since >= 0.5:
                return
        else:
            stable_since = None
        sleep(1 / settings["hz"])
    raise RuntimeError("Robot failed to settle; sample blocked")
