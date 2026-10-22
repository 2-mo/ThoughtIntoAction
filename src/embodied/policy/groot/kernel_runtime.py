"""GR00T on the arx kernel (todo P-08).

Reuses runtime.py's action decoding, per-step jump limits, workspace projection and scheduling;
state comes from the kernel and each target goes to the gateway as a Cartesian stream at 30 Hz.
Joint limits, the table plane, the 0.15 m lead and the step limit are the kernel's. The kernel
gripper is position controlled: OPEN targets the training open width and CLOSE targets 0 m,
instead of the legacy servo's 0.15 Nm torque-limited close (check during real-arm acceptance).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time
from typing import Any, Callable

import numpy as np

from legacy.arx5_control.cartesian import quaternion_xyzw_to_rpy
from legacy.arx5_control.teleoperation import GripperState, load_teleoperation_config
from embodied.devices.robots.arx5.paths import CONFIG_ROOT
from embodied.policy.groot.checkpoint import STATE_KEYS, VIDEO_KEYS
from embodied.policy.groot.runtime import (
    ACTION_PERIOD_NS, EXECUTION_HZ, REPLAN_STEPS, TRAINING_OPEN_GRIPPER_WIDTH_M, ActionRejected, ActionTarget,
    RuntimeFault, ScheduledActionTarget, build_observation, decode_action_chunk, limit_scheduled_actions)


@dataclass(frozen=True)
class KernelSnapshot:
    """The fields build_observation and decode_action_chunk read, from a kernel status."""
    eef_pose_6d: np.ndarray
    joint_position: np.ndarray
    gripper_position: float


def kernel_snapshot(session) -> KernelSnapshot:
    status = session.status()
    pose = np.asarray(status['pose'], dtype=np.float64)  # base TCP [x, y, z, qx, qy, qz, qw]
    return KernelSnapshot(np.r_[pose[:3], quaternion_xyzw_to_rpy(pose[3:])],
                          np.asarray(status['q'], dtype=np.float64), float(status['gripper_width']))


def gripper_width(target: ActionTarget) -> float:
    return TRAINING_OPEN_GRIPPER_WIDTH_M if target.gripper_state == GripperState.OPEN else 0.0


def run(session, worker, cameras, task: str, *, seconds: float, poll_command: Callable[[], str | None] = lambda: None,
        video_keys: tuple[str, ...] = VIDEO_KEYS, state_keys: tuple[str, ...] = STATE_KEYS,
        workspace: Any = None) -> dict[str, Any]:
    """Observe, infer, stream scheduled targets; between chunks the last target is repeated, so the
    kernel keeps following it, and when nothing is sent the kernel holds."""
    workspace = workspace or load_teleoperation_config(CONFIG_ROOT / "teleoperation.json").workspace
    start = kernel_snapshot(session)
    gripper_state = GripperState.OPEN if start.gripper_position >= .010 else GripperState.CLOSE
    last_target = ActionTarget(start.eef_pose_6d[:3], session.status()['pose'][3:], gripper_state, start.gripper_position)
    queue: deque[ScheduledActionTarget] = deque()
    summary = {'status': 'running', 'task': task, 'chunks': 0, 'rejected_chunks': 0, 'commands': 0,
               'kernel_runtime_id': session.identity['runtime_id']}
    pending, next_action_at, deadline = False, time.monotonic(), time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            if (command := poll_command()) is not None:
                summary.update(status='stopped', command=command)
                break
            snapshot = kernel_snapshot(session)
            result = worker.poll()
            if result is not None:
                pending = False
                if result.error is not None or result.actions is None:
                    raise RuntimeFault(result.error or 'Policy returned no actions')
                try:
                    targets = decode_action_chunk(result.actions, current_pose_6d=snapshot.eef_pose_6d, workspace=workspace,
                                                  previous_gripper_state=gripper_state, latency_s=0.0,
                                                  current_gripper_m=snapshot.gripper_position)
                    execute_from = time.monotonic_ns()
                    queue = limit_scheduled_actions(
                        deque(ScheduledActionTarget(execute_from + (i + 1) * ACTION_PERIOD_NS, target)
                              for i, target in enumerate(targets[:REPLAN_STEPS])),
                        current_pose_6d=snapshot.eef_pose_6d, workspace=workspace)
                    summary['chunks'] += 1
                except ActionRejected:
                    summary['rejected_chunks'] += 1
            if not pending and not queue:
                worker.submit(build_observation(cameras.latest(), snapshot, task, video_keys, state_keys),
                              time.monotonic_ns(), None)
                pending = True
            now = time.monotonic()
            if now >= next_action_at:
                due = time.monotonic_ns() + ACTION_PERIOD_NS // 2
                while queue and queue[0].execute_at_ns <= due:
                    last_target = queue.popleft().target
                gripper_state = last_target.gripper_state
                session.command('cartesian', pose=[*map(float, last_target.xyz_m), *map(float, last_target.quaternion_xyzw)],
                                gripper_width=gripper_width(last_target), ttl_ms=120, sent_monotonic_ns=time.monotonic_ns())
                summary['commands'] += 1
                next_action_at = max(next_action_at + 1 / EXECUTION_HZ, now + .001)
            time.sleep(.005)
        else:
            summary['status'] = 'time_limit'
    except KeyboardInterrupt:
        summary['status'] = 'interrupted'
    finally:
        try:
            session.command('hold')
        except Exception as exc:
            summary['hold_error'] = str(exc)
    return summary
