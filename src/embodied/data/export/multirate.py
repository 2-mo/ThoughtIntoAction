"""30 FPS observation anchors with a lossless 240 Hz robot sidecar."""
import numpy as np


def camera_rows(states, frames, path):
    clocks = np.asarray([row['host_monotonic_ns'] for row in states], dtype=np.int64)
    selected = []
    for frame in frames:
        stamp = frame['host_received_monotonic_ns']
        index = int(np.searchsorted(clocks, stamp, side='right') - 1)
        if index < 0 or stamp - clocks[index] > 100_000_000:
            continue  # Training-only exclusion; source archive retains every frame.
        state = states[index]
        selected.append((np.asarray([*state['actual_pos'], state['gripper_actual_pos']], np.float32),
                         np.asarray([*state['requested_pos'], state['requested_gripper_pos']], np.float32),
                         path, {**frame, 'wrist_valid': True}, state))
    return selected


def save_robot_sidecar(path, states):
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path,
        timestamp_ns=np.asarray([s['host_monotonic_ns'] for s in states], np.int64),
        action_timestamp_ns=np.asarray([s['requested_monotonic_ns'] for s in states], np.int64),
        state=np.asarray([[*s['actual_pos'], s['gripper_actual_pos']] for s in states], np.float64),
        action=np.asarray([[*s['requested_pos'], s['requested_gripper_pos']] for s in states], np.float64))
