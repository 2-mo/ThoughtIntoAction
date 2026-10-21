"""Experimental X5 quantile mapping; not evidence of base-checkpoint compatibility."""
import json
from pathlib import Path
import numpy as np

SCHEMA = 'x5_delta6_absolute_gripper_quantiles_v1'


def action_chunks(states, actions, horizon):
    states, actions = np.asarray(states), np.asarray(actions)
    if states.shape != actions.shape or states.ndim != 2 or states.shape[1] != 7 or not len(states):
        raise ValueError('Expected matching nonempty Nx7 state/action arrays')
    if horizon < 1 or not np.isfinite(states).all() or not np.isfinite(actions).all():
        raise ValueError('Invalid horizon or nonfinite data')
    indices = np.minimum(np.arange(len(states))[:, None] + np.arange(horizon), len(states) - 1)
    chunks = actions[indices].copy()
    chunks[..., :6] -= states[:, None, :6]  # Anchor state, not the preceding action.
    return chunks


class X5Mapping:
    def __init__(self, path):
        self.data = json.loads(Path(path).read_text())
        if self.data.get('schema') != SCHEMA or self.data.get('fps') != 30:
            raise ValueError('Unsupported X5 statistics format')
        self.bounds = {}
        for key in ('state', 'action'):
            stat = self.data[key]
            lo, hi = np.asarray(stat['q01'], dtype=float), np.asarray(stat['q99'], dtype=float)
            if lo.shape != (7,) or hi.shape != (7,) or not np.isfinite([lo, hi]).all() or np.any(hi < lo):
                raise ValueError('Invalid quantile bounds')
            self.bounds[key] = lo, hi

    def state_excess(self, state):
        state = np.asarray(state, dtype=float)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('Expected finite X5 state')
        lo, hi = self.bounds['state']
        normalized = 2 * (state - lo) / (hi - lo + 1e-6) - 1
        return float(np.maximum(np.abs(normalized) - 1, 0).max())

    def normalize_state(self, state):
        state = np.asarray(state, dtype=float)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('Expected finite X5 state')
        lo, hi = self.bounds['state']
        normalized = 2 * (state - lo) / (hi - lo + 1e-6) - 1
        outside = np.flatnonzero((normalized < -1) | (normalized > 1)).tolist()
        return np.pad(np.clip(normalized, -1, 1), (0, 25)).astype(np.float32), outside

    def absolute_actions(self, actions, state):
        actions, state = np.asarray(actions), np.asarray(state)
        if actions.ndim != 2 or actions.shape[1] != 32 or state.shape != (7,):
            raise ValueError('Expected Tx32 model actions and 7-state')
        if not np.isfinite(actions).all() or not np.isfinite(state).all():
            raise ValueError('Nonfinite model result')
        lo, hi = self.bounds['action']
        # OpenPI quantile inverse. Preserve out-of-range predictions for inspection.
        decoded = (actions[:, :7] + 1) / 2 * (hi - lo + 1e-6) + lo
        decoded[:, :6] += state[:6]
        return decoded
