"""Deterministic offline-model and failure-path checks; no robot connection."""
import numpy as np
import pytest

from embodied.contracts.control import RobotState
from embodied.devices.robots.realman.kinematics import MotionCheck


class Model:
    version = 'fake-test-model'
    lower = np.full(6, -180.)
    upper = np.full(6, 180.)
    code = 0
    singular = 0
    broken_fit = False

    def fk(self, q):
        return np.array([.3 + q[0]/1000 + (.01 if self.broken_fit else 0), 0, .4, 0, 0, 0, 1.])

    def solve(self, target, seed):
        q = np.zeros(6); q[0] = (target[0] - .3)*1000
        return self.code, q

    def singularity(self, q):
        return (self.singular if q[0] > .01 else 0), .2


@pytest.fixture
def inputs():
    model = Model()
    state = RobotState(np.zeros(6), np.zeros(6), model.fk(np.zeros(6)), float('nan'), 0)
    target = state.pose.copy(); target[0] += .0002
    return model, state, target


def test_offline_check_accepts_consistent_small_candidate(inputs):
    model, state, target = inputs
    check = MotionCheck({}, state, model=model)
    out = check.check(target, state)
    assert out['max_joint_change_deg'] == pytest.approx(.2)
    assert out['singularity_code'] == 0


@pytest.mark.parametrize('reason', ['fit', 'ik', 'limit', 'singular', 'jump', 'residual', 'nan'])
def test_offline_guard_rejects_invalid_models_and_candidates(inputs, reason):
    model, state, target = inputs
    check = MotionCheck({}, state, model=model)
    if reason == 'fit': model.broken_fit = True
    if reason == 'ik': model.code = 1
    if reason == 'limit': model.upper = np.full(6, .1)
    if reason == 'singular': model.singular = -2
    if reason == 'jump': target[0] += .002
    if reason == 'residual':
        target[0] += .002
        model.solve = lambda target, seed: (0, np.zeros(6))
    if reason == 'nan': model.solve = lambda target, seed: (0, np.full(6, np.nan))
    with pytest.raises((RuntimeError, ValueError)):
        check.check(target, state)
