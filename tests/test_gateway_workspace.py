"""Workspace gate: streaming joint targets follow the native teleop rules.

Native Quest teleoperation never runs a table-plane path check, and collection
recorded the same commands pressing into the table; a policy stream must keep
the same freedom. Planning commands stay checked in MotionPlanner.
"""
import threading
from types import SimpleNamespace

import pytest

from embodied.control.gateway import Gateway


class FakeWorkspace:
    def __init__(self):
        self.pose_checks = 0
        self.path_checks = 0

    def check_pose(self, pose):
        self.pose_checks += 1
        if pose[2] < 0:
            raise ValueError(f'TCP target z={pose[2]:.3f} m is below the table plane 0.000 m')

    def check_path(self, fk, qs):
        self.path_checks += 1
        raise ValueError('TCP path reaches z=-0.010 m, below the table plane 0.000 m')


def gateway(workspace):
    gw = Gateway.__new__(Gateway)
    gw.planner = SimpleNamespace(workspace=workspace, model=SimpleNamespace(fk=lambda q: [0., 0., -0.01, 0., 0., 0., 1.]))
    gw.config = SimpleNamespace(park_q=(0.,) * 6)
    gw.plan_lock = threading.RLock()
    return gw


def test_streamed_joint_target_below_the_table_is_accepted():
    workspace = FakeWorkspace()
    gateway(workspace)._check_workspace('joint', {'q': [0.] * 6})
    assert workspace.path_checks == 0


def test_streamed_cartesian_target_below_the_table_is_still_refused():
    workspace = FakeWorkspace()
    with pytest.raises(ValueError, match='below the table plane'):
        gateway(workspace)._check_workspace('cartesian', {'pose': [0., 0., -0.02, 0., 0., 0., 1.]})
    assert workspace.pose_checks == 1


def test_malformed_joint_target_is_left_to_the_command_engine():
    gateway(FakeWorkspace())._check_workspace('joint', {'q': ['bad']})
