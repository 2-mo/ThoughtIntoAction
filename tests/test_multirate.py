import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
from quest_teleop.multirate import camera_rows, save_robot_sidecar

class MultirateTests(unittest.TestCase):
    def test_camera_anchors_align_training_actions_and_archive_full_rate(self):
        origin=4552089789885210
        states=[]
        for i in range(241):
            states.append(dict(host_monotonic_ns=origin+round(i*1e9/240),
                requested_monotonic_ns=origin+round(i*1e9/240)-10,
                actual_pos=[i*.001]*6, gripper_actual_pos=.04,
                requested_pos=[i*.002]*6, requested_gripper_pos=.03))
        frames=[dict(host_received_monotonic_ns=origin+round(i*1e9/30)) for i in range(31)]
        rows=camera_rows(states,frames,Path("."))
        self.assertEqual(len(rows),31)
        self.assertEqual(rows[1][-1]['host_monotonic_ns'],states[8]['host_monotonic_ns'])
        with tempfile.TemporaryDirectory() as root:
            save_robot_sidecar(Path(root)/"robot/episode_000000.npz",states)
            data=np.load(Path(root)/"robot/episode_000000.npz")
            self.assertEqual(len(data["state"]),241)
            self.assertEqual(data["timestamp_ns"][0],origin)
            np.testing.assert_allclose(data["action"][:,0],np.arange(241)*.002)
        np.testing.assert_allclose([row[1][0] for row in rows], np.arange(31)*8*.002, atol=1e-7)
