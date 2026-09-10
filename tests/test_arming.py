import unittest
from quest_teleop.arming import DownHoldGate


class ArmingTests(unittest.TestCase):
    def setUp(self):
        self.gate = DownHoldGate()

    def step(self, axes, now, released=True):
        return self.gate.update(axes, now, released=released)

    def hold(self, now):
        self.assertFalse(self.step((0, 0), now))
        self.assertFalse(self.step((0, -1), now + .25))
        self.assertFalse(self.step((0, -1), now + 1.24))
        return self.step((0, -1), now + 1.25)

    def test_one_second_hold_fires_once_and_requires_center_before_next(self):
        self.assertTrue(self.hold(0))
        for t in (1.5, 2, 3, 4):
            self.assertFalse(self.step((0, -1), t))
        self.assertTrue(self.hold(5))

    def test_held_on_start_does_not_count(self):
        for t in (0, .1, .5, 1, 2):
            self.assertFalse(self.step((0, -1), t))
        self.assertTrue(self.hold(3))

    def test_short_holds_do_not_accumulate(self):
        for start in (0, 1, 2):
            self.step((0, 0), start)
            self.assertFalse(self.step((0, -1), start + .1))
            self.assertFalse(self.step((0, -1), start + .9))
        self.assertTrue(self.hold(3))

    def test_blocked_buttons_cancel_hold_and_require_center(self):
        self.step((0, 0), 0)
        self.step((0, -1), .25)
        self.assertFalse(self.step((0, -1), 1, released=False))
        for t in (1.25, 2, 3):
            self.assertFalse(self.step((0, -1), t))
        self.assertTrue(self.hold(4))

    def test_noise_diagonal_and_wrong_direction_cancel_hold(self):
        for axes in ((1, 0), (-1, 0), (0, 1), (.8, -1),
                     (float('nan'), -1), (0, float('inf'))):
            with self.subTest(axes=axes):
                self.step((0, 0), 0)
                self.step((0, -1), .25)
                self.assertFalse(self.step(axes, .9))
                self.assertFalse(self.step((0, -1), 1.25))
                self.assertFalse(self.step((0, -1), 3))
                self.assertTrue(self.hold(4))

    def test_gradual_downward_travel_and_partial_release_restart_timer(self):
        self.step((0, 0), 0)
        self.assertFalse(self.step((0, -.4), .1))
        self.assertFalse(self.step((0, -.7), .2))
        self.assertFalse(self.step((0, -1), .25))
        self.assertFalse(self.step((0, -.5), 1))
        self.assertFalse(self.step((0, -1), 1.25))
        self.assertFalse(self.step((0, -1), 2.24))
        self.assertTrue(self.step((0, -1), 2.25))

    def test_explicit_reset_discards_pending_hold(self):
        self.step((0, 0), 0)
        self.step((0, -1), .25)
        self.gate.reset()
        self.assertFalse(self.step((0, -1), 2))
        self.assertTrue(self.hold(3))
