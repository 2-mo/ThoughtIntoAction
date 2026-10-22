"""Physical presses and episode transitions without robot/camera hardware."""
import threading
import unittest
from unittest.mock import Mock
from embodied.devices.inputs.gestures import Press, StickAction
from legacy.quest_teleop.collection import EpisodeActions
from legacy.quest_teleop.runtime.session import TeleopSession


class PressTests(unittest.TestCase):
    def events(self, button, samples):
        return [event for now, pressed in samples if (event := button.update(pressed, now))]

    def test_short_release_bounce_and_repeat(self):
        for long_s in (None, 1.):
            button = Press(long_s)
            events = self.events(button, [(0, False), (.1, True), (.11, False), (.12, True),
                (.16, True), (.3, False), (.31, True), (.32, False), (.36, False),
                (.4, False), (.5, True), (.54, True), (.6, False), (.64, False)])
            self.assertEqual(events, ["short", "short"])

    def test_long_stick_click_never_also_short_or_repeated(self):
        self.assertEqual(self.events(Press(1.), [(0, False), (.1, True), (.14, True),
            (1.1, True), (2., True), (3., True), (3.1, False), (3.14, False)]), ["long"])

    def test_held_b_emits_only_one_discard_release(self):
        self.assertEqual(self.events(Press(), [(0, False), (.1, True), (.14, True),
            (1.1, True), (3., True), (3.1, False), (3.14, False), (4., False)]), ["short"])

    def test_press_started_while_busy_cannot_trigger_after_busy_finishes(self):
        button = Press()
        button.update(False, 0)
        button.update(True, .1, enabled=False)
        button.update(True, .14, enabled=True)
        self.assertIsNone(button.update(False, .2))
        self.assertIsNone(button.update(False, .24))

    def test_release_near_threshold_and_missing_poll(self):
        self.assertEqual(self.events(Press(1.), [(0, False), (.1, True), (.14, True),
            (1.09, False), (1.14, False)]), ["short"])
        self.assertEqual(self.events(Press(1.), [(0, False), (.1, True), (.14, True),
            (1.2, False), (1.24, False)]), ["long"])

    def test_held_on_start_is_ignored_until_release(self):
        self.assertEqual(self.events(Press(1.), [(0, True), (.1, True), (2., True),
            (2.1, False), (2.14, False), (3., True), (3.04, True),
            (3.2, False), (3.24, False)]), ["short"])


class StickTests(unittest.TestCase):
    def test_directions_fire_immediately_once(self):
        for axes, action in [([-1, 0], "start_task"), ([1, 0], "pause_task"), ([0, 1], "discard_previous")]:
            stick = StickAction()
            self.assertIsNone(stick.update(axes, 0))  # Held at startup.
            stick.update([0, 0], .1)
            self.assertEqual(stick.update(axes, .11), action)
            self.assertIsNone(stick.update(axes, .12))
            self.assertIsNone(stick.update(axes, 2))
            stick.update([0, 0], 3)
            self.assertEqual(stick.update(axes, 3.01), action)

    def test_crossing_to_other_direction_requires_neutral(self):
        stick = StickAction()
        stick.update([0, 0], 0)
        self.assertEqual(stick.update([-1, 0], .01), "start_task")
        self.assertIsNone(stick.update([1, 0], .02))
        stick.update([0, 0], .03)
        self.assertEqual(stick.update([1, 0], .04), "pause_task")

    def test_down_diagonal_invalid_and_busy_require_neutral(self):
        for axes, enabled in [([0, -1], True), ([1, .8], True), ([float("nan"), 0], True), ([-1, 0], False)]:
            stick = StickAction()
            stick.update([0, 0], 0)
            self.assertIsNone(stick.update(axes, .01, enabled=enabled))
            self.assertIsNone(stick.update([-1, 0], .02))
            stick.update([0, 0], .03)
            self.assertEqual(stick.update([-1, 0], .04), "start_task")

    def test_small_drift_does_not_trigger(self):
        stick = StickAction()
        stick.update([0, 0], 0)
        self.assertIsNone(stick.update([.4, 0], .01))
        self.assertEqual(stick.update([.9, 0], .02), "pause_task")


class ActionsTests(unittest.TestCase):
    def make_session(self):
        session = TeleopSession.__new__(TeleopSession)
        session.operations = threading.Lock()
        session.closed = False
        session.recording = Mock()
        session.task_active = True
        session.task_paused = False
        def stop(outcome):
            session.recording = None
            return {"lifecycle": "complete"}
        session._stop_recording = Mock(side_effect=stop)
        return session

    def test_a_saves_and_b_discards_current_episode(self):
        for action, outcome in (("stop_episode", "unknown"), ("discard", "discarded"), ("home", "discarded")):
            session = self.make_session()
            session.dispatch({"command": "collection." + action})
            session._stop_recording.assert_called_once_with(outcome)
            self.assertIsNone(session.recording)

    def test_discard_in_ready_is_noop(self):
        session = self.make_session()
        session.recording = None
        session.dispatch({"command": "collection.discard"})
        session._stop_recording.assert_not_called()

    def test_home_waits_for_discard_and_busy_press_is_not_queued(self):
        entered, finish = threading.Event(), threading.Event()
        session = Mock()
        def dispatch(payload):
            entered.set()
            self.assertTrue(finish.wait(2))
        session.dispatch.side_effect = dispatch
        actions = EpisodeActions(lambda: session, "pick")
        self.addCleanup(actions.close)
        self.assertTrue(actions.submit("home"))
        self.assertTrue(entered.wait(1))
        self.assertFalse(actions.take_home())
        self.assertFalse(actions.submit("stop_episode"))
        finish.set()
        actions.thread.join(1)
        self.assertTrue(actions.take_home())
        self.assertFalse(actions.take_home())
        session.dispatch.assert_called_once_with({"command": "collection.home", "task": "pick"})

    def test_failure_does_not_request_home(self):
        session = Mock()
        session.dispatch.side_effect = RuntimeError("disk error")
        actions = EpisodeActions(lambda: session, "pick")
        self.addCleanup(actions.close)
        actions.submit("home")
        actions.thread.join(1)
        self.assertFalse(actions.take_home())
        self.assertEqual(actions.error, "disk error")

    def test_b_during_save_is_applied_after_save_once(self):
        entered, finish = threading.Event(), threading.Event()
        session = Mock()
        session.recording.id = "just-saved"
        calls = []
        def dispatch(payload):
            calls.append(payload)
            if payload["command"] == "collection.stop_episode":
                entered.set()
                self.assertTrue(finish.wait(2))
        session.dispatch.side_effect = dispatch
        actions = EpisodeActions(lambda: session, "pick")
        self.addCleanup(actions.close)
        actions.submit("stop_episode")
        self.assertTrue(entered.wait(1))
        self.assertTrue(actions.submit("discard_previous"))
        self.assertTrue(actions.submit("discard_previous"))
        self.assertFalse(actions.submit("stop_episode"))
        finish.set()
        actions.thread.join(1)
        self.assertEqual([p["command"] for p in calls], ["collection.stop_episode", "collection.discard_previous"])
        self.assertEqual(calls[1]["episode_id"], "just-saved")
        self.assertFalse(actions.busy)

    def test_failed_save_does_not_discard_older_episode(self):
        entered, finish = threading.Event(), threading.Event()
        session = Mock()
        def dispatch(payload):
            entered.set()
            finish.wait(2)
            raise RuntimeError("save failed")
        session.dispatch.side_effect = dispatch
        actions = EpisodeActions(lambda: session, "pick")
        self.addCleanup(actions.close)
        actions.submit("stop_episode")
        self.assertTrue(entered.wait(1))
        actions.submit("discard_previous")
        finish.set()
        actions.thread.join(1)
        session.dispatch.assert_called_once()
        self.assertEqual(actions.error, "save failed")

    def test_pause_cancels_delayed_home(self):
        entered, finish = threading.Event(), threading.Event()
        session = Mock()
        session.dispatch.side_effect = lambda payload: (entered.set(), finish.wait(2))
        actions = EpisodeActions(lambda: session, "pick")
        self.addCleanup(actions.close)
        actions.submit("home")
        self.assertTrue(entered.wait(1))
        actions.cancel_home()
        finish.set()
        actions.thread.join(1)
        self.assertFalse(actions.take_home())
