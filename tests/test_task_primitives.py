"""Task failure propagation and recorded evidence, using the real Motion.wait API."""
import json
from pathlib import Path
import tempfile
import unittest

from embodied import Robot
from embodied.applications.operations.robot import Gripper, Motion
from embodied.applications.operations.tasks import GraspCheck, GraspUnconfirmed, Observation, TaskFailed


class FakeGripper:
    _validate_width = Gripper._validate_width

    def __init__(self, robot):
        self.robot = robot

    def open(self):
        return self.move(.08)

    def move(self, width):
        self._validate_width(width)
        self.robot.calls.append(("gripper", width))
        return width


class FakeRobot:
    task = Robot.task
    _pose_target = Robot._pose_target

    def __init__(self, run):
        self.run = Path(run)
        self.backend = "sim"
        self.pose = [.3, 0., .25, 0., 0., 0., 1.]
        self.calls = []
        self.operations = {}
        self.gripper = FakeGripper(self)
        self.failure = None

    @property
    def xyz(self):
        return self.pose[:3]

    def describe(self):
        return {"robot": {"gripper_max_width": .08}}

    def _connected(self):
        return self

    def _acquire(self):
        return self

    def _cancel_on_error(self):
        self.calls.append(("cancel",))

    def movel(self, pose=None, *, xyz=None, offset=None, frame="base", wait=True, **limits):
        target = self._pose_target(pose, xyz, offset, frame)
        operation_id = f"move-{len(self.operations)}"
        self.operations[operation_id] = target
        self.calls.append(("move", target, limits))
        motion = Motion(self, {"times": [0., .2]}, operation_id)
        if wait:
            motion.wait()
        return motion

    def wait(self, operation_id, *, timeout):
        self.calls.append(("wait", operation_id, timeout))
        if self.failure:
            raise self.failure
        self.pose = list(self.operations[operation_id])
        return {"operation_id": operation_id, "status": "completed"}


class TaskPrimitiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.robot = FakeRobot(self.temp.name)

    @staticmethod
    def confirmed(robot):
        return GraspCheck.contact(stop_reason="contact", width=.025, maximum_width=.04)

    def test_success_waits_every_motion_and_records_monotonic_stage_evidence(self):
        events = []
        with self.robot.task("pick/place", on_event=events.append) as task:
            task.pick(xyz=[.3, 0., .25], width=.025, verify_grasp=self.confirmed, timeout=4.)
            task.place(xyz=[.32, 0., .25])
        phases = task.result["phases"]
        self.assertEqual([row["phase"] for row in phases],
                         ["open", "approach", "descend", "grasp", "verify_grasp", "lift",
                          "approach", "place", "release", "retreat"])
        self.assertTrue(all(row["status"] == "completed" for row in phases))
        self.assertEqual(task.result["status"], "completed")
        calls = self.robot.calls
        for index, call in enumerate(calls):
            if call[0] == "move":
                self.assertEqual(calls[index + 1][0], "wait")
        self.assertEqual(next(call[2] for call in calls if call[0] == "wait"), 4.)
        saved = json.loads((task.directory / "result.json").read_text())
        self.assertEqual(saved, task.result)
        recorded = [json.loads(line) for line in (task.directory / "events.jsonl").read_text().splitlines()]
        self.assertEqual(recorded, events)
        self.assertEqual(len(recorded), 2 * len(phases))
        self.assertEqual([e["phase_index"] for e in recorded], [i for i in range(len(phases)) for _ in (0, 1)])
        self.assertEqual([e["host_monotonic_ns"] for e in recorded],
                         sorted(e["host_monotonic_ns"] for e in recorded))
        self.assertLessEqual(saved["started_monotonic_ns"], recorded[0]["host_monotonic_ns"])
        self.assertGreaterEqual(saved["finished_monotonic_ns"], recorded[-1]["host_monotonic_ns"])

    def test_unconfirmed_grasp_stops_lift_and_place_even_when_exception_is_caught(self):
        events = []
        with self.assertRaises(TaskFailed):
            with self.robot.task("empty grasp", on_event=events.append) as task:
                with self.assertRaises(GraspUnconfirmed):
                    task.pick(xyz=self.robot.xyz, width=.025,
                              verify_grasp=lambda robot: GraspCheck(False, "no contact", {"sensor": "fixture"}))
                count = len(self.robot.operations)
                with self.assertRaises(TaskFailed):
                    task.place(xyz=self.robot.xyz)
                self.assertEqual(len(self.robot.operations), count)
        self.assertEqual(count, 2)  # approach and descend only
        self.assertEqual(task.result["status"], "failed")
        last = events[-1]
        self.assertEqual((last["phase"], last["status"]), ("verify_grasp", "failed"))
        self.assertEqual(last["result"]["evidence"], {"sensor": "fixture"})
        self.assertFalse(last["result"]["confirmed"])
        self.assertTrue(any(call[0] == "cancel" for call in self.robot.calls))

    def test_second_observation_repositions_before_descending_and_keeps_identity(self):
        poses = [[.3, 0., .25, 0., 0., 0., 1.], [.305, 0., .25, 0., 0., 0., 1.]]
        seen = []

        def observe(robot):
            seen.append(list(robot.pose))
            return Observation(poses[len(seen) - 1], "block-a", {"frame_index": len(seen)})

        with self.robot.task("refine") as task:
            task.pick(observe=observe, width=.025, verify_grasp=self.confirmed)
        self.assertEqual(len(seen), 2)
        self.assertAlmostEqual(seen[1][2], .3)
        targets = list(self.robot.operations.values())
        self.assertEqual(targets[1][:2], [.305, 0.])
        self.assertAlmostEqual(targets[1][2], .3)
        self.assertEqual(targets[2], poses[1])

    def test_changed_target_aborts_before_descent(self):
        ids = iter(["a", "b"])
        with self.assertRaisesRegex(TaskFailed, "target changed"):
            with self.robot.task("lost target") as task:
                task.pick(observe=lambda robot: Observation(list(robot.pose), next(ids)),
                          width=.025, verify_grasp=self.confirmed)
        self.assertEqual(len(self.robot.operations), 1)
        self.assertEqual(task.result["phases"][-1]["phase"], "observe")
        self.assertEqual(task.result["status"], "failed")

    def test_motion_timeout_preserves_original_error_and_never_closes_gripper(self):
        self.robot.failure = TimeoutError("feedback never settled")
        with self.assertRaises(TimeoutError) as caught:
            with self.robot.task("timeout") as task:
                task.pick(xyz=self.robot.xyz, width=.025, verify_grasp=self.confirmed)
        self.assertIs(caught.exception, self.robot.failure)
        self.assertEqual([call for call in self.robot.calls if call[0] == "gripper"], [("gripper", .08)])
        self.assertEqual(task.result["phases"][-1]["status"], "failed")

    def test_invalid_width_cannot_start_any_stage_or_motion(self):
        with self.assertRaises(ValueError):
            with self.robot.task("bad target") as task:
                task.pick(xyz=self.robot.xyz, width=.2, verify_grasp=self.confirmed)
        self.assertEqual(self.robot.operations, {})
        self.assertEqual(task.result["phases"], [])
        self.assertEqual(task.result["status"], "failed")

    def test_contact_requires_stop_reason_and_plausible_width(self):
        for reason, width, expected in (("contact", .03, True), ("position", .03, False),
                                        ("timeout", .03, False), ("contact", .06, False)):
            self.assertEqual(GraspCheck.contact(stop_reason=reason, width=width, maximum_width=.04).confirmed,
                             expected)
        with self.assertRaises(ValueError):
            GraspCheck.contact(stop_reason="contact", width=float("nan"), maximum_width=.04)

    def test_examples_are_explicit_about_empty_scene_failure_and_real_dependencies(self):
        from examples.tasks.approach_retreat import run as approach
        from examples.tasks.pick_place import run as pick_place
        self.assertEqual(approach(self.robot)["status"], "completed")
        result = pick_place(self.robot)
        self.assertEqual(result["status"], "failed")
        self.assertNotIn("place", [row["phase"] for row in result["phases"]])
        self.robot.backend = "real"
        count = len(self.robot.operations)
        with self.assertRaisesRegex(ValueError, "observe"):
            pick_place(self.robot)
        self.assertEqual(len(self.robot.operations), count)


if __name__ == "__main__":
    unittest.main()
