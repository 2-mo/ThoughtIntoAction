#!/usr/bin/env python3
"""Hand-eye collector arm port on the MuJoCo kernel: frames, collector motion helpers, teaching, gripper, emergency."""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np

from embodied.calibration.hand_eye.collector import _automatic_motion_duration, _movej_for_duration
from embodied.calibration.hand_eye.core.transforms import pose6_to_transform, quaternion_xyzw_from_matrix, transform_error
from embodied.calibration.hand_eye.kernel_arm import KernelArm
from embodied.contracts.runtime.control import ControlClient
from embodied.workstation.control import sim_kernel
from legacy.arx5_control import ControlError, ControllerMode, Pose, SafetyError

READY = np.array([0., .9, .9, -.4, 0., 0.])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    with sim_kernel(output / 'runtime.log') as directory:
        arm = KernelArm(directory)
        arm.connect(stabilize=True)
        try:
            assert arm.mode is ControllerMode.ACTIVE
            # Joint 3 first: lifting joint 2 from the folded zero pose would cross the table plane.
            for target in (np.r_[0., 0., .9, 0., 0., 0.], READY):
                delta = target - arm.get_state().joint_position
                _movej_for_duration(arm, target, _automatic_motion_duration(arm, delta, 1.), allow_large_motion=True)
            state = arm.get_state()
            assert np.max(np.abs(state.joint_position - READY)) < .02, state.joint_position
            # The collector's T_base_eef: measured pose and FK of measured joints agree.
            error = transform_error(pose6_to_transform(state.eef_pose_6d),
                                    pose6_to_transform(arm.forward_kinematics(state.joint_position)))
            assert error[0] < 1e-4 and error[1] < .01, error

            # Pose targets: IK seeded from the live joints, then a kernel MoveJ.
            goal = state.eef_pose_6d.copy()
            goal[1] += .03
            transform = pose6_to_transform(goal)
            pose = Pose(transform[:3, 3], quaternion_xyzw_from_matrix(transform[:3, :3]))
            plan = arm.plan_movej(pose, velocity_scale=1.)
            assert len(plan.path_xyz_m) >= 2 and plan.duration_s > 0
            _movej_for_duration(arm, plan.target_joint_position, 1.)
            reached = arm.get_state().eef_pose_6d
            assert abs(reached[1] - goal[1]) < .003 and np.max(np.abs(reached[[0, 2]] - goal[[0, 2]])) < .003, reached
            line = arm.plan_movel(Pose(transform[:3, 3] + [0, 0, .02], pose.orientation_xyzw))
            assert line.path_xyz_m[-1][2] - line.path_xyz_m[0][2] > .015
            try:
                arm.plan_movel(Pose(transform[:3, 3] + [0, 0, .2], pose.orientation_xyzw))
                raise AssertionError('MoveL cap not applied')
            except SafetyError:
                pass

            # Gripper: absolute move, then a force close that reaches the minimum width (no object in simulation).
            arm.move_gripper(.06, 1.5)
            assert abs(arm.get_state().gripper_position - .06) < .003
            result = arm.force_close_gripper(.3, minimum_width_m=.03, timeout_s=5.)
            assert result.stop_reason == 'minimum_width' and abs(result.final_width_m - .03) < .003, result
            arm.teach_with_gripper_hold()
            time.sleep(.5)
            status = ControlClient(directory).status()
            assert status['phase'] == 'free-drive' and abs(status['gripper_width'] - .03) < .003, status['phase']
            arm.hold_current()
            time.sleep(.3)
            assert ControlClient(directory).status()['phase'] == 'gripper'

            # Emergency from another thread aborts a blocking move; the kernel holds; hold refuses afterwards.
            start = arm.get_state().joint_position

            def trip():
                observer = ControlClient(directory)
                deadline = time.monotonic() + 10
                while abs(observer.status()['q'][0] - start[0]) < .05 and time.monotonic() < deadline:
                    time.sleep(.01)
                arm.emergency_damping('check')
            threading.Thread(target=trip, daemon=True).start()
            try:
                arm.movej(start + [.5, 0, 0, 0, 0, 0], velocity_scale=.4, allow_large_motion=True)
                raise AssertionError('emergency did not abort the move')
            except ControlError as exc:
                assert not isinstance(exc, SafetyError), exc
            moved = abs(ControlClient(directory).status()['q'][0] - start[0])
            time.sleep(.5)
            after = abs(ControlClient(directory).status()['q'][0] - start[0])
            assert arm.mode is ControllerMode.FAULT and .05 <= moved < .3 and after < .3, (moved, after)
            try:
                arm.hold_current()
                raise AssertionError('hold after fault')
            except ControlError:
                pass
        finally:
            arm.close()
        assert ControlClient(directory).status()['owner'] is None, 'lease not released'
        summary = {'teach_phase': 'free-drive', 'force_close': result.stop_reason, 'emergency_motion_rad': moved}
        (output / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(f'PASS: hand-eye arm port on kernel {directory.name}: collector moves, IK pose target, MoveL cap, '
              f'gripper move/force close, teaching with gripper hold, emergency abort ({moved:.3f} rad), lease released.')


if __name__ == '__main__':
    main()
