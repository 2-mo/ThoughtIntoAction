"""One XR stream, with an independent clutch/target/gripper for each robot."""
from contextlib import ExitStack
import json
import sys
import time

import numpy as np

from embodied.contracts.control import ControlCommand, Mode
from .gripper import Buttons, SessionJog, GripperNotReady, require_ready, status, move, DEFAULT_SPEED
from .teleop import RelativeTarget, GripGate, XRClock
from .kinematics import CandidateRejected


def stop_arms(controls):
    original_error = sys.exc_info()[1]
    failures = []
    for control in controls:
        try:
            control.backend.stop()
        except Exception as exc:
            failures.append(f'{control.hand} {control.config["host"]}: {exc}')
    if failures:
        message = 'Stop delivery failed; verify physical stop: ' + '; '.join(failures)
        print(message, file=sys.stderr, flush=True)
        if original_error is not None:
            original_error.add_note(message)
        else:
            raise RuntimeError(message)


class ArmControl:
    def __init__(self, hand, backend, xr, *, position_only, gripper, gripper_step):
        self.hand, self.backend = hand, backend
        self.config = backend.config
        self.get_grip = getattr(xr, f'get_{hand}_grip')
        self.get_pose = getattr(xr, f'get_{hand}_controller_pose')
        self.open_button = 'X' if hand == 'left' else 'A'
        self.get_open = getattr(xr, f'get_{self.open_button}_button') if gripper else None
        self.get_trigger = getattr(xr, f'get_{hand}_trigger') if gripper else None
        self.tracker = RelativeTarget(self.config, position_only=position_only, incremental=True)
        self.tracker.origin = backend.origin.copy()
        self.gate, self.buttons = GripGate(), Buttons()
        self.jog = SessionJog(step=gripper_step)
        self.gripper = gripper
        self.last_input = self.last_gripper_report = None
        self.last_print = float('-inf')
        self.previews = self.gripper_actions = 0
        self.latest_gripper = None
        self.last_hand_motion = float('-inf')
        self.stationary_target = None
        self.blocked_reason = None

    def emit(self, fields):
        print(json.dumps({'hand': self.hand, 'host': self.config['host'], **fields}), flush=True)

    def prepare_gripper(self, initialize_open, speed):
        if not self.gripper:
            return
        self.backend.client.allow_gripper = True
        if initialize_open:
            report = move(self.backend.client, 'open', speed=speed, initialize_open=True)
            self.emit({'gripper_initialization': report})
            if not report['accepted']:
                raise RuntimeError(f'{self.hand}: Initial gripper open not accepted; speed was not applied')
        try:
            feedback = status(self.backend.client)
            self.latest_gripper = feedback
            require_ready(feedback)
            self.emit({'gripper': {'enabled_for_session': True, 'feedback': feedback}})
        except GripperNotReady as exc:
            self.latest_gripper = exc.feedback
            self.jog.unavailable = exc.feedback
            self.emit({'gripper': {'enabled_for_session': False,
                'reason': 'not ready; release open button/Trigger for read-only recovery checks',
                'feedback': exc.feedback}})

    def plan(self, fresh):
        self.grip = float(self.get_grip())
        if not np.isfinite(self.grip) or not 0 <= self.grip <= 1:
            raise ValueError(f'{self.hand}: Invalid Grip reading')
        self.following = self.gate.update(self.grip, fresh)
        if fresh and self.grip < .2:
            self.blocked_reason = None
        self.target = None
        if self.following:
            starting = self.tracker.reference is None
            if starting:
                # Each squeeze is a new clutch: discard any old command and
                # anchor all six pose components to measured robot feedback.
                self.backend.stop()
                baseline = self.backend.state.pose
            else:
                baseline = self.backend.last_target if self.backend.last_target is not None else self.backend.state.pose
            target = self.tracker.update(self.get_pose(), baseline)
            if starting:
                self.emit({'clutch_zero': {'hand_m_xyzw': self.tracker.reference.tolist(),
                    'robot_m_xyzw': baseline.tolist()}})
            if starting or self.tracker.moved:
                self.target = target
                self.last_hand_motion = time.monotonic()
            elif time.monotonic() - self.last_hand_motion < .08:
                self.target = self.stationary_target
        else:
            self.tracker.reset()
            self.last_hand_motion = float('-inf')
            self.stationary_target = None
        if self.gripper:
            self.open_down = bool(self.get_open())
            self.trigger = float(self.get_trigger())
            self.action = self.buttons.update(self.open_down, self.trigger, fresh=fresh)
            if self.jog.unavailable is not None:
                self.buttons.ready = False
                self.buttons.blocked_reason = 'gripper_unavailable'
                self.action = None

    def reject_target(self, error):
        self.backend.stop()
        self.gate = GripGate()  # A held Grip cannot automatically retry.
        self.following = False
        self.target = self.stationary_target = None
        self.tracker.reset()
        self.last_hand_motion = float('-inf')
        self.blocked_reason = str(error)
        self.emit({'arm_blocked_reason': self.blocked_reason,
            'recovery': 'release this Grip, then squeeze to set a new measured zero'})

    def send_target(self, execute, now):
        if self.target is None:
            return
        self.previews += 1
        if execute:
            try:
                self.backend.step(ControlCommand(Mode.COMMAND_FOLLOWING,
                    pose_target=self.target, engaged=True), now)
                # Retain only the accepted, rate-limited step during short gaps
                # between XR frames. Never catch up to an unsent hand target.
                self.stationary_target = self.backend.last_target.copy()
            except Exception:
                self.emit({'command_diagnostics': self.backend.telemetry()['command']})
                raise
        if now - self.last_print > .5:
            self.emit({'preview': not execute, 'target_m_xyzw': self.target.tolist(),
                'limits': sorted(set(self.tracker.limits)),
                'command_diagnostics': self.backend.telemetry()['command'] if execute else None})
            self.last_print = now

    def update_gripper(self, fresh):
        if not self.gripper:
            return
        key = (self.open_down, round(self.trigger, 1), fresh, self.following,
               self.buttons.ready, self.buttons.blocked_reason)
        if key != self.last_input:
            self.emit({'gripper_input': {
                f'{self.hand}_{self.open_button}': self.open_down,
                f'{self.hand}_trigger': self.trigger, f'{self.hand}_grip': self.grip,
                'xr_fresh': fresh, 'arm_following': self.following,
                'buttons_armed': self.buttons.ready, 'blocked_reason': self.buttons.blocked_reason,
                'requested_action': self.action}})
            self.last_input = key
        report = self.jog.update(self.backend.client, self.action, time.monotonic(),
            controls_neutral=fresh and not self.open_down and self.trigger < .2)
        if report is not None:
            self.latest_gripper = report.get('feedback', self.latest_gripper)
            self.gripper_actions += int(report.get('accepted', False))
            key = (report.get('reason'), report.get('position_target_raw'))
            if key != self.last_gripper_report:
                self.emit({'gripper': report})
                self.last_gripper_report = key


def run_quest(bindings, xr, *, execute, duration, startup_timeout, position_only,
              gripper, gripper_step, initialize_open=False, gripper_speed=DEFAULT_SPEED,
              standby=False, mirror=None):
    """Backends are already leased; any exit attempts every arm stop before XR close."""
    controls = [ArmControl(hand, backend, xr, position_only=position_only,
        gripper=gripper, gripper_step=gripper_step) for hand, backend in bindings]
    for control in controls:
        if control.backend.state.fault:
            raise RuntimeError(f'{control.hand}: {control.backend.state.fault}')
    with ExitStack() as stack:
        stack.callback(xr.close)
        stack.callback(stop_arms, controls)
        xr.init()
        # Both robot connections and startup states have been checked before
        # any explicitly requested full-open action is sent.
        for control in controls:
            control.prepare_gripper(initialize_open and not standby, gripper_speed)
        dual = len(controls) == 2
        print(('EXECUTE' if execute else 'READ-ONLY PREVIEW') +
              ': each Grip controls its assigned arm; ' + ('B starts/pauses; Y pauses.' if standby else
              'B' + ('/Y' if dual else '') + ' stops and exits.'), flush=True)
        print('Release Grip first. No automatic enable/home.', flush=True)
        if standby:
            print('手柄待机：松开全部控制键后，B 开始/暂停，Y 暂停；Grip 跟随。'
                  + ('首次开始会完全张开两侧夹爪并设置速度。' if initialize_open else ''), flush=True)
        for control in controls:
            c = control.config
            control.emit({'teleop': {
                'mode': 'position_only' if position_only else 'position_and_rotation',
                **{key: c[key] for key in ('position_scale', 'linear_speed_m_s', 'angular_speed_rad_s',
                    'translation_radius_m', 'rotation_radius_rad', 'xr_to_robot_basis')},
                'speed_percent': c.get('speed_percent'),
                'linear_speed_percent': c.get('linear_speed_percent'),
                'joint_speed_deg_s': c.get('joint_speed_deg_s'),
                'controller_speed_limits': {key: control.backend.info[key] for key in
                    ('get_arm_max_line_speed', 'get_arm_max_angular_speed') if key in control.backend.info},
                'workspace_center_m_xyzw': control.tracker.origin.tolist()}})
            if gripper:
                print(f'{control.hand} {c["host"]}: HOLD {control.open_button}=open, '
                      f'HOLD Trigger=close in {gripper_step}/1000 position segments. '
                      'Works while Grip follows; current segment may finish after release.', flush=True)
        if not gripper:
            print('Gripper control disabled; add --gripper with --execute to enable it.', flush=True)
        clock = XRClock(min(control.config['stale_s'] for control in controls))
        started = time.monotonic()
        deadline = started + duration if duration else float('inf')
        startup_deadline = started + startup_timeout
        next_identity = started + 1.
        next_status = started
        seen_xr = False
        paused = standby
        buttons_released = False
        initialized = not initialize_open or not standby
        last_xr = started
        next_reconnect = started + 3.
        reason = 'waiting_for_B' if standby else 'active'
        samples = fresh_samples = 0
        hz = controls[0].config['control_hz']
        while time.monotonic() < deadline:
            tick = time.monotonic()
            b = bool(xr.get_B_button())
            y = bool(xr.get_Y_button()) if dual else False
            if not standby and (b or y):
                break
            # Check both feedback states before either arm gets a new command.
            for control in controls:
                state = control.backend.read_state(tick)
                if state.fault:
                    raise RuntimeError(f'{control.hand} {control.config["host"]}: {state.fault}')
                if tick >= next_identity:
                    for key in ('get_current_tool_frame', 'get_current_work_frame'):
                        if control.backend.client.query(key) != control.backend.info[key]:
                            raise RuntimeError(f'{control.hand}: Robot coordinate/tool configuration changed during session')
            if tick >= next_identity:
                next_identity = tick + 1.
            timestamp = int(xr.get_time_stamp_ns())
            fresh = clock.fresh(timestamp, time.monotonic())
            if fresh:
                last_xr = time.monotonic()
                next_reconnect = last_xr + 3.
            if standby:
                neutral = all(float(c.get_grip()) < .2 and
                    (not gripper or (not c.get_open() and float(c.get_trigger()) < .2)) for c in controls)
                if not fresh:
                    if not paused:
                        stop_arms(controls)
                    paused = True
                    buttons_released = False
                    reason = 'xr_stale'
                elif y or (b and buttons_released and not paused):
                    paused = True
                    buttons_released = False
                    stop_arms(controls)
                    reason = 'paused_by_button'
                elif fresh and paused and reason == 'xr_stale' and not b:
                    reason = 'waiting_for_B'
                    buttons_released = not y
                elif b and buttons_released and paused:
                    buttons_released = False
                    if neutral:
                        try:
                            for control in controls:
                                control.backend.check_motion_ready()
                        except CandidateRejected as exc:
                            stop_arms(controls)
                            reason = 'posture_rejected'
                            print(f'{control.hand} {control.config["host"]} 当前姿态不适合遥操：{exc}。'
                                  '保持待机显示；请用示教器调整姿态，松键后按 B 重新检查。', flush=True)
                            continue
                        if not initialized:
                            for control in controls:
                                control.prepare_gripper(True, gripper_speed)
                            initialized = True
                        paused = False
                        reason = 'active'
                        print('手柄会话已开始；先松开再握住 Grip 跟随。', flush=True)
                    else:
                        reason = 'release_grip_and_gripper_buttons'
                        print('未开始：请松开两侧 Grip、X/A 与 Trigger，再按 B。', flush=True)
                elif not b and not y:
                    buttons_released = True
                if paused:
                    # Passing stale to input gates discards held controls and reference poses.
                    for control in controls:
                        control.plan(False)
                        control.backend.stop()
                    if mirror is not None:
                        mirror.update(controls, fresh=fresh, timestamp=timestamp, paused=True, reason=reason)
                    if tick >= next_status:
                        print(json.dumps({'standby': True, 'xr_fresh': fresh,
                            'waiting_for_first_xr': not seen_xr and not fresh,
                            'reason': reason, 'xr_timestamp_ns': timestamp,
                            'controls': [{ 'hand': c.hand, 'grip': c.grip,
                                'trigger': getattr(c, 'trigger', 0.), 'open': getattr(c, 'open_down', False)}
                                for c in controls]}), flush=True)
                        next_status = tick + 1.
                    if not fresh and time.monotonic() >= next_reconnect:
                        # Only one SDK reader owns this session. Re-subscribe while
                        # paused; stale buttons can never restart arm motion.
                        print('XR 数据未更新，暂停中重新订阅 SDK；恢复后请松键再按 B。', flush=True)
                        xr.close()
                        xr.init()
                        clock = XRClock(min(c.config['stale_s'] for c in controls))
                        # A reinitialized binding may retain its cached last timestamp.
                        # Require a newer callback, not merely a nonzero cached value.
                        clock.previous = int(xr.get_time_stamp_ns())
                        buttons_released = False
                        next_reconnect = time.monotonic() + 3.
                    seen_xr = seen_xr or fresh
                    time.sleep(max(0, 1/hz - (time.monotonic() - tick)))
                    continue
            samples += 1
            fresh_samples += int(fresh)
            for control in controls:
                control.plan(fresh)
            if tick >= next_status:
                for control in controls:
                    control.emit({'xr_fresh': fresh, f'{control.hand}_grip': control.grip,
                        'robot_fault': control.backend.state.fault, 'armed': control.gate.armed,
                        'following': control.following, 'preview': not execute,
                        'gripper_enabled_for_session': gripper and control.jog.unavailable is None,
                        'xr_timestamp_ns': timestamp, 'waiting_for_first_xr': not seen_xr and not fresh})
                next_status = tick + 1.
            # Releases stop their arms before following targets or gripper I/O.
            for control in controls:
                if not control.following or control.target is None:
                    control.backend.stop()
            if not fresh:
                if execute and seen_xr:
                    raise RuntimeError('XR data stalled or timestamp rolled back after connection; restart session')
                if execute and time.monotonic() > startup_deadline:
                    raise RuntimeError(f'No first XR frame within {startup_timeout:g} s; check Toolkit connection')
            else:
                seen_xr = True
                for control in controls:
                    try:
                        control.send_target(execute, tick)
                    except CandidateRejected as exc:
                        if not standby:
                            raise
                        control.reject_target(exc)
                        print(f'{control.hand} {control.config["host"]} 候选目标被拒绝，'
                              f'该侧停止：{exc}。松开该侧 Grip，再握下重新归零；另一侧和夹爪可继续操作。', flush=True)
            for control in controls:
                control.update_gripper(fresh)
            if mirror is not None:
                mirror.update(controls, fresh=fresh, timestamp=timestamp, paused=False, reason='active')
            time.sleep(max(0, 1/hz - (time.monotonic() - tick)))
        for control in controls:
            control.emit({'samples': samples, 'fresh_xr_samples': fresh_samples,
                'target_samples': control.previews, 'execute_requested': execute,
                'gripper_actions': control.gripper_actions,
                'gripper_enabled_for_session': gripper and control.jog.unavailable is None})
