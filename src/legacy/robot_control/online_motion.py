"""Service-owned jerk-limited online joint trajectory; never touches devices."""
import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result, ControlInterface

DEFAULTS = dict(teleop_preview_time_s=.01, max_joint_acceleration_rad_s2=2., max_joint_jerk_rad_s3=10.,
                home_speed_rad_s=.3, home_final_speed_rad_s=0., home_acceleration_rad_s2=.5, home_jerk_rad_s3=2.,
                preset_speed_rad_s=.5, preset_acceleration_rad_s2=1., preset_jerk_rad_s3=4.)


class OnlineMotion:
    def __init__(self, q, velocity, hardware_speed, speed, settings, dt, lower, upper):
        self.settings = settings
        self.speed = np.asarray(hardware_speed) if speed == 0 else np.minimum(hardware_speed, speed)
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        margin = settings['joint_soft_limit_margin_rad']
        self.target_lower = np.minimum(self.lower + margin, (self.lower + self.upper)/2)
        self.target_upper = np.maximum(self.upper - margin, (self.lower + self.upper)/2)
        self.otg, self.input, self.output = Ruckig(6, dt), InputParameter(6), OutputParameter(6)
        self.input.current_position = list(q)
        self.input.current_velocity = list(velocity)
        self.input.current_acceleration = [0.] * 6
        self.input.target_position = list(q)
        self.input.target_velocity = self.input.target_acceleration = [0.] * 6
        self._home_goal = None
        self.home_phase = None
        self.profile = None
        self.configure('teleop')
        self.stopping = False
        self.finished = False

    def configure(self, profile):
        if profile not in ('teleop', 'home', 'preset'):
            raise ValueError('Invalid trajectory profile')
        if profile == 'teleop':
            speed = self.speed
            acceleration = self.settings['max_joint_acceleration_rad_s2']
            jerk = self.settings['max_joint_jerk_rad_s3']
        else:
            limit = self.settings[profile + '_speed_rad_s']
            # Preset speed 0 disables its extra cap, retaining hardware/session limits.
            speed = self.speed if profile == 'preset' and limit == 0 else np.minimum(self.speed, limit)
            if profile == 'home' and self.home_phase == 'final':
                speed = np.minimum(speed, self.settings['home_final_speed_rad_s'])
            acceleration = self.settings[profile + '_acceleration_rad_s2']
            jerk = self.settings[profile + '_jerk_rad_s3']
        self.input.max_velocity = list(speed)
        self.input.max_acceleration = [acceleration]*6
        self.input.max_jerk = [jerk]*6
        self.profile = profile

    def target(self, q, profile='teleop'):
        if profile not in ('teleop', 'home', 'preset'):
            raise ValueError('Invalid trajectory profile')
        if profile != self.profile and (np.max(np.abs(self.input.current_velocity)) > 1e-8
                                        or np.max(np.abs(self.input.current_acceleration)) > 1e-8):
            # A lower preset jerk limit must not prolong braking while still
            # carrying the previous mode's acceleration. Stop with those limits
            # first, then the next target update selects the new profile at rest.
            self.stop()
            return
        goal = np.clip(q, self.target_lower, self.target_upper)
        if profile == 'home' and self.settings.get('home_final_speed_rad_s', 0) > 0:
            if self._home_goal is None or not np.array_equal(goal, self._home_goal):
                self._home_goal = goal.copy()
                self._home_midpoint = (np.asarray(self.input.current_position) + goal) / 2
                self.home_phase = 'first'
            goal = self._home_midpoint if self.home_phase == 'first' else self._home_goal
        else:
            self._home_goal, self.home_phase = None, None
        self.configure(profile)
        self.input.control_interface = ControlInterface.Position
        self.input.target_position = goal.tolist()
        self.stopping = False

    def stop(self):
        # Retain current v/a and active limits, replace old positional goal by v=a=0.
        self._home_goal, self.home_phase = None, None
        self.input.control_interface = ControlInterface.Velocity
        self.stopping = True

    def step(self):
        result = self.otg.update(self.input, self.output)
        if result not in (Result.Working, Result.Finished):
            raise RuntimeError(f'Online trajectory failed: {result}')
        q = np.array(self.output.new_position)
        if np.any(q < self.lower - 1e-8) or np.any(q > self.upper + 1e-8):
            raise RuntimeError('Online trajectory would cross a hard joint limit')
        self.output.pass_to_input(self.input)
        self.finished = result == Result.Finished
        if self.finished and not self.stopping and self.home_phase == 'first':
            # Stop at the joint-space midpoint before lowering the speed bound.
            # This avoids carrying a 2 rad/s command into the slower half.
            self.home_phase = 'final'
            self.configure('home')
            self.input.target_position = self._home_goal.tolist()
            self.finished = False
        return q, np.array(self.output.new_velocity)

    def snapshot(self):
        return dict(position=list(self.input.current_position), velocity=list(self.input.current_velocity),
                    acceleration=list(self.input.current_acceleration), profile=self.profile,
                    stopping=self.stopping, finished=self.finished, home_phase=self.home_phase)

    def path_above_floor(self, q, profile, tcp_model, minimum_z):
        """Check a candidate Ruckig path before accepting it; do not mutate planner.

        This is a sampled model check with a configurable clearance, not a
        continuous collision certificate or a measurement of servo overshoot.
        """
        from ruckig import Trajectory, RuckigError
        candidate = OnlineMotion(self.input.current_position, self.input.current_velocity,
            self.speed, 0, self.settings, self.otg.delta_time, self.lower, self.upper)
        candidate.input.current_acceleration = list(self.input.current_acceleration)
        candidate._home_goal = None if self._home_goal is None else self._home_goal.copy()
        candidate.home_phase = self.home_phase
        if self._home_goal is not None:
            candidate._home_midpoint = self._home_midpoint.copy()
        candidate.configure(self.profile)
        candidate.target(q, profile)
        # A profile switch first brakes; also inspect the following rest-to-rest leg.
        for leg in range(3):
            trajectory = Trajectory(6)
            try:
                result = candidate.otg.calculate(candidate.input, trajectory)
            except RuckigError:
                return False
            if result not in (Result.Working, Result.Finished):
                return False
            if not np.isfinite(trajectory.duration) or trajectory.duration > 30:
                return False
            for t in np.linspace(0, trajectory.duration, max(2, int(np.ceil(trajectory.duration / .01)) + 1)):
                position, _, _ = trajectory.at_time(float(t))
                try:
                    if tcp_model.fk(position)[2, 3] < minimum_z - 1e-6:
                        return False
                except ValueError:
                    return False
            if not candidate.stopping:
                if candidate.home_phase != 'first':
                    return True
                candidate.home_phase = 'final'

            position, velocity, acceleration = trajectory.at_time(trajectory.duration)
            candidate.input.current_position = position
            candidate.input.current_velocity = velocity
            candidate.input.current_acceleration = acceleration
            candidate.target(q, profile)
        return False
