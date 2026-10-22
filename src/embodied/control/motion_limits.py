"""Per-run planning limits; configuration is loaded once, never while moving.

These are platform planning bounds, not manufacturer-certified hardware limits.
"""
import json
import math
from pathlib import Path

import numpy as np


from embodied.contracts.paths import CONTROL_CONFIG
from embodied.motion.validation import TCP_UNITS, tcp_limits

DEFAULT_PATH = CONTROL_CONFIG / 'motion.json'
UNITS = {'speed': 'rad/s', 'acceleration': 'rad/s^2', 'jerk': 'rad/s^3'}


def positive_number(value, name, *, zero_allowed=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or value == 0 and not zero_allowed):
        raise ValueError(f'{name} must be a finite ' + ('nonnegative' if zero_allowed else 'positive') + ' number')
    return float(value)


class MotionLimits:
    def __init__(self, defaults, settling, cartesian, native_router, *, joint_velocity_limits, source, control_hz):
        if not isinstance(defaults, dict) or set(defaults) != set(UNITS):
            raise ValueError('Motion defaults must define speed, acceleration and jerk')
        self.defaults = {name: positive_number(defaults[name], name, zero_allowed=name == 'speed') for name in UNITS}
        if not isinstance(cartesian, dict) or set(cartesian) != set(TCP_UNITS):
            raise ValueError('Motion cartesian configuration must define all six TCP bounds (null disables a bound)')
        self.cartesian = {name: None if value is None else positive_number(value, name)
                          for name, value in cartesian.items()}
        self.native_router = validate_native_router(native_router)
        velocity = np.asarray(joint_velocity_limits, dtype=float)
        if velocity.ndim != 1 or velocity.size == 0 or not np.isfinite(velocity).all() or np.any(velocity <= 0):
            raise ValueError('Joint velocity capability must contain finite positive per-joint values')
        self.joint_velocity_limits = velocity.copy()
        self.control_hz = positive_number(control_hz, 'control_hz')
        self.source = dict(source)
        keys = {'position_tolerance_rad', 'velocity_tolerance_rad_s', 'duration_s', 'timeout_s', 'max_sample_gap_s'}
        if not isinstance(settling, dict) or set(settling) != keys:
            raise ValueError('Motion settling configuration is incomplete')
        self.settling = {name: positive_number(settling[name], name) for name in keys}

    @classmethod
    def load(cls, config=None, *, capabilities=None, backend='offline', path=DEFAULT_PATH):
        if config is None:
            from embodied.control.settings import load_settings
            config = load_settings().robot
        if capabilities is None:
            if backend == 'real':
                raise ValueError('Real motion requires capabilities reported by the connected driver')
            capabilities = {'joint_velocity_limits_rad_s': config.joint_velocity_max,
                            'source': {'kind': 'offline_configuration', 'description': 'Configured nominal capability'}}
        velocity = capabilities['joint_velocity_limits_rad_s']
        if len(velocity) != len(config.park_q):
            raise ValueError('Capability joint count does not match robot profile')
        return cls(**json.loads(Path(path).read_text()), joint_velocity_limits=velocity,
                   source=capabilities['source'], control_hz=config.control_hz)

    def resolve_cartesian(self, request):
        # Explicit call parameters override the runtime snapshot. Null in the
        # config means joint bounds alone; clients never invent TCP defaults.
        return {**{key: value for key, value in self.cartesian.items() if value is not None},
                **tcp_limits(request)}

    def resolve(self, request):
        limits = {}
        for name in UNITS:
            value = positive_number(request.get(name, self.defaults[name]), name, zero_allowed=name == 'speed')
            limits[name] = value
        velocity = self.joint_velocity_limits if limits['speed'] == 0 else np.minimum(self.joint_velocity_limits, limits['speed'])
        scale = positive_number(request.get('speed_scale', 1.), 'speed_scale')
        if scale > 1:
            raise ValueError('speed_scale must be in (0, 1]')
        limits['speed_scale'] = scale
        limits['joint_velocity_rad_s'] = (velocity * scale).tolist()
        return limits

    def step_limits(self, limits=None):
        resolved = self.resolve(self.defaults if limits is None else limits)
        return np.asarray(resolved['joint_velocity_rad_s']) / self.control_hz

    def describe(self):
        return {'defaults': self.defaults.copy(), 'joint_velocity_limits_rad_s': self.joint_velocity_limits.tolist(),
                'default_joint_velocity_limits_rad_s': self.resolve(self.defaults)['joint_velocity_rad_s'],
                'source': self.source.copy(), 'control_hz': self.control_hz, 'units': UNITS.copy(),
                'speed_zero': 'Use only the backend per-joint velocity limits',
                'acceleration_jerk_source': 'Application planning parameters, not hardware ratings',
                'cartesian': self.cartesian.copy(), 'cartesian_units': TCP_UNITS.copy(),
                'native_router': {key: value.copy() if isinstance(value, dict) else value
                                  for key, value in self.native_router.items()},
                'configuration': 'configs/control/motion.json',
                'settling': self.settling.copy()}


def validate_native_router(settings):
    """Native Ruckig accepts finite positive bounds even when smoothing is off."""
    if not isinstance(settings, dict) or set(settings) != {'smoothing_enabled', 'cartesian', 'gripper'}:
        raise ValueError('Native router configuration is incomplete')
    if type(settings['smoothing_enabled']) is not bool:
        raise ValueError('Native router smoothing_enabled must be boolean')
    result = {'smoothing_enabled': settings['smoothing_enabled']}
    for group, names in [('cartesian', {f'max_{frame}_{order}' for frame in ('linear', 'angular')
                                       for order in ('velocity', 'acceleration', 'jerk')}),
                         ('gripper', {'max_velocity', 'max_acceleration', 'max_jerk'})]:
        if not isinstance(settings[group], dict) or set(settings[group]) != names:
            raise ValueError(f'Native router {group} bounds are incomplete')
        result[group] = {key: positive_number(value, key) for key, value in settings[group].items()}
    return result
