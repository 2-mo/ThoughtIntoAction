"""Small numerical checks shared by models and planners."""
import math
import numpy as np


def finite_vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


TCP_UNITS = {'tcp_speed': 'm/s', 'tcp_acceleration': 'm/s^2', 'tcp_jerk': 'm/s^3',
             'angular_speed': 'rad/s', 'angular_acceleration': 'rad/s^2', 'angular_jerk': 'rad/s^3'}

def tcp_limits(request):
    result = {}
    for name in TCP_UNITS:
        if name in request:
            value = request[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive ({TCP_UNITS[name]})')
            result[name] = float(value)
    return result
