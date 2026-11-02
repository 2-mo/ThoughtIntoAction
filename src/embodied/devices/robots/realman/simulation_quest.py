"""Quest input over an owned RM65 simulation's existing gateway and lease."""
from contextlib import ExitStack
import json
import sys
import time

import numpy as np

from embodied.contracts.paths import PROJECT_ROOT
from embodied.contracts.runtime.control import ControlSession
from .teleop import RelativeTarget, GripGate, XRClock
from .gripper import Buttons


def mapping_config():
    # Reuse the deployed input conventions, without importing/opening its TCP backend.
    raw = json.loads((PROJECT_ROOT / 'configs/robots/realman/rm65-19.json').read_text())
    keys = ('control_hz', 'stale_s', 'position_scale', 'xr_to_robot_basis',
            'translation_radius_m', 'rotation_radius_rad', 'linear_speed_m_s', 'angular_speed_rad_s')
    config = {key: raw[key] for key in keys}
    basis = np.asarray(config['xr_to_robot_basis'])
    if basis.shape != (3, 3) or not np.allclose(basis.T @ basis, np.eye(3)) or not np.isclose(np.linalg.det(basis), 1):
        raise ValueError('XR basis must be a proper rotation')
    if any(not np.isfinite(config[key]) or config[key] <= 0 for key in keys if key != 'xr_to_robot_basis'):
        raise ValueError('Invalid RM65 input timing/scale/bounds')
    return config


def run(session, xr, config, duration, *, now=time.monotonic, wait=time.sleep):
    """Fresh tracking, release-to-arm and bounded relative targets; B exits."""
    from .geometry import limited_rotation
    tracker, gate = RelativeTarget(config), GripGate()
    clock, buttons = XRClock(config['stale_s']), Buttons()
    started = last = now()
    deadline = started + duration if duration else float('inf')
    seen = False
    previous = np.asarray(session.status()['pose'])
    tracker.origin = previous.copy()
    session.command('hold')
    last_report = float('-inf')
    samples = 0
    try:
        while now() < deadline:
            tick = now()
            state = session.status()
            pose = np.asarray(state['pose'])
            fresh = clock.fresh(int(xr.get_time_stamp_ns()), tick)
            grip = float(xr.get_right_grip())
            if not np.isfinite(grip) or not 0 <= grip <= 1:
                raise ValueError('Invalid Grip reading')
            following = gate.update(grip, fresh)
            if xr.get_B_button():
                break
            if fresh:
                seen = True
            if not fresh:
                session.command('hold')
                tracker.reset()
                if seen or tick - started > 5:
                    raise RuntimeError('Quest data expired; restart simulation input after reconnecting')
            elif following:
                target = tracker.update(xr.get_right_controller_pose(), pose)
                dt = min(max(tick - last, 0), 1 / config['control_hz'])
                delta = target[:3] - previous[:3]
                delta *= min(1., config['linear_speed_m_s'] * dt / max(np.linalg.norm(delta), 1e-12))
                target[:3] = previous[:3] + delta
                target[3:] = limited_rotation(target[3:], previous[3:], config['angular_speed_rad_s'] * dt)
                session.command('cartesian', pose=target.tolist(), source_sequence=samples, ttl_ms=100)
                previous = target.copy()
            else:
                tracker.reset()
                previous = pose.copy()
                session.command('hold')
            action = buttons.update(bool(xr.get_A_button()), float(xr.get_right_trigger()),
                                    fresh=fresh, grip=grip, now=tick)
            if action is not None and not following:
                maximum = session.parameters['effective_robot_config']['gripper_max_width']
                speed = session.parameters['effective_robot_config']['gripper_velocity_max_m_s']
                delta = speed / config['control_hz'] * (1 if action == 'open' else -1)
                width = float(np.clip(state['gripper_width'] + delta, 0, maximum))
                session.command('gripper', gripper_width=width, source_sequence=samples, ttl_ms=100)
            if tick - last_report >= 1:
                print(json.dumps({'simulation_only': True, 'xr_fresh': fresh,
                                  'following': following, 'gripper_action': action}), flush=True)
                last_report = tick
            samples += 1
            last = tick
            wait(max(0, 1 / config['control_hz'] - (now() - tick)))
    finally:
        if sys.exc_info()[0] is None:
            session.command('hold')
        else:
            try:
                session.command('hold')
            except (OSError, RuntimeError, ValueError, EOFError):
                pass  # Preserve the original error; the lease still expires independently.
    return {'samples': samples, 'simulation_only': True}


def connect(directory, duration):
    config = mapping_config()
    session = ControlSession(directory, source='quest', purpose='teleop',
                             metadata={'simulation_only': True, 'mapping': config})
    if session.identity['backend'] != 'sim' or session.parameters['effective_robot_config']['simulation_plugin'] != 'realman.sim':
        raise ValueError('Quest simulation entry requires an RM65 simulation runtime')
    with ExitStack() as stack:
        stack.enter_context(session)
        import xrobotoolkit_sdk as xr
        stack.callback(xr.close)
        xr.init()
        print('RM65 仿真：先松开 Grip/A/Trigger；Grip 跟随，松开后 A 张开 / Trigger 闭合；B 退出。', flush=True)
        return run(session, xr, config, duration)
