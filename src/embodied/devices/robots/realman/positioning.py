"""Explicit low-speed SDK joint zero move, separate from Cartesian teleoperation."""
import time
import numpy as np
from .kinematics import OfflineModel, load_sdk


def zero_preview(backend):
    for key in ('get_DH_data', 'get_joint_min_pos', 'get_joint_max_pos'):
        backend.info[key] = backend.client.query(key)
    model = OfflineModel(backend.info)
    target = np.zeros(6)
    if np.any(target < model.lower) or np.any(target > model.upper):
        raise ValueError('Zero joints outside controller limits')
    return {'target_joint_deg': target.tolist(), 'speed_percent': 5,
            'current_joint_deg': np.rad2deg(backend.state.q).tolist(),
            'zero_tcp_m_xyzw': model.fk(target).tolist(),
            'zero_singularity_code': model.singularity(target)[0],
            'collision_clearance_verified': False,
            'note': 'Joint zero is a singular calibration reference; use a bent posture for Cartesian teleoperation.'}


def execute_zero(backend, *, sdk=None):
    """Caller holds the device lease; no implicit enable, fault reset or tool writes."""
    initial = backend.read_state(time.monotonic())
    if initial.fault:
        raise RuntimeError(initial.fault)
    start_q = initial.q.copy()
    time.sleep(.2)
    state = backend.read_state(time.monotonic())
    if state.fault or np.max(np.abs(np.rad2deg(state.q - start_q))) > .05:
        raise RuntimeError('Arm must be stationary and fault-free before joint zero')
    sdk = load_sdk() if sdk is None else sdk
    arm = sdk.RoboticArm(sdk.rm_thread_mode_e.RM_DUAL_MODE_E)
    connected = False
    motion_attempted = False
    complete = False
    try:
        handle = arm.rm_create_robot_arm(backend.config['host'], backend.config['port'])
        if handle.id < 0:
            raise RuntimeError('SDK robot connection failed')
        connected = True
        # Read the SDK state too: reject a mismatched connection/unit convention.
        code, raw = arm.rm_get_current_arm_state()
        if code != 0:
            raise RuntimeError(f'SDK state query failed: {code}')
        joints = np.asarray(raw.get('joint'), dtype=float)
        if joints.shape != (6,) or not np.isfinite(joints).all() or not np.allclose(
                joints, np.rad2deg(state.q), atol=.1, rtol=0):
            raise RuntimeError('SDK joint feedback disagrees with JSON state')
        motion_attempted = True
        code = arm.rm_movej([0.]*6, 5, 0, 0, 0)
        if code != 0:
            raise RuntimeError(f'SDK movej rejected/failed: {code}')
        deadline = time.monotonic() + 90.
        stable = 0
        while time.monotonic() < deadline:
            state = backend.read_state(time.monotonic())
            if state.fault:
                raise RuntimeError(state.fault)
            stable = stable + 1 if np.max(np.abs(np.rad2deg(state.q))) <= .1 else 0
            if stable >= 3:
                complete = True
                return {'completed': True, 'joint_deg': np.rad2deg(state.q).tolist()}
            time.sleep(.1)
        raise RuntimeError('SDK joint zero did not finish within 90 s')
    except BaseException as exc:
        if motion_attempted and not complete:
            try:
                result = arm.rm_set_arm_stop()
                if result != 0:
                    raise RuntimeError(f'SDK stop failed: {result}; verify physical stop')
            except BaseException as stop_exc:
                exc.add_note(f'Stop delivery failed: {stop_exc}; verify physical stop')
        raise
    finally:
        if connected:
            arm.rm_delete_robot_arm()
