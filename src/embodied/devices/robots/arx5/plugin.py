"""ARX driver plugins: preparation, resource identity and SDK capabilities live here."""

def model_snapshot(config):
    from pathlib import Path
    import json
    from embodied.contracts.paths import PROJECT_ROOT
    model = Path(config.model_path) if config.model_path else PROJECT_ROOT / 'assets/robots/x5/runtime/x5_2025.xml'
    return {'model': {'xml_path': str(model.resolve()), 'xml': model.read_text(),
                      'urdf': model.with_suffix('.urdf').read_text(),
                      'manifest': json.loads(model.with_suffix('.model.json').read_text()),
                      'tool': json.loads(Path(config.tool_path or PROJECT_ROOT / 'configs/robots/arx5/tool.json').read_text())},
            'gain_policy': {'command-following': {'kp': config.kp, 'kd': config.kd},
                            'disabled': {'kp': config.kp, 'kd': config.kd},
                            'free-drive': {'kp': [0.] * config.joint_count, 'kd': [0.] * config.joint_count}},
            'controller_policy': {'ik': {'algorithm': 'NERO local SE3 DLS', 'damping': .01,
                                         'anchor_weight': .005, 'max_iterations': 50, 'tolerance': 1e-4,
                                         'finite_partial': 'clip and apply', 'invalid': 'retain last valid goal/seed'},
                                  'disabled_ramp_s': 'max(1, max(abs(home-start)) + 0.5)',
                                  'real_shutdown': 'SDK damping, no new Home trajectory'}}


class Simulation:
    snapshot = staticmethod(model_snapshot)

    def shutdown(self, config, backend, *, initial, publish, step, check):
        import time
        import numpy as np
        from embodied.control.runtime import PeriodicClock
        from embodied.contracts.control import ControlCommand, Mode
        if initial is None:
            return {'policy': 'home-and-hold', 'park_completed': False}
        clock = PeriodicClock(config.control_hz)
        state = backend.read_state(time.monotonic())
        home_duration = max(1., float(np.max(np.abs(state.q - config.park_q))) + .5)
        end = time.monotonic() + max(4., home_duration + 1.)
        while time.monotonic() < end:
            now = time.monotonic()
            state = backend.read_state(now)
            check(state, now, 'shutdown-parking')
            step(state, ControlCommand(Mode.DISABLED, gripper_mode=Mode.DISABLED), now, 'shutdown-parking')
            state = backend.read_state(now)
            check(state, now, 'shutdown-parking-feedback')
            publish(state, now)
            clock.wait()
        error = float(np.max(np.abs(state.q - config.park_q)))
        if error > .05 or abs(state.gripper_width - config.gripper_max_width) > .005:
            raise RuntimeError(f'Parking did not reach target: joint error {error:.4f} rad')
        return {'policy': 'home-and-hold', 'park_completed': True, 'park_joint_error_rad': error}

    def preflight(self, config):
        from .kinematics import X5Model
        X5Model(config)
        return {"ready": True, "errors": []}

    def resources(self, config):
        return ()

    def open(self, config, *, cancel_requested=None, **options):
        from .simulation import MujocoBackend
        return MujocoBackend(config, **options)

    def capabilities(self, config, parameters):
        return {"joint_velocity_limits_rad_s": list(config.joint_velocity_max),
                "gripper_velocity_max_m_s": config.gripper_velocity_max_m_s,
                "source": {"kind": "simulation_configuration", "description": "Configured model capability; no hardware measurement"}}


class Hardware:
    @staticmethod
    def environment(env):
        """Resolve the local SDK before launching; imports do not open CAN."""
        import os
        import subprocess
        from embodied.contracts.paths import PROJECT_ROOT, CONTROL_BUILD, CONTROL_PYTHON
        library = CONTROL_PYTHON.parent.parent / 'lib/python3.10/site-packages/arx5_interface.libs'
        env = {**env, 'PYTHONPATH': os.pathsep.join((str(CONTROL_BUILD / 'arx-gripper/source/python'), str(PROJECT_ROOT / 'src'))),
               'QUEST_ARX_GRIPPER_TURNS': '1', 'LD_LIBRARY_PATH': str(library)}
        result = subprocess.run([str(CONTROL_PYTHON), '-c',
            'import arx5_interface as sdk; assert getattr(sdk, "quest_gripper_turns_version", None) == 1'],
            cwd=PROJECT_ROOT, env=env, capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise ValueError('ARX Python 3.10 gripper patch unavailable; run ./run.sh control build-sdk: ' + result.stderr.strip())
        return env

    snapshot = staticmethod(model_snapshot)

    def shutdown(self, config, backend, **callbacks):
        # ARX releases motors into damping. Never introduce a new Home move.
        backend.request_stop()
        return {'policy': 'sdk-damping', 'stop_requested': True}

    def preflight(self, config):
        from .real import preflight
        from .kinematics import X5Model
        checks = preflight(config)
        if checks["ready"]:
            X5Model(config)
        return checks

    def resources(self, config):
        # Same shared lease as maintenance tools and the old service.
        return (config.can_interface,)

    def open(self, config, **options):
        from .real import RealBackend
        return RealBackend(config, **options)

    def capabilities(self, config, parameters):
        robot = parameters["sdk_runtime"]["robot"]
        return {"joint_velocity_limits_rad_s": robot["joint_vel_max"],
                "gripper_velocity_max_m_s": robot["gripper_vel_max"],
                "source": {"kind": "sdk_runtime_configuration", "description": "Active SDK capability, not measured rated hardware speed"}}
