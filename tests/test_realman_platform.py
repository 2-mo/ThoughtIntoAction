"""Deployment and client contracts; no device connections or services."""
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import threading
import time
from unittest.mock import Mock, patch

import pytest

from embodied.applications.workstation import bench, pair_state
from embodied.applications.workstation.robot_pair import prepare
from embodied.applications.workstation.workbench import profile, set_native_enabled
from embodied.control.settings import load_settings
from embodied.core.presets import pregrasp_joints
from embodied.devices.inputs.idle import NativeSession
from embodied.devices.inputs.quest.shared import Routing
from embodied.policy.kernel_runner import validate_policy_robot


def test_standard_profiles_use_realman_without_an_arx_controller():
    for name in ('sim-manual', 'sim-phone', 'sim-quest', 'sim-native-demo',
                 'real-manual', 'real-phone', 'real-quest'):
        deployment, plugin = profile(name)
        settings = load_settings(deployment['config'])
        assert plugin.name == ('realman.real' if plugin.hardware else 'realman.sim')
        assert 'controller' not in deployment
        assert settings.robot.model_plugin.startswith('realman')
        assert len(pregrasp_joints(asdict(settings.robot))) == 6
    assert load_settings().robot.simulation_plugin == 'realman.sim'


def test_single_quest_routing_uses_robot_rate_and_own_input_namespace(tmp_path):
    settings = load_settings('configs/robots/realman/sim.json')
    paths = Routing.configure(tmp_path, 'robot-test', settings)
    assert 'robot-test/xr/right_controller' in paths['coordinator'].read_text()
    assert 'gripper-coordinator' in paths
    assert 'robot-test/gripper-group' in paths['gripper'].read_text()


def test_pair_sim_uses_standard_realman_profiles(tmp_path):
    _, _, deployments = prepare(tmp_path, backend='sim')
    for side, path in deployments.items():
        config, plugin = profile(path)
        assert config['robot'] == 'realman.sim' and not plugin.hardware
        assert config['source'] == 'quest' and config['native']['device'] == 'quest.shared'
        assert config['pair']['side'] == side


def test_pair_selection_is_shared_and_commands_pin_the_selected_run(tmp_path):
    runs = {}
    for side in ('left', 'right'):
        directory = tmp_path/side
        directory.mkdir()
        (directory/'runtime.json').write_text(json.dumps({'backend': 'sim', 'runtime_id': side}))
        (directory/'commands.sock').touch()
        runs[side] = str(directory)
    pair = {'runtime_id': 'pair-test', 'runs': runs}
    with patch.object(pair_state, 'active_pair', return_value=pair):
        assert pair_state.selected_run()[0] == 'left'
        pair_state.select_side('right')
        found = bench.current_run()
        assert found[0] == tmp_path/'right'
        rows = bench.actions(*found)
        for key in ('b', 'x', 't', 'c', 'h', 'j', 'g'):
            command = next(row[2] for row in rows if row[0] == key)
            assert str(tmp_path/'right') in command
        pair_state.select_side('left')
        assert str(tmp_path/'right') in rows[2][2]  # Already rendered commands never retarget.


def test_native_source_releases_and_reacquires_its_own_lease(tmp_path):
    (tmp_path/'workbench.json').write_text(json.dumps({'source': 'quest'}))
    acquired, released = threading.Event(), threading.Event()
    session = Mock()
    session.error = None
    session.__enter__ = Mock(side_effect=lambda: acquired.set())
    session.close.side_effect = released.set
    with patch('embodied.devices.inputs.idle.ControlSession', return_value=session), \
         patch('embodied.contracts.runtime.control.ControlClient') as client:
        client.return_value.status.return_value = {'owner': None}
        thread = threading.Thread(target=NativeSession(tmp_path).run, kwargs={'duration': .5})
        thread.start()
        try:
            assert acquired.wait(.2)
            set_native_enabled(tmp_path)
            assert released.wait(.2)
            acquired.clear()
            set_native_enabled(tmp_path)
            assert acquired.wait(.2)
        finally:
            thread.join(1)
        assert not thread.is_alive()
        assert session.__enter__.call_count == 2
        assert session.close.call_count == 2


def test_policy_rejects_x5_weights_even_with_the_same_joint_count():
    contract = {'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
        'joint_names': [f'joint{i}' for i in range(1, 7)], 'joint_unit': 'rad',
        'gripper': {'name': 'eg2_nominal_width_m', 'unit': 'm', 'range': [0., .07]},
        'robot_type': 'rm65_hardware_eg2_4c2_nominal_width'}
    parameters = {'data_contract': contract, 'effective_robot_config': {'hardware_plugin': 'realman.real'}}
    with pytest.raises(ValueError, match='RM65-trained'):
        validate_policy_robot(parameters, {})
    spec = {'adapter_spec': {'data_contract': contract, 'camera_roles': {'left_wrist_0_rgb': 'wrist_rgb'}}}
    assert validate_policy_robot(parameters, spec)['data_contract'] == contract
    mismatched = json.loads(json.dumps(spec))
    mismatched['adapter_spec']['data_contract']['gripper']['range'][1] = .082
    with pytest.raises(ValueError, match='contract differs'):
        validate_policy_robot(parameters, mismatched)


def test_gateway_description_exposes_realman_presets_to_existing_clients():
    from types import SimpleNamespace
    from embodied.control.gateway import Gateway
    gateway = Gateway.__new__(Gateway)
    gateway.config = load_settings('configs/robots/realman/left.json').robot
    gateway.capabilities = {}
    gateway.engine = SimpleNamespace(runtime_id='fake')
    gateway.planner = SimpleNamespace(workspace=None, motion_limits=None)
    gateway.motion_limits = None
    gateway.model_id = 'fake-model'
    gateway.simulation = None
    robot = gateway.describe()['robot']
    assert robot['hardware_plugin'] == 'realman.real'
    assert pregrasp_joints(robot) == list(gateway.config.pregrasp_q)
