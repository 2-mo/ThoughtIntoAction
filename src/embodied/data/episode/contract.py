"""Recorded vector semantics, independent of robot SDKs and training frameworks.

The runtime wire currently carries absolute joint positions and one gripper width.
Other action representations require their own adapter; never pad or reinterpret them.
"""
import copy
import math

SCHEMA = 'embodied.joint_vectors/1'


def validate_contract(contract):
    if not isinstance(contract, dict):
        raise ValueError('Robot vector contract must be an object')
    contract = copy.deepcopy(contract)
    if contract.get('schema') != SCHEMA or contract.get('representation') != 'absolute_joint_position':
        raise ValueError('Unsupported robot vector contract')
    names = contract.get('joint_names')
    if (not isinstance(names, list) or not names or
            any(not isinstance(name, str) or not name.strip() for name in names) or len(set(names)) != len(names)):
        raise ValueError('joint_names must be nonempty unique names in wire order')
    if contract.get('joint_unit') != 'rad':
        raise ValueError('Runtime joint vectors require radians')
    if 'gripper' not in contract:
        raise ValueError('Declare gripper metadata or explicit null for a robot without a gripper')
    gripper = contract['gripper']
    if gripper is None:
        return contract
    if not isinstance(gripper, dict) or not isinstance(gripper.get('name'), str) or not gripper['name'] or gripper['name'] in names or gripper.get('unit') != 'm':
        raise ValueError('A distinct gripper width name and metre unit are required')
    limits = gripper.get('range')
    if (not isinstance(limits, list) or len(limits) != 2 or
            type(limits[0]) not in (int, float) or not math.isfinite(limits[0]) or limits[0] < 0 or
            (limits[1] is not None and (type(limits[1]) not in (int, float) or not math.isfinite(limits[1]) or limits[1] <= limits[0]))):
        raise ValueError('Invalid gripper width range (null upper bound means undeclared)')
    return contract


def runtime_contract(descriptor):
    """Obtain semantics from the recorded descriptor, including historic X5 snapshots."""
    if 'data_contract' in descriptor:
        contract = validate_contract(descriptor['data_contract'])
    else:
        parameters = descriptor.get('parameters', {})
        profile = parameters.get('effective_robot_config', {})
        names = descriptor.get('joint_names') or parameters.get('model', {}).get('joint_names')
        count = descriptor.get('joint_count')
        if count is None and profile.get('park_q') is not None:
            count = len(profile['park_q'])
        # Old wire/episode versions were explicitly X5, before joint_count existed.
        legacy = count is None and descriptor.get('layout', {}).get('version', 1) < 3
        if count is None:
            count = len(names) if names else 6 if legacy else None
        if type(count) is not int or count < 1:
            raise ValueError('Runtime descriptor must declare joint_count')
        width = profile.get('gripper_max_width', descriptor.get('gripper_max_width'))
        if width is None and legacy:
            width = .082
        contract = validate_contract({'schema': SCHEMA, 'representation': 'absolute_joint_position',
            'joint_names': names or [f'joint{i + 1}' for i in range(count)], 'joint_unit': 'rad',
            'gripper': {'name': 'gripper_m', 'unit': 'm', 'range': [0., width]},
            'robot_type': descriptor.get('robot_type') or profile.get('name') or ('stanford_x5' if legacy else 'joint_robot')})
        if len(contract['joint_names']) != count:
            raise ValueError('Joint names disagree with runtime joint_count')
    if descriptor.get('joint_count') is not None and len(contract['joint_names']) != descriptor['joint_count']:
        raise ValueError('Vector contract disagrees with runtime joint_count')
    return contract


def episode_contract(metadata):
    if 'data_contract' in metadata:
        contract = validate_contract(metadata['data_contract'])
        count = (metadata.get('runtime') or {}).get('joint_count')
        if count is not None and count != len(contract['joint_names']):
            raise ValueError('Recorded contract disagrees with runtime joint_count')
        return contract
    if metadata.get('runtime') is not None:
        return runtime_contract(metadata['runtime'])
    # Schema 1 was the six-axis X5 service; use its recorded model joint order.
    robot = metadata.get('robot', {})
    return validate_contract({'schema': SCHEMA, 'representation': 'absolute_joint_position',
        'joint_names': robot.get('joint_names', [f'joint{i + 1}' for i in range(6)]), 'joint_unit': 'rad',
        'gripper': {'name': 'gripper_m', 'unit': 'm', 'range': [0., metadata.get('control', {}).get('gripper_width_m', .082)]},
        'robot_type': 'stanford_x5'})


def feature_names(contract):
    return [*contract['joint_names'], *([contract['gripper']['name']] if contract['gripper'] else [])]


def action_definition(contract):
    return 'sent_joint_position' + (' + sent_gripper_width' if contract['gripper'] else '')


def vector(row, contract):
    values = [*row['q'], *([row['gripper_width']] if contract['gripper'] else [])]
    if len(values) != len(feature_names(contract)) or any(type(x) not in (int, float) or not math.isfinite(x) for x in values):
        raise ValueError('Robot vector dimension or finite values disagree with its contract')
    if contract['gripper']:
        low, high = contract['gripper']['range']
        if values[-1] < low or (high is not None and values[-1] > high):
            raise ValueError('Gripper outside recorded calibrated width range')
    return values
