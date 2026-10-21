"""The kernel command protocol (`control/run/<RUN>/commands.sock`, version 1), as data (todo C-01).

One length-prefixed JSON request per connection, answered by {"ok": true, "result": ...} or
{"ok": false, "error": "<Type>: <message>"}. The gateway (control/adapters/gateway.py,
commands.py) enforces it; this table documents it for clients, generates
docs/reference/kernel-protocol.md, and validates the golden samples in tests/golden/kernel/.
Units: joint angles rad, lengths m, poses [x, y, z, qx, qy, qz, qw] of the TCP (`tcp_link`) in
`base_link`, times host CLOCK_MONOTONIC ns unless a name says otherwise.
"""

VERSION = 1

# Field: (type, meaning). Types: int, float, str, bool, object, q (6 floats), pose (7 floats).
ENVELOPE = {
    'version': ('int', 'always 1'),
    'runtime_id': ('str', 'runtime.json runtime_id; a request for another run is refused'),
    'action': ('str', 'one of ACTIONS'),
    'sent_monotonic_ns': ('int', 'client send time; queued requests older than 250 ms are refused'),
}
LEASED = {
    'token': ('str', 'from acquire'),
    'epoch': ('int', 'from acquire; any revocation increments it'),
    'sequence': ('int', 'strictly increasing per lease'),
}
TARGET = {
    'ttl_ms': ('int', '20..250, default 100; the kernel holds when the newest target expires'),
    'source_sequence': ('int', 'optional, -1..2^63, recorded with each tick'),
}

ACTIONS = {
    # name: (lease, request fields {name: (type, meaning, required)}, result keys, summary)
    'describe': (False, {}, ('protocol', 'runtime_id', 'model_id', 'robot', 'workspace', 'actions'),
                 'Robot limits, control rate, park pose, gripper range, CAN interface, model id, workspace; no feedback needed.'),
    'status': (False, {}, ('runtime_id', 'model_id', 'epoch', 'owner', 'last_revocation', 'lease_remaining_s', 'stopped',
                           'phase', 'q', 'dq', 'pose', 'gripper_width', 'feedback_monotonic_ns', 'operations', 'ingresses'),
               'Latest measured state, owner and operations; clients treat feedback older than 150 ms as stale.'),
    'acquire': (False, {'source': ('str', 'ASCII identifier, at most 31 characters', True),
                        'metadata': ('object', 'JSON object, at most 8 KiB, recorded in source-metadata.json', False),
                        'ingress': ('str', 'rpc (default) or an installed native ingress', False),
                        'purpose': ('str', 'operation (default) or teleop', False)},
                ('source', 'token', 'epoch', 'ingress', 'purpose', 'q', 'pose', 'gripper_width', 'status'),
                'Take the single control lease; refused while owned or while the arm moves (>0.05 rad/s).'),
    'stop': (False, {}, ('status', 'stopped'), 'Latch a stop for the rest of the run (no lease needed).'),
    'heartbeat': (True, {}, ('status',), 'Renew the lease (expires 0.5 s after the last leased request); never renews targets.'),
    'source-metadata': (True, {'metadata': ('object', 'JSON object, at most 8 KiB', False)}, ('status', 'source', 'epoch'),
                        'Append metadata for the current lease.'),
    'release': (True, {}, ('status',), 'Give up the lease; the kernel holds the measured pose.'),
    'hold': (True, {}, ('status', 'physical_stopped'), 'Drop targets and cancel the operation; hold the measured pose.'),
    'cancel': (True, {}, ('status', 'physical_stopped'), 'Same as hold.'),
    'preview': (False, {'kind': ('str', 'movej, movel, home or jog', False),
                        'q': ('q', 'movej / jog goal', False), 'pose': ('pose', 'movel goal (unit quaternion)', False),
                        'speed': ('float', 'joint speed bound 0.01..0.5 rad/s (default 0.15)', False),
                        'gripper_width': ('float', 'width held during the move, 0..gripper_max_width (default: current)', False)},
                ('kind', 'epoch', 'owner_present', 'model_id', 'q', 'times', 'gripper_width', 'plan_id', 'expires',
                 'collision_check', 'tcp', 'workspace'),
                'Plan without moving: joint path and times, TCP path (<=160 poses), checked against limits, derivative '
                'bounds and the table plane. Valid for 30 s and only for the same owner and model.'),
    'execute': (True, {'plan_id': ('str', 'from preview', True)}, ('operation_id', 'status'),
                'Run a preview from where it started (within 0.03 rad, at rest); poll status.operations until it settles.'),
    'free-drive': (True, {**{k: (*v, False) for k, v in TARGET.items()},
                          'gripper_width': ('float', 'optional: hold the gripper at this width while the arm is free', False)},
                   ('status',), 'Arm free to move by hand while renewed.'),
    'joint': (True, {**{k: (*v, False) for k, v in TARGET.items()},
                     'q': ('q', 'joint target within limits', True),
                     'gripper_width': ('float', 'gripper target (default: current)', False)},
              ('status',), 'Stream a joint target; the kernel moves toward it within its per-tick step limit.'),
    'cartesian': (True, {**{k: (*v, False) for k, v in TARGET.items()},
                         'pose': ('pose', 'TCP target within 0.15 m of the current TCP and above the table plane', True),
                         'gripper_width': ('float', 'gripper target (default: current)', False)},
                  ('status',), 'Stream a TCP target through the kernel IK.'),
    'gripper': (True, {**{k: (*v, False) for k, v in TARGET.items()},
                       'gripper_width': ('float', 'gripper target, 0..gripper_max_width', False)},
                ('status',), 'Hold the arm and stream a gripper target.'),
}
# Nested result objects.
STATUS_OWNER = ('source', 'epoch', 'ingress', 'purpose')
OPERATION = ('operation_id', 'status', 'started', 'kind')  # plus reason when it failed or was cancelled
OPERATION_STATUS = ('running', 'completed', 'failed', 'cancelled')

TYPES = {'int': int, 'float': (int, float), 'str': str, 'bool': bool, 'object': dict}


def validate_request(request):
    """Errors in a request's shape (empty when well formed); value ranges are the kernel's to check."""
    errors = []
    if not isinstance(request, dict):
        return ['request is not an object']
    action = request.get('action')
    if action not in ACTIONS:
        return [f'unknown action {action!r}']
    lease, fields, _, _ = ACTIONS[action]
    expected = {**{k: (t, m, True) for k, (t, m) in ENVELOPE.items()},
                **({k: (t, m, True) for k, (t, m) in LEASED.items()} if lease else {}), **fields}
    if action in ('describe', 'status', 'preview'):  # answered by the gateway thread, never queued
        expected['sent_monotonic_ns'] = (*ENVELOPE['sent_monotonic_ns'], False)
    for name, (kind, _, required) in expected.items():
        if name not in request:
            if required:
                errors.append(f'{action}: missing {name}')
            continue
        value = request[name]
        if kind in ('q', 'pose'):
            size = 6 if kind == 'q' else 7
            if not (isinstance(value, list) and len(value) == size and all(type(v) in (int, float) for v in value)):
                errors.append(f'{action}: {name} must be {size} numbers')
        elif type(value) is bool and kind != 'bool' or not isinstance(value, TYPES[kind]):
            errors.append(f'{action}: {name} must be {kind}')
    extra = set(request) - set(expected)
    if extra:
        errors.append(f'{action}: unexpected fields {sorted(extra)}')
    if request.get('version') != VERSION:
        errors.append(f'version must be {VERSION}')
    return errors


def render():
    """docs/reference/kernel-protocol.md body."""
    def table(fields):
        return '\n'.join(f'| `{name}` | {spec[0]} | {"required" if len(spec) < 3 or spec[2] else "optional"} | {spec[1]} |'
                         for name, spec in fields.items())
    lines = ['| Field | Type | | Meaning |', '| --- | --- | --- | --- |', table(ENVELOPE), '',
             'Leased actions also carry:', '', '| Field | Type | | Meaning |', '| --- | --- | --- | --- |', table(LEASED), '']
    for name, (lease, fields, result, summary) in ACTIONS.items():
        lines += [f'### `{name}`' + (' (lease)' if lease else ''), '', summary, '']
        if fields:
            lines += ['| Field | Type | | Meaning |', '| --- | --- | --- | --- |', table(fields), '']
        lines += ['Result: ' + ', '.join(f'`{key}`' for key in result), '']
    lines += [f'`status.owner`: {", ".join(f"`{k}`" for k in STATUS_OWNER)}. '
              f'`status.operations[]`: {", ".join(f"`{k}`" for k in OPERATION)} (+ `reason`), '
              f'status one of {", ".join(f"`{k}`" for k in OPERATION_STATUS)}.']
    return '\n'.join(lines).rstrip() + '\n'


DOC = 'docs/reference/kernel-protocol.md'


def updated(text):
    """`text` with the generated section between the protocol markers replaced."""
    start, end = '<!-- protocol:start -->\n', '<!-- protocol:end -->'
    head, rest = text.split(start, 1)
    return head + start + render() + end + rest.split(end, 1)[1]


if __name__ == '__main__':
    from pathlib import Path
    path = Path(__file__).resolve().parents[3] / DOC
    path.write_text(updated(path.read_text()))
    print(f'updated {path}')
