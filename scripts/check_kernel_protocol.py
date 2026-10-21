#!/usr/bin/env python3
"""Every commands.sock action against a MuJoCo kernel: requests and results must match embodied.contracts.protocol.

--update rewrites tests/golden/kernel/samples.json from this run (lists shortened, ids normalized).
"""
import argparse
import json
from pathlib import Path
import time

from embodied.contracts.protocol import ACTIONS, OPERATION, STATUS_OWNER, VERSION, validate_request
from embodied.contracts.rpc import request
from embodied.workstation.control import sim_kernel

GOLDEN = Path(__file__).resolve().parents[1] / 'tests/golden/kernel/samples.json'


def shorten(value):
    if isinstance(value, dict):
        return {key: shorten(item) for key, item in value.items()}
    if isinstance(value, list) and len(value) > 8:  # paths and operation lists; q / pose stay whole
        return [shorten(item) for item in value[:3]] + ['…']
    if isinstance(value, list):
        return [shorten(item) for item in value]
    return value


class Client:
    def __init__(self, directory):
        self.directory = directory
        self.runtime_id = json.loads((directory / 'runtime.json').read_text())['runtime_id']
        self.lease, self.sequence, self.samples = None, 0, []

    def send(self, action, **fields):
        message = {'version': VERSION, 'runtime_id': self.runtime_id, 'action': action, 'sent_monotonic_ns': time.monotonic_ns()}
        if ACTIONS[action][0]:
            self.sequence += 1
            message.update(token=self.lease['token'], epoch=self.lease['epoch'], sequence=self.sequence)
        message.update(fields)
        errors = validate_request(message)
        assert not errors, errors
        result = request(self.directory / 'commands.sock', message)
        keys = set(ACTIONS[action][2])
        assert set(result) == keys, (action, sorted(set(result) ^ keys))
        self.samples.append({'request': message, 'result': result})
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--update', action='store_true', help='Rewrite the golden samples from this run')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    with sim_kernel(args.output / 'runtime.log') as directory:
        client = Client(directory)
        robot = client.send('describe')['robot']
        status = client.send('status')
        client.lease = client.send('acquire', source='protocol', metadata={'check': 'kernel protocol'}, purpose='operation')
        client.send('heartbeat')
        client.send('source-metadata', metadata={'step': 'motion'})
        plan = client.send('preview', kind='movej', q=[0., 0., .3, 0., 0., 0.], speed=.5)
        operation = client.send('execute', plan_id=plan['plan_id'])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            client.send('heartbeat')  # status never renews the lease
            status = client.send('status')
            entry = next((op for op in status['operations'] if op['operation_id'] == operation['operation_id']), None)
            if entry and entry['status'] != 'running':
                break
            time.sleep(.05)
        assert entry and entry['status'] == 'completed' and set(OPERATION) <= set(entry), entry
        assert set(status['owner']) == set(STATUS_OWNER), status['owner']
        q, pose = status['q'], status['pose']
        client.send('joint', q=q, gripper_width=robot['gripper_max_width'] / 2, ttl_ms=100, source_sequence=1)
        client.send('cartesian', pose=pose, ttl_ms=100)
        client.send('gripper', gripper_width=robot['gripper_max_width'] / 2, ttl_ms=100)
        client.send('free-drive', ttl_ms=100, gripper_width=robot['gripper_max_width'] / 2)
        client.send('hold')
        client.send('cancel')
        client.send('release')
        client.send('stop')
    covered = {sample['request']['action'] for sample in client.samples}
    assert covered == set(ACTIONS), sorted(set(ACTIONS) - covered)
    if args.update:
        samples = {}
        for sample in client.samples:
            samples.setdefault(sample['request']['action'], shorten(sample))
        text = json.dumps(samples, indent=2, ensure_ascii=False)
        for secret in (client.runtime_id, client.lease['token']):
            text = text.replace(secret, '<runtime_id>' if secret == client.runtime_id else '<token>')
        GOLDEN.parent.mkdir(parents=True, exist_ok=True)
        GOLDEN.write_text(text + '\n')
        print(f'wrote {GOLDEN}')
    print(f'PASS: {len(covered)} actions, {len(client.samples)} requests match embodied.contracts.protocol on {directory.name}')


if __name__ == '__main__':
    main()
