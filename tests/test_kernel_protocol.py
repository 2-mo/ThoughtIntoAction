"""The kernel command protocol table matches the gateway, the golden samples, the client and the docs (todo C-01)."""
import ast
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from embodied.contracts import protocol
from embodied.contracts.protocol import ACTIONS, validate_request

ROOT = Path(__file__).resolve().parents[1]
GOLDEN = json.loads((ROOT / 'tests/golden/kernel/samples.json').read_text())


def gateway_actions():
    tree = ast.parse((ROOT / 'control/adapters/gateway.py').read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(getattr(target, 'id', None) == 'ACTIONS' for target in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('gateway ACTIONS not found')


class KernelProtocolTests(unittest.TestCase):
    def test_table_lists_exactly_the_gateway_actions(self):
        self.assertEqual(set(ACTIONS), set(gateway_actions()))

    def test_golden_samples_cover_every_action_and_match_the_table(self):
        self.assertEqual(set(GOLDEN), set(ACTIONS))
        for action, sample in GOLDEN.items():
            with self.subTest(action=action):
                self.assertEqual(validate_request(sample['request']), [])
                self.assertEqual(set(sample['result']), set(ACTIONS[action][2]))

    def test_malformed_requests_are_reported(self):
        joint = dict(GOLDEN['joint']['request'])
        self.assertEqual(validate_request(joint), [])
        self.assertIn('joint: q must be 6 numbers', validate_request({**joint, 'q': [0] * 5}))
        self.assertIn('joint: missing token', validate_request({k: v for k, v in joint.items() if k != 'token'}))
        self.assertIn("joint: unexpected fields ['speed']", validate_request({**joint, 'speed': 1}))
        self.assertIn('version must be 1', validate_request({**joint, 'version': 2}))
        self.assertEqual(validate_request({'action': 'fly'}), ["unknown action 'fly'"])

    def test_control_session_sends_well_formed_requests(self):
        from embodied.contracts.runtime import control
        sent = []

        def fake(path, message):
            sent.append(message)
            if message['action'] == 'acquire':
                return {'token': 't', 'epoch': 1}
            if message['action'] == 'preview':
                return {'plan_id': 'p'}
            return {'status': 'applied', 'feedback_monotonic_ns': message['sent_monotonic_ns'], 'operations': []}
        client = object.__new__(control.ControlSession)
        client.directory, client.identity = Path('/run'), {'runtime_id': 'r'}
        client.source, client.ingress, client.purpose, client.metadata = 'test', 'rpc', 'operation', {}
        client.sequence, client.error = 0, None
        client.lock = __import__('threading').RLock()
        with patch.object(control, 'request', fake):
            client.lease = control.ControlClient.request(client, 'acquire', source='test', metadata={}, ingress='rpc', purpose='operation')
            client.describe()
            client.execute(client.preview('movej', q=[0.] * 6, speed=.2))
            client.command('joint', q=[0.] * 6, gripper_width=.04, ttl_ms=120, sent_monotonic_ns=1)
            client.command('free-drive', gripper_width=.04, ttl_ms=200, sent_monotonic_ns=1)
            client.command('hold')
        for message in sent:
            with self.subTest(action=message['action']):
                self.assertEqual(validate_request(message), [])

    def test_reference_page_is_generated_from_the_table(self):
        path = ROOT / protocol.DOC
        self.assertEqual(path.read_text(), protocol.updated(path.read_text()),
                         'run PYTHONPATH=src python3 -m embodied.contracts.protocol')


if __name__ == '__main__':
    unittest.main()
