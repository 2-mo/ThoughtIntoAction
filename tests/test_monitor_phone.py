"""HTTP phone observation never acquires control; commands are explicit."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import Request, ProxyHandler, build_opener
from urllib.error import HTTPError

from embodied.applications.monitor.scene import Scene
from embodied.devices.inputs import phone_service


class MonitorPhoneTests(unittest.TestCase):
    def test_read_start_and_explicit_control_boundaries(self):
        scene = Scene(None, runtime_monitor=SimpleNamespace(identity={'runtime_id': 'test', 'backend': 'sim'}))
        self.addCleanup(scene.close)
        http = build_opener(ProxyHandler({}))
        url = scene.url + 'api/phone'
        def post(data, origin=None):
            headers = {'Content-Type': 'application/json'}
            if origin: headers['Origin'] = origin
            return json.loads(http.open(Request(url, json.dumps(data).encode(), headers)).read())
        with patch.object(phone_service, 'monitor', return_value={'available': True}) as monitor, \
             patch.object(phone_service, 'ensure_service') as start, \
             patch.object(phone_service, 'command', return_value={'enabled': True}) as command:
            state = json.loads(http.open(url).read())
            self.assertEqual(state['kernel_runtime_id'], 'test')
            command.assert_not_called()
            start.assert_not_called()
            self.assertTrue(post({'action': 'start'})['ok'])
            start.assert_called_once()
            command.assert_not_called()
            self.assertTrue(post({'action': 'enable', 'runtime_id': 'test'})['ok'])
            command.assert_called_once_with('enable', runtime_id='test')
            command.reset_mock()
            with self.assertRaises(HTTPError) as error:
                post({'action': 'enable'}, 'https://unrelated.invalid')
            self.assertEqual(error.exception.code, 403)
            command.assert_not_called()
            self.assertIn('error', post({'action': 'execute'}))
            self.assertIn('error', post(['enable']))
            scene.runtime_monitor = None
            self.assertIn('error', post({'action': 'enable'}))
            command.assert_not_called()


if __name__ == '__main__':
    unittest.main()
