"""Read-only native hand display; no SDK, robot or control publication."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import json
from pathlib import Path
import tempfile
import unittest

from embodied.control.observer import open_input_observer
from embodied.devices.inputs.quest.workbench import InputObserver


class InputObserverTests(unittest.TestCase):
    def test_observer_waits_for_first_backend_tick_before_loading_input_parameters(self):
        from embodied.control import observer
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            (directory/'runtime.json').write_text(json.dumps({'runtime_id': 'robot-test',
                'input': 'idle', 'native': {'device': 'quest.shared'}}))
            parameters = {'effective_input_config': {'input_prefix': 'robot-pair-test'}}
            def first_tick():
                (directory/'parameters.json').write_text(json.dumps(parameters))
                return SimpleNamespace(sequence=1)
            hands = Mock()
            hands.snapshot.return_value = {'hands': {'left': None, 'right': None}}
            with patch.object(observer, 'TickSubscriber') as subscriber, \
                 patch.object(observer, 'open_input_observer', return_value=hands) as open_input, \
                 patch.object(observer.signal, 'signal'), \
                 patch.object(observer, 'to_dict', return_value={'source': 'idle', 'terminal': True}):
                subscriber.return_value.receive.side_effect = first_tick
                observer.run(directory)
                self.assertEqual(open_input.call_args.args[1], parameters)
                hands.snapshot.assert_called_once()
                hands.close.assert_called_once()

    def test_snapshot_keeps_per_hand_freshness_and_does_not_renew_old_frames(self):
        from embodied.devices.inputs.quest import workbench
        observer = InputObserver.__new__(InputObserver)
        observer.inputs = {'left': 'left-input', 'right': 'right-input'}
        observer.poses = {'left': 'left-pose', 'right': 'right-pose'}
        observer.samples, observer.positions = {}, {}
        observer.latest, observer.next_sample = None, 0.
        samples = {}
        for side in observer.inputs:
            samples[side+'-input'] = SimpleNamespace(timestamp_us=10_000_000, tracking_valid=True,
                connection_state=1, grip=.7, trigger=.3, joystick_x=.1, joystick_y=.2, buttons_bitset=4)
            samples[side+'-pose'] = SimpleNamespace(timestamp_us=10_000_000, values=[0, 0, 0, 0, 0, 0, 1])
        with patch.object(workbench, 'drain_latest', side_effect=lambda key: samples.pop(key, None)), \
             patch.object(workbench.time, 'time', return_value=10.) as wall, \
             patch.object(workbench.time, 'monotonic', return_value=5.) as now:
            first = observer.snapshot()
            self.assertTrue(first['hands']['left']['buttons']['X'])
            self.assertTrue(first['hands']['right']['buttons']['A'])
            self.assertEqual(first['hands']['left']['trigger'], .3)
            wall.return_value = 10.2
            now.return_value = 5.2
            stale = observer.snapshot()
            self.assertEqual(stale['hands'], {'left': None, 'right': None})
            self.assertEqual(stale['device_timestamp_ns'], first['device_timestamp_ns'])

    def test_optional_device_observer_uses_shared_namespace_without_starting_sdk(self):
        with patch('embodied.devices.inputs.quest.workbench.InputObserver') as observer:
            open_input_observer({'runtime_id': 'robot-left', 'native': {'device': 'quest.shared'}},
                                {'effective_input_config': {'input_prefix': 'robot-pair-test'}})
            observer.assert_called_once_with('robot-pair-test')
        self.assertIsNone(open_input_observer({'runtime_id': 'robot-manual'}, {}))


if __name__ == '__main__':
    unittest.main()
