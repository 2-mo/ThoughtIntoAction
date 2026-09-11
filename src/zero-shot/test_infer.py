"""Device-free checks for state/image alignment and PI05 input semantics."""
import gzip
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from infer import episode_observations, prepare_state
from tokenizer import tokenize


class InputsTest(unittest.TestCase):
    def test_physical_range_and_padding(self):
        low = [-3.14, -.05, -.1, -1.6, -1.57, -2., 0.]
        high = [2.618, 3.5, 3.2, 1.55, 1.57, 2., .082]
        for values, expected in [(low, -1), (high, 1)]:
            state, _ = prepare_state(values, 'joint-limits', .082)
            np.testing.assert_allclose(state[:7], expected, atol=1e-6)
            np.testing.assert_array_equal(state[7:], 0)
        with self.assertRaises(ValueError):
            prepare_state([0] * 6 + [float('nan')], 'raw', .082)

    def test_tokenizer_has_bos_and_masked_padding(self):
        class FakeTokenizer:
            def encode(self, prompt, add_bos):
                self.prompt, self.bos = prompt, add_bos
                return [2, 9, 10]
        tokenizer = FakeTokenizer()
        tokens, mask = tokenize(tokenizer, 'pick_up\nblock', np.zeros(32), 5)
        self.assertEqual(tokens, [2, 9, 10, 0, 0])
        self.assertEqual(mask, [True, True, True, False, False])
        self.assertTrue(tokenizer.bos)
        self.assertIn('Task: pick up block, State: 128 128', tokenizer.prompt)
        with self.assertRaises(ValueError):
            tokenize(tokenizer, 'pick', np.zeros(32), 2)

    def test_aligns_previous_state_not_future_and_converts_bgr(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'episode.json').write_text(json.dumps({
                'lifecycle': 'complete', 'control': {'gripper_width_m': .082}}))
            states = [{'host_monotonic_ns': t, 'actual_pos': [q] * 6,
                       'gripper_actual_pos': .04} for t, q in [(100, 1), (200, 2)]]
            frames = [{'host_received_monotonic_ns': t} for t in [150, 99, 200_000_201]]
            for name, data in [('states', states), ('frames', frames)]:
                with gzip.open(root / f'{name}.jsonl.gz', 'wt') as f:
                    f.write('\n'.join(map(json.dumps, data)))
            with patch('quest_teleop.video.read_frame', return_value=np.array([[[10, 20, 30]]], dtype=np.uint8)):
                observations, _ = episode_observations(root, [0])
                np.testing.assert_array_equal(observations[0][0], [[[30, 20, 10]]])
                np.testing.assert_array_equal(observations[0][1][:6], [1] * 6)
                for index in [1, 2, -1, 3]:
                    with self.assertRaises(ValueError):
                        episode_observations(root, [index])


if __name__ == '__main__':
    unittest.main()
