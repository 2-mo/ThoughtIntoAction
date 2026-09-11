import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from x5_mapping import SCHEMA, X5Mapping, action_chunks


class MappingTests(unittest.TestCase):
    def test_delta_chunk_uses_anchor_and_repeats_tail(self):
        state = np.array([[1]*6+[.02], [2]*6+[.03]], dtype=float)
        action = np.array([[3]*6+[.04], [4]*6+[.05]], dtype=float)
        chunks = action_chunks(state, action, 3)
        np.testing.assert_array_equal(chunks[0, :, 0], [2, 3, 3])
        np.testing.assert_array_equal(chunks[1, :, 0], [2, 2, 2])
        np.testing.assert_array_equal(chunks[0, :, 6], [.04, .05, .05])

    def test_quantiles_and_absolute_joint_reconstruction(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'stats.json'
            path.write_text(json.dumps({'schema': SCHEMA, 'fps': 30,
                'state': {'q01': [-1]*7, 'q99': [1]*7},
                'action': {'q01': [-.1]*6+[0], 'q99': [.1]*6+[.08]}}))
            mapping = X5Mapping(path)
            norm, outside = mapping.normalize_state(np.zeros(7))
            self.assertFalse(outside)
            np.testing.assert_allclose(norm[:7], 0, atol=1e-6)
            np.testing.assert_array_equal(norm[7:], 0)
            decoded = mapping.absolute_actions(np.zeros((2, 32)), np.ones(7))
            np.testing.assert_allclose(decoded[:, :6], 1, atol=1e-6)
            np.testing.assert_allclose(decoded[:, 6], .04, atol=1e-6)
            with self.assertRaises(ValueError):
                mapping.absolute_actions(np.full((2,32), np.nan), np.ones(7))


if __name__ == '__main__':
    unittest.main()
