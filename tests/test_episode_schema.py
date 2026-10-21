"""Episode formats are identified by namespaced ids, not colliding schema_version numbers (todo D-04)."""
import json
from pathlib import Path
import tempfile
import unittest

from embodied.data.episode.schema import GROOT_RAW, RUNTIME, TELEOP, identify


class EpisodeSchemaTests(unittest.TestCase):
    def test_formats_with_the_same_version_number_are_distinct(self):
        self.assertEqual(identify({'kind': 'quest_teleop_episode', 'schema_version': 1}), TELEOP)
        self.assertEqual(identify({'kind': 'quest_runtime_trace', 'schema_version': 2}), RUNTIME)
        self.assertEqual(identify({'schema_version': 1, 'session_id': 's', 'stage': 'accepted'}), GROOT_RAW)
        self.assertEqual(identify({'schema': RUNTIME, 'schema_version': 2}), RUNTIME)
        for unknown in ({'schema_version': 1}, {'kind': 'quest_teleop_episode', 'schema_version': 2}, {'schema': 'x5.episode/9'}, []):
            with self.assertRaises(ValueError):
                identify(unknown)

    def test_lerobot_export_names_the_right_exporter_for_groot_raw(self):
        from embodied.data.export.exporters import for_episode
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'groot'
            path.mkdir()
            (path / 'episode.json').write_text(json.dumps({'schema_version': 1, 'session_id': 's', 'stage': 'accepted'}))
            with self.assertRaisesRegex(ValueError, 'collect groot export'):
                for_episode(path)

    def test_new_kernel_episodes_record_their_format(self):
        from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
        with tempfile.TemporaryDirectory() as directory:
            writer = RuntimeEpisodeWriter(directory, {'runtime_id': 'test'}, 'task')
            manifest = json.loads((writer.path / 'episode.json').read_text())
            writer.close('duration')
        self.assertEqual(manifest['schema'], RUNTIME)


if __name__ == '__main__':
    unittest.main()
