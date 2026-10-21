import io
import pickle
import unittest
from embodied.policy.pi05_sft.checkpoint import DEFAULT_CHECKPOINT,inspect_checkpoint,MetadataReader


class CheckpointTests(unittest.TestCase):
    def test_actual_training_metadata(self):
        spec=inspect_checkpoint(DEFAULT_CHECKPOINT)
        self.assertEqual(spec['step'],2500)
        self.assertEqual(spec['model']['action_horizon'],15)
        self.assertEqual(spec['asset_id'],'local/x5_blue_cube_to_white')

    def test_metadata_does_not_import_arbitrary_globals(self):
        with self.assertRaises(pickle.UnpicklingError):
            MetadataReader(io.BytesIO(pickle.dumps(eval))).load()


if __name__=='__main__':unittest.main()
