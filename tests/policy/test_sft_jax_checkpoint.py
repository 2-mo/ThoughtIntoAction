import json
from pathlib import Path
import tempfile
import unittest
from embodied.policy.pi05_sft.checkpoint import inspect_jax_checkpoint


class JaxCheckpointTests(unittest.TestCase):
    def test_numeric_step_selection_and_horizon(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for step in (900,15000):
                p=root/str(step)
                (p/'params').mkdir(parents=True)
                (p/'params/_METADATA').write_text('{}')
                a=p/'assets/local/x5';a.mkdir(parents=True)
                (a/'norm_stats.json').write_text(json.dumps({'norm_stats':{
                    key:{field:[0.]*7 for field in ('mean','std','q01','q99')}
                    for key in ('state','actions')}}))
            spec=inspect_jax_checkpoint(root)
            self.assertEqual(spec['step'],15000)
            self.assertEqual(spec['model']['action_horizon'],15)
            self.assertEqual(inspect_jax_checkpoint(root/'900',horizon=20)['model']['action_horizon'],20)
            with self.assertRaises(ValueError):inspect_jax_checkpoint(root,horizon=0)
            (root/'15000/assets/local/x5/norm_stats.json').write_text('{}')
            with self.assertRaises(KeyError):inspect_jax_checkpoint(root)


if __name__=='__main__':unittest.main()
