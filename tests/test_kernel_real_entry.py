"""Check the startup shell using a fake launcher; never opens CAN or a robot."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class KernelRealEntryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.entry = self.root / 'scripts/arm/kernel_real.sh'
        self.entry.parent.mkdir(parents=True)
        source = Path(__file__).resolve().parents[1] / 'scripts/arm/kernel_real.sh'
        shutil.copyfile(source, self.entry)

    def launch(self, launcher, *, confirm=True):
        fake = self.root / 'run.sh'
        fake.write_text('#!/bin/bash\n' + launcher)
        fake.chmod(0o755)
        return subprocess.run(['bash', str(self.entry), '--profile', 'real-manual',
                               *(['--yes'] if confirm else []), '--headless'],
                              input='n\n', text=True, capture_output=True, timeout=3)

    def test_realman_never_prepares_can_and_forwards_arguments(self):
        result = self.launch('if [[ "$1" == can-up ]]; then exit 99; '
                             'else printf "launched: %s\\n" "$*"; fi\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('USB-CAN', result.stdout)
        self.assertIn('RealMan', result.stdout)
        self.assertIn('kernel profiles start --profile real-manual --headless', result.stdout)
        self.assertFalse((self.root / 'outputs').exists())

    def test_runtime_failure_is_preserved(self):
        result = self.launch('echo test-runtime-error >&2; exit 23\n')
        self.assertEqual(result.returncode, 23)
        self.assertIn('test-runtime-error', result.stderr)

    def test_declining_confirmation_does_not_prepare_devices(self):
        result = self.launch('echo unexpected-device-access; exit 99\n', confirm=False)
        self.assertEqual(result.returncode, 0)
        self.assertIn('未启动', result.stdout)
        self.assertNotIn('unexpected-device-access', result.stdout)
        self.assertFalse((self.root / 'outputs').exists())
