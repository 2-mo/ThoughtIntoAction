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

    def test_can_details_go_to_log_and_arguments_reach_control(self):
        result = self.launch('if [[ "$1" == can-up ]]; then echo low-level-can-details; '
                             'else printf "launched: %s\\n" "$*"; fi\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('low-level-can-details', result.stdout)
        self.assertIn('USB-CAN · 已就绪', result.stdout)
        self.assertIn('kernel profiles start --profile real-manual --headless', result.stdout)
        log, = (self.root / 'outputs/logs/bench').glob('can-*.log')
        self.assertIn('low-level-can-details', log.read_text())

    def test_can_failure_preserves_reason_and_never_starts_control(self):
        result = self.launch('if [[ "$1" == can-up ]]; then echo test-can-error >&2; '
                             'exit 23; else echo unexpected-control-start; fi\n')
        self.assertEqual(result.returncode, 23)
        self.assertIn('test-can-error', result.stderr)
        self.assertIn('退出码 23', result.stderr)
        self.assertNotIn('unexpected-control-start', result.stdout)

    def test_declining_confirmation_does_not_prepare_devices(self):
        result = self.launch('echo unexpected-device-access; exit 99\n', confirm=False)
        self.assertEqual(result.returncode, 0)
        self.assertIn('未启动', result.stdout)
        self.assertNotIn('unexpected-device-access', result.stdout)
        self.assertFalse((self.root / 'outputs').exists())
