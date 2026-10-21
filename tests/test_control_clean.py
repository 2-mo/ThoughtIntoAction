import json
import os
from pathlib import Path

from embodied.workstation.control import stale_runs

BOOT = Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def run(root, name, pid=999_999_999, live=False):
    directory = root / name
    directory.mkdir()
    (directory / 'runtime.json').write_text(json.dumps({'supervisor_pid': pid, 'boot_id': BOOT}))
    if live:
        (directory / 'commands.sock').touch()
        (directory / 'runtime.ready').touch()
    return directory


def test_keeps_the_newest_and_running_kernels(tmp_path):
    oldest_live = run(tmp_path, '20260101-000000-arx5-a', pid=os.getpid(), live=True)
    old = [run(tmp_path, f'2026010{i}-000000-arx5-b') for i in (2, 3)]
    newest = [run(tmp_path, f'2026010{i}-000000-arx5-c') for i in (4, 5)]
    (tmp_path / 'check-x5.json').write_text('{}')
    stale = stale_runs(2, tmp_path)
    assert stale == old
    assert oldest_live not in stale and not set(newest) & set(stale)
    assert stale_runs(10, tmp_path) == []
