"""Finalize LeRobot outside control threads and outside the robot Python environment."""
from pathlib import Path
import os
import subprocess
from robot_core.project import PROJECT_ROOT
from robot_core.storage import save_json


def finalize_lerobot(raw_path, fps, output=None):
    raw_path = Path(raw_path).resolve()
    python = PROJECT_ROOT/'outputs/envs/lerobot-export/bin/python'
    if not python.is_file():
        raise RuntimeError('LeRobot 环境缺失；请先运行 scripts/datasets/setup_export.sh，原始数据已保留')
    output = Path(output).resolve() if output is not None else raw_path/'lerobot'
    status = raw_path/'lerobot-status.json'
    save_json(status, {'state': 'saving', 'fps': fps})
    env = {**os.environ, 'PYTHONPATH': str(PROJECT_ROOT/'src')}
    with (raw_path/'lerobot-save.log').open('w') as log:
        result = subprocess.run([str(python), '-m', 'quest_teleop.export_lerobot', '--source', str(raw_path),
            '--output', str(output), '--fps', str(fps), '--preserve-samples'],
            env=env, stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        save_json(status, {'state': 'failed', 'returncode': result.returncode})
        raise RuntimeError(f'LeRobot 保存失败，原始记录保留；详见 {raw_path / "lerobot-save.log"}')
    save_json(status, {'state': 'complete', 'path': str(output), 'fps': fps})
    return output


def finalize_task(raw_root, episode_ids, output, fps):
    """Export only accepted episodes in this task; retain selection for retry/audit."""
    import json
    raw_root, output = Path(raw_root).resolve(), Path(output).resolve()
    selection = raw_root / "tasks" / output.name
    selection.mkdir(parents=True, exist_ok=True)
    for link in selection.glob("*"):
        if link.is_symlink():
            link.unlink()
    for episode_id in episode_ids:
        if Path(episode_id).name != episode_id:
            raise ValueError("Invalid episode id")
        source = raw_root / episode_id
        meta = json.loads((source / "episode.json").read_text())
        if meta.get("lifecycle") != "complete" or meta.get("outcome") in ("discarded", "failure"):
            raise ValueError("Task contains incomplete or rejected episode: " + episode_id)
        (selection / episode_id).symlink_to(source, target_is_directory=True)
    return finalize_lerobot(selection, fps, output)
