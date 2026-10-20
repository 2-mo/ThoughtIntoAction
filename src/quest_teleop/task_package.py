"""Portable task archive. No LeRobot imports, video re-encoding or robot commands."""
import hashlib
import json
from pathlib import Path
import zipfile
from robot_core.storage import save_json


def finalize_task(raw_root, episode_ids, output, fps):
    raw_root, output = Path(raw_root).resolve(), Path(output).resolve()
    archive = output.with_name(output.name + ".zip")
    if archive.exists():
        raise FileExistsError(archive)
    status = raw_root / "tasks" / output.name / "package-status.json"
    status.parent.mkdir(parents=True, exist_ok=True)
    save_json(status, {"state": "saving", "episodes": len(episode_ids), "path": str(archive)})
    temporary = archive.with_name(archive.name + ".partial")
    files, episodes = [], []
    try:
        if not episode_ids or len(set(episode_ids)) != len(episode_ids):
            raise ValueError("Expected nonempty unique episode IDs")
        run_id = None
        schema = None
        for episode_id in episode_ids:
            if Path(episode_id).name != episode_id or episode_id in (".", ".."):
                raise ValueError("Invalid episode id")
            source = (raw_root / episode_id).resolve()
            if not source.is_relative_to(raw_root):
                raise ValueError("Episode path escapes raw root")
            meta = json.loads((source / "episode.json").read_text())
            version = meta.get('schema_version', 1)
            if version not in (1, 2) or schema is not None and version != schema:
                raise ValueError('Unsupported/mixed episode schemas; action definitions must remain separate')
            schema = version
            if meta.get("lifecycle") == "recording":
                raise ValueError("Stop the active recording before packing: " + episode_id)
            if run_id is not None and meta.get("task_run_id") != run_id:
                raise ValueError("Mixed task runs")
            run_id = meta.get("task_run_id")
            members = sorted(file for file in source.rglob("*") if file.is_file())
            for file in members:
                if not file.resolve().is_relative_to(source):
                    raise ValueError("Media path escapes episode")
                files.append((file, Path("episodes") / episode_id / file.relative_to(source)))
            episodes.append({"id": episode_id, "task": meta["task"], "counts": meta.get("counts", {}),
                             "outcome": meta.get("outcome"), "lifecycle": meta.get("lifecycle"),
                             "error": meta.get("error"), "dropped": meta.get("dropped", {})})
        manifest = {"format": "quest-task-archive", "version": 2, "recording_rate_hz": fps,
                    "task_run_id": run_id, "episodes": episodes, "files": {},
                    "selection_policy": "all task episodes; no outcome/lifecycle filtering",
                    "camera_fps": 30, "schema_version": schema,
                    "conversion_layout": "aligned30_raw240_v1" if schema == 1 else "aligned30_raw250_v2",
                    "timestamp_note": "int64 host monotonic ns; RGB receipt is not exposure; device clock is separate"}
        archive.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as package:
            for source, relative in files:
                digest = hashlib.sha256()
                with source.open("rb") as stream:
                    while chunk := stream.read(1024 * 1024):
                        digest.update(chunk)
                manifest["files"][relative.as_posix()] = {"sha256": digest.hexdigest(), "bytes": source.stat().st_size}
                package.write(source, str(Path(output.name) / relative))
            package.writestr(output.name + "/manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
            package.writestr(output.name + "/README.md", package_readme(output.name))
            from robot_core.project import PROJECT_ROOT
            for folder in ("integrations/openpi", "scripts/datasets"):
                for file in sorted((PROJECT_ROOT / folder).rglob("*")):
                    if file.is_file() and "__pycache__" not in file.parts:
                        package.write(file, output.name + "/tools/" + str(file.relative_to(PROJECT_ROOT)))
            for filename in ("export_lerobot.py", "export_video.py", "export_decode.py", "multirate.py", "video.py", "mcap_io.py"):
                package.write(PROJECT_ROOT / "src/quest_teleop" / filename,
                              output.name + "/tools/src/quest_teleop/" + filename)
            package.writestr(output.name + "/tools/src/quest_teleop/__init__.py", "")
            # The portable converter must carry the extracted storage package;
            # compatibility shims alone would work only inside this checkout.
            for module in ('episode_io', 'dataset_tools', 'observations', 'embodied_contracts'):
                for file in sorted((PROJECT_ROOT / 'src' / module).rglob('*.py')):
                    if '__pycache__' not in file.parts:
                        package.write(file, output.name + '/tools/' + str(file.relative_to(PROJECT_ROOT)))
            package.writestr(output.name + '/tools/src/robot_core/__init__.py', '')
            package.write(PROJECT_ROOT / 'src/robot_core/storage.py', output.name + '/tools/src/robot_core/storage.py')
            for registry in ('configs/plugins.toml', 'control/config/components.json'):
                package.write(PROJECT_ROOT / registry, output.name + '/tools/' + registry)
            write_executable(package, output.name + "/tools/convert.sh",
                "#!/usr/bin/env bash\nset -euo pipefail\n"
                'ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"\n'
                'exec bash "$ROOT/scripts/datasets/convert.sh" "$@"\n')
            write_executable(package, output.name + "/转换为LeRobot.sh",
                "#!/usr/bin/env bash\nset -euo pipefail\n"
                'ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"\n'
                'exec bash "$ROOT/tools/convert.sh" "$@"\n')
        with zipfile.ZipFile(temporary) as package:
            if package.testzip() is not None:
                raise RuntimeError("Archive CRC verification failed")
        temporary.replace(archive)
        save_json(status, {"state": "complete", "episodes": len(episodes), "path": str(archive),
                           "bytes": archive.stat().st_size})
        return archive
    except BaseException as exc:
        temporary.unlink(missing_ok=True)
        save_json(status, {"state": "failed", "error": str(exc)})
        raise


def write_executable(package, name, content):
    info = zipfile.ZipInfo(name)
    info.create_system = 3
    info.external_attr = 0o100755 << 16
    package.writestr(info, content)


def package_readme(name):
    return f"""# Quest task archive v2 / 数据包说明

This is a raw acquisition archive, NOT an already filtered LeRobot training dataset.
本包保留本轮所有已停止录制的条目，包括 discarded、failure、interrupted 及其错误/丢样标记。
Do not treat unknown as success. Decide selection on the server; preserve the original archive.

## Files / 文件
- manifest.json: episode IDs, outcomes, lifecycle, counts, and SHA256/size for every episode file.
- episodes/<id>/episode.json: original task, camera calibration, robot model/control settings.
- episode.mcap: robot snapshots and RGB frame indices, lossless Zstd; keep the media files alongside it.
- wrist_rgb.mp4 / exterior_rgb.mp4: independent original RGB videos; exterior exists for dual-camera episodes.
- quality.json: post-recording integrity report; a pass does not label task success.
- tools/: bundled conversion code, dependency setup, and OpenPI adapter snapshot.
- 转换为LeRobot.sh: runnable conversion entry; paths are relative to the script, not your terminal.

## Timing and action meaning
host_monotonic_ns / requested_monotonic_ns: integer nanoseconds, do not convert to float.
RGB host_received_monotonic_ns is receipt time, not exposure time. Device timestamps have their own clock domain.
actual_pos: six measured joint angles (rad); gripper_actual_pos: measured width (m).
requested_pos / requested_gripper_pos: service-side requested targets, not future measured states.
The request timestamp is NOT the original Quest button or pose timestamp.
No state/action samples are removed from this archive.

## Server conversion / 服务器转换
1. Extract ZIP; verify manifest SHA256 before changing any labels.
2. Decide which episodes to train on. Default conversion rejects discarded/failed/incomplete data.
   --skip-invalid explicitly skips rejected episodes and writes reasons to meta/quest_export.json.
   Preserve original labels; relabel a WORKING COPY only after review.
3. On Linux with uv and FFmpeg shared libraries installed, run the bundled entry (no project checkout needed):
   bash 转换为LeRobot.sh
   First run installs an isolated CPU environment automatically (network required).
   默认读取包内 episodes/，写入包内 lerobot_dataset/；不会覆盖已有输出。
4. To explicitly skip rejected episodes or choose an output path:
   bash 转换为LeRobot.sh --skip-invalid --output /path/to/lerobot_dataset
   tools/convert.sh remains an equivalent entry. Run --help for options.

转换显示检查、总帧数、当前条目、速度、剩余时间、统计写入和验证进度。
TorchCodec batches CPU decoding; H264 encoding is streamed, with only statistics-sample PNGs.
Only one episode's raw records are kept in memory. No 240 FPS image expansion.
默认 --decoder torchcodec --decode-workers 0 --decode-batch-size 32，避免小任务进程启动成本。
--decode-workers 2/4 启用多进程；任务按提交顺序消费，最多保留进程数加 1 个解码批次。
不会预读整套视频到内存；--decoder opencv 可切换兼容后端。
默认 --video-preset veryfast --video-threads 4（不足 4 核则减少）。
可调 --video-threads 8 或 --video-preset fast；线程更多不保证更快。
--no-progress disables progress bars. Elapsed time and throughput are saved in meta/quest_export.json.

## Training layout
meta/data/videos: LeRobot v2.1 with one observation per actual RGB frame at nominal 30 Hz.
robot/episode_NNNNNN.npz: all robot state/action rows and exact int64 timestamps (~240 Hz).
The raw archive keeps full metadata/diagnostics, even if a training row is excluded.
Use tools/integrations/openpi/run.py with this dataset. Each training row contains a real
30 FPS image, the latest preceding robot state and requested action at that image timestamp.
OpenPI's STANDARD LeRobot loader reads 15 consecutive 30 Hz actions (~0.5 s) per observation.
The robot/*.npz sidecar is an archival copy of all 240 Hz samples, NOT the training action source.
Raw integer timestamps remain intact. No images are duplicated to 240 FPS.
Dual-camera exports use wrist anchors and the latest preceding exterior frame within the configured
receipt-time tolerance (60 ms by default). Exterior frames can repeat; counts and offsets are reported.
Original archives retain every frame from both cameras. Different camera sets export separately.
The package includes all outcomes; only the server's explicit selection determines training episodes.
Training/inference control rate and action units must match. No hardware actuation is performed by conversion.
"""
