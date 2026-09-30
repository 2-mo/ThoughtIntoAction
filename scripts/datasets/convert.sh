#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE="$HOME/work/data/quest_x5"
OUTPUT="$HOME/work/data/quest_x5/lerobot/x5_$(date +%Y%m%d-%H%M%S)"
if [[ -f "$ROOT/../manifest.json" && -d "$ROOT/../episodes" ]]; then
    SOURCE="$ROOT/../episodes"
    OUTPUT="$ROOT/../lerobot_dataset"
fi
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    cat <<EOF
将 Quest 采集数据转换为 LeRobot v2.1。首次运行自动安装隔离的 CPU 导出环境。
用法: bash "$0" [选项]
  --source PATH          原始 episode 或 episodes 目录（默认：$SOURCE）
  --output PATH          新数据集目录（默认：$OUTPUT；不覆盖已有目录）
  --skip-invalid         显式跳过失败、丢弃、不完整等无效条目，并记录原因
  --task TEXT            仅导出指定任务
  --repo-id ID           数据集标识（默认 local/quest_x5，不上传）
  --fps 30               视频观测频率；完整机器人样本另存 NPZ
  --video-threads N      H264 编码线程数（默认最多 4）
  --video-preset NAME    编码速度档位（默认 veryfast；可用 fast、medium 等）
  --decoder NAME        解码后端：torchcodec（默认）或 opencv
  --decode-workers N    解码进程数（默认 0，在主进程内解码；2/4 可启用多进程）
  --decode-batch-size N  每批帧数（默认 32；调小可减少内存）
  --no-progress          关闭进度条
可设置 LEROBOT_EXPORT_PYTHON 使用已有的固定版本导出 Python。
EOF
    exit 0
fi
PYTHON="${LEROBOT_EXPORT_PYTHON:-$ROOT/outputs/envs/lerobot-export/bin/python}"
if [[ -z "${LEROBOT_EXPORT_PYTHON:-}" ]]; then
    if [[ ! -x "$PYTHON" ]] || ! "$PYTHON" -c 'import numpy, av, tqdm, mcap, zstandard; import importlib.util; assert all(importlib.util.find_spec(name) for name in ("lerobot", "torchcodec"))' >/dev/null 2>&1; then
        printf '首次运行：准备 LeRobot 转换环境，下载依赖可能需要几分钟。\n' >&2
        bash "$ROOT/scripts/datasets/setup_export.sh"
    fi
fi
if [[ ! -x "$PYTHON" ]]; then
    printf '导出 Python 不可执行：%s\n' "$PYTHON" >&2
    exit 1
fi
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
exec "$PYTHON" -m quest_teleop.export_lerobot --source "$SOURCE" --output "$OUTPUT" "$@"
