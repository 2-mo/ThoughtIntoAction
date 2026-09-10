#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_DIR="$ROOT/outputs/envs/lerobot-export"
if [[ ! -x "$ENV_DIR/bin/python" ]]; then uv venv --python 3.12 "$ENV_DIR"; fi
uv pip install --python "$ENV_DIR/bin/python" torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
uv pip install --python "$ENV_DIR/bin/python" -r "$ROOT/scripts/datasets/export_requirements.txt"
uv pip install --python "$ENV_DIR/bin/python" --no-deps 'lerobot @ https://github.com/huggingface/lerobot/archive/0cf864870cf29f4738d3ade893e6fd13fbd7cdb5.tar.gz'
