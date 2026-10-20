#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ENV_DIR="$ROOT/outputs/envs/pi05-zero-shot"
if [[ ! -x "$ENV_DIR/bin/python" ]]; then uv venv --python 3.12 "$ENV_DIR"; fi
uv pip install --python "$ENV_DIR/bin/python" torch==2.7.1+cu128 torchvision==0.22.1+cu128 --index-url https://download.pytorch.org/whl/cu128
uv pip install --python "$ENV_DIR/bin/python" -r "$ROOT/src/embodied/policy/pi05_zeroshot/requirements.txt"
"$ENV_DIR/bin/python" "$ROOT/src/embodied/policy/pi05_zeroshot/tokenizer.py"
