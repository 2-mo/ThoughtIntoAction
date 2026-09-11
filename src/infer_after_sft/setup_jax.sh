#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_DIR="$ROOT/outputs/envs/pi05-jax"
if [[ ! -d "$ROOT/outputs/inference/openpi-215abfb217dbac7d5f1273282331b9b1866c0479" ]]; then
    echo '请先运行 bash src/infer_after_sft/setup.sh 准备固定版本 OpenPI 源码'
    exit 1
fi
if [[ ! -x "$ENV_DIR/bin/python" ]]; then uv venv --python 3.12 "$ENV_DIR"; fi
uv pip install --python "$ENV_DIR/bin/python" -r "$ROOT/src/infer_after_sft/requirements-jax.txt" --extra-index-url https://download.pytorch.org/whl/cu128 --index-strategy unsafe-best-match
"$ENV_DIR/bin/python" "$ROOT/src/zero-shot/tokenizer.py"
JAX_PLATFORMS=cuda "$ENV_DIR/bin/python" -c 'import jax; print(jax.devices())'
