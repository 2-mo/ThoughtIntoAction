#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_DIR="$ROOT/outputs/envs/pi05-sft"
REVISION=215abfb217dbac7d5f1273282331b9b1866c0479
CHECKOUT="$ROOT/outputs/inference/openpi-$REVISION"
if [[ ! -d "$CHECKOUT" ]]; then
    mkdir -p "$ROOT/outputs/inference"
    ARCHIVE="$(mktemp)"
    trap 'rm -f "$ARCHIVE"' EXIT
    curl --fail -L "https://github.com/Physical-Intelligence/openpi/archive/$REVISION.tar.gz" -o "$ARCHIVE"
    printf '%s  %s\n' 20fc377546b741bb31d1e12aa4003e6e6a54fa0ee3c7b7d8b438157cffb3c683 "$ARCHIVE" | sha256sum --check
    tar -xzf "$ARCHIVE" -C "$ROOT/outputs/inference"
fi
if [[ ! -x "$ENV_DIR/bin/python" ]]; then uv venv --python 3.12 "$ENV_DIR"; fi
uv pip install --python "$ENV_DIR/bin/python" torch==2.7.1+cu128 torchvision==0.22.1+cu128 --index-url https://download.pytorch.org/whl/cu128
uv pip install --python "$ENV_DIR/bin/python" -r "$ROOT/src/infer_after_sft/requirements.txt"
"$ENV_DIR/bin/python" - "$CHECKOUT" <<'PY'
from pathlib import Path
import shutil,sys,transformers
assert transformers.__version__=='4.53.2'
shutil.copytree(Path(sys.argv[1])/'src/openpi/models_pytorch/transformers_replace',
                Path(transformers.__file__).parent,dirs_exist_ok=True)
PY
"$ENV_DIR/bin/python" "$ROOT/src/zero-shot/tokenizer.py"
printf 'SFT 环境准备完成。运行 bash src/infer_after_sft/run.sh --preview 验证现场推理。\n'
