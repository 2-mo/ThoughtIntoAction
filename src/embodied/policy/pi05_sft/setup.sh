#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ENV_DIR="$ROOT/outputs/envs/pi05-sft"
python3 "$ROOT/scripts/setup/dependencies.py" --fetch --name openpi
CHECKOUT="$(PYTHONPATH="$ROOT/src" python3 -c 'from embodied.core.dependencies import dependency_root; print(dependency_root("openpi"))')"
if [[ ! -x "$ENV_DIR/bin/python" ]]; then uv venv --python 3.12 "$ENV_DIR"; fi
uv pip install --python "$ENV_DIR/bin/python" torch==2.7.1+cu128 torchvision==0.22.1+cu128 --index-url https://download.pytorch.org/whl/cu128
uv pip install --python "$ENV_DIR/bin/python" -r "$ROOT/src/embodied/policy/pi05_sft/requirements.txt"
# Runtime uses installed packages. The managed checkout remains only a build/training resource.
uv pip install --python "$ENV_DIR/bin/python" --no-deps "$CHECKOUT" "$CHECKOUT/packages/openpi-client"
"$ENV_DIR/bin/python" - "$CHECKOUT" <<'PY'
from pathlib import Path
import shutil,sys,transformers
assert transformers.__version__=='4.53.2'
shutil.copytree(Path(sys.argv[1])/'src/openpi/models_pytorch/transformers_replace',
                Path(transformers.__file__).parent,dirs_exist_ok=True)
PY
env PYTHONPATH="$ROOT/src" "$ENV_DIR/bin/python" -m embodied.policy.pi05_zeroshot.tokenizer
printf 'SFT 环境准备完成。运行 bash src/embodied/policy/pi05_sft/run.sh --preview 验证现场推理。\n'
