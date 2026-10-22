#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ENV_DIR="$ROOT/outputs/envs/pi05-jax"
python3 "$ROOT/scripts/setup/dependencies.py" --fetch --name openpi
CHECKOUT="$(PYTHONPATH="$ROOT/src" python3 -c 'from embodied.core.dependencies import dependency_root; print(dependency_root("openpi"))')"
if [[ ! -x "$ENV_DIR/bin/python" ]]; then uv venv --python 3.12 "$ENV_DIR"; fi
uv pip install --python "$ENV_DIR/bin/python" -r "$ROOT/src/embodied/policy/pi05_sft/requirements-jax.txt" --extra-index-url https://download.pytorch.org/whl/cu128 --index-strategy unsafe-best-match
uv pip install --python "$ENV_DIR/bin/python" --no-deps "$CHECKOUT" "$CHECKOUT/packages/openpi-client"
env PYTHONPATH="$ROOT/src" "$ENV_DIR/bin/python" -m embodied.policy.pi05_zeroshot.tokenizer
JAX_PLATFORMS=cuda "$ENV_DIR/bin/python" -c 'import jax; print(jax.devices())'
