#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
exec bash "$ROOT/src/embodied/policy/pi05_sft/run.sh" --backend jax "$@"
