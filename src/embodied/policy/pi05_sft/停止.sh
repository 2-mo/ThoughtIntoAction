#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
exec bash "$ROOT/scripts/inference/停止Pi05.sh" "$@"
