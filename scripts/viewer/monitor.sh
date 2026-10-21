#!/usr/bin/env bash
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ $# -eq 0 ]]; then
  exec "$ROOT_DIR/run.sh" viewer-control start
fi
exec "$ROOT_DIR/run.sh" view-arm --camera "$@"
