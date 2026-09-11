#!/usr/bin/env bash
set -Eeuo pipefail
WEB_PORT=8081  # 固定地址 http://127.0.0.1:8081/，可用 --port 临时覆盖
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/." && pwd)"
if [[ $# -eq 0 ]]; then
  exec "$ROOT_DIR/run.sh" viewer-control start
fi
exec "$ROOT_DIR/run.sh" view-arm --camera --port "$WEB_PORT" "$@"
