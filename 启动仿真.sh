#!/usr/bin/env bash
set -Eeuo pipefail
WEB_PORT=8081  # 固定地址 http://127.0.0.1:8081/，可用 --port 临时覆盖
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/." && pwd)"
exec "$ROOT_DIR/run.sh" view-arm --camera --port "$WEB_PORT" "$@"
