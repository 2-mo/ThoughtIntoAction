#!/usr/bin/env bash
set -euo pipefail
SIM_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$SIM_PROJECT_DIR/scripts/shortcuts/启动工作台.sh" start --mode simulation "$@"
