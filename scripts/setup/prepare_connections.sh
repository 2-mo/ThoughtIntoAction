#!/usr/bin/env bash
# Prepare USB and CAN connections; start teleoperation separately.
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    printf 'Usage: ./run.sh prepare-connections\nPrepare Quest USB forwarding and the configured CAN interface.\n'
    exit 0
fi
if [[ $# -gt 0 ]]; then
    printf 'Unexpected argument: %s\n' "$1" >&2
    exit 2
fi
"$ROOT_DIR/run.sh" setup-quest-usb
"$ROOT_DIR/run.sh" can-up
printf 'Connections prepared. Start PC Service with ./run.sh start-xr-service.\n'
