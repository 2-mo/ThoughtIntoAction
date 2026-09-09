#!/usr/bin/env bash
set -Eeuo pipefail
SERVICE_SCRIPT="${QUEST_XR_SERVICE_SCRIPT:-/opt/apps/roboticsservice/runService.sh}"
if [[ ! -f "$SERVICE_SCRIPT" ]]; then
    printf 'XRoboToolkit PC Service not found: %s\nInstall the matching .deb first.\n' "$SERVICE_SCRIPT" >&2
    exit 1
fi
cd -- "$(dirname -- "$SERVICE_SCRIPT")"
exec env -u LD_LIBRARY_PATH bash "$SERVICE_SCRIPT" "$@"
