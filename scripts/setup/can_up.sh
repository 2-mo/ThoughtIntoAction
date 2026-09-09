#!/usr/bin/env bash
# SocketCAN setup only; never sends an arm command or restarts an existing CAN interface.
set -Eeuo pipefail
CAN_INTERFACE="${QUEST_CAN_PORT:-can0}"
CAN_DEVICE="${QUEST_CAN_DEVICE:-/dev/arx_r5}"
STATUS_ONLY=0
DRY_RUN=0
while (($#)); do
    case "$1" in
        --interface) CAN_INTERFACE="${2:?missing interface}"; shift 2 ;;
        --device) CAN_DEVICE="${2:?missing device}"; shift 2 ;;
        --status) STATUS_ONLY=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help)
            printf 'Usage: can-up [--interface can0] [--device /dev/arx_r5] [--status|--dry-run]\n'
            exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
    esac
done
if [[ ! "$CAN_INTERFACE" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    printf 'Invalid CAN interface: %s\n' "$CAN_INTERFACE" >&2
    exit 2
fi
if ((STATUS_ONLY)); then
    exec ip -details -statistics link show dev "$CAN_INTERFACE"
fi
if ip link show dev "$CAN_INTERFACE" >/dev/null 2>&1; then
    ip -details -statistics link show dev "$CAN_INTERFACE"
    printf 'Interface exists; leaving its state unchanged.\n'
    exit 0
fi
if [[ ! -c "$CAN_DEVICE" ]]; then
    printf 'USB-CAN device not found: %s\nSet robot.can_device in configs/local.toml.\n' "$CAN_DEVICE" >&2
    exit 1
fi
if ((DRY_RUN)); then
    printf 'Would create %s from %s using SLCAN s8 (1 Mbit/s), then bring it up.\n' "$CAN_INTERFACE" "$CAN_DEVICE"
    exit 0
fi
command -v slcand >/dev/null
PRIVILEGED=()
if ((EUID != 0)); then PRIVILEGED=(sudo); fi
"${PRIVILEGED[@]}" slcand -o -f -s8 "$CAN_DEVICE" "$CAN_INTERFACE"
# slcand creates the interface asynchronously.
for ((attempt=0; attempt<30; attempt++)); do
    if ip link show dev "$CAN_INTERFACE" >/dev/null 2>&1; then
        "${PRIVILEGED[@]}" ip link set dev "$CAN_INTERFACE" up
        exec ip -brief link show dev "$CAN_INTERFACE"
    fi
    sleep 0.1
done
printf 'Timed out waiting for %s.\n' "$CAN_INTERFACE" >&2
exit 1
