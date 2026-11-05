#!/usr/bin/env bash
# SocketCAN setup only; never sends an arm command or restarts an existing CAN interface.
set -Eeuo pipefail
CAN_INTERFACE="${QUEST_CAN_PORT:-can0}"
CAN_DEVICE="${QUEST_CAN_DEVICE:-/dev/arx_r5}"
STATUS_ONLY=0
DRY_RUN=0
ALL=0
while (($#)); do
    case "$1" in
        --interface) CAN_INTERFACE="${2:?missing interface}"; shift 2 ;;
        --device) CAN_DEVICE="${2:?missing device}"; shift 2 ;;
        --all) ALL=1; shift ;;
        --status) STATUS_ONLY=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help)
            printf 'Usage: can-up [--interface can0] [--device /dev/arx_r5] [--all] [--status|--dry-run]\n'
            printf '  --all  create one interface per ARX USB2CAN adapter, in serial order: can0, can1, ...\n'
            exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
    esac
done
if ((ALL)); then
    # Dual-arm rigs carry one USB2CAN adapter per arm. Name them by serial
    # order so can0/can1 stay stable across replugging.
    mapfile -t LINKS < <(ls -1 /dev/serial/by-id/usb-ARX_USB2CAN_*-if00 2>/dev/null | LC_ALL=C sort)
    if ((${#LINKS[@]} == 0)); then
        printf 'No ARX USB2CAN adapter found under /dev/serial/by-id.\n' >&2
        exit 1
    fi
    PRIVILEGED=()
    if ((EUID != 0)); then
        printf '需要管理员权限，将 %d 个 USB-CAN 设备创建为 can0..can%d 并启用。\n' "${#LINKS[@]}" "$((${#LINKS[@]} - 1))"
        printf '这一步仅准备通信接口，不会使能电机。\n'
        PRIVILEGED=(sudo)
    fi
    for index in "${!LINKS[@]}"; do
        interface="can$index"
        device="$(readlink -f "${LINKS[$index]}")"
        if ip link show dev "$interface" >/dev/null 2>&1; then
            ip -brief link show dev "$interface"
            printf 'Interface exists; leaving its state unchanged.\n'
            continue
        fi
        if ((DRY_RUN)); then
            printf 'Would create %s from %s\n' "$interface" "$device"
            continue
        fi
        "${PRIVILEGED[@]}" slcand -o -f -s8 "$device" "$interface"
        for ((attempt = 0; attempt < 30; attempt++)); do
            ip link show dev "$interface" >/dev/null 2>&1 && break
            sleep 0.1
        done
        "${PRIVILEGED[@]}" ip link set dev "$interface" up
        ip -brief link show dev "$interface"
    done
    exit 0
fi
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
if ((EUID != 0)); then
    printf '需要管理员权限，将 USB-CAN 设备 %s 创建为通信接口 %s 并启用。\n' "$CAN_DEVICE" "$CAN_INTERFACE"
    printf '这一步仅准备通信接口，不会使能电机。若 sudo 提示密码，请输入当前 Ubuntu 用户的登录密码（输入时不显示字符）。\n'
    PRIVILEGED=(sudo)
fi
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
