#!/usr/bin/env bash
set -Eeuo pipefail
case "${1:-start}" in
    start|wear)
        adb -d shell input keyevent KEYCODE_WAKEUP
        adb -d shell am broadcast -a com.oculus.vrpowermanager.prox_close
        if [[ "${1:-start}" == start ]]; then
            adb -d shell am start -n com.xrobotoolkit.client.quest/com.unity3d.player.UnityPlayerActivity
        fi
        ;;
    unwear)
        adb -d shell am broadcast -a com.oculus.vrpowermanager.prox_far
        ;;
    stop)
        adb -d shell am broadcast -a com.oculus.vrpowermanager.prox_far
        adb -d shell input keyevent KEYCODE_SLEEP
        ;;
    *)
        printf 'Usage: quest-wake [start|wear|unwear|stop]\n' >&2
        exit 2
        ;;
esac
