#!/usr/bin/python3
"""Grant plugdev access to D435i IIO attributes, without following sysfs symlinks."""
import grp
import os
from pathlib import Path
import stat
import sys


def apply(device):
    device = Path(device).resolve(strict=True)
    if not device.is_relative_to('/sys/devices') or not device.name.startswith('iio:device'):
        raise ValueError('Expected an IIO device under /sys/devices')
    usb = next((p for p in device.parents if (p / 'idVendor').exists()), None)
    if usb is None or (usb / 'idVendor').read_text().strip() != '8086' or (usb / 'idProduct').read_text().strip() != '0b3a':
        raise ValueError('Not a RealSense D435i')
    gid = grp.getgrnam('plugdev').gr_gid
    for root, dirs, files in os.walk(device, followlinks=False):
        dirs[:] = [name for name in dirs if not (Path(root) / name).is_symlink()]
        for path in [Path(root), *(Path(root) / name for name in files)]:
            mode = path.lstat().st_mode
            if not (stat.S_ISREG(mode) or stat.S_ISDIR(mode)):
                continue
            # Mirror owner permissions to the device group; preserve read-only attributes.
            os.chown(path, -1, gid, follow_symlinks=False)
            os.chmod(path, stat.S_IMODE(mode) | ((mode & 0o700) >> 3))


if __name__ == '__main__':
    apply(sys.argv[1])
