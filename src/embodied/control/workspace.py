"""Kernel workspace limits for commands that enter through the gateway.

The native Quest route keeps upstream arx-televr behaviour. Every gateway source
(phone, CLI, policies, operations, calibration) is checked here, once, so clients
do not carry their own copies. Settings: config/workspace.json, base frame, metres.
"""
import json
import math
from pathlib import Path

from embodied.contracts.paths import CONTROL_CONFIG

DEFAULT_PATH = CONTROL_CONFIG / 'workspace.json'


class Workspace:
    def __init__(self, table_enabled=False, table_z_m=0., table_clearance_m=0.):
        if type(table_enabled) is not bool:
            raise ValueError('table_enabled must be boolean')
        for name, value in (('table_z_m', table_z_m), ('table_clearance_m', table_clearance_m)):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number')
        if table_clearance_m < 0:
            raise ValueError('table_clearance_m must be nonnegative')
        self.minimum_tcp_z = float(table_z_m + table_clearance_m) if table_enabled else None

    @classmethod
    def load(cls, path=DEFAULT_PATH):
        path = Path(path)
        return cls(**json.loads(path.read_text())) if path.is_file() else cls()

    def describe(self):
        return {'minimum_tcp_z_m': self.minimum_tcp_z}

    def check_pose(self, pose):
        if self.minimum_tcp_z is not None and pose[2] < self.minimum_tcp_z - 1e-6:
            raise ValueError(f'TCP target z={pose[2]:.3f} m is below the table plane {self.minimum_tcp_z:.3f} m')

    def check_path(self, fk, qs):
        """Every sample of the path, like the legacy service; a sampled check, not collision proof."""
        if self.minimum_tcp_z is None:
            return
        lowest = min(float(fk(q)[2]) for q in qs)
        if lowest < self.minimum_tcp_z - 1e-6:
            raise ValueError(f'TCP path reaches z={lowest:.3f} m, below the table plane {self.minimum_tcp_z:.3f} m')
