"""Shared Cartesian safety boundary, expressed in robot base coordinates."""
import math
import numpy as np


class TablePlane:
    def __init__(self, table_z=None, clearance=0.):
        if table_z is not None and not math.isfinite(table_z):
            raise ValueError('Table height must be finite')
        if not math.isfinite(clearance) or clearance < 0:
            raise ValueError('Table clearance must be finite and nonnegative')
        self.minimum_z = None if table_z is None else table_z + clearance

    def clamp(self, position):
        position = np.asarray(position, dtype=float).copy()
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('TCP target must contain three finite coordinates')
        if self.minimum_z is not None:
            position[2] = max(position[2], self.minimum_z)
        return position
