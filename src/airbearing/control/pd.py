from __future__ import annotations

import numpy as np

from airbearing.control.base import Controller, ControllerOutput
from airbearing.dynamics import rot2, wrap_angle
from airbearing.spec import SatelliteSpec


class PDController(Controller):
    name = "pd"

    def __init__(self, spec: SatelliteSpec, kp_xy: float = 1.2, kd_xy: float = 3.5, kp_yaw: float = 0.8, kd_yaw: float = 1.6):
        self.spec = spec
        self.kp_xy = kp_xy
        self.kd_xy = kd_xy
        self.kp_yaw = kp_yaw
        self.kd_yaw = kd_yaw

    def compute(self, state: np.ndarray, ref: np.ndarray) -> ControllerOutput:
        x, y, th, vx, vy, om = state
        rx, ry, rth = ref[:3]
        ex, ey = rx - x, ry - y
        eth = wrap_angle(rth - th)
        # inertial force command, then rotate to body
        Fx_i = self.kp_xy * ex - self.kd_xy * vx
        Fy_i = self.kp_xy * ey - self.kd_xy * vy
        F_b = rot2(th).T @ np.array([Fx_i, Fy_i])
        Mz = self.kp_yaw * eth - self.kd_yaw * om
        # clip to a plausible wrench box from B
        B = self.spec.allocation_matrix()
        fmax = np.max(np.abs(B), axis=1)
        F_b = np.clip(F_b, -fmax[:2], fmax[:2])
        Mz = float(np.clip(Mz, -fmax[2], fmax[2]))
        return ControllerOutput(np.array([F_b[0], F_b[1], Mz]))
