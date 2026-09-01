from __future__ import annotations

import numpy as np
from scipy import linalg

from airbearing.control.base import Controller, ControllerOutput
from airbearing.dynamics import rot2, wrap_angle
from airbearing.spec import SatelliteSpec


def linear_AB(mass: float, Iz: float, theta: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Discrete (A, B) for inertial state, body-frame wrench input, frozen theta."""
    A = np.eye(6)
    A[0, 3] = dt
    A[1, 4] = dt
    A[2, 5] = dt
    c, s = np.cos(theta), np.sin(theta)
    # vdot inertial = R(theta) F_body / m
    B = np.zeros((6, 3))
    B[3, 0] = c / mass * dt
    B[3, 1] = -s / mass * dt
    B[4, 0] = s / mass * dt
    B[4, 1] = c / mass * dt
    B[5, 2] = dt / Iz
    return A, B


class LQRController(Controller):
    name = "lqr"

    def __init__(self, spec: SatelliteSpec):
        self.spec = spec
        self.Q = np.diag([12.0, 12.0, 8.0, 4.0, 4.0, 2.0])
        self.R = np.diag([0.8, 0.8, 0.4])
        self._K_cache: dict[int, np.ndarray] = {}

    def _K(self, theta: float) -> np.ndarray:
        key = int(round(theta * 20))  # ~3 deg bins
        if key in self._K_cache:
            return self._K_cache[key]
        A, B = linear_AB(self.spec.mass, self.spec.Iz, theta, self.spec.control_dt)
        P = linalg.solve_discrete_are(A, B, self.Q, self.R)
        K = np.linalg.solve(self.R + B.T @ P @ B, B.T @ P @ A)
        self._K_cache[key] = K
        return K

    def compute(self, state: np.ndarray, ref: np.ndarray) -> ControllerOutput:
        st = np.asarray(state, dtype=float).copy()
        r = np.zeros(6)
        r[:3] = ref[:3]
        err = st - r
        err[2] = wrap_angle(st[2] - r[2])
        u = -self._K(st[2]) @ err
        Balloc = self.spec.allocation_matrix()
        fmax = np.sum(np.abs(Balloc), axis=1)
        u = np.clip(u, -fmax, fmax)
        return ControllerOutput(u)
