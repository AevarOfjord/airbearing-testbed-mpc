from __future__ import annotations

import time

import cvxpy as cp
import numpy as np

from airbearing.control.base import Controller, ControllerOutput
from airbearing.control.lqr import linear_AB
from airbearing.dynamics import wrap_angle
from airbearing.spec import SatelliteSpec


class LinearMPC(Controller):
    """Linear MPC in inertial coordinates, body-frame wrench input, theta frozen over the horizon."""

    name = "mpc"

    def __init__(
        self,
        spec: SatelliteSpec,
        horizon: int = 12,
        q_xy: float = 18.0,
        q_yaw: float = 10.0,
        q_v: float = 2.0,
        q_w: float = 1.2,
        r_f: float = 0.15,
        r_m: float = 0.08,
    ):
        self.spec = spec
        self.N = horizon
        self.Q = np.diag([q_xy, q_xy, q_yaw, q_v, q_v, q_w])
        self.R = np.diag([r_f, r_f, r_m])
        self.Qf = self.Q * 4.0
        self._theta_built: float | None = None
        self._prob = None
        self._x0 = None
        self._xref = None
        self._U = None
        self._X = None

    def _wrench_limits(self) -> tuple[np.ndarray, np.ndarray]:
        B = self.spec.allocation_matrix()
        lo_u, hi_u = self.spec.cmd_bounds()
        # Axis-aligned box around the actuator cube (ignores coupling). Conservative enough for teaching.
        hi = np.zeros(3)
        lo = np.zeros(3)
        for j in range(3):
            col = B[j]
            hi[j] = float(np.sum(np.maximum(col * hi_u, col * lo_u)))
            lo[j] = float(np.sum(np.minimum(col * hi_u, col * lo_u)))
        return lo, hi

    def _build(self, theta: float) -> None:
        N, dt = self.N, self.spec.control_dt
        A, B = linear_AB(self.spec.mass, self.spec.Iz, theta, dt)
        lo, hi = self._wrench_limits()
        X = cp.Variable((6, N + 1))
        U = cp.Variable((3, N))
        x0 = cp.Parameter(6)
        xref = cp.Parameter((6, N + 1))
        q = np.sqrt(np.diag(self.Q))
        r = np.sqrt(np.diag(self.R))
        qf = np.sqrt(np.diag(self.Qf))
        cost = 0
        cons = [X[:, 0] == x0]
        for k in range(N):
            cost += cp.sum_squares(cp.multiply(q, X[:, k] - xref[:, k]))
            cost += cp.sum_squares(cp.multiply(r, U[:, k]))
            cons += [X[:, k + 1] == A @ X[:, k] + B @ U[:, k]]
            cons += [U[:, k] <= hi, U[:, k] >= lo]
        cost += cp.sum_squares(cp.multiply(qf, X[:, N] - xref[:, N]))
        self._prob = cp.Problem(cp.Minimize(cost), cons)
        self._x0 = x0
        self._xref = xref
        self._U = U
        self._X = X
        self._theta_built = theta

    def compute(self, state: np.ndarray, ref: np.ndarray) -> ControllerOutput:
        st = np.asarray(state, dtype=float).copy()
        goal = np.zeros(6)
        goal[:3] = ref[:3]
        # error-state yaw: rotate problem so linear wrap is small
        yaw_err = wrap_angle(st[2] - goal[2])
        theta_lin = st[2]
        if self._prob is None or abs(wrap_angle(theta_lin - (self._theta_built or 0.0))) > 0.15:
            self._build(theta_lin)
        x0 = st.copy()
        x0[2] = yaw_err  # relative yaw; A,B still use actual theta for force rotation
        xref = np.zeros((6, self.N + 1))
        xref[0, :] = goal[0]
        xref[1, :] = goal[1]
        self._x0.value = x0
        self._xref.value = xref
        t0 = time.perf_counter()
        status = "fail"
        for solver in (cp.OSQP, cp.CLARABEL):
            try:
                self._prob.solve(solver=solver, warm_start=True, verbose=False)
                if self._U.value is not None:
                    status = str(self._prob.status)
                    break
            except Exception:
                continue
        ms = (time.perf_counter() - t0) * 1000.0
        if self._U.value is None:
            return ControllerOutput(np.zeros(3), solver_ms=ms, status=status, feasible=False)
        u0 = np.asarray(self._U.value[:, 0], dtype=float)
        return ControllerOutput(u0, solver_ms=ms, status=status, feasible=True)
