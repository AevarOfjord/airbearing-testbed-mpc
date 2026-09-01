"""Thruster-count-agnostic wrench allocator.

Maps a body-frame wrench [Fx, Fy, Mz] onto whatever actuators the JSON lists.
Binary solenoids are solved as a [0, 1] relaxation; callers may round or apply
the duty as PWM over the control interval.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import cvxpy as cp
import numpy as np

from airbearing.spec import SatelliteSpec


@dataclass
class Allocation:
    cmd: np.ndarray  # in [cmd_min, cmd_max] per thruster
    cmd_rounded: np.ndarray
    wrench_achieved: np.ndarray
    residual: np.ndarray
    solver_ms: float
    status: str
    rw_torque: float = 0.0


def _solve_qp(B: np.ndarray, wrench: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> tuple[np.ndarray, str, float]:
    n = B.shape[1]
    u = cp.Variable(n)
    residual = B @ u - wrench
    # Prefer smaller total effort so unused thrusters stay off.
    cost = cp.sum_squares(residual) + 1e-4 * cp.sum_squares(u)
    prob = cp.Problem(cp.Minimize(cost), [u >= lo, u <= hi])
    t0 = time.perf_counter()
    status = "fail"
    for solver in (cp.OSQP, cp.CLARABEL):
        try:
            prob.solve(solver=solver, verbose=False, warm_start=True)
            if u.value is not None:
                status = str(prob.status)
                break
        except Exception:
            continue
    solver_ms = (time.perf_counter() - t0) * 1000.0
    if u.value is None:
        return np.zeros(n), "infeasible", solver_ms
    cmd = np.clip(np.asarray(u.value, dtype=float).reshape(n), lo, hi)
    return cmd, status, solver_ms


def allocate(
    spec: SatelliteSpec,
    wrench: np.ndarray,
    round_binary: bool = False,
    rw_torque_request: float = 0.0,
) -> Allocation:
    wrench = np.asarray(wrench, dtype=float).reshape(3)
    B = spec.allocation_matrix()
    lo, hi = spec.cmd_bounds()
    w = wrench.copy()
    rw_t = 0.0
    if spec.reaction_wheel is not None:
        lim = spec.reaction_wheel.max_torque
        rw_t = float(np.clip(rw_torque_request, -lim, lim))
        w[2] = w[2] - rw_t  # wheel torque on spacecraft is -wheel motor torque convention:
        # request Mz includes wheel; we apply +rw_t on spacecraft via wheel, rest via thrusters.
        # Here: wrench[2] is desired spacecraft Mz. Wheel can provide rw_t, thrusters provide rest.
        w[2] = wrench[2] - rw_t

    cmd, status, solver_ms = _solve_qp(B, w, lo, hi)
    cmd_rounded = cmd.copy()
    for i, t in enumerate(spec.thrusters):
        if t.type == "binary_solenoid":
            if round_binary:
                cmd_rounded[i] = 1.0 if cmd[i] >= 0.5 else 0.0
            else:
                cmd_rounded[i] = cmd[i]  # duty / PWM-equivalent
        elif t.type == "pwm_fan":
            cmd_rounded[i] = cmd[i]
        else:
            cmd_rounded[i] = cmd[i]
    achieved = B @ cmd_rounded
    if spec.reaction_wheel is not None:
        achieved = achieved + np.array([0.0, 0.0, rw_t])
    return Allocation(
        cmd=cmd,
        cmd_rounded=cmd_rounded,
        wrench_achieved=achieved,
        residual=achieved - wrench,
        solver_ms=solver_ms,
        status=status,
        rw_torque=rw_t,
    )


def pinv_allocate(spec: SatelliteSpec, wrench: np.ndarray) -> np.ndarray:
    """Unconstrained least-squares fallback (clipped)."""
    B = spec.allocation_matrix()
    lo, hi = spec.cmd_bounds()
    u = np.linalg.pinv(B) @ np.asarray(wrench, dtype=float).reshape(3)
    return np.clip(u, lo, hi)
