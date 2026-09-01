"""Parameter estimation from a log: F_max scale and optional command delay.

Layout (positions, directions, types) stays in the JSON the student drew.
Default PE fit is a single F_max scale plus an optional 0–2 step delay.
`--full` also fits mass, Iz, and per-thruster F_max.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares

from airbearing.logschema import LogSchemaError, load_log, write_pose_csv
from airbearing.spec import SatelliteSpec, load_vehicle, save_vehicle, spec_to_dict


def _unit_B(spec: SatelliteSpec) -> np.ndarray:
    B = np.zeros((3, spec.n_thrusters))
    for i, t in enumerate(spec.thrusters):
        r = t.position - spec.com
        f = t.force_direction
        B[:, i] = [f[0], f[1], r[0] * f[1] - r[1] * f[0]]
    return B


def _lag_commands(u: np.ndarray, t: np.ndarray, taus: np.ndarray) -> np.ndarray:
    y = np.zeros(u.shape[1])
    out = np.zeros_like(u)
    for i in range(len(t)):
        dt = float(t[i] - t[i - 1]) if i else 1e-2
        for k, tau in enumerate(taus):
            if tau < 1e-4:
                y[k] = u[i, k]
            else:
                y[k] += dt * (u[i, k] - y[k]) / max(tau, 1e-4)
        out[i] = y
    return out


def _delay_u(u: np.ndarray, steps: int) -> np.ndarray:
    if steps <= 0:
        return u
    pad = np.zeros((steps, u.shape[1]))
    return np.vstack([pad, u[:-steps]])


def _meas_wrench(log: dict, mass: float, Iz: float, dlin: float, drot: float):
    t = log["t"]
    vx, vy, om = log["vx"], log["vy"], log["omega"]
    yaw = log["yaw"]
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    alpha = np.gradient(om, t)
    c, s = np.cos(yaw), np.sin(yaw)
    Fx_i = mass * ax + dlin * vx
    Fy_i = mass * ay + dlin * vy
    Fx_b = c * Fx_i + s * Fy_i
    Fy_b = -s * Fx_i + c * Fy_i
    Mz = Iz * alpha + drot * om
    return np.vstack([Fx_b, Fy_b, Mz])


def _residual_plot(t, meas, pred, path: Path) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = ["Fx (N)", "Fy (N)", "Mz (N m)"]
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, meas[i], label="measured", lw=1.2)
        ax.plot(t, pred[i], label="model", lw=1.0, alpha=0.85)
        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("Identify residual (SI)")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def identify_from_log(
    log_csv: str | Path,
    vehicle: str | Path,
    *,
    out: str | Path | None = None,
    mode: str = "fmax_scale",
    delay_steps: int | str | None = "auto",
    residual_png: str | Path | None = None,
) -> dict[str, Any]:
    spec = load_vehicle(vehicle)
    try:
        log = load_log(log_csv)
    except LogSchemaError:
        raise
    if "u" not in log:
        raise LogSchemaError("identify needs command columns u0..uN (SI duty / on-off)")
    for col in ("vx", "vy", "omega"):
        if col not in log:
            raise LogSchemaError(f"identify needs {col}")
    t = log["t"]
    u = log["u"]
    if u.shape[1] != spec.n_thrusters:
        raise LogSchemaError(
            f"log has {u.shape[1]} command columns, vehicle has {spec.n_thrusters} thrusters"
        )
    taus = np.array([float(th.tau) for th in spec.thrusters])
    u_lag = _lag_commands(u, t, taus)
    B1 = _unit_B(spec)
    F0 = np.array([th.F_max for th in spec.thrusters], dtype=float)
    dlin = float(spec.linear_damping)
    drot = float(spec.rotational_damping)
    delays = [0, 1, 2] if delay_steps in (None, "auto") else [int(delay_steps)]

    def eval_delay(d: int, full: bool) -> tuple[Any, np.ndarray, int]:
        u_eff = _delay_u(u_lag, d)
        if not full:
            meas = _meas_wrench(log, spec.mass, spec.Iz, dlin, drot)

            def residual(p):
                s = float(p[0])
                pred = (B1 * (s * F0)) @ u_eff.T
                return np.concatenate([meas[0] - pred[0], meas[1] - pred[1], meas[2] - pred[2]])

            sol = least_squares(residual, [1.0], bounds=([0.05], [20.0]), max_nfev=120)
            return sol, residual(sol.x), d
        x0 = np.concatenate([[spec.mass, spec.Iz], F0])
        n = spec.n_thrusters

        def residual(p):
            m, Iz = p[0], p[1]
            F = p[2:]
            meas = _meas_wrench(log, m, Iz, dlin, drot)
            pred = (B1 * F) @ u_eff.T
            return np.concatenate([meas[0] - pred[0], meas[1] - pred[1], meas[2] - pred[2]])

        lo = np.concatenate([[0.05, 1e-4], np.full(n, 1e-4)])
        hi = np.concatenate([[500.0, 50.0], np.full(n, 50.0)])
        sol = least_squares(residual, x0, bounds=(lo, hi), max_nfev=200)
        return sol, residual(sol.x), d

    full = mode in ("full", "mass_iz_fmax")
    scored = []
    for d in delays:
        sol, fun, dd = eval_delay(d, full)
        rmse = float(np.sqrt(np.mean(fun ** 2)))
        scored.append((rmse, sol, fun, dd))
    scored.sort(key=lambda z: z[0])
    rmse, sol, fun, best_d = scored[0]

    data = spec_to_dict(spec)
    if full:
        m, Iz = float(sol.x[0]), float(sol.x[1])
        F = np.array(sol.x[2:], dtype=float)
        scale = float(np.mean(F / np.maximum(F0, 1e-9)))
        data["mass"] = m
        data["Iz"] = Iz
        for i, th in enumerate(data["thrusters"]):
            th["F_max"] = float(F[i])
    else:
        scale = float(sol.x[0])
        m, Iz = spec.mass, spec.Iz
        F = scale * F0
        for i, th in enumerate(data["thrusters"]):
            th["F_max"] = float(F[i])
    data["name"] = spec.name + "_identified"
    data["notes"] = (
        f"Identified from {Path(log_csv).as_posix()}; mode={mode}; "
        f"F_max_scale={scale:.4g}; delay_steps={best_d}; residual_rmse={rmse:.4g}. "
        "Layout copied from the source JSON."
    )
    u_eff = _delay_u(u_lag, best_d)
    pred = (B1 * F) @ u_eff.T
    meas = _meas_wrench(log, m, Iz, dlin, drot)

    png = None
    if residual_png is not None or out is not None:
        dest_png = Path(residual_png) if residual_png else Path(out).with_suffix(".residual.png") if out else Path("residual.png")
        png = str(_residual_plot(t, meas, pred, dest_png))

    result: dict[str, Any] = {
        "mass": m,
        "Iz": Iz,
        "F_max": [float(x) for x in F],
        "F_max_scale": scale,
        "delay_steps": int(best_d),
        "rmse": rmse,
        "success": bool(sol.success) or rmse < 5.0,
        "mode": mode,
        "data": data,
        "residual_png": png,
    }
    if out is not None:
        dest = Path(out)
        try:
            save_vehicle(data, dest, allow_any=True)
        except Exception:
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_text(json.dumps(data, indent=2) + "\n")
        result["out"] = str(dest)
    return result


def synthesize_excitation_log(
    spec: SatelliteSpec,
    path: Path,
    duration: float = 6.0,
    *,
    kind: str = "chirp",
    delay_steps: int = 0,
) -> Path:
    """Open-loop PRBS or chirp per thruster — tests, Lab 3, and PE experiments."""
    from airbearing.dynamics import Plant

    plant = Plant(spec)
    plant.reset(-0.2, 0.0, 0.1)
    dt = spec.sim_dt
    t = 0.0
    rows_t = []
    rows_st = []
    rows_u = []
    n = spec.n_thrusters
    rng = np.random.default_rng(0)
    bits = rng.choice([-1.0, 1.0], size=(int(duration / dt) + 8, n))
    kstep = 0
    pending: list[np.ndarray] = []
    while t < duration:
        cmd = np.zeros(n)
        if kind == "prbs":
            hold = max(1, int(0.08 / dt))
            cmd = 0.7 * bits[kstep // hold]
            if spec.thrusters[0].type == "binary_solenoid":
                cmd = (cmd > 0).astype(float)
        else:
            k = int(t / (duration / max(n, 1))) % n
            cmd[k] = 0.7 * np.sin(2.2 * t) + 0.2 * np.sin(7.0 * t)
            if spec.thrusters[k].type == "binary_solenoid":
                cmd[k] = 1.0 if cmd[k] > 0 else 0.0
        pending.append(cmd.copy())
        if delay_steps <= 0:
            applied = cmd
        elif len(pending) <= delay_steps:
            applied = np.zeros(n)
        else:
            applied = pending[-(delay_steps + 1)]
        st = plant.step(applied, dt=dt)
        rows_t.append(t)
        rows_st.append(st.copy())
        rows_u.append(cmd.copy())  # logged command (host), plant may be delayed
        t += dt
        kstep += 1
    t_arr = np.array(rows_t)
    st_arr = np.vstack(rows_st)
    u_arr = np.vstack(rows_u)
    path = Path(path)
    write_pose_csv(path, t_arr, st_arr, u_arr)
    return path
