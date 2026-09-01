"""System ID from runs/<id>/log.csv → vehicles/<name>_identified.json.

Layout (positions, directions, types) stays in the JSON the student drew.
This fits mass, Iz, and per-thruster F_max from logged motion + commands.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares

from airbearing.spec import SatelliteSpec, load_vehicle, save_vehicle, spec_to_dict, vehicles_dir


def load_log(path: str | Path) -> dict[str, np.ndarray]:
    path = Path(path)
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty log {path}")
    keys = rows[0].keys()
    out: dict[str, np.ndarray] = {}
    numeric = [k for k in keys if k not in ("safety", "status")]
    for k in numeric:
        out[k] = np.array([float(r[k]) for r in rows], dtype=float)
    u_cols = sorted([k for k in keys if k.startswith("u") and k[1:].isdigit()], key=lambda s: int(s[1:]))
    if u_cols:
        out["u"] = np.column_stack([out[k] for k in u_cols])
    return out


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


def identify_from_log(log_csv: str | Path, vehicle: str | Path, *, out: str | Path | None = None) -> dict[str, Any]:
    spec = load_vehicle(vehicle)
    log = load_log(log_csv)
    t = log["t"]
    vx, vy, om = log["vx"], log["vy"], log["omega"]
    yaw = log["yaw"]
    u = log["u"]
    taus = np.array([float(th.tau) for th in spec.thrusters])
    u_eff = _lag_commands(u, t, taus)
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    alpha = np.gradient(om, t)
    B1 = _unit_B(spec)
    n = spec.n_thrusters
    x0 = np.concatenate([[spec.mass, spec.Iz], np.array([th.F_max for th in spec.thrusters])])
    dlin = float(spec.linear_damping)
    drot = float(spec.rotational_damping)

    def residual(p):
        m, Iz = p[0], p[1]
        F = p[2:]
        c, s = np.cos(yaw), np.sin(yaw)
        # body wrench from inertial accel + known JSON damping
        Fx_i = m * ax + dlin * vx
        Fy_i = m * ay + dlin * vy
        Fx_b = c * Fx_i + s * Fy_i
        Fy_b = -s * Fx_i + c * Fy_i
        Mz = Iz * alpha + drot * om
        pred = (B1 * F) @ u_eff.T  # 3 x T
        return np.concatenate([Fx_b - pred[0], Fy_b - pred[1], Mz - pred[2]])

    lo = np.concatenate([[0.05, 1e-4], np.full(n, 1e-4)])
    hi = np.concatenate([[500.0, 50.0], np.full(n, 50.0)])
    sol = least_squares(residual, x0, bounds=(lo, hi), max_nfev=200)
    m, Iz = float(sol.x[0]), float(sol.x[1])
    F = sol.x[2:]
    data = spec_to_dict(spec)
    data["mass"] = m
    data["Iz"] = Iz
    data["name"] = spec.name + "_identified"
    for i, th in enumerate(data["thrusters"]):
        th["F_max"] = float(F[i])
    data["notes"] = (
        f"Identified from {Path(log_csv).as_posix()}; cost={float(sol.cost):.4g}. "
        "Layout copied from the source JSON — do not edit Python."
    )
    rmse = float(np.sqrt(np.mean(sol.fun ** 2)))
    result = {
        "mass": m,
        "Iz": Iz,
        "F_max": [float(x) for x in F],
        "rmse": rmse,
        "success": bool(sol.success),
        "data": data,
    }
    if out is not None:
        dest = Path(out)
        allow = dest.parent.resolve() != vehicles_dir().resolve()
        # identified files belong under vehicles/; tests may use tmp
        try:
            save_vehicle(data, dest, allow_any=True)
        except Exception:
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_text(json.dumps(data, indent=2) + "\n")
        result["out"] = str(dest)
    return result


def synthesize_excitation_log(spec: SatelliteSpec, path: Path, duration: float = 6.0) -> Path:
    """Open-loop chirp per thruster — used by tests and Lab 3."""
    from airbearing.dynamics import Plant

    plant = Plant(spec)
    plant.reset(-0.2, 0.0, 0.1)
    dt = spec.sim_dt
    t = 0.0
    rows = []
    n = spec.n_thrusters
    while t < duration:
        cmd = np.zeros(n)
        k = int(t / (duration / max(n, 1))) % n
        cmd[k] = 0.7 * np.sin(2.2 * t) + 0.2 * np.sin(7.0 * t)
        if spec.thrusters[k].type == "binary_solenoid":
            cmd[k] = 1.0 if cmd[k] > 0 else 0.0
        st = plant.step(cmd, dt=dt)
        rows.append((t, st.copy(), cmd.copy()))
        t += dt
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["t", "x", "y", "yaw", "vx", "vy", "omega"] + [f"u{i}" for i in range(n)]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for t, st, cmd in rows:
            d = {"t": t, "x": st[0], "y": st[1], "yaw": st[2], "vx": st[3], "vy": st[4], "omega": st[5]}
            for i, c in enumerate(cmd):
                d[f"u{i}"] = c
            w.writerow(d)
    return path
