"""Compare a simulation log to a hardware (or replay) log. RMSE in SI."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from airbearing.logschema import REQUIRED_COLUMNS, load_log


def _unwrap_yaw(y: np.ndarray) -> np.ndarray:
    return np.unwrap(y)


def _interp(t_src: np.ndarray, y: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    return np.interp(t_dst, t_src, y)


def _shift(t: np.ndarray, cols: dict[str, np.ndarray], steps: int) -> dict[str, np.ndarray]:
    """Delay the real trajectory by `steps` samples (command-delay mismatch)."""
    if steps <= 0:
        return cols
    out = {}
    n = len(t)
    for k, v in cols.items():
        pad = np.repeat(v[:1], steps)
        out[k] = np.concatenate([pad, v])[:n]
    return out


def compare_logs(
    sim_path: str | Path,
    real_path: str | Path,
    *,
    delay_steps: int = 0,
    mismatch_delay: int | None = None,
) -> dict[str, Any]:
    sim = load_log(sim_path, required=REQUIRED_COLUMNS)
    real = load_log(real_path, required=REQUIRED_COLUMNS)
    t_s, t_r = sim["t"], real["t"]
    t0 = max(float(t_s[0]), float(t_r[0]))
    t1 = min(float(t_s[-1]), float(t_r[-1]))
    if t1 <= t0:
        raise ValueError("sim and real logs have no overlapping time")
    n = max(8, int(min(len(t_s), len(t_r))))
    t = np.linspace(t0, t1, n)

    def series(log, delay: int) -> dict[str, np.ndarray]:
        cols = {k: log[k] for k in ("x", "y", "yaw")}
        cols = _shift(log["t"], cols, delay)
        return {
            "x": _interp(log["t"], cols["x"], t),
            "y": _interp(log["t"], cols["y"], t),
            "yaw": _interp(log["t"], _unwrap_yaw(cols["yaw"]), t),
        }

    a = series(sim, 0)
    b = series(real, delay_steps)
    dx = a["x"] - b["x"]
    dy = a["y"] - b["y"]
    pos = np.hypot(dx, dy)
    yaw = a["yaw"] - b["yaw"]
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
    out: dict[str, Any] = {
        "sim": str(sim_path),
        "real": str(real_path),
        "n": n,
        "t0": t0,
        "t1": t1,
        "delay_steps": delay_steps,
        "rmse_position_m": float(np.sqrt(np.mean(pos ** 2))),
        "rmse_x_m": float(np.sqrt(np.mean(dx ** 2))),
        "rmse_y_m": float(np.sqrt(np.mean(dy ** 2))),
        "rmse_yaw_rad": float(np.sqrt(np.mean(yaw ** 2))),
        "units": "SI",
        "label": "aligned" if delay_steps == 0 else f"delay {delay_steps} step(s)",
    }
    if mismatch_delay is not None:
        b2 = series(real, mismatch_delay)
        dx2 = a["x"] - b2["x"]
        dy2 = a["y"] - b2["y"]
        pos2 = np.hypot(dx2, dy2)
        yaw2 = a["yaw"] - b2["yaw"]
        yaw2 = (yaw2 + np.pi) % (2 * np.pi) - np.pi
        out["delay_mismatch"] = {
            "label": f"delay mismatch ({mismatch_delay} step)",
            "delay_steps": mismatch_delay,
            "rmse_position_m": float(np.sqrt(np.mean(pos2 ** 2))),
            "rmse_yaw_rad": float(np.sqrt(np.mean(yaw2 ** 2))),
        }
    return out


def format_compare(rep: dict[str, Any]) -> str:
    lines = [
        f"sim:  {rep['sim']}",
        f"real: {rep['real']}",
        f"case: {rep['label']}",
        f"RMSE position = {rep['rmse_position_m']:.4g} m",
        f"RMSE yaw      = {rep['rmse_yaw_rad']:.4g} rad",
    ]
    mm = rep.get("delay_mismatch")
    if mm:
        lines.append(
            f"case: {mm['label']}  RMSE pos={mm['rmse_position_m']:.4g} m  "
            f"yaw={mm['rmse_yaw_rad']:.4g} rad"
        )
    return "\n".join(lines) + "\n"
