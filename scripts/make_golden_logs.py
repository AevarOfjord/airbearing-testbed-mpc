#!/usr/bin/env python3
"""Write compact schema-valid golden logs under examples/logs/ (offline compare)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from airbearing.control.mpc import LinearMPC
from airbearing.logschema import comment_line, write_pose_csv
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.spec import load_vehicle

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "examples" / "logs"
FAN = REPO / "examples" / "vehicles" / "fan_plus.json"


def _annotate(path: Path, extra_comments: list[str]) -> None:
    text = path.read_text()
    lines = text.splitlines(keepends=True)
    if not lines:
        return
    insert = "".join(c if c.endswith("\n") else c + "\n" for c in extra_comments)
    # keep the magic comment first so load_log sees schema_version
    path.write_text(lines[0] + insert + "".join(lines[1:]))


def main() -> int:
    spec = load_vehicle(FAN)
    mission = point_to_point(spec.table_size)
    mission.duration = 4.0
    res = Runtime(
        spec,
        LinearMPC(spec, horizon=8),
        mission,
        runs_root=REPO / "runs" / "_golden",
        seed=7,
    ).run()
    t = np.round(np.array([row.t for row in res.logs], dtype=float), 4)
    st = np.array([row.state for row in res.logs], dtype=float)
    OUT.mkdir(parents=True, exist_ok=True)

    sim_path = OUT / "sim.csv"
    write_pose_csv(sim_path, t, np.round(st, 6))
    _annotate(
        sim_path,
        [
            "# closed-loop MPC simulation (fan_plus, point-to-point, seed=7, 4 s)",
            "# shipped golden log for `airbearing compare` on a fresh clone",
        ],
    )

    rng = np.random.default_rng(11)
    delay = 1
    delayed = np.vstack([st[:1], st[:-delay]]) if delay else st.copy()
    noise = np.zeros_like(delayed)
    noise[:, 0] = rng.normal(0.0, 0.003, len(t))  # ~3 mm mocap
    noise[:, 1] = rng.normal(0.0, 0.003, len(t))
    noise[:, 2] = rng.normal(0.0, 0.008, len(t))  # yaw rad
    noise[:, 3] = rng.normal(0.0, 0.012, len(t))
    noise[:, 4] = rng.normal(0.0, 0.012, len(t))
    noise[:, 5] = rng.normal(0.0, 0.02, len(t))
    hw = delayed + noise
    hw_path = OUT / "hardware.csv"
    write_pose_csv(hw_path, t, np.round(hw, 6))
    _annotate(
        hw_path,
        [
            "# example hardware-shaped log (synthetic mocap noise)",
            "# same trajectory family as sim.csv + 1-step delay + Gaussian pose noise; not a floor recording",
        ],
    )
    print(f"wrote {sim_path}  ({len(t)} samples)")
    print(f"wrote {hw_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
