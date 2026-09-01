# Instructor packet — airbearing (20 minutes)

Planar *(x, y, yaw)* GNC package for cubesats on air-bearing tables. Students describe **their** vehicle in one JSON file. **Not flight software.** SI units. Python **3.11+**. Solvers are OSQP / Clarabel via cvxpy — **no Gurobi**.

Clone: <https://github.com/AevarOfjord/airbearing-testbed-mpc>

Print this page (`docs/instructor.pdf`) or keep this file open. Brief a TA/PI in ~20 minutes, then they can run a lab session.

## Install (3 lines)

```bash
git clone https://github.com/AevarOfjord/airbearing-testbed-mpc.git
cd airbearing-testbed-mpc && python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev,viz]"
```

Then `airbearing --help` and `make test`. Windows: `.venv\Scripts\activate`.

## Labs 1–5 (software; 1–2 h each)

Hardware is **not** required. `make lab1` … `make lab5` run the staff demos.

| Lab | Hours | Topic | Done looks like |
|-----|-------|--------|-----------------|
| 1 | 1–2 h | Editor + `check` | `airbearing check vehicles/<name>.json` prints **both-signs YES**; JSON lives under `vehicles/*.json` |
| 2 | 1–2 h | PD vs LQR vs MPC | Methods table from `airbearing report` / `methods.txt` for all three controllers |
| 3 | 1–2 h | Identify `F_max` | Identified JSON **beats the guess on RMSE**; residual plot written |
| 4 | 1–2 h | Actuators | Binary vs PWM vs `F_max` mismatch compared (`make lab4`) |
| 5 | 1–2 h | Navigation | Onboard vs mocap vs fused on sim sensors (`make lab5`) |

Student write-ups: `labs/`. Staff notes on shipped examples: `labs/staff/`.

## Where student files go

| Path | What |
|------|------|
| `vehicles/` | **Their** satellite JSON. Editor / `new-vehicle` will not save anywhere else. |
| `runs/` | Each `airbearing run` writes `log.csv`, `summary.json`, `methods.txt`, plots. |
| `examples/vehicles/` | Shipped starting guesses — copy, then calibrate. Do not edit `src/` to add a satellite. |

Offline sim-vs-log demo (fresh clone, no table):

```bash
airbearing compare --sim examples/logs/sim.csv --real examples/logs/hardware.csv
```

## Hardware (optional)

Labs 1–5 are software. A real table is extra credit / a later session:

```bash
airbearing run --vehicle vehicles/mine.json --armed --port /dev/ttyUSB0 --dashboard
```

`--armed` is required. The runtime **refuses null mocap** (zeros commands and aborts). Gateways implement a **~100 ms deadman**. See `docs/HARDWARE.md` and `firmware/`.

## Grading — artifacts to collect

- **Lab 1:** `vehicles/*.json` plus `airbearing check` output showing both-signs **YES**.
- **Lab 2:** three `runs/<id>/summary.json` (or `methods.txt`) — PD, LQR, MPC on the same hop.
- **Lab 3:** `vehicles/*_identified.json` and the residual plot (`*.png`); identified RMSE smaller than the uncalibrated guess.
- **Lab 4:** the binary / PWM / mismatch numbers from `make lab4` (or equivalent `summary.json`).
- **Lab 5:** `runs/lab5/summary.json` showing fused pose RMSE below noisy mocap passthrough.

## Navigation (onboard vs external)

Sensors are declared on the **same** vehicle JSON under `navigation`. MPC still consumes a 6-state. Students pick a mode by editing that block (not Python):

- omit `onboard` → external / mocap only
- omit `external` → onboard IMU only
- both → fuse (`estimator: ekf`)
- omit `navigation` entirely → sim passthrough (old behaviour)

`examples/vehicles/fan_plus.json` ships sim passthrough; `fan_plus_fused.json` ships a fused EKF. `--armed` refuses invalid estimates. Actuator serial (`--port`) is not the IMU serial. Webcam ArUco is an optional extra (`pip install -e ".[webcam]"`); tests skip if OpenCV is missing.

Hashes in `methods.txt` (`vehicle_sha256`, optional git hash) make it obvious which JSON produced the run.

## Safety

This kit is laboratory / teaching software, **not flight software**. The Python supervisor never *adds* thrust — it only zeros commands. Serial gateways force every actuator off if no host packet arrives within ~100 ms. `--armed` is required for hardware; null telemetry is refused rather than flown open-loop. Keep the table clear. A dashboard e-stop is not a substitute for a physical kill switch. If you bypass `--armed` or the deadman, this package will not save a runaway vehicle.
