# airbearing-testbed-mpc

**Generation 4** student kit: planar (x, y, yaw) GNC for air-bearing satellites.

You describe *your* vehicle in **one JSON file** — eight UK compressed-air solenoids, four computer fans, six PWM fans, or three thrusters — and the same runtime, allocator, and linear MPC run in simulation (and optionally on a serial gateway). **This is laboratory / teaching software. It is not flight software.**

Students **never edit Python to add a satellite.** Hardware is `vehicles/<name>.json` validated by `schemas/vehicle.schema.json`.

Author: **Ævar Öfjörð** ([AevarOfjord](https://github.com/AevarOfjord)). License: MIT.

![Solenoid octagon demo](docs/assets/solenoid_demo.png)
![Solenoids vs fans](docs/assets/compare_actuators.png)

## What this is / is not

| Is | Is not |
|----|--------|
| A clone-and-hack kit for *your* table | A copy of a Windows/Gurobi thesis tree |
| One loop for sim and hardware | Fifteen overlapping handoff guides |
| JSON vehicles, thruster-count agnostic | Hard-coded 8 thrusters / COM20 |
| Click-to-place visual builder | Mass/`F_max` constants buried in `src/` |
| `cvxpy` + Clarabel/OSQP | A Gurobi requirement for `make run` |
| Deadman + refuse-null-telemetry | Flight-qualified GNC |

Hardware numbers in `vehicles/*.json` are **typical starting guesses**. Calibrate `F_max` on a scale ([docs/LAB.md](docs/LAB.md)).

## 60-second start

```bash
git clone https://github.com/AevarOfjord/airbearing-testbed-mpc.git
cd airbearing-testbed-mpc
python3 -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
make install
make test
make run                 # solenoid octagon, animation + CSV in runs/<id>/
make compare-actuators   # same mission, solenoids vs fans
```

Python **3.11+**. No Gurobi license. No MATLAB.

## Model a new satellite (dead simple)

**Primary path — visual builder** (pygame top-down table):

```bash
python -m airbearing edit-vehicle
# or
make edit-vehicle
```

Click to add/move/delete thrusters. Type `solenoid` / `pwm_fan` / `continuous`, set `F_max`, bidirectional, direction, mass, \(I_z\), COM, table size. The HUD shows live ±Fx ±Fy ±Mz and **warns if plus-frame fans are radial (Mz ≡ 0)**. **Save writes JSON only under `vehicles/`.**

Then:

```bash
python -m airbearing check vehicles/mine.json --png docs/assets/mine.png
make run VEHICLE=vehicles/mine.json
```

CLI wizard still exists (also writes JSON, never Python):

```bash
make new-vehicle
python -m airbearing new-vehicle --noninteractive --name my_fans --n 4 --type pwm_fan --bidirectional --out vehicles/my_fans.json
```

Actuator types in the spec:

- `binary_solenoid` — on/off valves (UK compressed-air testbed). Allocator uses a `[0,1]` relaxation; apply as PWM duty or `--round-binary`.
- `pwm_fan` — computer fans, duty `0..1` or bidirectional `[-1,1]`, first-order spin-up `tau`.
- `continuous` — ideal ±F_max (sim / offboard ESC).

Shipped vehicles:

| File | What |
|------|------|
| `vehicles/uk_solenoid_octagon.json` | ~24 kg, 8 compressed-air thrusters, 6 m table |
| `vehicles/fan_quadrotor_plus.json` | cheap 4-fan plus layout |
| `vehicles/fan_hex.json` | 6 PWM fans + optional sim-only reaction wheel |
| `vehicles/micro_3thruster.json` | 3 binary jets (allocator is not stuck at 8; controllability *should* warn) |

## Live twin, replay, dashboard

```bash
python -m airbearing view                 # pygame table, plumes, trail, HUD
make view
python -m airbearing view --record        # headless → docs/assets/live_twin.png (gif if possible)
# arrows / WASD teleop, T teleop, M handover to MPC, space e-stop

python -m airbearing run --replay labs/data/example_mocap.csv
python -m airbearing run --dashboard --dashboard-port 8765
```

`--dashboard` is a tiny **stdlib** HTTP page (`/`, `/status.json`, `POST /estop`). The e-stop flag zeros commands; it is not a substitute for a physical kill switch.

## System ID

```bash
python -m airbearing identify runs/<id>/log.csv --vehicle vehicles/mine.json --out vehicles/mine_identified.json
```

Fits `mass`, `Iz`, and per-thruster `F_max`; layout stays whatever you drew. Details: [docs/LAB.md](docs/LAB.md).

## Labs (master / PhD)

| | |
|--|--|
| Lab 1 | editor + `check` |
| Lab 2 | PD vs LQR vs MPC |
| Lab 3 | identify vs uncalibrated RMSE |
| Lab 4 | binary vs PWM vs `F_max` mismatch |

```bash
make lab1
make lab2
make lab3
make lab4
```

Write-ups: [labs/README.md](labs/README.md). Staff notes: [labs/staff/](labs/staff/).

## Runtime

```bash
python -m airbearing run --vehicle vehicles/uk_solenoid_octagon.json --controller mpc
python -m airbearing run --vehicle vehicles/fan_quadrotor_plus.json --controller pd
# hardware (refuses to fire if mocap returns null; requires --armed)
python -m airbearing run --vehicle vehicles/uk_solenoid_octagon.json --armed --port /dev/ttyUSB0 --dashboard
```

Each run writes `runs/<timestamp>_<name>/{log.csv,trajectory.png,animation.gif,summary.json}`.

## Docs (still four pages, plus labs)

- [ARCHITECTURE.md](ARCHITECTURE.md) — one-pager
- [docs/YOUR_SATELLITE.md](docs/YOUR_SATELLITE.md) — JSON field guide
- [docs/MATH.md](docs/MATH.md) — 3-DOF, MPC, allocator
- [docs/HARDWARE.md](docs/HARDWARE.md) — fans, solenoids, firmware protocol
- [docs/LAB.md](docs/LAB.md) — scale calibration + system ID
- [labs/README.md](labs/README.md) — Lab 1–4

## Safety

Gateways implement a **~100 ms deadman** (no host packet → actuators 0). The Python supervisor never *adds* thrust. Real mode **refuses null telemetry**. Keep the table clear; this kit will not save a runaway vehicle if you bypass `--armed`.
