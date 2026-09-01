# airbearing-testbed-mpc

**Generation 3** student kit: planar (x, y, yaw) GNC for air-bearing satellites.

You describe *your* vehicle in JSON — eight UK compressed-air solenoids, four computer fans, six PWM fans, or three thrusters — and the same runtime, allocator, and linear MPC run in simulation (and optionally on a serial gateway). **This is laboratory / teaching software. It is not flight software.**

Author: **Ævar Öfjörð** ([AevarOfjord](https://github.com/AevarOfjord)). License: MIT.

![Solenoid octagon demo](docs/assets/solenoid_demo.png)
![Solenoids vs fans](docs/assets/compare_actuators.png)

## What this is / is not

| Is | Is not |
|----|--------|
| A clone-and-hack kit for *your* table | A copy of a Windows/Gurobi thesis tree |
| One loop for sim and hardware | Fifteen overlapping handoff guides |
| JSON vehicles, thruster-count agnostic | Hard-coded 8 thrusters / COM20 |
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

## Bring your own satellite

Students should **not** edit Python to change hardware.

```bash
cp vehicles/YOUR_SATELLITE.json.example vehicles/my_fans.json
# edit mass, Iz, COM, table_size, thruster list
python -m airbearing check vehicles/my_fans.json
python -m airbearing run --vehicle vehicles/my_fans.json
```

Or the wizard (writes JSON + prints a both-signs Fx/Fy/Mz controllability check):

```bash
make new-vehicle
# non-interactive:
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

## Runtime

```bash
python -m airbearing run --vehicle vehicles/uk_solenoid_octagon.json --controller mpc
python -m airbearing run --vehicle vehicles/fan_quadrotor_plus.json --controller pd
# hardware (refuses to fire if mocap returns null; requires --armed)
python -m airbearing run --vehicle vehicles/uk_solenoid_octagon.json --armed --port /dev/ttyUSB0
```

Each run writes `runs/<timestamp>_<name>/{log.csv,trajectory.png,animation.gif,summary.json}`.

## Docs (four pages, not fifteen)

- [ARCHITECTURE.md](ARCHITECTURE.md) — one-pager
- [docs/YOUR_SATELLITE.md](docs/YOUR_SATELLITE.md) — JSON field guide
- [docs/MATH.md](docs/MATH.md) — 3-DOF, MPC, allocator
- [docs/HARDWARE.md](docs/HARDWARE.md) — fans, solenoids, firmware protocol
- [docs/LAB.md](docs/LAB.md) — calibrate `F_max` with a kitchen scale

## Safety

Gateways implement a **~100 ms deadman** (no host packet → actuators 0). The Python supervisor never *adds* thrust. Real mode **refuses null telemetry**. Keep the table clear; this kit will not save a runaway vehicle if you bypass `--armed`.
