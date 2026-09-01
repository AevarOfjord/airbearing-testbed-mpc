# Labs (master / PhD)

Hardware lives in **one JSON** under `vehicles/`. Do not edit Python to add a satellite.

| Lab | Command | What you do |
|-----|---------|-------------|
| 1 Editor + check | `make lab1` | Open the visual builder, place thrusters, `check`, save JSON |
| 2 Controllers | `make lab2` | Same mission: PD vs LQR vs MPC |
| 3 System ID | `make lab3` | Fit mass / Iz / F_max from a log; compare to a wrong JSON |
| 4 Actuators | `make lab4` | Solenoid (relax vs binary) vs PWM fans vs F_max mismatch |

Staff notes for shipped vehicles: [`staff/`](staff/).

Recorded mocap: [`data/example_mocap.csv`](data/example_mocap.csv) — replay with

```bash
python -m airbearing view --record --replay labs/data/example_mocap.csv
python -m airbearing run --replay labs/data/example_mocap.csv --duration 4
```
