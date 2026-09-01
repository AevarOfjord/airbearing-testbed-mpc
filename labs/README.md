# Labs

Hardware lives in **one JSON** file. Do not edit Python to add a satellite. Copy an example from `examples/vehicles/` into `vehicles/`, or use the editor.

| Lab | Command | What you do |
|-----|---------|-------------|
| 1 Editor + check | `make lab1` | Open the visual builder, place thrusters, `check`, save JSON |
| 2 Controllers | `make lab2` | Same mission: PD vs LQR vs MPC; cite `airbearing report` |
| 3 System ID | `make lab3` | Fit `F_max` scale from a log; residual plot |
| 4 Actuators | `make lab4` | Solenoid (relax vs binary) vs PWM fans vs `F_max` mismatch |

Notes for shipped examples: [`staff/`](staff/).

Recorded mocap (SI CSV): [`data/example_mocap.csv`](data/example_mocap.csv)

```bash
airbearing view --record --replay labs/data/example_mocap.csv
airbearing run --replay labs/data/example_mocap.csv --duration 4
airbearing compare --sim runs/<sim>/log.csv --real labs/data/example_mocap.csv --mismatch-delay 1
```
