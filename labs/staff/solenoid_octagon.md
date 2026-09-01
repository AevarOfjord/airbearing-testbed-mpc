# Notes — `examples/vehicles/solenoid_octagon.json`

Eight compressed-air solenoids, large table.

- Allocator uses a `[0,1]` relaxation. `--round-binary` is jerkier and usually a slightly higher RMSE (Lab 4).
- Deadman 100 ms is in JSON *and* firmware. Do not “fix” missed deadlines by removing it.
- `F_max` in the file is a starting guess; scale calibration / Lab 3 (docs/LAB.md) is the real number.
