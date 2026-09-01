# Staff notes — `uk_solenoid_octagon.json`

Eight compressed-air solenoids, ~24 kg, 6 m table.

- Allocator uses a `[0,1]` relaxation. `--round-binary` is jerkier and usually a slightly higher RMSE (Lab 4).
- Deadman 100 ms is in JSON *and* firmware. Students must not “fix” missed deadlines by removing it.
- `F_max` in the file is a starting guess; Lab 3 / scale calibration (docs/LAB.md) is the real number.
