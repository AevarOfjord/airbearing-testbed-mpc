# Lab 3 — Identify `F_max` from a log

Use a persistently exciting experiment (open-loop chirp/PRBS, or a closed-loop log that used the thrusters):

```bash
make lab3
airbearing identify runs/<id>/log.csv --vehicle vehicles/mine.json \
  --out vehicles/mine_identified.json --residual vehicles/mine_residual.png
```

Default fit: one `F_max` scale + optional delay. Layout (positions/directions) must not change. Inspect the residual plot (body wrench vs `B u`).
