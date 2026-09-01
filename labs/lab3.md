# Lab 3 — Identify vs uncalibrated

```bash
make lab3
python -m airbearing identify runs/<id>/log.csv --vehicle vehicles/mine.json --out vehicles/mine_identified.json
```

Compare closed-loop RMSE of the guessed JSON vs the identified one. Layout (positions/directions) must not change.
