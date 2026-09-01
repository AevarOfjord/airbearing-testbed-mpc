# Calibrate, identify, campaign

Numbers in example JSON are guesses. Your fans, regulators, and table friction will differ.

## Kitchen scale (`F_max`)

1. Put a digital scale on a rigid stand. Zero it.
2. Rig **one** thruster so its jet axis is vertical (pick a sign and stick to it).
3. Command a known duty:
   - solenoid: `CMD:<bit>:<duration_ms>` for a long pulse (watch the deadman: send at ≥10 Hz).
   - fan: `PWM:1,0,0,...`
4. Record grams. \(F_{\max} = m g\) with \(g = 9.81\,\mathrm{m/s^2}\). A 70 g reading is **0.69 N**.
5. Repeat at two or three duties; fans are often nonlinear near 0. Put the peak (or a linear fit’s slope × 1.0) into `F_max`.
6. Repeat for every thruster. Asymmetry is normal — put the truth in JSON.
7. Measure mass with the same scale. Estimate \(I_z\) from geometry or a bifilar pendulum: \(I_z = m g b^2 T^2 / (16\pi^2 L)\).

Then `airbearing run --vehicle vehicles/my_sat.json` and compare the first metre of motion to the log. If the sim races ahead, your `F_max` is high; if it crawls, `F_max` is low or the table is dirty.

## Visual builder (Lab 1)

```bash
airbearing edit-vehicle
airbearing check vehicles/mine.json --png labs/data/my_layout.png
airbearing run --vehicle vehicles/mine.json
```

Plus-frame fans that blow *through* the COM produce **Mz ≡ 0**. The HUD will warn; rotate each jet 90°.

## Parameter estimation (Lab 3)

Persistently exciting input: open-loop **PRBS** or **chirp** per thruster, or reuse a closed-loop `runs/<id>/log.csv` that actually moved the vehicle.

```bash
airbearing identify runs/<id>/log.csv --vehicle vehicles/mine.json \
  --out vehicles/mine_identified.json --residual vehicles/mine_residual.png
```

Default PE keeps your layout and fits a single **`F_max` scale** plus an optional **0–2 step command delay**. `--mode full` also fits mass, \(I_z\), and per-thruster `F_max`. The residual plot is body wrench (N, N m) vs the scaled `B u` model.

Then close the loop on the identified JSON and compare RMSE to the uncalibrated file (`airbearing compare --sim ... --real ...`).

A synthetic log generator is used in tests: `airbearing.identify.synthesize_excitation_log` (`kind="chirp"` or `"prbs"`).

## Methods table (Lab 2)

Every `airbearing run` writes `summary.json` and `methods.txt`. Print it with:

```bash
airbearing report runs/<id>
```

Columns: settling time, integrated |u|, solver p50/p95, deadline misses, seed, git hash (if the tree is a git checkout), SHA-256 of the vehicle JSON.
