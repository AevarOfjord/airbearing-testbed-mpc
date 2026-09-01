# Lab: calibrate `F_max`, then identify the rest

Numbers in `vehicles/*.json` are guesses. Your fans and regulators will differ.

## Kitchen scale (`F_max`)

1. Put a digital scale on a rigid stand. Zero it.
2. Rig **one** thruster so its jet axis is vertical, nozzle/fan pointing **at** the scale (or so the vehicle body pushes on the scale — pick one sign and stick to it).
3. Command a known duty:
   - solenoid: `CMD:<bit>:<duration_ms>` for a long pulse (watch the deadman: send at ≥10 Hz).
   - fan: `PWM:1,0,0,...`
4. Record grams. \(F_{\max} = m g\) with \(g = 9.81\). A 70 g reading is **0.69 N**.
5. Repeat at two or three duties; fans are often nonlinear near 0. Put the peak (or a linear fit’s slope × 1.0) into `F_max`.
6. Repeat for every thruster. Asymmetry is normal — put the truth in JSON, do not “average it away” in Python.
7. Measure mass with the same scale. Estimate \(I_z\) from geometry \(I_z \approx m r^2 / 2\) for a disk, or a bifilar pendulum: \(I_z = m g b^2 T^2 / (16\pi^2 L)\).

Then `python -m airbearing run --vehicle vehicles/my_sat.json` and compare the first metre of motion to the log. If the sim races ahead, your `F_max` is high; if it crawls, `F_max` is low or the table is dirty.

## Visual builder (Lab 1)

```bash
python -m airbearing edit-vehicle          # or: make edit-vehicle
# click to add thrusters, 1/2/3 type, r rotate, s save → vehicles/<name>.json
python -m airbearing check vehicles/mine.json --png docs/assets/mine.png
make run VEHICLE=vehicles/mine.json
```

Plus-frame fans that blow *through* the COM produce **Mz ≡ 0**. The HUD will warn; rotate each jet 90°.

## Batch system ID (Lab 3)

From a logged excitation (open-loop chirp or a closed-loop `runs/<id>/log.csv`):

```bash
python -m airbearing identify runs/<id>/log.csv --vehicle vehicles/mine.json --out vehicles/mine_identified.json
```

The fitter keeps your layout (positions, directions, types) and adjusts `mass`, `Iz`, and each `F_max`. Compare closed-loop RMSE of the uncalibrated JSON vs the identified one — that is Lab 3.

A synthetic log generator is used in tests (`airbearing.identify.synthesize_excitation_log`).
