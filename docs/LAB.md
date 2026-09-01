# Lab: calibrate `F_max` with a scale

Numbers in `vehicles/*.json` are guesses. Your fans and regulators will differ.

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
