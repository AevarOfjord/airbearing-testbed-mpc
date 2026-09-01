# Notes — `examples/vehicles/fan_plus.json`

Four tangential plus-frame PWM fans, bidirectional.

- Lab 1: if fans point *along* the arms (`force_direction` ∥ position), `check` must warn **Mz ≡ 0**. The shipped file is already tangent.
- Lab 2: on a short hop, MPC and LQR should both beat an untuned PD in RMSE; PD still makes progress. Solver time ≪ 50 ms. Cite `airbearing report`.
- Lab 3: a few seconds of chirp/PRBS recovers an `F_max` scale; `--mode full` also fits mass.
- Typical kitchen-scale `F_max` for 80 mm fans is 0.2–0.5 N — the value in JSON is a guess.
