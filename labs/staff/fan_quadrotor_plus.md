# Staff notes — `fan_quadrotor_plus.json`

Teaching vehicle. Tangential plus-frame PWM fans, bidirectional.

- Lab 1: if a student points fans *along* the arms (`force_direction` ∥ position), `check` must warn **Mz ≡ 0**. The shipped file is already tangent.
- Lab 2: on a 6 s hop, MPC and LQR should both beat an untuned PD in RMSE; PD still makes progress. Solver time ≪ 50 ms.
- Lab 3: a 5 s per-thruster chirp recovers mass within ~35%. F_max signs follow the JSON directions.
- Typical kitchen-scale `F_max` for 80 mm fans is 0.2–0.5 N — the 0.35 N in JSON is a guess.
