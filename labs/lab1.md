# Lab 1 — Editor + check

Model hardware **only** as JSON.

1. `python -m airbearing edit-vehicle` (or `make edit-vehicle`).
2. Click the table to place thrusters. Keys: `1` solenoid, `2` pwm_fan, `3` continuous, `b` bidirectional, `r` rotate, `[ ]` F_max, `s` save.
3. If plus-frame fans point through the COM, the HUD warns **Mz ≡ 0**. Rotate 90°.
4. `python -m airbearing check vehicles/<name>.json --png labs/data/my_layout.png`
5. `make run VEHICLE=vehicles/<name>.json`
