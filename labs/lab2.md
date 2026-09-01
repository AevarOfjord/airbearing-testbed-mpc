# Lab 2 — PD vs LQR vs MPC

Same vehicle, same hop:

```bash
make lab2
airbearing report runs/lab2/<id>
```

For each controller, copy the methods table (`settling_s`, `integrated_|u|`, `solver_p50_ms`, `solver_p95_ms`, `deadline_misses`, hashes). Why does PD lag on a nearly frictionless table?
