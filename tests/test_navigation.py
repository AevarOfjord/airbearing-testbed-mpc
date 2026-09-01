"""Onboard vs external estimation: schema, drivers, passthrough, EKF, --armed."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from airbearing.control.mpc import LinearMPC
from airbearing.dynamics import Plant
from airbearing.estimate import PassthroughEstimator, PlanarEKF, build_navigation
from airbearing.missions import point_to_point
from airbearing.runtime import Runtime
from airbearing.spec import load_vehicle, spec_from_dict, spec_to_dict, validate_dict
from airbearing.telemetry import (
    Measurement,
    SimulatedImu,
    SimulatedMocap,
    WebcamAruco,
    parse_imu_line,
)

REPO = Path(__file__).resolve().parents[1]


def test_schema_navigation_optional_and_strict():
    raw = json.loads((REPO / "examples" / "vehicles" / "fan_plus.json").read_text())
    validate_dict(raw)
    raw["navigation"]["unknown"] = True
    with pytest.raises(Exception):
        validate_dict(raw)
    fused = json.loads((REPO / "examples" / "vehicles" / "fan_plus_fused.json").read_text())
    validate_dict(fused)
    spec = load_vehicle(REPO / "examples" / "vehicles" / "solenoid_octagon.json")
    assert spec.navigation is None


def test_navigation_roundtrip_in_spec():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "fan_plus_fused.json")
    d = spec_to_dict(spec)
    again = spec_from_dict(d)
    assert again.navigation is not None
    assert again.navigation.estimator == "ekf"
    assert again.navigation.external.type == "sim"
    assert again.navigation.onboard.type == "sim"


def test_passthrough_matches_legacy_mocap():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "fan_plus.json")
    plant = Plant(spec)
    plant.reset(0.12, -0.08, 0.2, 0.01, -0.02, 0.05)
    mocap = SimulatedMocap(plant)
    old, ok_old = mocap.read()
    nav = build_navigation(spec, plant)
    nav.reset(plant.state)
    est, ok = nav.step(spec.control_dt)
    assert ok_old and ok
    np.testing.assert_allclose(old, est)


def test_passthrough_without_navigation_block():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "solenoid_octagon.json")
    assert spec.navigation is None
    plant = Plant(spec)
    plant.reset(0.0, 0.0, 0.1)
    nav = build_navigation(spec, plant)
    nav.reset(plant.state)
    est, ok = nav.step(spec.control_dt)
    mocap = SimulatedMocap(plant)
    old, _ = mocap.read()
    assert ok
    np.testing.assert_allclose(old, est)


def test_ekf_reduces_mocap_noise():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "fan_plus.json")
    plant = Plant(spec)
    plant.reset(-0.3, 0.2, 0.0)
    rng = np.random.default_rng(1)
    ekf = PlanarEKF(
        Q={"x": 1e-5, "y": 1e-5, "yaw": 1e-6, "vx": 5e-4, "vy": 5e-4, "omega": 1e-4},
        R={"x": 9e-4, "y": 9e-4, "yaw": 1.6e-3},
        timeout_s=1.0,
    )
    ekf.reset(plant.state)
    mocap_err = []
    ekf_err = []
    dt = spec.control_dt
    cmd = np.array([0.4, -0.2, 0.3, -0.1])
    for _ in range(60):
        nsub = max(1, int(round(dt / spec.sim_dt)))
        for _ in range(nsub):
            plant.step(cmd, dt=dt / nsub)
        truth = plant.state.copy()
        zx = truth[0] + rng.normal(0, 0.04)
        zy = truth[1] + rng.normal(0, 0.04)
        zyaw = truth[2] + rng.normal(0, 0.05)
        imu = Measurement(
            stamp=plant.t,
            values={
                "ax": float(plant.accel_body[0]),
                "ay": float(plant.accel_body[1]),
                "gyro_z": float(truth[5]),
            },
            valid=True,
        )
        ext = Measurement(stamp=plant.t, values={"x": zx, "y": zy, "yaw": zyaw}, valid=True)
        est, ok = ekf.step(dt, imu, ext)
        assert ok
        mocap_err.append(float(np.hypot(zx - truth[0], zy - truth[1])))
        ekf_err.append(float(np.hypot(est[0] - truth[0], est[1] - truth[1])))
    assert float(np.mean(ekf_err)) < 0.7 * float(np.mean(mocap_err))


def test_armed_null_external_refuses(tmp_path: Path):
    spec = load_vehicle(REPO / "examples" / "vehicles" / "fan_plus.json")
    mission = point_to_point(spec.table_size)
    mission.duration = 0.5

    class Dead:
        def read(self):
            return None, False

    rt = Runtime(
        spec,
        LinearMPC(spec, horizon=6),
        mission,
        armed=True,
        mocap=Dead(),
        runs_root=tmp_path,
    )
    res = rt.run()
    assert res.aborted
    assert "null" in res.abort_reason.lower() or "telemetry" in res.abort_reason.lower()


def test_parse_imu_line_fake_json():
    d = parse_imu_line('{"ax": 0.10, "ay": -0.20, "gyro_z": 0.01}\n')
    assert d["ax"] == pytest.approx(0.10)
    assert d["ay"] == pytest.approx(-0.20)
    assert d["gyro_z"] == pytest.approx(0.01)
    from airbearing.telemetry import SerialImu

    imu = SerialImu(port=None, stream=None)
    m = imu.feed_line('{"ax": 1.5, "ay": 0.0, "gyro_z": -0.3}')
    assert m.valid
    assert m.values["ax"] == pytest.approx(1.5)


def test_webcam_aruco_graceful_without_camera():
    w = WebcamAruco(camera=99)
    m = w.measure()
    assert m.valid is False
    if not w.available:
        pytest.skip("opencv not installed")


def test_sim_imu_reads_plant_body_accel():
    spec = load_vehicle(REPO / "examples" / "vehicles" / "fan_plus.json")
    plant = Plant(spec)
    plant.reset(0, 0, 0)
    plant.step(np.ones(spec.n_thrusters) * 0.5, dt=spec.sim_dt)
    imu = SimulatedImu(plant)
    m = imu.measure()
    assert m.valid
    assert "ax" in m.values and "gyro_z" in m.values
    np.testing.assert_allclose(m.values["ax"], plant.accel_body[0])
