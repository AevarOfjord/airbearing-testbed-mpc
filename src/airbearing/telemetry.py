"""Sensor drivers: mocap, IMU, replay, webcam. Estimators live in estimate.py.

Drivers return timestamped Measurement dicts. A measurement is invalid on
timeout or hardware failure. PoseSource.read() stays for the 6-state adapter.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from airbearing.dynamics import Plant

POSE_KEYS = ("x", "y", "yaw", "vx", "vy", "omega")
IMU_KEYS = ("ax", "ay", "gyro_z")
ALL_MEAS = POSE_KEYS + IMU_KEYS


@dataclass
class Measurement:
    """One timestamped sample. `valid` is False on timeout / dropout."""

    stamp: float
    values: dict[str, float] = field(default_factory=dict)
    valid: bool = True

    def get(self, key: str, default: float = 0.0) -> float:
        return float(self.values[key]) if key in self.values else default


class PoseSource(Protocol):
    def read(self) -> tuple[np.ndarray | None, bool]:
        """Return (state6 or None, ok)."""
        ...


class SensorDriver(Protocol):
    def measure(self, now: float | None = None) -> Measurement:
        ...


def _now(now: float | None) -> float:
    return time.monotonic() if now is None else float(now)


def parse_imu_line(line: str | bytes) -> dict[str, float]:
    """Parse one JSON IMU line: {\"ax\", \"ay\", \"gyro_z\"} (SI, body frame)."""
    if isinstance(line, bytes):
        line = line.decode("utf-8", errors="replace")
    line = line.strip()
    if not line:
        raise ValueError("empty IMU line")
    data = json.loads(line)
    if not isinstance(data, dict):
        raise ValueError("IMU line is not a JSON object")
    out: dict[str, float] = {}
    for k in IMU_KEYS:
        if k in data and data[k] is not None:
            out[k] = float(data[k])
    if not out:
        raise ValueError(f"no IMU fields in {line!r}")
    return out


def state_from_values(values: dict[str, float], prev: np.ndarray | None = None) -> np.ndarray:
    """Build a 6-state from a measurement dict. Missing pose keys → 0 or prev."""
    base = np.zeros(6) if prev is None else np.asarray(prev, dtype=float).reshape(6).copy()
    for i, k in enumerate(POSE_KEYS):
        if k in values:
            base[i] = float(values[k])
    return base


def has_pose(meas: Measurement | None) -> bool:
    if meas is None or not meas.valid:
        return False
    return "x" in meas.values and "y" in meas.values


def has_imu(meas: Measurement | None) -> bool:
    if meas is None or not meas.valid:
        return False
    return any(k in meas.values for k in IMU_KEYS)


def _apply_noise(
    values: dict[str, float],
    noise: dict[str, float],
    rng: np.random.Generator,
) -> dict[str, float]:
    if not noise:
        return values
    out = dict(values)
    for k, sig in noise.items():
        if k in out and sig:
            out[k] = float(out[k] + rng.normal(0.0, float(sig)))
    return out


def _stale(stamp: float | None, now: float, timeout_s: float) -> bool:
    if timeout_s <= 0 or stamp is None:
        return False
    return (now - stamp) > timeout_s


@dataclass
class SimulatedMocap:
    """Drop-in mock: the plant IS the 'camera'. Optional dropout / noise."""

    plant: Plant
    dropout: bool = False
    meas: tuple[str, ...] = POSE_KEYS
    noise: dict[str, float] = field(default_factory=dict)
    timeout_s: float = 0.0
    rng: np.random.Generator | None = None
    _rng: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = self.rng if self.rng is not None else np.random.default_rng()
        self.meas = tuple(self.meas) if self.meas else POSE_KEYS

    def measure(self, now: float | None = None) -> Measurement:
        stamp = _now(now)
        if self.dropout:
            return Measurement(stamp=stamp, values={}, valid=False)
        z = self.plant.state.copy()
        values = {k: float(z[i]) for i, k in enumerate(POSE_KEYS) if k in self.meas}
        if self.noise:
            values = _apply_noise(values, self.noise, self._rng)
        else:
            # modest quantization like a cheap mocap (legacy behaviour)
            if "x" in values:
                values["x"] = round(values["x"], 4)
            if "y" in values:
                values["y"] = round(values["y"], 4)
            if "yaw" in values:
                values["yaw"] = round(values["yaw"], 5)
        return Measurement(stamp=stamp, values=values, valid=True)

    def read(self) -> tuple[np.ndarray | None, bool]:
        m = self.measure()
        if not m.valid:
            return None, False
        return state_from_values(m.values, self.plant.state), True


@dataclass
class SimulatedImu:
    """Body-frame IMU from the plant: ax, ay (m/s^2) and gyro_z (rad/s)."""

    plant: Plant
    dropout: bool = False
    meas: tuple[str, ...] = IMU_KEYS
    noise: dict[str, float] = field(default_factory=dict)
    timeout_s: float = 0.0
    rng: np.random.Generator | None = None
    _rng: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = self.rng if self.rng is not None else np.random.default_rng()
        self.meas = tuple(self.meas) if self.meas else IMU_KEYS

    def measure(self, now: float | None = None) -> Measurement:
        stamp = _now(now)
        if self.dropout:
            return Measurement(stamp=stamp, values={}, valid=False)
        raw = self.plant.imu_sample()
        values = {k: float(raw[k]) for k in self.meas if k in raw}
        values = _apply_noise(values, self.noise, self._rng)
        return Measurement(stamp=stamp, values=values, valid=True)

    def read(self) -> tuple[np.ndarray | None, bool]:
        m = self.measure()
        return (None, False) if not m.valid else (None, True)


class HttpMocap:
    def __init__(
        self,
        endpoint: str,
        timeout_s: float = 0.05,
        meas: tuple[str, ...] | list[str] | None = None,
    ):
        self.endpoint = endpoint
        self.timeout_s = timeout_s
        self.meas = tuple(meas) if meas else ("x", "y", "yaw", "vx", "vy", "omega")
        self._last_ok: float | None = None
        self._last: Measurement | None = None

    def measure(self, now: float | None = None) -> Measurement:
        import urllib.request

        stamp = _now(now)
        try:
            with urllib.request.urlopen(self.endpoint, timeout=max(self.timeout_s, 0.01)) as r:
                data = json.loads(r.read().decode())
            values = {k: float(data[k]) for k in self.meas if k in data and data[k] is not None}
            # HTTP pose always needs x,y,yaw when advertised
            if "x" in self.meas and "x" not in values:
                raise KeyError("x")
            m = Measurement(stamp=stamp, values=values, valid=True)
            self._last_ok = stamp
            self._last = m
            return m
        except Exception:
            if self._last is not None and not _stale(self._last_ok, stamp, self.timeout_s):
                return Measurement(stamp=self._last.stamp, values=dict(self._last.values), valid=True)
            return Measurement(stamp=stamp, values={}, valid=False)

    def read(self) -> tuple[np.ndarray | None, bool]:
        m = self.measure()
        if not m.valid:
            return None, False
        return state_from_values(m.values), True


@dataclass
class CsvReplay:
    """Recorded mocap: sequential 6-state rows from labs/data/example_mocap.csv."""

    path: str
    meas: tuple[str, ...] = POSE_KEYS
    timeout_s: float = 0.0
    _rows: list = field(default_factory=list, init=False)
    _i: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        import csv

        rows = []
        with Path(self.path).open(newline="") as f:
            lines = [ln for ln in f if not ln.startswith("#")]
            r = csv.DictReader(lines)
            for d in r:
                st = np.array(
                    [
                        float(d["x"]),
                        float(d["y"]),
                        float(d["yaw"]),
                        float(d.get("vx") or 0.0),
                        float(d.get("vy") or 0.0),
                        float(d.get("omega") or 0.0),
                    ],
                    dtype=float,
                )
                rows.append(st)
        if not rows:
            raise ValueError(f"empty mocap csv: {self.path}")
        self._rows = rows
        self._i = 0
        self.meas = tuple(self.meas) if self.meas else POSE_KEYS

    def measure(self, now: float | None = None) -> Measurement:
        stamp = _now(now)
        if self._i >= len(self._rows):
            z = self._rows[-1]
        else:
            z = self._rows[self._i]
            self._i += 1
        values = {k: float(z[i]) for i, k in enumerate(POSE_KEYS) if k in self.meas}
        return Measurement(stamp=stamp, values=values, valid=True)

    def read(self) -> tuple[np.ndarray | None, bool]:
        m = self.measure()
        return state_from_values(m.values), True

    @property
    def n(self) -> int:
        return len(self._rows)


class SerialImu:
    """JSON-line IMU on a serial port distinct from the actuator gateway."""

    def __init__(
        self,
        port: str | None = None,
        baud: int = 115200,
        timeout_s: float = 0.05,
        meas: tuple[str, ...] | list[str] | None = None,
        stream: Any | None = None,
    ):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.meas = tuple(meas) if meas else IMU_KEYS
        self._last_ok: float | None = None
        self._last: Measurement | None = None
        self._buf = ""
        if stream is not None:
            self.ser = stream
        elif port:
            import serial  # type: ignore

            self.ser = serial.Serial(port, baud, timeout=0)
        else:
            self.ser = None

    def feed_line(self, line: str, now: float | None = None) -> Measurement:
        """Parse a fake or captured line (unit tests; no serial required)."""
        stamp = _now(now)
        values = parse_imu_line(line)
        values = {k: values[k] for k in self.meas if k in values}
        m = Measurement(stamp=stamp, values=values, valid=True)
        self._last_ok = stamp
        self._last = m
        return m

    def measure(self, now: float | None = None) -> Measurement:
        stamp = _now(now)
        line = self._readline()
        if line:
            try:
                return self.feed_line(line, now=stamp)
            except Exception:
                pass
        if self._last is not None and not _stale(self._last_ok, stamp, self.timeout_s):
            return Measurement(stamp=self._last.stamp, values=dict(self._last.values), valid=True)
        return Measurement(stamp=stamp, values={}, valid=False)

    def _readline(self) -> str | None:
        if self.ser is None:
            return None
        try:
            if hasattr(self.ser, "readline"):
                raw = self.ser.readline()
                if not raw:
                    return None
                if isinstance(raw, bytes):
                    return raw.decode("utf-8", errors="replace")
                return str(raw)
            # pyserial in non-blocking mode: drain bytes
            n = getattr(self.ser, "in_waiting", 0) or 0
            if n:
                chunk = self.ser.read(n)
                if isinstance(chunk, bytes):
                    self._buf += chunk.decode("utf-8", errors="replace")
            if "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                return line
        except Exception:
            return None
        return None

    def close(self) -> None:
        ser = getattr(self, "ser", None)
        if ser is not None and hasattr(ser, "close"):
            try:
                ser.close()
            except Exception:
                pass

    def read(self) -> tuple[np.ndarray | None, bool]:
        m = self.measure()
        return (None, m.valid)


class WebcamAruco:
    """Optional OpenCV ArUco pose. Constructs and reads as invalid if cv2 is missing."""

    def __init__(
        self,
        camera: int = 0,
        marker_id: int = 0,
        marker_size_m: float = 0.05,
        timeout_s: float = 0.1,
        meas: tuple[str, ...] | list[str] | None = None,
        table_size: float = 2.0,
    ):
        self.camera = camera
        self.marker_id = marker_id
        self.marker_size_m = marker_size_m
        self.timeout_s = timeout_s
        self.table_size = table_size
        self.meas = tuple(meas) if meas else ("x", "y", "yaw")
        self.available = False
        self._cv2 = None
        self._cap = None
        self._last_ok: float | None = None
        self._last: Measurement | None = None
        try:
            import cv2  # type: ignore

            self._cv2 = cv2
            self.available = True
        except Exception:
            self._cv2 = None
            self.available = False

    def measure(self, now: float | None = None) -> Measurement:
        stamp = _now(now)
        if not self.available or self._cv2 is None:
            return Measurement(stamp=stamp, values={}, valid=False)
        try:
            values = self._grab()
        except Exception:
            values = None
        if values:
            m = Measurement(stamp=stamp, values=values, valid=True)
            self._last_ok = stamp
            self._last = m
            return m
        if self._last is not None and not _stale(self._last_ok, stamp, self.timeout_s):
            return Measurement(stamp=self._last.stamp, values=dict(self._last.values), valid=True)
        return Measurement(stamp=stamp, values={}, valid=False)

    def _grab(self) -> dict[str, float] | None:
        cv2 = self._cv2
        if self._cap is None:
            self._cap = cv2.VideoCapture(self.camera)
        if self._cap is None or not self._cap.isOpened():
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids = _detect_aruco(cv2, gray)
        if ids is None or len(ids) == 0:
            return None
        ids = np.asarray(ids).reshape(-1)
        pick = 0
        if self.marker_id is not None:
            hits = np.where(ids == self.marker_id)[0]
            if len(hits) == 0:
                return None
            pick = int(hits[0])
        c = np.asarray(corners[pick]).reshape(-1, 2)
        cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
        h, w = gray.shape[:2]
        half = 0.5 * self.table_size
        # Cheap pinhole-free map: image centre → table origin.
        x = (cx / max(w, 1) - 0.5) * 2.0 * half
        y = -(cy / max(h, 1) - 0.5) * 2.0 * half
        d = c[1] - c[0] if len(c) >= 2 else np.array([1.0, 0.0])
        yaw = float(np.arctan2(d[1], d[0]))
        values = {"x": x, "y": y, "yaw": yaw}
        return {k: values[k] for k in self.meas if k in values}

    def close(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def read(self) -> tuple[np.ndarray | None, bool]:
        m = self.measure()
        if not m.valid:
            return None, False
        return state_from_values(m.values), True


def _detect_aruco(cv2, gray):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        params = cv2.aruco.DetectorParameters()
        det = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = det.detectMarkers(gray)
        return corners, ids
    except Exception:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        return corners, ids


@dataclass
class PoseSourceDriver:
    """Wrap a legacy PoseSource as a Measurement driver."""

    source: Any
    timeout_s: float = 0.0

    def measure(self, now: float | None = None) -> Measurement:
        stamp = _now(now)
        pose, ok = self.source.read()
        if not ok or pose is None:
            return Measurement(stamp=stamp, values={}, valid=False)
        values = {k: float(pose[i]) for i, k in enumerate(POSE_KEYS)}
        return Measurement(stamp=stamp, values=values, valid=True)

    def read(self) -> tuple[np.ndarray | None, bool]:
        return self.source.read()


def default_meas_for(kind: str, role: str) -> tuple[str, ...]:
    if kind == "serial_imu" or (kind == "sim" and role == "onboard"):
        return IMU_KEYS
    if kind in ("http_mocap", "webcam_aruco"):
        return ("x", "y", "yaw")
    if kind == "csv_replay":
        return POSE_KEYS
    if role == "onboard":
        return IMU_KEYS
    return POSE_KEYS


def _src_get(source, name, default=None):
    if isinstance(source, dict):
        val = source.get(name, default)
    else:
        val = getattr(source, name, default)
    return default if val is None else val


def make_driver(
    source,
    *,
    role: str,
    plant: Plant | None = None,
    rng: np.random.Generator | None = None,
    replay: str | Path | None = None,
) -> Any:
    """Build a driver from a SensorSource dataclass (or dict)."""
    kind = _src_get(source, "type")
    meas = tuple(_src_get(source, "meas", ()) or ())
    if not meas:
        meas = default_meas_for(kind, role)
    noise = dict(_src_get(source, "noise", {}) or {})
    timeout_s = float(_src_get(source, "timeout_s", 0.0) or 0.0)
    if kind == "sim":
        if plant is None:
            raise ValueError("sim sensor requires a plant")
        imu_like = set(meas) <= set(IMU_KEYS) or role == "onboard"
        if imu_like:
            return SimulatedImu(plant, meas=meas, noise=noise, timeout_s=timeout_s, rng=rng)
        return SimulatedMocap(plant, meas=meas, noise=noise, timeout_s=timeout_s, rng=rng)
    if kind == "http_mocap":
        endpoint = _src_get(source, "endpoint", "")
        if not endpoint:
            raise ValueError("http_mocap requires endpoint")
        to = timeout_s if timeout_s > 0 else 0.05
        return HttpMocap(endpoint, timeout_s=to, meas=meas)
    if kind == "csv_replay":
        path = replay or _src_get(source, "path")
        if not path:
            raise ValueError("csv_replay requires path")
        return CsvReplay(str(path), meas=meas, timeout_s=timeout_s)
    if kind == "serial_imu":
        port = _src_get(source, "port")
        baud = int(_src_get(source, "baud", 115200) or 115200)
        to = timeout_s if timeout_s > 0 else 0.05
        return SerialImu(port=port, baud=baud, timeout_s=to, meas=meas)
    if kind == "webcam_aruco":
        camera = int(_src_get(source, "camera", 0) or 0)
        marker_id = int(_src_get(source, "marker_id", 0) or 0)
        marker_size = float(_src_get(source, "marker_size_m", 0.05) or 0.05)
        table = float(_src_get(source, "table_size", 0.0) or 0.0)
        if table <= 0:
            table = plant.spec.table_size if plant is not None else 2.0
        to = timeout_s if timeout_s > 0 else 0.1
        return WebcamAruco(
            camera=camera,
            marker_id=marker_id,
            marker_size_m=marker_size,
            timeout_s=to,
            meas=meas,
            table_size=table,
        )
    raise ValueError(f"unknown sensor type {kind}")
