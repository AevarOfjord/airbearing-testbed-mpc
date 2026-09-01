"""Optional serial gateways. Simulation never imports pyserial.

Protocols
---------
solenoid:  CMD:<bitmask>:<duration_ms>\\n     bitmask is integer, bit i = thruster i
fan pwm:   PWM:<d0>,<d1>,...\\n               duties in [-1,1] or [0,1]
Both gateways implement a ~100 ms deadman: missing frames force actuators to 0.
"""

from __future__ import annotations

from airbearing.spec import SatelliteSpec


class NullGateway:
    def send(self, cmd) -> None:
        return

    def close(self) -> None:
        return


class SolenoidGateway:
    def __init__(self, port: str, baud: int = 115200, duration_ms: int = 80):
        import serial  # type: ignore

        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.duration_ms = duration_ms

    def send(self, cmd) -> None:
        mask = 0
        for i, c in enumerate(cmd):
            if c >= 0.5:
                mask |= 1 << i
        line = f"CMD:{mask}:{self.duration_ms}\n"
        self.ser.write(line.encode("ascii"))

    def close(self) -> None:
        try:
            self.ser.write(b"CMD:0:0\n")
            self.ser.close()
        except Exception:
            pass


class FanPwmGateway:
    def __init__(self, port: str, baud: int = 115200):
        import serial  # type: ignore

        self.ser = serial.Serial(port, baud, timeout=0.05)

    def send(self, cmd) -> None:
        payload = ",".join(f"{float(c):.3f}" for c in cmd)
        self.ser.write(f"PWM:{payload}\n".encode("ascii"))

    def close(self) -> None:
        try:
            zeros = ",".join("0" for _ in range(16))
            self.ser.write(f"PWM:{zeros}\n".encode("ascii"))
            self.ser.close()
        except Exception:
            pass


def open_gateway(spec: SatelliteSpec, port: str | None) -> NullGateway | SolenoidGateway | FanPwmGateway:
    if not port:
        return NullGateway()
    types = {t.type for t in spec.thrusters}
    if types <= {"binary_solenoid"}:
        return SolenoidGateway(port)
    return FanPwmGateway(port)
