/*
  Onboard IMU stream for the air-bearing kit.

  Prints one JSON object per line, ~100 Hz, body frame, SI:
    {"ax": <m/s^2>, "ay": <m/s^2>, "gyro_z": <rad/s>}

  Deadman stays on the actuator gateway (fan_pwm_gateway / solenoid_gateway),
  not on this IMU stream. Dropping IMU packets must not cut thrusters by itself.
  Use a second serial port (navigation.onboard.port); never share the PWM/CMD
  gateway cable.

  Lab equipment, not flight software.

  Default: analog pins A0/A1/A2 with a mid-scale zero. Replace analogRead with
  your IMU (MPU6050 / ICM-42688 / …) after a bench calibration.
*/

const unsigned PERIOD_MS = 10;  // ~100 Hz
const int PIN_AX = A0;
const int PIN_AY = A1;
const int PIN_GZ = A2;
// Counts → SI. Edit after calibration. 512 ≈ 0, 1 g ≈ 512 counts.
const float COUNTS_ZERO = 512.0;
const float AX_SCALE = 9.81 / 512.0;
const float AY_SCALE = 9.81 / 512.0;
const float GZ_SCALE = 4.0 / 512.0;  // rad/s per count; edit for your gyro

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long t0 = millis();
  float ax = (analogRead(PIN_AX) - COUNTS_ZERO) * AX_SCALE;
  float ay = (analogRead(PIN_AY) - COUNTS_ZERO) * AY_SCALE;
  float gz = (analogRead(PIN_GZ) - COUNTS_ZERO) * GZ_SCALE;
  Serial.print("{\"ax\":");
  Serial.print(ax, 5);
  Serial.print(",\"ay\":");
  Serial.print(ay, 5);
  Serial.print(",\"gyro_z\":");
  Serial.print(gz, 5);
  Serial.println("}");
  unsigned long dt = millis() - t0;
  if (dt < PERIOD_MS) delay(PERIOD_MS - dt);
}
