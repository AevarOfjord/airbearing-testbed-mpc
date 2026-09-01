/*
  Solenoid gateway for planar air-bearing thrusters.
  Protocol:  CMD:<bitmask>:<duration_ms>\n
  Deadman:   if no valid CMD arrives within DEADMAN_MS, all valves OFF.

  Map bit i -> digital pin PIN_MAP[i]. Active HIGH.
  This is lab equipment, not flight software.
*/

const uint8_t N = 8;
const uint8_t PIN_MAP[N] = {2, 3, 4, 5, 6, 7, 8, 9};
const unsigned DEADMAN_MS = 100;

unsigned long last_cmd_ms = 0;
unsigned long fire_until_ms = 0;
uint16_t mask = 0;

void allOff() {
  for (uint8_t i = 0; i < N; i++) digitalWrite(PIN_MAP[i], LOW);
}

void applyMask(uint16_t m) {
  for (uint8_t i = 0; i < N; i++) digitalWrite(PIN_MAP[i], (m & (1u << i)) ? HIGH : LOW);
}

void setup() {
  for (uint8_t i = 0; i < N; i++) {
    pinMode(PIN_MAP[i], OUTPUT);
    digitalWrite(PIN_MAP[i], LOW);
  }
  Serial.begin(115200);
  last_cmd_ms = millis();
}

void loop() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      if (n > 4 && buf[0] == 'C' && buf[1] == 'M' && buf[2] == 'D' && buf[3] == ':') {
        unsigned m = 0, d = 0;
        if (sscanf(buf + 4, "%u:%u", &m, &d) == 2) {
          mask = (uint16_t)m;
          unsigned long now = millis();
          fire_until_ms = now + d;
          last_cmd_ms = now;
          applyMask(mask);
        }
      }
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;
    }
  }
  unsigned long now = millis();
  if (now - last_cmd_ms >= DEADMAN_MS) {
    allOff();
    mask = 0;
  } else if (now >= fire_until_ms) {
    allOff();
  }
}
