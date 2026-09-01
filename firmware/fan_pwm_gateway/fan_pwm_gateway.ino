/*
  PWM fan gateway.
  Protocol:  PWM:<d0>,<d1>,<d2>,...\n     duties in [-1, 1] or [0, 1]
  Deadman:   no frame within DEADMAN_MS -> all duties 0.

  Unidirectional: duty 0..1 on PIN_PWM[i].
  Bidirectional:  sign on PIN_DIR[i], magnitude PWM.

  Lab equipment, not flight software.
*/

const uint8_t N = 6;
const uint8_t PIN_PWM[N] = {3, 5, 6, 9, 10, 11};  // analogWrite capable
const uint8_t PIN_DIR[N] = {2, 4, 7, 8, 12, 13};
const unsigned DEADMAN_MS = 100;

unsigned long last_cmd_ms = 0;
float duty[N];

void allZero() {
  for (uint8_t i = 0; i < N; i++) {
    duty[i] = 0;
    analogWrite(PIN_PWM[i], 0);
    digitalWrite(PIN_DIR[i], LOW);
  }
}

void apply() {
  for (uint8_t i = 0; i < N; i++) {
    float d = duty[i];
    if (d < 0) {
      digitalWrite(PIN_DIR[i], HIGH);
      d = -d;
    } else {
      digitalWrite(PIN_DIR[i], LOW);
    }
    if (d > 1) d = 1;
    analogWrite(PIN_PWM[i], (int)(d * 255.0));
  }
}

void setup() {
  for (uint8_t i = 0; i < N; i++) {
    pinMode(PIN_PWM[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
  }
  allZero();
  Serial.begin(115200);
  last_cmd_ms = millis();
}

void loop() {
  static char buf[96];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      if (n > 4 && buf[0] == 'P' && buf[1] == 'W' && buf[2] == 'M' && buf[3] == ':') {
        char *p = buf + 4;
        uint8_t i = 0;
        while (i < N && *p) {
          duty[i++] = atof(p);
          while (*p && *p != ',') p++;
          if (*p == ',') p++;
        }
        last_cmd_ms = millis();
        apply();
      }
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else n = 0;
  }
  if (millis() - last_cmd_ms >= DEADMAN_MS) {
    allZero();
  }
}
