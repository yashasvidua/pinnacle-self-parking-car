#include "sonar.h"
#include "pins.h"
#include "soc/soc.h"       // REG_READ
#include "soc/gpio_reg.h"  // GPIO_IN_REG, GPIO_IN1_REG

// ---- shared with the ISR (DRAM, volatile) ----
static uint8_t s_echoPin[SONAR_COUNT];       // non-const copy so the ISR reads RAM, not flash
static volatile int8_t s_active = -1;        // sensor currently being measured
static volatile bool s_rising = false;       // rising edge seen
static volatile bool s_done = false;         // falling edge seen -> s_durUs valid
static volatile uint32_t s_riseUs = 0;
static volatile uint32_t s_durUs = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- main-loop state ----
static float s_hist[SONAR_COUNT][3];
static uint8_t s_histIdx[SONAR_COUNT];
static float s_median[SONAR_COUNT];
static uint32_t s_lastFireMs = 0;
static bool s_begun = false;

static inline int IRAM_ATTR readLevel(uint8_t pin) {
  if (pin < 32) return (REG_READ(GPIO_IN_REG) >> pin) & 1U;
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1U;
}

static void IRAM_ATTR echoIsr(void* arg) {
  const int idx = (int)(intptr_t)arg;
  if (idx != s_active) return;  // only the sensor we fired counts
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&s_mux);
  if (readLevel(s_echoPin[idx])) {
    s_riseUs = now;
    s_rising = true;
  } else if (s_rising && !s_done) {
    s_durUs = now - s_riseUs;
    s_done = true;
    s_rising = false;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

static void pushReading(int i, float cm) {
  s_hist[i][s_histIdx[i]] = cm;
  s_histIdx[i] = (s_histIdx[i] + 1) % 3;
  float m = median3(s_hist[i][0], s_hist[i][1], s_hist[i][2]);
  s_median[i] = (m < 0) ? -1.0f : m;
}

void sonarBegin() {
  for (int i = 0; i < SONAR_COUNT; i++) {
    pinMode(SONAR_TRIG_PINS[i], OUTPUT);
    digitalWrite(SONAR_TRIG_PINS[i], LOW);
    s_echoPin[i] = SONAR_ECHO_PINS[i];
    pinMode(SONAR_ECHO_PINS[i], INPUT);  // externally driven through the 5V->3.3V divider
    s_hist[i][0] = s_hist[i][1] = s_hist[i][2] = -1.0f;
    s_histIdx[i] = 0;
    s_median[i] = -1.0f;
  }
  for (int i = 0; i < SONAR_COUNT; i++) {
    attachInterruptArg(digitalPinToInterrupt(SONAR_ECHO_PINS[i]), echoIsr,
                       (void*)(intptr_t)i, CHANGE);
  }
  s_active = -1;
  s_lastFireMs = millis();
  s_begun = true;
}

void sonarUpdate() {
  if (!s_begun) return;
  const uint32_t nowMs = millis();
  if (nowMs - s_lastFireMs < SONAR_GAP_MS) return;
  s_lastFireMs = nowMs;

  // 1) Close the measurement of the sensor fired SONAR_GAP_MS ago.
  int prev = s_active;
  if (prev >= 0) {
    bool done;
    uint32_t dur;
    portENTER_CRITICAL(&s_mux);
    done = s_done;
    dur = s_durUs;
    s_active = -1;  // ignore any late edges from it
    portEXIT_CRITICAL(&s_mux);
    float cm = -1.0f;
    if (done && dur > 0 && dur <= SONAR_TIMEOUT_US) {
      cm = dur / 58.0f;  // round trip at ~343 m/s
      if (cm > SONAR_MAX_CM) cm = -1.0f;
    }
    pushReading(prev, cm);
  }

  // 2) Fire the next one.
  int next = (prev + 1) % SONAR_COUNT;  // prev == -1 -> 0
  portENTER_CRITICAL(&s_mux);
  s_rising = false;
  s_done = false;
  s_durUs = 0;
  s_active = (int8_t)next;
  portEXIT_CRITICAL(&s_mux);
  const uint8_t trig = SONAR_TRIG_PINS[next];
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

float sonarGet(int i) {
  if (i < 0 || i >= SONAR_COUNT) return -1.0f;
  return s_median[i];
}
