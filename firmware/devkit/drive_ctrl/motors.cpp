#include "motors.h"
#include "pins.h"

static int s_left = 0;
static int s_right = 0;
static bool s_applied = false;  // false = force the next write

static int clamp255(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

static uint32_t dutyFor(int mag) {
  const uint32_t maxDuty = (1UL << PWM_BITS) - 1;
  return ((uint32_t)mag * maxDuty) / 255UL;
}

static void driveSide(uint8_t in1, uint8_t in2, uint8_t ch, int v) {
  if (v == 0) {
    ledcWrite(ch, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }
  if (v > 0) {
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  ledcWrite(ch, dutyFor(abs(v)));
}

static void setupSide(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t ch) {
  pinMode(in1, OUTPUT);
  digitalWrite(in1, LOW);
  pinMode(in2, OUTPUT);
  digitalWrite(in2, LOW);
  pinMode(pwmPin, OUTPUT);
  digitalWrite(pwmPin, LOW);
  ledcSetup(ch, PWM_FREQ, PWM_BITS);   // esp32 core 2.0.x API
  ledcAttachPin(pwmPin, ch);
  ledcWrite(ch, 0);
}

void motorsBegin() {
  setupSide(PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_LEFT_PWM, LEFT_LEDC_CH);
  setupSide(PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_RIGHT_PWM, RIGHT_LEDC_CH);
#ifdef PIN_MOTOR_STBY
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, HIGH);  // TB6612 enabled; the IN pins already hold it stopped
#endif
  s_left = 0;
  s_right = 0;
  s_applied = true;
}

void motorsSet(int left, int right) {
  left = clamp255(left);
  right = clamp255(right);
  if (s_applied && left == s_left && right == s_right) return;  // nothing changed
  s_left = left;
  s_right = right;
  s_applied = true;
  driveSide(PIN_LEFT_IN1, PIN_LEFT_IN2, LEFT_LEDC_CH, LEFT_INVERT ? -left : left);
  driveSide(PIN_RIGHT_IN1, PIN_RIGHT_IN2, RIGHT_LEDC_CH, RIGHT_INVERT ? -right : right);
}

void motorsStop() {
  motorsSet(0, 0);
}

int motorsLeft() { return s_left; }
int motorsRight() { return s_right; }
