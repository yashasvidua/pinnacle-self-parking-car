// drive_ctrl.ino - Pinacle DevKit drive controller (1:10 skid-steer self-parking car)
//
// Protocol and safety rules: docs/CONTRACT.md (binding). Summary:
//   UDP 4210 in : C,seq,l,r | S,seq | H,seq | R,seq,deg
//   UDP 4211 out: {"seq":n,"state":"OK|ESTOP|WATCHDOG","d":[fl,fr,bl,br,l,r],"yaw":deg|null,"busy":bool} @10 Hz
//   Watchdog 300 ms, ultrasonic hard stop < 8 cm (release >= 10 cm), R rotate in place.
//
// ---------------------------------------------------------------------------
// Arduino IDE settings
//   Board package : esp32 by Espressif Systems, version 2.0.17 (NOT 3.x: ledc API differs)
//   Board         : "DOIT ESP32 DEVKIT V1"  (or "ESP32 Dev Module")
//   Upload speed  : 921600 (drop to 115200 if uploads fail)
//   Flash freq 80 MHz, Partition "Default 4MB with spiffs", Core debug level "None"
//   Serial Monitor: 115200 baud, line ending "Newline" (or "Both NL & CR")
//   Before building: copy secrets.example.h to secrets.h and fill in the WiFi details.
//   No external libraries needed (WiFi.h, WiFiUdp.h, Wire.h are built in).
//
// Wiring summary (all pins are defined in pins.h; "TODO" = placeholder, confirm!)
//   Buzzer            GPIO18          LED                GPIO19
//   MPU6050 SDA/SCL   GPIO21 / GPIO22 (3V3, GND, AD0 low -> 0x68)
//   Left  motors      IN1 25, IN2 26, PWM/EN 27   (TODO)  2 BO motors in parallel
//   Right motors      IN1 32, IN2 33, PWM/EN 23   (TODO)  2 BO motors in parallel
//   TB6612 STBY       tie to 3V3, or define PIN_MOTOR_STBY in pins.h
//   HC-SR04 trig/echo (TODO)  fl 16/34, fr 17/35, bl 5/36, br 14/39, l 15/4, r 2/13
//     ECHO is 5 V: use a divider (e.g. 1k series + 2k to GND) on every echo line.
//   Common GND between the ESP32, the motor driver, the sensors and the battery.
//
// Startup: motors are forced to STOP first, then the sketch waits for Enter on the
// Serial Monitor (prints "ready - press Enter" every 3 s), then beeps, calibrates the
// gyro (~1 s, keep the car still) and joins WiFi.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <math.h>
#include "pins.h"
#include "motors.h"
#include "sonar.h"
#include "gyro.h"
#include "comms.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "secrets.h not found: copy secrets.example.h to secrets.h and fill in WIFI_SSID / WIFI_PASSWORD"
#endif
#if !defined(WIFI_SSID) || !defined(WIFI_PASSWORD)
#error "secrets.h must define WIFI_SSID and WIFI_PASSWORD"
#endif
#ifdef LAPTOP_IP
#define PINACLE_LAPTOP_IP LAPTOP_IP
#else
#define PINACLE_LAPTOP_IP nullptr
#endif

// Motion direction classes for the hard stop (contract: v = left + right)
#define MOVE_NONE 0
#define MOVE_FWD  1
#define MOVE_REV  2
#define MOVE_ROT  3

// Sensor indices (contract order)
#define S_FL 0
#define S_FR 1
#define S_BL 2
#define S_BR 3
#define S_L  4
#define S_R  5

// ---- commanded motion ----
static int g_cmdL = 0;
static int g_cmdR = 0;

// ---- watchdog ----
static uint32_t g_lastValidMs = 0;
static bool g_watchdog = true;  // nothing received yet -> WATCHDOG, motors off

// ---- R rotation ----
static bool g_rotActive = false;
static bool g_rotGyro = false;      // closed loop (true) or timed (false)
static float g_rotTargetYaw = 0.0f;
static int g_rotSign = 0;           // timed mode: +1 CCW, -1 CW
static uint32_t g_rotStartMs = 0;
static uint32_t g_rotDurMs = 0;     // timed mode duration
static int g_rotL = 0;
static int g_rotR = 0;

// ---- hard stop (ESTOP) ----
static bool g_estop = false;
static int g_estopDir = MOVE_NONE;
static bool g_estopHoldRot = false; // an R was cancelled by ESTOP: keep latched until a new command

// ---- outputs / timing ----
static uint32_t g_lastTelemMs = 0;
static uint32_t g_lastStatusMs = 0;
static bool g_buzzerOn = false;
static bool g_ledOn = false;

// ===========================================================================
// Small helpers
// ===========================================================================
static void buzzerWrite(bool on) {
  if (on == g_buzzerOn) return;
  g_buzzerOn = on;
#if BUZZER_ACTIVE
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
#else
  ledcWrite(BUZZER_LEDC_CH, on ? (1U << (PWM_BITS - 1)) : 0);  // 50% duty
#endif
}

static void ledWrite(bool on) {
  if (on == g_ledOn) return;
  g_ledOn = on;
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

static void waitForEnter() {
  uint32_t lastPrompt = 0;
  bool first = true;
  for (;;) {
    if (first || millis() - lastPrompt >= 3000) {
      first = false;
      lastPrompt = millis();
      Serial.println("ready - press Enter");
    }
    while (Serial.available() > 0) {
      int c = Serial.read();
      if (c == '\n' || c == '\r') {
        delay(20);  // let a trailing \n of "\r\n" arrive
        while (Serial.available() > 0) Serial.read();  // drain
        return;
      }
    }
    delay(10);
  }
}

static int motionDir(int l, int r) {
  int v = l + r;
  if (v > 0) return MOVE_FWD;
  if (v < 0) return MOVE_REV;
  if (l != 0) return MOVE_ROT;
  return MOVE_NONE;
}

static bool inRange(int idx, float thresholdCm) {
  float d = sonarGet(idx);
  return d >= 0.0f && d < thresholdCm;  // -1 (no echo) counts as clear
}

// Is the motion class `dir` blocked by a reading below thresholdCm?
static bool blocked(int dir, float thresholdCm) {
  switch (dir) {
    case MOVE_FWD: return inRange(S_FL, thresholdCm) || inRange(S_FR, thresholdCm);
    case MOVE_REV: return inRange(S_BL, thresholdCm) || inRange(S_BR, thresholdCm);
    case MOVE_ROT: return inRange(S_FL, thresholdCm) || inRange(S_FR, thresholdCm) ||
                         inRange(S_BL, thresholdCm) || inRange(S_BR, thresholdCm);
    default: return false;
  }
}

static const char* dirName(int dir) {
  switch (dir) {
    case MOVE_FWD: return "forward";
    case MOVE_REV: return "reverse";
    case MOVE_ROT: return "rotation";
    default: return "none";
  }
}

// ===========================================================================
// Rotation (R)
// ===========================================================================
static void cancelRotation(const char* why) {
  if (!g_rotActive) return;
  g_rotActive = false;
  g_rotL = 0;
  g_rotR = 0;
  Serial.printf("[rot] cancelled (%s)\n", why);
}

static void startRotation(float deg, uint32_t now) {
  g_cmdL = 0;  // R replaces any C motion; after the turn the car stays stopped
  g_cmdR = 0;
  g_rotL = 0;
  g_rotR = 0;
  g_rotStartMs = now;
  g_rotGyro = gyroOk();
  if (g_rotGyro) {
    g_rotTargetYaw = gyroYaw() + deg;
    g_rotActive = fabsf(deg) > ROTATE_TOL_DEG;
  } else {
    g_rotSign = (deg > 0) ? 1 : -1;
    g_rotDurMs = (uint32_t)(fabsf(deg) * ROTATE_MS_PER_DEG);
    g_rotActive = g_rotDurMs > 0;
  }
  Serial.printf("[rot] R %.1f deg, %s%s\n", deg, g_rotGyro ? "gyro" : "timed",
                g_rotActive ? "" : " (nothing to do)");
}

// positive pwm = CCW = left backward, right forward
static void setSpin(int signedPwm) {
  g_rotL = -signedPwm;
  g_rotR = signedPwm;
}

static void updateRotation(uint32_t now) {
  if (!g_rotActive) return;
  const uint32_t elapsed = now - g_rotStartMs;

  if (g_rotGyro) {
    if (!gyroOk()) {  // MPU died mid-turn: stop rather than guess
      cancelRotation("gyro lost");
      return;
    }
    const float err = g_rotTargetYaw - gyroYaw();
    if (fabsf(err) <= ROTATE_TOL_DEG) {
      g_rotActive = false;
      g_rotL = g_rotR = 0;
      Serial.printf("[rot] done, err %.1f deg, %lu ms\n", err, (unsigned long)elapsed);
      return;
    }
    if (elapsed >= ROTATE_TIMEOUT_MS) {
      g_rotActive = false;
      g_rotL = g_rotR = 0;
      Serial.printf("[rot] TIMEOUT, err %.1f deg\n", err);
      return;
    }
    float k = fabsf(err) / ROTATE_SLOW_DEG;
    if (k > 1.0f) k = 1.0f;
    int pwm = ROTATE_MIN_PWM + (int)((ROTATE_PWM - ROTATE_MIN_PWM) * k);
    setSpin(err > 0 ? pwm : -pwm);  // overshoot reverses the spin
  } else {
    if (elapsed >= g_rotDurMs) {
      g_rotActive = false;
      g_rotL = g_rotR = 0;
      Serial.printf("[rot] timed spin done, %lu ms\n", (unsigned long)elapsed);
      return;
    }
    setSpin(g_rotSign * ROTATE_PWM);
  }
}

// ===========================================================================
// Commands
// ===========================================================================
static void handleCommand(const DriveCommand& c, uint32_t now) {
  g_lastValidMs = now;
  if (g_watchdog) {
    g_watchdog = false;
    Serial.println("[wd] link OK");
  }
  switch (c.type) {
    case 'C':
      cancelRotation("C");
      g_estopHoldRot = false;
      g_cmdL = c.left;
      g_cmdR = c.right;
      break;
    case 'S':
      cancelRotation("S");
      g_estopHoldRot = false;
      g_cmdL = 0;
      g_cmdR = 0;
      break;
    case 'R':
      g_rotActive = false;  // a new R replaces the old one
      g_estopHoldRot = false;
      startRotation(c.deg, now);
      break;
    case 'H':
    default:
      break;  // heartbeat: feeds the watchdog only
  }
}

// ===========================================================================
// Setup / loop
// ===========================================================================
void setup() {
  // 1) Motors safe before anything else so the car can't twitch.
  motorsBegin();
  motorsStop();
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
#if !BUZZER_ACTIVE
  ledcSetup(BUZZER_LEDC_CH, BUZZER_TONE_HZ, PWM_BITS);
  ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CH);
  ledcWrite(BUZZER_LEDC_CH, 0);
#endif

  // 2) Startup gate.
  Serial.begin(115200);
  waitForEnter();
  Serial.println("\n[pinacle] drive_ctrl starting");

  // 3) Short boot beep (a delay in setup() is fine; loop() never blocks).
  buzzerWrite(true);
  delay(120);
  buzzerWrite(false);

  // 4) Peripherals. Keep the car still during the gyro calibration.
  bool gyro = gyroBegin();
  sonarBegin();
  commsBegin(WIFI_SSID, WIFI_PASSWORD, PINACLE_LAPTOP_IP);
  Serial.printf("[pinacle] gyro %s, watchdog %d ms, stop < %.0f cm\n",
                gyro ? "OK" : "OFF", WATCHDOG_MS, STOP_DIST_CM);

  g_lastValidMs = millis();
  g_watchdog = true;  // stays WATCHDOG until the laptop talks
}

void loop() {
  commsUpdate();
  sonarUpdate();
  gyroUpdate();

  const uint32_t now = millis();

  // ---- 1. incoming commands ----
  DriveCommand c;
  for (int i = 0; i < 8 && commsPoll(c); i++) handleCommand(c, now);

  // ---- 2. watchdog ----
  if (!g_watchdog && now - g_lastValidMs > WATCHDOG_MS) {
    g_watchdog = true;
    cancelRotation("watchdog");
    g_cmdL = 0;
    g_cmdR = 0;
    motorsStop();
    Serial.println("[wd] WATCHDOG: no valid packet for 300 ms, motors off");
  }

  // ---- 3. rotation controller ----
  updateRotation(now);

  int l = g_rotActive ? g_rotL : g_cmdL;
  int r = g_rotActive ? g_rotR : g_cmdR;

  // ---- 4. hard stop (latching, with hysteresis) ----
  int dir = motionDir(l, r);
  if (dir == MOVE_NONE && g_estopHoldRot) dir = MOVE_ROT;
  if (g_estop) {
    // Release when the motion no longer points at the obstacle (zero / other way),
    // or every relevant reading is back to >= STOP_DIST_CM + STOP_HYST_CM (or -1).
    if (dir != g_estopDir || !blocked(g_estopDir, STOP_DIST_CM + STOP_HYST_CM)) {
      g_estop = false;
      Serial.println("[estop] released");
    }
  }
  if (!g_estop && dir != MOVE_NONE && blocked(dir, STOP_DIST_CM)) {
    g_estop = true;
    g_estopDir = dir;
    Serial.printf("[estop] HARD STOP (%s) d=[%.1f,%.1f,%.1f,%.1f]\n", dirName(dir),
                  sonarGet(S_FL), sonarGet(S_FR), sonarGet(S_BL), sonarGet(S_BR));
    if (g_rotActive) {
      cancelRotation("ESTOP");
      g_estopHoldRot = true;
    }
  }
  if (!g_estop) g_estopHoldRot = false;

  // ---- 5. drive ----
  if (g_estop || g_watchdog) motorsStop();
  else motorsSet(l, r);

  const TelemetryState st = g_estop ? TS_ESTOP : (g_watchdog ? TS_WATCHDOG : TS_OK);

  // ---- 6. telemetry 10 Hz ----
  if (now - g_lastTelemMs >= TELEMETRY_PERIOD_MS) {
    g_lastTelemMs = now;
    float d[SONAR_COUNT];
    for (int i = 0; i < SONAR_COUNT; i++) d[i] = sonarGet(i);
    commsSendTelemetry(st, d, gyroOk(), gyroYaw(), g_rotActive);
  }

  // ---- 7. LED: solid = WiFi + telemetry flowing, else blink 2 Hz ----
  ledWrite(commsTelemetryFlowing() ? true : ((now / 250) % 2) == 0);

  // ---- 8. buzzer: boot beep, then beeping while ESTOP ----
  buzzerWrite(g_estop && (now % 300) < 120);

  // ---- 9. 1 Hz status line ----
  if (now - g_lastStatusMs >= 1000) {
    g_lastStatusMs = now;
    static const char* const kSt[] = {"OK", "ESTOP", "WATCHDOG"};
    char yawBuf[16];
    if (gyroOk()) snprintf(yawBuf, sizeof(yawBuf), "%.1f", gyroYaw());
    else snprintf(yawBuf, sizeof(yawBuf), "null");
    Serial.printf("[st] %-8s wifi=%d seq=%lu cmd=%d,%d out=%d,%d busy=%d yaw=%s "
                  "d=[%.1f %.1f %.1f %.1f %.1f %.1f]\n",
                  kSt[st], commsWifiConnected() ? 1 : 0, (unsigned long)commsLastSeq(),
                  l, r, motorsLeft(), motorsRight(), g_rotActive ? 1 : 0, yawBuf,
                  sonarGet(0), sonarGet(1), sonarGet(2), sonarGet(3), sonarGet(4), sonarGet(5));
  }
}
