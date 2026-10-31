#include "gyro.h"
#include "pins.h"

#if USE_MPU6050

#include <Wire.h>

#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_GYRO_ZOUT_H  0x47
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75
#define GYRO_LSB_PER_DPS     65.5f   // +-500 dps range
#define GYRO_MAX_FAILS       10      // consecutive bus errors before we declare the gyro dead

static bool s_ok = false;
static float s_biasRaw = 0.0f;
static float s_yaw = 0.0f;
static uint32_t s_lastSampleUs = 0;
static uint8_t s_fails = 0;

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom((uint8_t)MPU6050_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

static bool readGyroZ(int16_t& out) {
  uint8_t b[2];
  if (!readRegs(MPU_REG_GYRO_ZOUT_H, b, 2)) return false;
  out = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  return true;
}

bool gyroBegin() {
  s_ok = false;
  s_yaw = 0.0f;
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);  // ms; a dead bus must not stall the control loop for long

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[gyro] no MPU6050 at 0x68 - yaw disabled, R uses timed fallback");
    return false;
  }
  uint8_t who = 0;
  readRegs(MPU_REG_WHO_AM_I, &who, 1);
  Serial.printf("[gyro] MPU found, WHO_AM_I=0x%02X\n", who);

  bool ok = writeReg(MPU_REG_PWR_MGMT_1, 0x01);  // wake, clock = PLL on gyro X
  delay(50);
  ok = ok && writeReg(MPU_REG_CONFIG, 0x03);       // DLPF ~44 Hz
  ok = ok && writeReg(MPU_REG_GYRO_CONFIG, 0x08);  // +-500 dps
  if (!ok) {
    Serial.println("[gyro] config write failed - yaw disabled");
    return false;
  }
  delay(50);

  // Bias calibration: average GYRO_ZOUT for ~1 s with the car still.
  Serial.println("[gyro] calibrating bias, keep the car still...");
  int32_t sum = 0;
  int n = 0, fails = 0;
  int16_t lo = 32767, hi = -32768;
  const uint32_t t0 = millis();
  while (millis() - t0 < GYRO_CAL_MS) {
    int16_t z;
    if (readGyroZ(z)) {
      sum += z;
      n++;
      if (z < lo) lo = z;
      if (z > hi) hi = z;
    } else {
      fails++;
    }
    delay(5);
  }
  if (n < 50) {
    Serial.printf("[gyro] calibration failed (%d samples, %d errors) - yaw disabled\n", n, fails);
    return false;
  }
  s_biasRaw = (float)sum / n;
  Serial.printf("[gyro] bias %.2f dps (%d samples, spread %.2f dps)\n",
                s_biasRaw / GYRO_LSB_PER_DPS, n, (hi - lo) / GYRO_LSB_PER_DPS);
  if ((hi - lo) / GYRO_LSB_PER_DPS > 5.0f) {
    Serial.println("[gyro] WARNING: large spread - was the car moving during calibration?");
  }
  s_fails = 0;
  s_lastSampleUs = micros();
  s_ok = true;
  return true;
}

void gyroUpdate() {
  if (!s_ok) return;
  const uint32_t now = micros();
  const uint32_t dtUs = now - s_lastSampleUs;
  if (dtUs < GYRO_PERIOD_US) return;

  int16_t z;
  if (!readGyroZ(z)) {
    if (++s_fails >= GYRO_MAX_FAILS) {
      s_ok = false;
      Serial.println("[gyro] MPU6050 stopped answering - yaw disabled");
    }
    return;  // keep s_lastSampleUs: the next good sample integrates over the gap
  }
  s_fails = 0;
  s_lastSampleUs = now;

  float dt = dtUs * 1e-6f;
  if (dt > 0.05f) dt = 0.05f;  // don't integrate a long outage as if the rate held
  float dps = ((float)z - s_biasRaw) / GYRO_LSB_PER_DPS;
  if (fabsf(dps) < GYRO_DEADBAND_DPS) dps = 0.0f;
#if GYRO_INVERT
  dps = -dps;
#endif
  s_yaw += dps * dt;  // Z up, so positive rate = CCW
}

bool gyroOk() { return s_ok; }
float gyroYaw() { return s_ok ? s_yaw : 0.0f; }

#else  // USE_MPU6050 == 0: stubs

bool gyroBegin() {
  Serial.println("[gyro] USE_MPU6050=0 - yaw disabled, R uses timed fallback");
  return false;
}
void gyroUpdate() {}
bool gyroOk() { return false; }
float gyroYaw() { return 0.0f; }

#endif
