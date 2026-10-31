// pins.h - every GPIO and every tunable of the drive controller lives here.
// Board: ESP32 DevKit V1 (30-pin, ESP32-WROOM-32), esp32 core 2.0.17.
//
// Pins we avoid: 0, 1, 3 (boot / UART0), 6-11 (SPI flash), 12 (flash-voltage strap).
// 34, 35, 36 (VP), 39 (VN) are input only and have no pull-ups: fine for echo pins.
// Strapping pins 2, 5, 15 (and 14, which outputs PWM at boot) are only used as
// HC-SR04 TRIG outputs. A stray pulse there at boot just fires a sonar and can't
// move the car. The motor driver sits on "quiet" pins.
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Fixed wiring
// ---------------------------------------------------------------------------
#define PIN_BUZZER      18
#define PIN_LED         19
#define PIN_I2C_SDA     21   // MPU6050 SDA
#define PIN_I2C_SCL     22   // MPU6050 SCL

// Buzzer type: 1 = active buzzer (drive HIGH = sound), 0 = passive (driven by a PWM tone).
#define BUZZER_ACTIVE   1
#define BUZZER_TONE_HZ  2700
#define BUZZER_LEDC_CH  2    // only used when BUZZER_ACTIVE == 0 (timer 1; motors use timer 0)

// ---------------------------------------------------------------------------
// Motor driver: generic dual H-bridge (TB6612FNG or L298N style).
// Per side: IN1, IN2 (direction) + PWM (TB6612 PWMx / L298N ENx, remove the EN jumper).
// Each side drives 2 BO motors wired in parallel.
// ---------------------------------------------------------------------------
#define PIN_LEFT_IN1    25   // TODO: confirm wiring
#define PIN_LEFT_IN2    26   // TODO: confirm wiring
#define PIN_LEFT_PWM    27   // TODO: confirm wiring
#define PIN_RIGHT_IN1   32   // TODO: confirm wiring
#define PIN_RIGHT_IN2   33   // TODO: confirm wiring
#define PIN_RIGHT_PWM   23   // TODO: confirm wiring
// TB6612 STBY: uncomment and set a pin to have it driven HIGH at boot, or tie
// STBY to 3V3. All safe GPIOs are used by the placeholder map, so free one first.
// #define PIN_MOTOR_STBY  4  // TODO: confirm wiring

#define LEFT_LEDC_CH    0
#define RIGHT_LEDC_CH   1
#define PWM_FREQ        1000 // Hz. 1 kHz suits the L298N; the TB6612 is happy up to ~20 kHz
#define PWM_BITS        8    // duty 0..255

// Set to 1 if a side drives backwards when commanded forward (or swap its motor wires).
#define LEFT_INVERT     0
#define RIGHT_INVERT    0

// ---------------------------------------------------------------------------
// HC-SR04 ultrasonic sensors. Index order is fixed by the contract:
//   0 = fl (front-left), 1 = fr, 2 = bl, 3 = br, 4 = l (left side), 5 = r (right side)
// !! HC-SR04 ECHO is a 5 V signal. Put a divider on every ECHO line before the
// !! ESP32 (e.g. 1 k series + 2 k to GND, giving ~3.3 V). TRIG is fine at 3.3 V.
// ---------------------------------------------------------------------------
#define SONAR_COUNT     6
//                                            fl  fr  bl  br   l   r
static const uint8_t SONAR_TRIG_PINS[SONAR_COUNT] = {16, 17,  5, 14, 15,  2}; // TODO: confirm wiring
static const uint8_t SONAR_ECHO_PINS[SONAR_COUNT] = {34, 35, 36, 39,  4, 13}; // TODO: confirm wiring

#define SONAR_GAP_MS      30     // one sensor fired every 30 ms (round robin)
#define SONAR_TIMEOUT_US  25000  // echo longer than this = no echo (~430 cm)
#define SONAR_MAX_CM      400.0f // readings beyond this are reported as -1

// ---------------------------------------------------------------------------
// MPU6050 gyro (yaw)
// ---------------------------------------------------------------------------
#define USE_MPU6050        1      // 0 = no gyro: yaw is null, R uses the timed fallback
#define MPU6050_ADDR       0x68   // AD0 low
#define GYRO_INVERT        0      // 1 if the board is mounted upside down (yaw must be CCW positive)
#define GYRO_PERIOD_US     5000   // sample at 200 Hz
#define GYRO_CAL_MS        1000   // bias calibration time at boot (car must be still)
#define GYRO_DEADBAND_DPS  0.3f   // rates below this are treated as 0 (limits drift)

// ---------------------------------------------------------------------------
// R (rotate in place)
// ---------------------------------------------------------------------------
#define ROTATE_PWM          180    // PWM used for spinning
#define ROTATE_MIN_PWM      120    // PWM used right at the target (must still overcome friction)
#define ROTATE_SLOW_DEG     25.0f  // start slowing down this many degrees before the target
#define ROTATE_TOL_DEG      3.0f   // contract: done when |error| <= 3 deg
#define ROTATE_TIMEOUT_MS   4000   // contract: gyro rotation gives up after 4 s
#define ROTATE_MS_PER_DEG   6.0f   // timed fallback (no gyro): ms of spin per degree at ROTATE_PWM. Calibrate!

// ---------------------------------------------------------------------------
// Safety
// ---------------------------------------------------------------------------
#define WATCHDOG_MS     300     // no valid packet for this long -> stop
#define STOP_DIST_CM    8.0f    // hard stop below this
#define STOP_HYST_CM    2.0f    // release at STOP_DIST_CM + STOP_HYST_CM (10 cm)

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------
#define UDP_CMD_PORT        4210
#define UDP_TELEM_PORT      4211
#define TELEMETRY_PERIOD_MS 100    // 10 Hz
#define WIFI_RETRY_MS       10000  // re-issue WiFi.begin() this often while disconnected
