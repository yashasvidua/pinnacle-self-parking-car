// gyro.h - MPU6050 yaw from raw Wire registers (no library).
// Yaw = integrated gyro Z, degrees, CCW positive, unwrapped, zeroed at boot.
// With USE_MPU6050 0 every function is a stub and gyroOk() is false.
#pragma once
#include <Arduino.h>

bool gyroBegin();     // false if nothing ACKs at MPU6050_ADDR. Calibrates bias (~1 s, keep the car still).
void gyroUpdate();    // call every loop(); samples at 200 Hz
bool gyroOk();        // false if disabled, absent, or the bus failed at runtime
float gyroYaw();      // degrees (0 if !gyroOk())
