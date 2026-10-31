// motors.h - dual H-bridge (TB6612 / L298N) skid-steer drive.
// Each side = 2 BO motors in parallel on one H-bridge channel.
#pragma once
#include <Arduino.h>

// Configure pins and PWM and leave the motors stopped. Safe to call before anything else.
void motorsBegin();

// left/right in -255..255 (clamped). Sign = direction (+ = forward), magnitude = PWM duty.
void motorsSet(int left, int right);

// Stop = COAST: IN1 = IN2 = LOW and PWM = 0 (TB6612 "stop" / L298N "free run").
// We coast rather than brake so a stop never jerks the car or browns out the supply.
void motorsStop();

// Last applied values (after clamping, before inversion).
int motorsLeft();
int motorsRight();
