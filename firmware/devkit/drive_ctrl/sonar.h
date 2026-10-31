// sonar.h - 6x HC-SR04, non-blocking round robin.
// One sensor is fired every SONAR_GAP_MS. Its echo is timed by a CHANGE interrupt
// (micros()), so the main loop never waits for an echo.
// Index order (contract): 0 fl, 1 fr, 2 bl, 3 br, 4 l, 5 r.
#pragma once
#include <Arduino.h>

void sonarBegin();
void sonarUpdate();      // call every loop()
float sonarGet(int i);   // median of the last 3 readings, cm; -1 = no echo / out of range / bad index
