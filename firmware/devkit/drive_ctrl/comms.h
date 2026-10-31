// comms.h - WiFi station + UDP command parser (port 4210) + JSON telemetry (port 4211).
// Protocol: docs/CONTRACT.md section 1.
#pragma once
#include <Arduino.h>

enum TelemetryState { TS_OK = 0, TS_ESTOP = 1, TS_WATCHDOG = 2 };

struct DriveCommand {
  char type;      // 'C', 'S', 'H' or 'R'
  uint32_t seq;
  int left;       // C only, clamped to -255..255
  int right;      // C only, clamped to -255..255
  float deg;      // R only, relative, CCW positive
};

// laptopIp may be nullptr (or "") -> no telemetry until the first valid packet arrives.
void commsBegin(const char* ssid, const char* password, const char* laptopIp);

// Call every loop(): keeps WiFi connected, (re)opens the UDP socket.
void commsUpdate();

// Returns true and fills `out` with the next VALID command, if any is waiting.
// Malformed datagrams are silently dropped. Call repeatedly until it returns false.
bool commsPoll(DriveCommand& out);

// Sends one telemetry datagram (if connected and a destination is known).
// d = 6 distances in contract order, -1 = no echo. yawValid=false -> "yaw":null.
void commsSendTelemetry(TelemetryState st, const float d[6], bool yawValid, float yaw, bool busy);

bool commsWifiConnected();
bool commsTelemetryFlowing();   // connected and telemetry sent within the last 500 ms
uint32_t commsLastSeq();        // seq of the last valid laptop packet
