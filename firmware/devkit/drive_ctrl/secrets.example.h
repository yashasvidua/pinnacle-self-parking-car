// secrets.example.h - copy to secrets.h (same folder) and fill in.
// secrets.h is gitignored; never commit real credentials.
#pragma once

#define WIFI_SSID     "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// Optional: where to send telemetry before the first laptop packet arrives.
// Without it, telemetry starts once the laptop sends anything (its heartbeat does).
// #define LAPTOP_IP  "192.168.1.50"
