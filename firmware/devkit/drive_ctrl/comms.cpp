#include "comms.h"
#include "pins.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static WiFiUDP s_udp;
static const char* s_ssid = nullptr;
static const char* s_pass = nullptr;
static bool s_wasConnected = false;
static bool s_udpOpen = false;
static uint32_t s_lastWifiAttemptMs = 0;

static IPAddress s_dest;
static bool s_haveDest = false;
static uint32_t s_lastSeq = 0;
static uint32_t s_telemSeq = 0;
static uint32_t s_lastTelemMs = 0;
static bool s_telemSentOnce = false;

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
void commsBegin(const char* ssid, const char* password, const char* laptopIp) {
  s_ssid = ssid;
  s_pass = password;
  if (laptopIp && laptopIp[0]) {
    IPAddress ip;
    if (ip.fromString(laptopIp)) {
      s_dest = ip;
      s_haveDest = true;
      Serial.printf("[comms] telemetry -> LAPTOP_IP %s until a packet arrives\n", laptopIp);
    } else {
      Serial.printf("[comms] LAPTOP_IP '%s' is not a valid IP, ignored\n", laptopIp);
    }
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);         // power save adds 100+ ms of latency; also avoids GPIO36/39 glitches
  WiFi.setAutoReconnect(true);
  WiFi.begin(s_ssid, s_pass);
  s_lastWifiAttemptMs = millis();
  Serial.printf("[comms] connecting to '%s'...\n", s_ssid);
}

void commsUpdate() {
  const bool connected = (WiFi.status() == WL_CONNECTED);
  const uint32_t now = millis();

  if (connected && !s_wasConnected) {
    Serial.printf("[comms] WiFi connected, IP %s, RSSI %d dBm, UDP listen %d -> telemetry %d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI(), UDP_CMD_PORT, UDP_TELEM_PORT);
    if (s_udpOpen) s_udp.stop();
    s_udpOpen = s_udp.begin(UDP_CMD_PORT) == 1;
    if (!s_udpOpen) Serial.println("[comms] UDP begin failed");
  } else if (!connected && s_wasConnected) {
    Serial.println("[comms] WiFi lost, reconnecting...");
    s_lastWifiAttemptMs = now;
  }
  s_wasConnected = connected;

  if (!connected && now - s_lastWifiAttemptMs >= WIFI_RETRY_MS) {
    s_lastWifiAttemptMs = now;
    Serial.println("[comms] WiFi retry");
    WiFi.disconnect();
    WiFi.begin(s_ssid, s_pass);
  }
}

bool commsWifiConnected() { return s_wasConnected; }

bool commsTelemetryFlowing() {
  return s_wasConnected && s_telemSentOnce && (millis() - s_lastTelemMs < 500);
}

uint32_t commsLastSeq() { return s_lastSeq; }

// ---------------------------------------------------------------------------
// Parser. Accepts exactly: C,seq,l,r | S,seq | H,seq | R,seq,deg  (trailing \r\n ok)
// ---------------------------------------------------------------------------
static bool parseSeq(const char*& p, uint32_t& out) {
  if (!isdigit((unsigned char)*p)) return false;
  char* end;
  unsigned long v = strtoul(p, &end, 10);
  if (end == p) return false;
  out = (uint32_t)v;
  p = end;
  return true;
}

static bool parseInt(const char*& p, long& out) {
  const char* q = p;
  if (*q == '+' || *q == '-') q++;
  if (!isdigit((unsigned char)*q)) return false;
  char* end;
  long v = strtol(p, &end, 10);
  if (end == p) return false;
  out = v;
  p = end;
  return true;
}

static bool parseFloat(const char*& p, float& out) {
  const char* q = p;
  if (*q == '+' || *q == '-') q++;
  if (!isdigit((unsigned char)*q) && *q != '.') return false;  // rejects "nan", "inf", ""
  if (q[0] == '0' && (q[1] == 'x' || q[1] == 'X')) return false;  // no hex floats
  char* end;
  float v = strtof(p, &end);
  if (end == p || !isfinite(v)) return false;
  out = v;
  p = end;
  return true;
}

static bool expectComma(const char*& p) {
  if (*p != ',') return false;
  p++;
  return true;
}

static int clamp255(long v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return (int)v;
}

static bool parseLine(char* line, DriveCommand& c) {
  // strip trailing whitespace / CR / LF
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r' || line[n - 1] == ' ')) line[--n] = '\0';
  if (n < 3) return false;

  const char* p = line;
  c.type = *p++;
  c.left = 0;
  c.right = 0;
  c.deg = 0.0f;
  if (!expectComma(p) || !parseSeq(p, c.seq)) return false;

  switch (c.type) {
    case 'C': {
      long l, r;
      if (!expectComma(p) || !parseInt(p, l)) return false;
      if (!expectComma(p) || !parseInt(p, r)) return false;
      c.left = clamp255(l);
      c.right = clamp255(r);
      break;
    }
    case 'R':
      if (!expectComma(p) || !parseFloat(p, c.deg)) return false;
      break;
    case 'S':
    case 'H':
      break;
    default:
      return false;
  }
  return *p == '\0';  // no trailing junk
}

bool commsPoll(DriveCommand& out) {
  if (!s_udpOpen) return false;
  char buf[64];
  // Loop so a malformed datagram doesn't hide a valid one behind it.
  for (int guard = 0; guard < 8; guard++) {
    int size = s_udp.parsePacket();
    if (size <= 0) return false;
    int len = s_udp.read((uint8_t*)buf, sizeof(buf) - 1);
    if (len <= 0) continue;
    if (size > (int)sizeof(buf) - 1) continue;  // oversized: not one of ours
    buf[len] = '\0';
    if (!parseLine(buf, out)) continue;
    s_dest = s_udp.remoteIP();  // telemetry goes back to whoever is driving
    s_haveDest = true;
    s_lastSeq = out.seq;        // recorded, never used to reject (contract)
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Telemetry: {"seq":n,"state":"OK","d":[fl,fr,bl,br,l,r],"yaw":deg|null,"busy":bool}
// ---------------------------------------------------------------------------
static int appendDist(char* buf, size_t cap, int pos, float v) {
  if (pos < 0 || (size_t)pos >= cap) return pos;
  if (!(v >= 0.0f) || !isfinite(v)) return pos + snprintf(buf + pos, cap - pos, "-1");
  return pos + snprintf(buf + pos, cap - pos, "%.1f", v);
}

void commsSendTelemetry(TelemetryState st, const float d[6], bool yawValid, float yaw, bool busy) {
  if (!s_wasConnected || !s_udpOpen || !s_haveDest) return;

  static const char* const kNames[] = {"OK", "ESTOP", "WATCHDOG"};
  const char* name = (st >= TS_OK && st <= TS_WATCHDOG) ? kNames[st] : "OK";

  char buf[192];
  const size_t cap = sizeof(buf);
  int pos = snprintf(buf, cap, "{\"seq\":%lu,\"state\":\"%s\",\"d\":[",
                     (unsigned long)s_telemSeq, name);
  for (int i = 0; i < 6; i++) {
    if (i > 0 && pos >= 0 && (size_t)pos < cap) pos += snprintf(buf + pos, cap - pos, ",");
    pos = appendDist(buf, cap, pos, d[i]);
  }
  if (pos >= 0 && (size_t)pos < cap) {
    if (yawValid && isfinite(yaw)) pos += snprintf(buf + pos, cap - pos, "],\"yaw\":%.1f", yaw);
    else pos += snprintf(buf + pos, cap - pos, "],\"yaw\":null");
  }
  if (pos >= 0 && (size_t)pos < cap) {
    pos += snprintf(buf + pos, cap - pos, ",\"busy\":%s}", busy ? "true" : "false");
  }
  if (pos <= 0 || (size_t)pos >= cap) return;  // truncated: never send broken JSON

  if (s_udp.beginPacket(s_dest, UDP_TELEM_PORT) == 1) {
    s_udp.write((const uint8_t*)buf, (size_t)pos);
    if (s_udp.endPacket() == 1) {
      s_lastTelemMs = millis();
      s_telemSentOnce = true;
    }
  }
  s_telemSeq++;
}
