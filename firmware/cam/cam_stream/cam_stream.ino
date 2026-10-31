/*
 * cam_stream.ino -- Pinacle ESP32-CAM MJPEG streamer (GC2145 sensor)
 *
 * Joins a phone hotspot (station mode) and serves:
 *   http://<ip>/         tiny HTML page showing the stream
 *   http://<ip>/stream   MJPEG (multipart/x-mixed-replace)  <- laptop FrameSource uses this
 *   http://<ip>/capture  single JPEG
 *
 * Arduino IDE settings (esp32 board package 2.0.17 -- do NOT use 3.x):
 *   Board:          "AI Thinker ESP32-CAM"
 *   PSRAM:          Enabled
 *   Partition:      "Huge APP (3MB No OTA)"
 *   Upload speed:   115200
 *   Serial Monitor: 115200 baud, line ending "Newline" (or "Both NL & CR")
 *
 * Setup: copy secrets.example.h to secrets.h and fill in WIFI_SSID / WIFI_PASSWORD.
 *
 * NOTE: the GC2145 has NO hardware JPEG encoder (unlike the OV2640). We capture
 * RGB565 at QVGA (320x240) and JPEG-encode each frame in software with
 * frame2jpg(), so expect roughly 8-12 fps.
 *
 * NOTE: esp_http_server runs all handlers on one task. While a /stream client
 * is connected, other requests (/, /capture, a second viewer) wait until that
 * stream ends. Use one stream client at a time.
 *
 * Startup gate: after reset the sketch waits for Enter on Serial before touching
 * the camera or WiFi, so a crash can't turn into a flood of reboots.
 */

#include <WiFi.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include "secrets.h"   // defines WIFI_SSID and WIFI_PASSWORD (gitignored)

// ---------------------------------------------------------------------------
// AI-Thinker ESP32-CAM pin map (GC2145 module)
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26   // SCCB SDA
#define SIOC_GPIO_NUM   27   // SCCB SCL
#define Y2_GPIO_NUM      5   // D0
#define Y3_GPIO_NUM     18   // D1
#define Y4_GPIO_NUM     19   // D2
#define Y5_GPIO_NUM     21   // D3
#define Y6_GPIO_NUM     36   // D4
#define Y7_GPIO_NUM     39   // D5
#define Y8_GPIO_NUM     34   // D6
#define Y9_GPIO_NUM     35   // D7
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
static const uint8_t  JPEG_QUALITY      = 15;     // frame2jpg quality (1-100, higher = better/larger)
static const int      CAM_INIT_ATTEMPTS = 5;
static const int      WARMUP_FRAMES     = 15;     // discarded so auto-exposure settles
static const uint32_t WIFI_TIMEOUT_MS   = 15000;  // per connection attempt
static const int      WIFI_ATTEMPTS     = 5;
static const uint32_t WIFI_CHECK_MS     = 10000;  // loop() link-check interval (long enough for a join to finish)
static const uint32_t FPS_REPORT_MS     = 5000;

// ---------------------------------------------------------------------------
// MJPEG multipart framing
// ---------------------------------------------------------------------------
#define PART_BOUNDARY "pinacleframeboundary"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t httpServer = NULL;

// ---------------------------------------------------------------------------
// Startup gate: wait for Enter on Serial before doing anything else.
// ---------------------------------------------------------------------------
static void waitForEnter() {
  Serial.println("ready - press Enter");
  uint32_t lastPrompt = millis();
  for (;;) {
    while (Serial.available()) {
      int c = Serial.read();
      if (c == '\n' || c == '\r') {
        // Drain whatever came with it (e.g. the '\n' after a '\r').
        delay(50);
        while (Serial.available()) Serial.read();
        return;
      }
    }
    if (millis() - lastPrompt >= 3000) {
      Serial.println("ready - press Enter");
      lastPrompt = millis();
    }
    delay(10);
  }
}

// Stop here without rebooting; repeat the reason now and then.
static void haltForever(const char *why) {
  for (;;) {
    Serial.printf("HALTED: %s (fix and press the reset button)\n", why);
    delay(10000);
  }
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
static void powerCycleCamera() {
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);  // sensor powered down
  delay(200);
  digitalWrite(PWDN_GPIO_NUM, LOW);   // sensor powered up
  delay(200);
}

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;   // GC2145: no hardware JPEG
  config.frame_size   = FRAMESIZE_QVGA;     // 320x240
  config.jpeg_quality = 12;                 // ignored for RGB565; set for completeness
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  for (int attempt = 1; attempt <= CAM_INIT_ATTEMPTS; attempt++) {
    powerCycleCamera();
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
      Serial.printf("Camera init OK (attempt %d/%d)\n", attempt, CAM_INIT_ATTEMPTS);
      return true;
    }
    Serial.printf("Camera init failed (attempt %d/%d): error 0x%x\n",
                  attempt, CAM_INIT_ATTEMPTS, (unsigned)err);
    esp_camera_deinit();   // release anything half-initialised before retrying
    delay(500);
  }
  return false;
}

static void warmUpCamera() {
  Serial.printf("Discarding %d frames so auto-exposure settles...\n", WARMUP_FRAMES);
  for (int i = 0; i < WARMUP_FRAMES; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static void printUrls() {
  String ip = WiFi.localIP().toString();
  Serial.printf("WiFi connected, IP: %s\n", ip.c_str());
  Serial.printf("Stream URL: http://%s/stream\n", ip.c_str());
  Serial.printf("Viewer:     http://%s/\n", ip.c_str());
}

static bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // modem sleep wrecks streaming throughput/latency
  for (int attempt = 1; attempt <= WIFI_ATTEMPTS; attempt++) {
    Serial.printf("Connecting to \"%s\" (attempt %d/%d)", WIFI_SSID, attempt, WIFI_ATTEMPTS);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      printUrls();
      return true;
    }
    Serial.printf("WiFi attempt %d failed (status %d)\n", attempt, (int)WiFi.status());
  }
  return false;
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------
static esp_err_t indexHandler(httpd_req_t *req) {
  String ip = WiFi.localIP().toString();
  char html[512];
  snprintf(html, sizeof(html),
           "<!doctype html><html><head><meta charset=\"utf-8\">"
           "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
           "<title>Pinacle cam</title></head>"
           "<body style=\"font-family:sans-serif;background:#111;color:#eee\">"
           "<h3>Pinacle cam - %s</h3>"
           "<p>Stream: http://%s/stream</p>"
           "<img src=\"/stream\" style=\"max-width:100%%\">"
           "</body></html>",
           ip.c_str(), ip.c_str());
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("capture: frame grab failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  uint8_t *jpg = NULL;
  size_t len = 0;
  bool ok = frame2jpg(fb, JPEG_QUALITY, &jpg, &len);
  esp_camera_fb_return(fb);
  if (!ok || !jpg) {
    Serial.println("capture: frame2jpg failed");
    if (jpg) free(jpg);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)jpg, len);
  free(jpg);
  return res;
}

static esp_err_t streamHandler(httpd_req_t *req) {
  char partHeader[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  Serial.println("stream: client connected");
  uint32_t frames = 0;
  uint32_t windowStart = millis();

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("stream: frame grab failed");
      res = ESP_FAIL;
      break;
    }

    uint8_t *jpg = NULL;
    size_t len = 0;
    if (!frame2jpg(fb, JPEG_QUALITY, &jpg, &len) || !jpg) {
      // Conversion failed: release everything and try the next frame.
      Serial.println("stream: frame2jpg failed, skipping frame");
      if (jpg) free(jpg);
      esp_camera_fb_return(fb);
      delay(5);
      continue;
    }

    // Boundary, part header (Content-Type + Content-Length), then JPEG body.
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      int hlen = snprintf(partHeader, sizeof(partHeader), STREAM_PART, (unsigned)len);
      res = httpd_resp_send_chunk(req, partHeader, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)jpg, len);
    }

    free(jpg);
    esp_camera_fb_return(fb);

    if (res != ESP_OK) break;   // client went away

    frames++;
    uint32_t now = millis();
    if (now - windowStart >= FPS_REPORT_MS) {
      float fps = frames * 1000.0f / (float)(now - windowStart);
      Serial.printf("stream: %.1f fps (last JPEG %u bytes)\n", fps, (unsigned)len);
      frames = 0;
      windowStart = now;
    }
  }

  Serial.println("stream: client disconnected");
  return res;
}

static void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  // Single server, default ctrl_port (32768). If a second server is ever added,
  // give it a different server_port AND ctrl_port (e.g. 81 / 32769).
  config.lru_purge_enable = true;  // drop the oldest socket if we run out

  httpd_uri_t indexUri   = { .uri = "/",        .method = HTTP_GET, .handler = indexHandler,   .user_ctx = NULL };
  httpd_uri_t streamUri  = { .uri = "/stream",  .method = HTTP_GET, .handler = streamHandler,  .user_ctx = NULL };
  httpd_uri_t captureUri = { .uri = "/capture", .method = HTTP_GET, .handler = captureHandler, .user_ctx = NULL };

  if (httpd_start(&httpServer, &config) != ESP_OK) {
    haltForever("HTTP server failed to start");
  }
  httpd_register_uri_handler(httpServer, &indexUri);
  httpd_register_uri_handler(httpServer, &streamUri);
  httpd_register_uri_handler(httpServer, &captureUri);
  Serial.println("HTTP server started on port 80");
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  waitForEnter();               // nothing else runs until Enter is pressed

  Serial.println();
  Serial.println("Pinacle cam_stream (GC2145, RGB565 -> software JPEG)");

  if (!psramFound()) {
    Serial.println("ERROR: PSRAM not found. In Arduino IDE set Tools > PSRAM > Enabled "
                   "(board \"AI Thinker ESP32-CAM\"). The frame buffers need PSRAM.");
    haltForever("no PSRAM");
  }
  Serial.printf("PSRAM OK: %u bytes free\n", (unsigned)ESP.getFreePsram());

  if (!initCamera()) {
    Serial.printf("ERROR: camera init failed after %d attempts. Check the ribbon cable "
                  "and that the sensor is a GC2145.\n", CAM_INIT_ATTEMPTS);
    haltForever("camera init failed");
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) Serial.printf("Sensor PID: 0x%x\n", (unsigned)s->id.PID);
  warmUpCamera();

  if (!connectWiFi()) {
    Serial.println("WiFi not connected yet; will keep retrying in loop().");
  }

  startServer();
}

void loop() {
  static uint32_t lastCheck = 0;
  static bool wasConnected = true;   // so a drop at any time gets reported

  if (millis() - lastCheck >= WIFI_CHECK_MS) {
    lastCheck = millis();
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (!connected) {
      if (wasConnected) Serial.println("WiFi link lost, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    } else if (!wasConnected) {
      printUrls();   // IP may have changed after the reconnect
    }
    wasConnected = connected;
  }
  delay(100);
}
