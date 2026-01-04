/*
  LoGetter v1 — ESP32 Feather V2 + Ultimate GPS (UART) + LittleFS trip files + InfluxDB v2 upload
  - Logs 1Hz GPS samples to /trips/trip_boot_<ms>.csv, then renames to trip_YYYY-MM-DD_HH-MM-SS.csv once GPS time is valid
  - Skips the bogus 2000-00-00/00:00:00 rows (won't write until GPS date/time is valid)
  - Records TTFF (time to first fix) in ms (from trip start -> first GPS.fix with valid time)
  - Uploads all trip files to InfluxDB v2 using Line Protocol with the GPS timestamp (ns) per point
  - Deletes trip file only after successful upload (HTTP 204)
  - Button SW38 (GPIO38 / BUTTON): short press = upload now, long press = end trip + upload + sleep
  - NeoPixel status:
      Yellow blink     = waiting for GPS fix/time
      Purple pulse     = logging
      Green solid      = WiFi connected
      Green fast blink = uploading
      Red solid        = WiFi failure / error
      Blue slow pulse  = on battery / car off (only when VBUS sense enabled)
      Blue fast blink  = closing + preparing sleep
      Off              = deep sleep

  Wiring:
    - GPS UART: GPS TX -> ESP32 RX, GPS RX -> ESP32 TX, GND -> GND, 3V -> 3V (or VIN per module)
    - Feather ESP32 V2: use Serial1 on GPIO7 (RX) / GPIO8 (TX) if that's your wiring:
          Serial1.begin(9600, SERIAL_8N1, 7, 8);
    - Optional VBUS sense (recommended for car use):
          USB VBUS (5V) -> 100k -> GPIO34 -> 100k -> GND
          Then set USE_VBUS_SENSE = 1 and tune VBUS_ON_MV/VBUS_OFF_MV after printing ADC values
    - Battery voltage monitor:
          Use the built in voltage divider on IO35 
    - NeoPixel: built-in on PIN_NEOPIXEL (GPIO0), power enable on NEOPIXEL_I2C_POWER (GPIO2)

  Requires:
    - Adafruit GPS library
    - LittleFS (ESP32 core)
    - secrets.h:
        #pragma once
        #define WIFI_SSID      "SSID"
        #define WIFI_PASS      "DEADBEEF"
        #define INFLUX_TOKEN   "..."
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include "FS.h"
#include "LittleFS.h"

#include "secrets.local.h"      // Rename as "secrets.h" cause I'm too lazy and will accidentally post my secrets.


// -----------------------------------------------------------------------------
// Wi‑Fi Provisioning Portal (SoftAP + Captive DNS + Landing Page)
// Product policy:
//   - FIRST BOOT: always start portal and stop (no logging) until provisioned.
//   - AFTER: never auto-start portal again.
//   - HARD RESET: user holds BOOT (GPIO0) at power-up to clear creds + provisioned flag.
// Stores:
//   - Wi-Fi creds in Preferences namespace "wifi" (keys: ssid, pass)
//   - Provisioned flag in Preferences namespace "cfg" (key: provisioned)
// -----------------------------------------------------------------------------
static const byte  DNS_PORT = 53;
static DNSServer   g_dns;
static WebServer   g_web(80);
static bool        g_provisioning = false;

// BOOT pin on Feather ESP32 V2 is GPIO0 (also NeoPixel data); check it BEFORE driving NeoPixel.
#ifndef FORCE_PORTAL_PIN
#define FORCE_PORTAL_PIN 0
#endif
#ifndef FORCE_PORTAL_HOLD_MS
#define FORCE_PORTAL_HOLD_MS 300
#endif

static String apName() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[32];
  snprintf(buf, sizeof(buf), "LoGetter-%04X", (uint16_t)(mac & 0xFFFF));
  return String(buf);
}

static const char SETUP_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>LoGetter Setup</title>
  <style>
    :root { font-family: -apple-system, system-ui, Segoe UI, Roboto, Arial; }
    body { margin:0; background:#0b0f19; color:#e8eefc; }
    .wrap { max-width:420px; margin:0 auto; padding:24px; }
    .card { background:#121a2a; border:1px solid #1f2a44; border-radius:16px; padding:18px; box-shadow: 0 8px 24px rgba(0,0,0,.35);}
    h1 { font-size:22px; margin:0 0 6px; }
    p { opacity:.85; line-height:1.35; margin:0 0 14px; }
    label { display:block; font-size:13px; opacity:.85; margin:10px 0 6px; }
    input, select { width:100%; padding:12px; border-radius:12px; border:1px solid #2a385c; background:#0b1222; color:#e8eefc; }
    button { width:100%; margin-top:14px; padding:12px; border-radius:12px; border:0; background:#4c7dff; color:#fff; font-weight:600; }
    .hint { font-size:12px; opacity:.7; margin-top:10px; }
    .row { display:flex; gap:10px; }
    .row > * { flex:1; }
    .small { font-size:12px; opacity:.75; margin-top:6px; }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Set up LoGetter</h1>
      <p>Enter your Wi‑Fi details so LoGetter can upload trips when you're parked.</p>

      <form method="POST" action="/save" autocomplete="on">
        <label for="ssid">Wi‑Fi network (SSID)</label>
        <input id="ssid" name="ssid" placeholder="e.g. MyHomeWiFi" required />

        <div class="row">
          <div>
            <label for="pass">Wi‑Fi password</label>
            <input id="pass" name="pass" type="password" placeholder="Password" />
          </div>
          <div>
            <label for="show">Show</label>
            <select id="show" onchange="togglePass()">
              <option value="0" selected>Hidden</option>
              <option value="1">Visible</option>
            </select>
          </div>
        </div>

        <button type="submit">Save &amp; Connect</button>
      </form>

      <div class="hint">Tip: If your Wi‑Fi has no password, leave it blank.</div>
      <div class="small">If this page doesn't pop up automatically, open <b>http://192.168.4.1</b></div>
    </div>
  </div>

<script>
function togglePass() {
  var sel = document.getElementById('show');
  var p = document.getElementById('pass');
  p.type = (sel.value === '1') ? 'text' : 'password';
}
</script>
</body>
</html>
)HTML";

// ---- NVS helpers ----
static bool wifiHaveSaved(String &ssid, String &pass) {
  Preferences p;
  p.begin("wifi", true);
  ssid = p.getString("ssid", "");
  pass = p.getString("pass", "");
  p.end();
  return ssid.length() > 0;
}

static void wifiSave(const String &ssid, const String &pass) {
  Preferences p;
  p.begin("wifi", false);
  p.putString("ssid", ssid);
  p.putString("pass", pass);
  p.end();
}

static void wifiClearSaved() {
  Preferences p;
  p.begin("wifi", false);
  p.clear();
  p.end();
}

static bool isProvisioned() {
  Preferences p;
  p.begin("cfg", true);
  bool v = p.getBool("provisioned", false);
  p.end();
  return v;
}

static void setProvisioned(bool v) {
  Preferences p;
  p.begin("cfg", false);
  p.putBool("provisioned", v);
  p.end();
}

static void factoryResetProvisioning() {
  Serial.println("Factory reset: clearing saved Wi-Fi creds + provisioning flag.");
  wifiClearSaved();
  setProvisioned(false);
}

// ---- Wi-Fi connect (STA) ----
static bool wifiTryConnect(const String& ssid, const String& pass, uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  delay(100);

  WiFi.begin(ssid.c_str(), pass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
    yield();
  }
  return WiFi.status() == WL_CONNECTED;
}

// ---- Force-portal input ----
static bool shouldForcePortalAtBoot() {
  pinMode(FORCE_PORTAL_PIN, INPUT_PULLUP);
  delay(FORCE_PORTAL_HOLD_MS);
  return digitalRead(FORCE_PORTAL_PIN) == LOW;
}

// ---- Web handlers ----
static void portalHandleRoot() {
  g_web.send_P(200, "text/html", SETUP_HTML);
}

static void portalHandleSave() {
  String ssid = g_web.arg("ssid");
  String pass = g_web.arg("pass");
  ssid.trim();

  if (ssid.length() == 0) {
    g_web.send(400, "text/plain", "Missing SSID");
    return;
  }

  // Save then test connect
  wifiSave(ssid, pass);

  bool ok = wifiTryConnect(ssid, pass, 15000);
  if (ok) {
    setProvisioned(true);

    String msg = "<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>"
                 "<style>body{font-family:-apple-system,system-ui;margin:24px;}</style>"
                 "<h2>Connected ✅</h2>"
                 "<p>LoGetter connected to <b>" + ssid + "</b>.<br/>You can close this page.</p>";
    g_web.send(200, "text/html", msg);
    delay(1200);
    ESP.restart();
  } else {
    g_web.send(200, "text/html",
      "<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:-apple-system,system-ui;margin:24px;}</style>"
      "<h2>Couldn't connect</h2><p>Check the password and try again.</p><p><a href='/'>Back</a></p>");
    // stay in AP mode
    WiFi.mode(WIFI_AP);
  }
}

static void portalHandleNotFound() {
  g_web.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  g_web.sendHeader("Pragma", "no-cache");
  g_web.sendHeader("Expires", "-1");
  g_web.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  g_web.send(302, "text/plain", "");
}


static void startProvisioningPortal() {
  g_provisioning = true;

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  // Deterministic portal IP
  IPAddress apIP(192,168,4,1);
  IPAddress netMsk(255,255,255,0);

  String name = apName();
  WiFi.softAPConfig(apIP, apIP, netMsk);
  delay(150); // give AP stack a moment to come up
  bool ok = WiFi.softAP(name.c_str());

  Serial.println();
  Serial.println("=== Provisioning Mode ===");
  Serial.printf("softAP() ok: %d\n", (int)ok);
  Serial.print("AP SSID: "); Serial.println(name);
  Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());
  Serial.println("If captive portal doesn't open, browse to http://192.168.4.1");

  g_dns.start(DNS_PORT, "*", WiFi.softAPIP());

  g_web.on("/", HTTP_GET, portalHandleRoot);
  g_web.on("/save", HTTP_POST, portalHandleSave);
  

    // Captive portal detection endpoints (iOS/macOS/Android/Windows)
  g_web.on("/generate_204", HTTP_GET, portalHandleRoot);          // Android sometimes hits this
  g_web.on("/gen_204",      HTTP_GET, portalHandleRoot);          // Android variant
  g_web.on("/hotspot-detect.html", HTTP_GET, portalHandleRoot);   // iOS/macOS
  g_web.on("/library/test/success.html", HTTP_GET, portalHandleRoot); // iOS/macOS
  g_web.on("/ncsi.txt", HTTP_GET, portalHandleRoot);             // Windows
  g_web.on("/connecttest.txt", HTTP_GET, portalHandleRoot);      // Windows
  g_web.on("/redirect", HTTP_GET, portalHandleRoot);             // common

  // Anything else -> root
  g_web.onNotFound(portalHandleNotFound);
  g_web.begin();
}
// -----------------------------------------------------------------------------
// End provisioning portal
// -----------------------------------------------------------------------------


// =========================
// InfluxDB v2 config
// =========================
static const char* INFLUX_BASE_URL  = "https://influx.eggseax.com"; // your server
static const char* INFLUX_ORG       = "lab";                   // your org name
static const char* INFLUX_BUCKET    = "car_trips";             // your bucket name
static const char* INFLUX_PRECISION = "ns";                    // keep ns
static const char* DEVICE_TAG       = "car1";                  // optional tag to distinguish devices

static String influxWriteUrl() {
  return String(INFLUX_BASE_URL) +
         "/api/v2/write?org=" + INFLUX_ORG +
         "&bucket=" + INFLUX_BUCKET +
         "&precision=" + INFLUX_PRECISION;
}


// =========================
// GPS UART pins (your wiring)
// =========================
static const int GPS_BAUD = 9600;
static const int GPS_RX_PIN = 7;  // ESP32 receives from GPS TX
static const int GPS_TX_PIN = 8;  // ESP32 transmits to GPS RX

HardwareSerial GPSSerial(1);
Adafruit_GPS GPS(&GPSSerial);

// =========================
// NeoPixel (Feather ESP32 V2)
// =========================
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 0
#endif
#ifndef NEOPIXEL_I2C_POWER
#define NEOPIXEL_I2C_POWER 2
#endif

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// =========================
// Button (SW38)
// =========================
#ifndef BUTTON
#define BUTTON 38
#endif

static const uint32_t BTN_DEBOUNCE_MS   = 30;
static const uint32_t BTN_LONGPRESS_MS  = 1500;

// =========================
// Storage + trip behavior
// =========================
static const char* TRIP_DIR = "/trips";
static const uint32_t LOG_PERIOD_MS = 1000;  // 1Hz

// Trip end conditions
// 1) Recommended: detect car USB removal (VBUS sense)
//
// Wiring (simple + robust):
//   USB VBUS (5V) -> 10k -> GPIO34 (A2) -> 10k -> GND
// This makes ~2.5V at the pin when VBUS is present.
static const int USE_VBUS_SENSE = 1;          // set to 0 to disable VBUS sensing
static const int VBUS_SENSE_PIN = 34;         // Feather ESP32 V2: A2 = GPIO34

// Calibrated thresholds (mV at the ADC pin, *after* the divider):
// Your readings: unplugged ~142 mV, plugged ~2560 mV.
// Use a little hysteresis to avoid chatter.
static const int VBUS_ON_MV  = 1500;          // consider VBUS "present" above this
static const int VBUS_OFF_MV = 800;           // consider VBUS "gone" below this



// =========================
// Battery monitor (Feather ESP32 V2 LiPoly monitor)
// =========================
// Feather ESP32 V2 has a 200K/200K divider into GPIO35 (VOLTAGE_MONITOR).
// Read the ADC pin and multiply by 2 to estimate battery voltage.
static const int   USE_BATT_MONITOR = 1;
static const int   BATT_MON_PIN     = 35;
static const float BATT_DIVIDER     = 2.0f;

// Basic sanity limits for a 1-cell LiPo
static const float BATT_VALID_MIN_V = 2.80f;  // below this is likely bogus
static const float BATT_VALID_MAX_V = 4.35f;  // above this is likely bogus

// If readings jump around more than this across a short sample window, treat as floating/no-batt
static const float BATT_JITTER_MAX_V = 0.25f;

// 2) Dev fallback: end trip if "stopped" for N seconds
static const float SPEED_STOP_MPH = 1.0f;
static const uint32_t STOP_HOLD_SEC = 120;   // 2 minutes

// Wi-Fi + sleep
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static const uint32_t SLEEP_SECONDS = 120;   // wake every 2 min and try again (if car off)


// =========================
// State
// =========================
String currentTripPath;
String currentTripId;   // filename stem, used as Influx tag

// Derive a stable trip_id from a trip file path like '/trips/trip_YYYY-MM-DD_HH-MM-SS.csv'
// Result: 'trip_YYYY-MM-DD_HH-MM-SS' (no directory, no extension).
static String tripIdFromPath(const String& path) {
  int slash = path.lastIndexOf('/');
  int start = (slash >= 0) ? (slash + 1) : 0;
  int dot = path.lastIndexOf('.');
  int end = (dot > start) ? dot : path.length();
  return path.substring(start, end);
}

bool tripRenamed = false;

uint32_t lastLogMs = 0;
uint32_t tripStartMs = 0;
bool ttffRecorded = false;
uint32_t ttffMs = 0;

uint32_t stoppedSinceMs = 0;

// Button state
bool btnLastRaw = true;
uint32_t btnChangeMs = 0;
uint32_t btnPressStartMs = 0;
bool btnPressedStable = false;

// =========================
// Helpers: NeoPixel patterns
// =========================
static uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) { return pixel.Color(r, g, b); }

static void pixelOff() {
  pixel.clear();
  pixel.show();
}

static void pixelSolid(uint32_t c, uint8_t brightness = 40) {
  pixel.setBrightness(brightness);
  pixel.setPixelColor(0, c);
  pixel.show();
}

static void pixelPulse(uint32_t c, uint16_t periodMs) {
  uint32_t now = millis();
  float phase = (now % periodMs) / (float)periodMs;              // 0..1
  float tri = phase < 0.5f ? (phase * 2.f) : (2.f - phase * 2.f);// 0..1..0
  uint8_t b = (uint8_t)(10 + tri * 80);                          // 10..90
  pixel.setBrightness(b);
  pixel.setPixelColor(0, c);
  pixel.show();
}

static void pixelBlink(uint32_t c, uint16_t onMs, uint16_t offMs, uint8_t brightness = 40) {
  static uint32_t t = 0;
  static bool on = false;
  uint32_t now = millis();
  if (!on && now - t >= offMs) { on = true; t = now; }
  if (on && now - t >= onMs)   { on = false; t = now; }
  pixel.setBrightness(brightness);
  pixel.setPixelColor(0, on ? c : 0);
  pixel.show();
}

// =========================
// Helpers: math/time
// =========================
static float knots_to_mph(float k) { return k * 1.150779f; }

static bool isLeap(int y) { return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0); }

static uint32_t daysBeforeYear(int y) {
  uint32_t days = 0;
  for (int yr = 1970; yr < y; yr++) days += isLeap(yr) ? 366 : 365;
  return days;
}

static uint16_t daysBeforeMonth(int y, int m) {
  static const uint16_t cumNorm[] = {0,31,59,90,120,151,181,212,243,273,304,334};
  static const uint16_t cumLeap[] = {0,31,60,91,121,152,182,213,244,274,305,335};
  return (isLeap(y) ? cumLeap : cumNorm)[m - 1];
}

static bool ymdhms_to_epoch_ns(int y, int mo, int d, int hh, int mm, int ss, int ms, uint64_t &outNs) {
  if (y < 1970 || mo < 1 || mo > 12 || d < 1 || d > 31) return false;
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 60 || ms < 0 || ms > 999) return false;

  uint32_t days = daysBeforeYear(y) + daysBeforeMonth(y, mo) + (uint32_t)(d - 1);
  uint32_t secs = days * 86400UL + (uint32_t)hh * 3600UL + (uint32_t)mm * 60UL + (uint32_t)ss;
  outNs = (uint64_t)secs * 1000000000ULL + (uint64_t)ms * 1000000ULL;
  return true;
}

// =========================
// Power sense
// =========================

static bool carPowerPresent() {
  if (!USE_VBUS_SENSE) return true;

  static bool state = true;  // assume on at boot
  uint32_t mv = analogReadMilliVolts(VBUS_SENSE_PIN);

  if (state)  state = (mv > VBUS_OFF_MV);
  else        state = (mv > VBUS_ON_MV);

  return state;
}

// =========================
// Filesystem + trip filenames
// =========================
static void ensureTripsDir() {
  if (!LittleFS.exists(TRIP_DIR)) LittleFS.mkdir(TRIP_DIR);
}

static bool gpsTimeValid() {
  return (GPS.year > 0 && GPS.month > 0 && GPS.day > 0);
}

static String makeTripBootFilename() {
  char buf[96];
  snprintf(buf, sizeof(buf), "%s/trip_boot_%lu.csv", TRIP_DIR, (unsigned long)millis());
  return String(buf);
}

static String makeTripFilenameFromGPS() {
  char buf[96];
  snprintf(buf, sizeof(buf),
           "%s/trip_%04d-%02d-%02d_%02d-%02d-%02d.csv",
           TRIP_DIR,
           2000 + GPS.year, GPS.month, GPS.day,
           GPS.hour, GPS.minute, GPS.seconds);
  return String(buf);
}

static void writeHeaderIfNew(File &f) {
  if (f.size() == 0) {
    f.println("utc_date,utc_time,fix,fixquality,sats,lat,lon,alt_m,speed_mph,course_deg,hdop,batt_v,ttff_ms");
  }
}

static void maybeRenameTripFile() {
  if (tripRenamed) return;
  if (!gpsTimeValid()) return;

  String newPath = makeTripFilenameFromGPS();
  if (newPath == currentTripPath) { tripRenamed = true; return; }

  if (LittleFS.exists(newPath)) {
    newPath.replace(".csv", "_dup.csv");
  }

  Serial.println();
  Serial.println("=== Trip Rename ===");
  Serial.printf("Old: %s\n", currentTripPath.c_str());
  Serial.printf("New: %s\n", newPath.c_str());

  if (LittleFS.rename(currentTripPath, newPath)) {
    currentTripPath = newPath;
    tripRenamed = true;

    // keep trip_id in sync with final filename
    currentTripId = currentTripPath;
    int slash = currentTripId.lastIndexOf('/');
    if (slash >= 0) currentTripId = currentTripId.substring(slash + 1);
    if (currentTripId.endsWith(".csv")) currentTripId.remove(currentTripId.length() - 4);

    Serial.println("Rename: OK");
  // Keep trip_id in sync with the final on-disk filename.
  currentTripId = tripIdFromPath(newPath);

  } else {
    Serial.println("Rename: FAILED (will retry)");
  }
}

// =========================
// Battery level 
// =========================

static float readBatteryVoltage() {
  if (!USE_BATT_MONITOR) return -1.0f;

  // Take a few samples (ESP32 ADC can be noisy, and floating can look "high")
  const int N = 6;
  float vmin =  999.0f;
  float vmax = -999.0f;
  float sum  = 0.0f;

  for (int i = 0; i < N; i++) {
    float v;

    #if defined(ARDUINO_ARCH_ESP32)
      uint32_t mv = analogReadMilliVolts(BATT_MON_PIN); // mV at ADC pin
      v = (mv / 1000.0f) * BATT_DIVIDER;                // undo divider
    #else
      int raw = analogRead(BATT_MON_PIN);               // 0..4095 @ 12-bit
      float v_pin = (raw / 4095.0f) * 3.3f;
      v = v_pin * BATT_DIVIDER;
    #endif

    sum += v;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;

    
    delay(2);
    yield();
  }

  float avg = sum / (float)N;

  // Reject obvious garbage
  if (avg < BATT_VALID_MIN_V || avg > BATT_VALID_MAX_V) return -1.0f;

  // Reject floating/no-battery behavior (usually unstable or pegged high)
  if ((vmax - vmin) > BATT_JITTER_MAX_V) return -1.0f;

  // With 1M bleeder, unplugged should be near 0V
  if (avg < 1.0f) return -1.0f;

  return avg;
}




// =========================
// CSV logging
// =========================
static String buildCsvLine() {
  char dateBuf[16];
  char timeBuf[20];
  snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d", 2000 + GPS.year, GPS.month, GPS.day);
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d.%03d", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds);

  float speed_mph = knots_to_mph(GPS.speed);

  String line;
  line.reserve(180);
  line += dateBuf; line += ",";
  line += timeBuf; line += ",";
  line += String((int)GPS.fix); line += ",";
  line += String((int)GPS.fixquality); line += ",";
  line += String((int)GPS.satellites); line += ",";
  line += String(GPS.latitudeDegrees, 6); line += ",";
  line += String(GPS.longitudeDegrees, 6); line += ",";
  line += String(GPS.altitude, 2); line += ",";
  line += String(speed_mph, 2); line += ",";
  line += String(GPS.angle, 2); line += ",";
  line += String(GPS.HDOP, 2); line += ",";

  float batt_v = readBatteryVoltage();
  if (batt_v < 0) line += "-1";
  else            line += String(batt_v, 2);
  line += ",";


  line += String(ttffRecorded ? (int)ttffMs : -1);
  return line;
}

// =========================
// Influx: CSV row -> Line Protocol
// =========================
static bool parseDate(const String& s, int &y, int &mo, int &d) {
  if (s.length() < 10) return false;
  y  = s.substring(0, 4).toInt();
  mo = s.substring(5, 7).toInt();
  d  = s.substring(8, 10).toInt();
  return true;
}

static bool parseTime(const String& s, int &hh, int &mm, int &ss, int &ms) {
  if (s.length() < 8) return false;
  hh = s.substring(0, 2).toInt();
  mm = s.substring(3, 5).toInt();
  ss = s.substring(6, 8).toInt();
  ms = 0;
  int dot = s.indexOf('.');
  if (dot >= 0 && dot + 1 < (int)s.length()) {
    ms = s.substring(dot + 1).toInt();
  }
  return true;
}

static bool csvToLineProtocol(const String& csvLine, String &outLP) {
  // utc_date,utc_time,fix,fixquality,sats,lat,lon,alt_m,speed_mph,course_deg,hdop,batt_v,ttff_ms
  String p[13];
  int idx = 0;
  int start = 0;

  for (int i = 0; i < (int)csvLine.length() && idx < 13; i++) {
    char c = csvLine[i];
    if (c == ',' || c == '\n' || c == '\r') {
      p[idx++] = csvLine.substring(start, i);
      start = i + 1;
    }
  }

  // Capture last field if line didn't end with newline
  if (idx < 13 && start < (int)csvLine.length()) {
    String tail = csvLine.substring(start);
    tail.trim();
    if (tail.length()) p[idx++] = tail;
}

  if (idx < 13) return false;

  int y, mo, d, hh, mm, ss, ms;
  if (!parseDate(p[0], y, mo, d)) return false;
  if (!parseTime(p[1], hh, mm, ss, ms)) return false;

  uint64_t ts_ns;
  if (!ymdhms_to_epoch_ns(y, mo, d, hh, mm, ss, ms, ts_ns)) return false;

  outLP.reserve(256);
  outLP = "gps,device=";
  outLP += DEVICE_TAG;
  outLP += ",trip_id=";
  outLP += currentTripId;

  // Fields (numeric)
  outLP += " lat="; outLP += p[5];
  outLP += ",lon="; outLP += p[6];
  outLP += ",alt_m="; outLP += p[7];
  outLP += ",speed_mph="; outLP += p[8];
  outLP += ",course_deg="; outLP += p[9];
  outLP += ",sats="; outLP += p[4];
  outLP += ",fix="; outLP += p[2];
  outLP += ",fixquality="; outLP += p[3];
  outLP += ",hdop="; outLP += p[10];

  // batt_v: allow -1 (no battery) — you can choose to omit it instead if you want
  if (p[11] != "-1" && p[11].length()) {
    outLP += ",batt_v="; outLP += p[11];
  }

  outLP += ",ttff_ms="; outLP += p[12];
  outLP += " ";
  outLP += String((unsigned long long)ts_ns);
  return true;
}


// =========================
// Wi-Fi + HTTP write (verbose)
// =========================
static bool wifiConnectVerbose() {
  Serial.println();
  Serial.println("=== WiFi Connect ===");

  // Product behavior:
  //   - Only connect using credentials saved via provisioning portal (Preferences/NVS).
  //   - Never auto-start the portal from here (first-boot only).
  if (g_provisioning) {
    Serial.println("Provisioning active; Wi-Fi connect deferred.");
    return false;
  }

  String ssid, pass;
  if (!wifiHaveSaved(ssid, pass)) {
    Serial.println("No saved Wi-Fi credentials.");
    return false;
  }

  Serial.printf("SSID: %s", ssid.c_str());

  bool ok = wifiTryConnect(ssid, pass, WIFI_CONNECT_TIMEOUT_MS);

  if (ok) {
    pixelSolid(rgb(0, 255, 0), 25); // green solid: connected
    Serial.printf("WiFi connected! IP: %s RSSI: %d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }

  pixelSolid(rgb(255, 0, 0), 60); // solid red: failed
  Serial.println("WiFi connect FAILED. (No portal fallback)");
  return false;
}

static bool influxPostVerbose(const String& payload) {
  String url = influxWriteUrl();

  Serial.println();
  Serial.println("=== Influx Write ===");
  Serial.printf("URL: %s\n", url.c_str());
  Serial.printf("Bytes: %d\n", (int)payload.length());

  WiFiClientSecure client;
  client.setInsecure();          // ok for now with Cloudflare
  client.setTimeout(12000);     // wait

  HTTPClient http;
  http.setTimeout(15000);   // 15s overall
  http.setReuse(false);    // IMPORTANT: don't keep sockets open between requests

  // IMPORTANT: this must be the overload that accepts (client, url)
  if (!http.begin(client, url)) {
    Serial.println("HTTP begin() failed");
    return false;
  }

  http.addHeader("Authorization", String("Token ") + INFLUX_TOKEN);
  http.addHeader("Content-Type", "text/plain; charset=utf-8");
  http.addHeader("Connection", "close");

  int code = http.POST((uint8_t*)payload.c_str(), payload.length());
  String body = http.getString();

  Serial.printf("HTTP code: %d\n", code);
  if (body.length()) {
    Serial.println("--- response body ---");
    Serial.println(body);
    Serial.println("---------------------");
  }

  http.end();
  delay(20); // give lwIP a breath

  return (code == 204);
}


static bool uploadTripFile(const String& path) {
  // Tag all points from this file with a stable trip_id derived from the filename.
  currentTripId = tripIdFromPath(path);
  Serial.printf("Trip ID (upload): %s\n", currentTripId.c_str());

  Serial.println();
  Serial.println("=== Upload Trip File ===");
  Serial.printf("File: %s\n", path.c_str());
  Serial.printf("Exists: %d\n", LittleFS.exists(path));

  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.println("ERROR: cannot open file for upload.");
    return false;
  }

  const int MAX_LINES_PER_POST = 200;
  String payload;
  payload.reserve(3500);

  uint32_t rows = 0;
  uint32_t postedBatches = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    if (line.startsWith("utc_date")) continue;

    String lp;
    // capture last field if line didn't end with newline/comma
    if (csvToLineProtocol(line + "\n", lp)) {
      payload += lp;
      payload += "\n";
      rows++;
    }

    // Post every N lines OR if payload gets large
    if ((rows % MAX_LINES_PER_POST) == 0 || payload.length() > 3000) {
      Serial.printf("Posting batch %lu (rows so far: %lu)\n",
                    (unsigned long)(postedBatches + 1),
                    (unsigned long)rows);

      pixelBlink(rgb(0, 255, 0), 60, 60, 60); // fast green blink: uploading

      if (!influxPostVerbose(payload)) {
        Serial.println("ERROR: Influx write failed (batch).");
        f.close();
        return false;
      }
      postedBatches++;
      payload = "";
      delay(30);
      yield();
    }

    yield();
  }

  // Final partial batch
  if (payload.length()) {
    Serial.printf("Posting final batch %lu (total rows: %lu)\n",
                  (unsigned long)(postedBatches + 1),
                  (unsigned long)rows);

    pixelBlink(rgb(0, 255, 0), 60, 60, 60);

    if (!influxPostVerbose(payload)) {
      Serial.println("ERROR: Influx write failed (final).");
      f.close();
      return false;
    }
  }

  f.close();

  Serial.printf("Uploaded %lu rows from %s\n", (unsigned long)rows, path.c_str());
  return true;
}


static void uploadAllTrips() {
  Serial.println();
  Serial.println("=== Upload All Trips ===");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected; skipping upload sweep.");
    return;
  }

  File dir = LittleFS.open(TRIP_DIR);
  if (!dir || !dir.isDirectory()) {
    Serial.println("No trips directory (or not a directory).");
    return;
  }

  for (File file = dir.openNextFile(); file; file = dir.openNextFile()) {
    // If WiFi drops mid-sweep, stop cleanly
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi dropped during upload sweep.");
      break;
    }

    String name = String(file.name());   // may be "trip_x.csv" OR "/trips/trip_x.csv"
    bool isDir = file.isDirectory();
    file.close();

    if (isDir) continue;
    if (!name.endsWith(".csv")) continue;

    // Normalize to an absolute /trips/... path
    String path = name;

    // ensure leading "/"
    if (!path.startsWith("/")) path = String("/") + path;

    // ensure it’s inside /trips
    if (!path.startsWith(String(TRIP_DIR) + "/")) {
      String base = path;
      int slash = base.lastIndexOf('/');
      if (slash >= 0) base = base.substring(slash + 1);
      path = String(TRIP_DIR) + "/" + base;
    }

    Serial.printf("Found trip: %s\n", path.c_str());

    if (uploadTripFile(path)) {
      Serial.printf("Delete trip (success): %s\n", path.c_str());
      LittleFS.remove(path);
      delay(50); // small breather helps stability
    } else {
      Serial.printf("Keep trip (upload failed): %s\n", path.c_str());
      break; // try later
    }

    yield();
  }

  Serial.println("Upload sweep done.");
}


static void goToSleep(uint32_t seconds) {
  Serial.println();
  Serial.println("=== Sleep ===");
  Serial.printf("Sleeping for %lu seconds\n", (unsigned long)seconds);

  uint32_t t0 = millis();
  while (millis() - t0 < 900) {
    pixelPulse(rgb(0, 0, 255), 1200); // blue breathe: preparing sleep
    delay(10);
    yield();
  }

  pixelOff();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

// =========================
// Trip control + logging
// =========================
static void startNewTrip() {
  tripRenamed = false;
  ttffRecorded = false;
  ttffMs = 0;
  tripStartMs = millis();
  stoppedSinceMs = 0;

  currentTripPath = makeTripBootFilename();

  // trip_id = filename without directory + extension
  currentTripId = currentTripPath;
  int slash = currentTripId.lastIndexOf('/');
  if (slash >= 0) currentTripId = currentTripId.substring(slash + 1);
  if (currentTripId.endsWith(".csv")) currentTripId.remove(currentTripId.length() - 4);

  Serial.println();
  Serial.println("=== Trip Start ===");
  Serial.printf("Trip file: %s\n", currentTripPath.c_str());
}

static bool shouldEndTrip() {
  if (USE_VBUS_SENSE) {
    return !carPowerPresent();
  }
  if (stoppedSinceMs == 0) return false;
  uint32_t stoppedFor = (millis() - stoppedSinceMs) / 1000;
  return stoppedFor >= STOP_HOLD_SEC;
}

static void updateLedStatus() {
  bool carOn = carPowerPresent();

  if (USE_VBUS_SENSE && !carOn) {
    pixelPulse(rgb(0, 0, 255), 1800);                 // blue slow pulse: on battery
    return;
  }

  if (!gpsTimeValid() || !GPS.fix) {
    pixelBlink(rgb(255, 255, 0), 120, 120, 40);       // yellow: no time/fix yet
    return;
  }

  pixelPulse(rgb(160, 0, 255), 1500);                 // purple: logging
}

static void logOneSample() {
  updateLedStatus();

  // Skip writing garbage until GPS date/time is valid
  if (!gpsTimeValid()) {
    Serial.println("LOG: (skipping) waiting for GPS date/time...");
    return;
  }

  // Rename trip_boot_* -> trip_YYYY... once time becomes valid
  maybeRenameTripFile();

  // Record TTFF once we have time + a fix
  if (GPS.fix && !ttffRecorded) {
    ttffRecorded = true;
    ttffMs = millis() - tripStartMs;
    Serial.printf("TTFF recorded: %lu ms (fixquality=%d)\n",
                  (unsigned long)ttffMs, (int)GPS.fixquality);
  }

  // Write sample to file (you can uncomment to only log when fix is valid)
  // if (!GPS.fix) return;

  File f = LittleFS.open(currentTripPath, "a");
  if (!f) {
    Serial.println("ERROR: cannot open trip file for append.");
    pixelSolid(rgb(255, 0, 0), 80);
    return;
  }
  writeHeaderIfNew(f);

  String csv = buildCsvLine();
  f.println(csv);
  f.close();

  Serial.print("LOG: ");
  Serial.println(csv);

  // Stop detection in dev fallback mode
  float speed_mph = knots_to_mph(GPS.speed);
  if (speed_mph < SPEED_STOP_MPH) {
    if (stoppedSinceMs == 0) stoppedSinceMs = millis();
  } else {
    stoppedSinceMs = 0;
  }
}

// =========================
// Button handler
// =========================
static void handleButton() {
  bool raw = digitalRead(BUTTON);   // HIGH = not pressed, LOW = pressed
  uint32_t now = millis();

  if (raw != btnLastRaw) {
    btnLastRaw = raw;
    btnChangeMs = now;
  }

  if (now - btnChangeMs < BTN_DEBOUNCE_MS) return;

  bool pressed = (raw == LOW);

  if (pressed && !btnPressedStable) {
    btnPressedStable = true;
    btnPressStartMs = now;
    Serial.println("BTN: down");
  }

  if (!pressed && btnPressedStable) {
    btnPressedStable = false;
    uint32_t held = now - btnPressStartMs;

    if (held >= BTN_LONGPRESS_MS) {
      Serial.printf("BTN: long press (%lu ms) -> end trip + upload + sleep\n", (unsigned long)held);

      // Closing indication
      uint32_t t0 = millis();
      while (millis() - t0 < 800) {
        pixelBlink(rgb(0, 0, 255), 50, 50, 80); // blue quick blink
        delay(10);
        yield();
      }

      if (wifiConnectVerbose()) {
        uploadAllTrips();
      } else {
        Serial.println("BTN long-press: WiFi connect failed");
        pixelSolid(rgb(255, 0, 0), 60);
        delay(800);
      }

      goToSleep(SLEEP_SECONDS);
    } else {
      Serial.printf("BTN: short press (%lu ms) -> upload now\n", (unsigned long)held);

      if (wifiConnectVerbose()) {
        uploadAllTrips();
      } else {
        Serial.println("BTN short-press: WiFi connect failed");
        pixelSolid(rgb(255, 0, 0), 60);
        delay(800);
      }
    }
  }
}

// =========================
// Setup / Loop
// =========================
void setup() {
  delay(300); // helps USB-serial auto-reset on some setups

  Serial.begin(115200);
  delay(250);


  // ---------------------------------------------------------------------------
  // Provisioning policy:
  //   - FIRST BOOT: always start AP portal.
  //   - AFTER provisioning: never auto-start portal again.
  //   - HARD RESET: hold BOOT (GPIO0) at power-up to clear creds + provisioned flag.
  // ---------------------------------------------------------------------------

  // If user is holding BOOT at boot, treat it as a hard reset request.
  if (shouldForcePortalAtBoot()) {
    factoryResetProvisioning();
  }

  // FIRST BOOT behavior: start portal immediately and stop normal startup.
  if (!isProvisioned()) {
    Serial.println("First boot (not provisioned) -> starting Wi-Fi provisioning portal");
    startProvisioningPortal();
    return; // IMPORTANT: don't continue into logger mode
  }

  Serial.println();
  Serial.println("======================================");
  Serial.println(" LoGetter v1: GPS Trip Logger -> Influx");
  Serial.println("======================================");
  Serial.printf("Influx URL : %s\n", INFLUX_BASE_URL);
  Serial.printf("Org        : %s\n", INFLUX_ORG);
  Serial.printf("Bucket     : %s\n", INFLUX_BUCKET);
  Serial.printf("Device tag : %s\n", DEVICE_TAG);
  Serial.printf("VBUS sense enabled: %d\n", USE_VBUS_SENSE);

  // NeoPixel power enable + init
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
  delay(10);

  pixel.begin();
  pixelOff();
  pixelSolid(rgb(0, 0, 255), 20); // brief blue on boot
  delay(200);
  pixelOff();

  // Button (SW38 has pull-up onboard)
  pinMode(BUTTON, INPUT);

  // ADC init (needed for VBUS sense and/or battery monitor)
  if (USE_VBUS_SENSE || USE_BATT_MONITOR) {
    analogReadResolution(12);
  }

  if (USE_VBUS_SENSE) {
    pinMode(VBUS_SENSE_PIN, INPUT);

    #if defined(ARDUINO_ARCH_ESP32)
      analogSetPinAttenuation(VBUS_SENSE_PIN, ADC_11db);
    #endif

    Serial.printf("VBUS sense pin: %d  ON:%d mV  OFF:%d mV", VBUS_SENSE_PIN, VBUS_ON_MV, VBUS_OFF_MV);
  }


  if (USE_BATT_MONITOR) {
    pinMode(BATT_MON_PIN, INPUT);
    #if defined(ARDUINO_ARCH_ESP32)
      // Widens measurable range on the ESP32 ADC so higher voltages don't clip
      analogSetPinAttenuation(BATT_MON_PIN, ADC_11db);
    #endif
    Serial.printf("Battery monitor pin: %d\n", BATT_MON_PIN);
  }


  // FS
  Serial.println("Mounting LittleFS...");
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount failed.");
    pixelSolid(rgb(255, 0, 0), 80);
    while (true) delay(1000);
  }
  ensureTripsDir();
  Serial.println("LittleFS mounted.");

  // GPS
  Serial.println("Starting GPS...");
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(800);
  Serial.println("GPS started.");

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("Wake cause: %d\n", (int)cause);

  startNewTrip();
}

void loop() {
  if (g_provisioning) {
    g_dns.processNextRequest();
    g_web.handleClient();
    delay(1);
    return;
  }


  // Button can force upload/sleep
  handleButton();

  // GPS read + parse
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  uint32_t now = millis();
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    logOneSample();

    if (shouldEndTrip()) {
      Serial.println();
      Serial.println("=== Trip End Condition Met ===");

      // Blue fast blink while closing
      uint32_t t0 = millis();
      while (millis() - t0 < 1000) {
        pixelBlink(rgb(0, 0, 255), 50, 50, 80);
        delay(10);
        yield();
      }

      if (wifiConnectVerbose()) {
        uploadAllTrips();
      } else {
        Serial.println("Upload skipped (no Wi-Fi).");
      }

      goToSleep(SLEEP_SECONDS);
    }
  }

  delay(2);
  yield();
}