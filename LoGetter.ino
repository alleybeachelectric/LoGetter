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
          Then set USE_VBUS_SENSE = 1 and tune VBUS_ON_THRESH after printing ADC values
    - NeoPixel: built-in on PIN_NEOPIXEL (GPIO0), power enable on NEOPIXEL_I2C_POWER (GPIO2)

  Requires:
    - Adafruit GPS library
    - LittleFS (ESP32 core)
    - secrets.h:
        #pragma once
        #define WIFI_SSID      "BitcoinFarm"
        #define WIFI_PASS      "..."
        #define INFLUX_TOKEN   "..."
*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include "FS.h"
#include "LittleFS.h"

#include "secrets.local.h"

// =========================
// InfluxDB v2 config
// =========================
static const char* INFLUX_HOST      = "http://10.0.1.13:8086"; // your server
static const char* INFLUX_ORG       = "lab";                   // your org name
static const char* INFLUX_BUCKET    = "car_trips";             // your bucket name
static const char* INFLUX_PRECISION = "ns";                    // keep ns
static const char* DEVICE_TAG       = "car1";                  // optional tag to distinguish devices

static String influxWriteUrl() {
  return String(INFLUX_HOST) + "/api/v2/write?org=" + INFLUX_ORG +
         "&bucket=" + INFLUX_BUCKET + "&precision=" + INFLUX_PRECISION;
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
static const int USE_VBUS_SENSE = 0;         // <-- set to 1 after you add the divider
static const int VBUS_SENSE_PIN = 34;
static const int VBUS_ON_THRESH = 600;       // tune with serial print

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
  if (!USE_VBUS_SENSE) return true; // cannot detect car USB removal in dev mode
  int v = analogRead(VBUS_SENSE_PIN);
  return v > VBUS_ON_THRESH;
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
    f.println("utc_date,utc_time,fix,fixquality,sats,lat,lon,alt_m,speed_mph,course_deg,hdop,ttff_ms");
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
    Serial.println("Rename: OK");
  } else {
    Serial.println("Rename: FAILED (will retry)");
  }
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
  // utc_date,utc_time,fix,fixquality,sats,lat,lon,alt_m,speed_mph,course_deg,hdop,ttff_ms
  String p[12];
  int idx = 0, start = 0;

  for (int i = 0; i < (int)csvLine.length() && idx < 12; i++) {
    char c = csvLine[i];
    if (c == ',' || c == '\n' || c == '\r') {
      p[idx++] = csvLine.substring(start, i);
      start = i + 1;
    }
  }
  if (idx < 11) return false;

  int y, mo, d, hh, mm, ss, ms;
  if (!parseDate(p[0], y, mo, d)) return false;
  if (!parseTime(p[1], hh, mm, ss, ms)) return false;

  uint64_t ts_ns;
  if (!ymdhms_to_epoch_ns(y, mo, d, hh, mm, ss, ms, ts_ns)) return false;

  outLP = "gps";
  outLP += ",device="; outLP += DEVICE_TAG;

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
  outLP += ",ttff_ms="; outLP += p[11];

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
  Serial.printf("SSID: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    pixelBlink(rgb(255, 0, 0), 80, 80, 30); // red blink while connecting
    Serial.print(".");
    delay(250);
    yield();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    pixelSolid(rgb(0, 255, 0), 25); // green solid: connected
    Serial.printf("WiFi connected! IP: %s RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }

  pixelSolid(rgb(255, 0, 0), 60); // solid red: failed
  Serial.println("WiFi connect FAILED.");
  return false;
}

static bool influxPostVerbose(const String& payload) {
  String url = influxWriteUrl();

  Serial.println();
  Serial.println("=== Influx Write ===");
  Serial.printf("URL: %s\n", url.c_str());
  Serial.printf("Bytes: %d\n", (int)payload.length());

  WiFiClient client;
  client.setTimeout(5000); // 5s socket read timeout

  HTTPClient http;
  http.setTimeout(7000);   // 7s overall
  http.setReuse(false);    // IMPORTANT: don't keep sockets open between requests

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
  Serial.begin(115200);
  delay(250);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" LoGetter v1: GPS Trip Logger -> Influx");
  Serial.println("======================================");
  Serial.printf("Influx host: %s\n", INFLUX_HOST);
  Serial.printf("Org: %s  Bucket: %s\n", INFLUX_ORG, INFLUX_BUCKET);
  Serial.printf("Write URL: %s\n", influxWriteUrl().c_str());
  Serial.printf("Device tag: %s\n", DEVICE_TAG);
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

  if (USE_VBUS_SENSE) {
    analogReadResolution(12);
    pinMode(VBUS_SENSE_PIN, INPUT);
    Serial.printf("VBUS sense pin: %d thresh: %d\n", VBUS_SENSE_PIN, VBUS_ON_THRESH);
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
