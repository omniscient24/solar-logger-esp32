/************************************************************
  Solar Logger — Feather ESP32 + Adafruit INA219 (0x40)

  Pins (Feather ESP32 silkscreen):
    • I2C: SDA = GPIO23, SCL = GPIO22
    • INA219 @ 0x40

  Features:
    • Uses Adafruit_INA219 library
    • Single sampler loop, web reads cached snapshot
    • Day-of-year energy integration (software only, no RTC)
    • Web dashboard + JSON endpoints
    • Browser OTA at /update
*************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>
#include <Adafruit_INA219.h>

// ===== USER CONFIG =====
const char* WIFI_SSID     = "weenieNet4";
const char* WIFI_PASSWORD = "un-moc-cod-ek-u";
const char* OTA_HOSTNAME  = "solar-logger";
const char* OTA_PASSWORD  = "solar123";   // change me

// ===== PINS =====
constexpr int I2C_SDA = 23;
constexpr int I2C_SCL = 22;

// ===== CADENCE =====
constexpr unsigned long SAMPLE_INTERVAL_MS = 800;

// ===== OBJECTS =====
WebServer   server(80);
Adafruit_INA219 ina219(0x40);   // explicit address

// ===== STATE =====
String  i2cSeen = "none";
bool    haveINA = false;

unsigned long lastSampleMs    = 0;
unsigned long lastEnergyMs    = 0;
float         energyToday_Wh  = 0.0f;

// ===== SNAPSHOT =====
struct Sample {
  float v_bus_V     = NAN;
  float v_shunt_mV  = NAN;
  float current_mA  = NAN;
  float power_mW    = NAN;
  float v_source_V  = NAN;
  uint32_t ms       = 0;
};

static Sample SNAP[2];
static volatile uint32_t SNAP_SEQ = 0; // even=stable, odd=writing

static inline Sample getSnapshotCopy() {
  Sample out;
  for (;;) {
    uint32_t s1 = SNAP_SEQ;
    if (s1 & 1) continue;
    out = SNAP[s1 & 1];
    uint32_t s2 = SNAP_SEQ;
    if (s1 == s2 && !(s2 & 1)) break;
    delay(1);
  }
  return out;
}

static inline void publishSnapshot(const Sample& s) {
  uint32_t n = SNAP_SEQ + 1;
  SNAP_SEQ = n;
  SNAP[n & 1] = s;
  SNAP_SEQ = n + 1;
}

// ===== UTILS =====
static inline String safeStr(float v, int digits) {
  return isnan(v) ? String("NaN") : String(v, digits);
}

void detectI2CDevices() {
  String s; int n=0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (n++) s += " ";
      char buf[6]; snprintf(buf, sizeof(buf), "0x%02X", a);
      s += buf;
    }
  }
  i2cSeen = s.length() ? s : "none";
}

// ===== SAMPLER =====
void sampleINA() {
  Sample s; s.ms = millis();

  if (!haveINA) { publishSnapshot(s); return; }

  ina219.setCalibration_32V_2A();

  float busV    = ina219.getBusVoltage_V();
  float shunt_mV= ina219.getShuntVoltage_mV();
  float curr_mA = ina219.getCurrent_mA();
  float pow_mW  = ina219.getPower_mW();

  s.v_bus_V    = busV;
  s.v_shunt_mV = shunt_mV;
  s.current_mA = curr_mA;
  s.power_mW   = pow_mW;
  if (!isnan(busV) && !isnan(shunt_mV)) s.v_source_V = busV + shunt_mV/1000.0f;

  publishSnapshot(s);
}

// ===== WEB =====
void handleRoot() {
  Sample s = getSnapshotCopy();
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta http-equiv='refresh' content='5'>"
                "<title>Solar Logger</title>"
                "<style>"
                "body{font-family:Arial;background:#111;color:#eee;margin:20px}"
                ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:16px}"
                ".card{background:#222;border:1px solid #333;border-radius:10px;padding:16px}"
                ".label{font-size:12px;color:#aaa;text-transform:uppercase}"
                ".value{font-size:26px;font-weight:700;margin-top:6px}"
                ".unit{font-size:14px;color:#aaa}"
                "a{color:#7fbfff}"
                "</style></head><body>";
  html += "<h1>🌞 Solar Panel Monitor</h1><div class='grid'>";

  html += "<div class='card'><div class='label'>Voltage (Source)</div>"
          "<div class='value'>" + safeStr(s.v_source_V, 3) + "<span class='unit'> V</span></div></div>";
  html += "<div class='card'><div class='label'>Voltage (Load)</div>"
          "<div class='value'>" + safeStr(s.v_bus_V, 3) + "<span class='unit'> V</span></div></div>";
  html += "<div class='card'><div class='label'>Shunt ΔV</div>"
          "<div class='value'>" + safeStr(s.v_shunt_mV, 3) + "<span class='unit'> mV</span></div></div>";
  html += "<div class='card'><div class='label'>Current</div>"
          "<div class='value'>" + safeStr(s.current_mA, 3) + "<span class='unit'> mA</span></div></div>";
  html += "<div class='card'><div class='label'>Power</div>"
          "<div class='value'>" + safeStr(s.power_mW/1000.0f, 3) + "<span class='unit'> W</span></div></div>";
  html += "<div class='card'><div class='label'>Energy (today)</div>"
          "<div class='value'>" + safeStr(energyToday_Wh, 3) + "<span class='unit'> Wh</span></div></div>";
  html += "<div class='card'><div class='label'>I2C Devices (boot)</div>"
          "<div class='value' style='font-size:18px'>" + i2cSeen + "</div></div>";

  html += "</div><p><a href='/data'>/data</a> · <a href='/health'>/health</a> · <a href='/update'>/update</a></p></body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  Sample s = getSnapshotCopy();
  String json = "{";
  json += "\"i2c_seen\":\"" + i2cSeen + "\",";
  json += "\"voltage_source\":" + safeStr(s.v_source_V, 3) + ",";
  json += "\"voltage_load\":"   + safeStr(s.v_bus_V, 3) + ",";
  json += "\"shunt_mV\":"       + safeStr(s.v_shunt_mV, 3) + ",";
  json += "\"current_mA\":"     + safeStr(s.current_mA, 3) + ",";
  json += "\"power_mW\":"       + safeStr(s.power_mW, 2) + ",";
  json += "\"energy_today_Wh\":" + safeStr(energyToday_Wh, 3);
  json += "}";
  server.send(200, "application/json", json);
}

void handleHealth() {
  String out = "{";
  out += "\"i2c_seen\":\"" + i2cSeen + "\",";
  out += "\"ina219\":" + String(haveINA ? 1 : 0) + ",";
  out += "\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0) + ",";
  out += "\"ip\":\"" + (WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\"";
  out += "}";
  server.send(200, "application/json", out);
}

// ===== OTA =====
void setupArduinoOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.onStart([](){ Serial.println("OTA start"); });
  ArduinoOTA.onEnd([](){ Serial.println("OTA end → reboot"); });
  ArduinoOTA.onError([](ota_error_t e){ Serial.printf("OTA error: %u\n", e); });
  ArduinoOTA.begin();
}

// ===== SETUP / LOOP =====
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Solar Logger Boot (ESP32 + INA219 only) ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  detectI2CDevices();
  Serial.print("I2C seen at boot: "); Serial.println(i2cSeen); // expect "0x40"

  haveINA = ina219.begin(&Wire);
  if (!haveINA) Serial.println("INA219: begin() FAILED");
  else {
    ina219.setCalibration_32V_2A();
    Serial.println("INA219: initialized");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) { Serial.print("WiFi IP: "); Serial.println(WiFi.localIP()); setupArduinoOTA(); }
  else { Serial.println("WiFi not connected"); }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/health", handleHealth);
  server.on("/update", HTTP_GET, [](){
    String page = R"HTML(
      <html><head><meta charset='utf-8'><title>OTA Update</title>
      <style>body{font-family:Arial;margin:20px}</style></head><body>
      <h1>OTA Update</h1>
      <form method='POST' action='/update' enctype='multipart/form-data'>
        <input type='file' name='firmware' accept='.bin'>
        <input type='submit' value='Upload'>
      </form>
      <p><a href='/'>Back</a></p>
      </body></html>)HTML";
    server.send(200, "text/html", page);
  });
  server.on("/update", HTTP_POST,
    [](){ server.send(200, "text/plain", Update.hasError() ? "Update FAILED" : "Update OK, rebooting..."); delay(200); if (!Update.hasError()) ESP.restart(); },
    [](){ HTTPUpload& up = server.upload(); if (up.status == UPLOAD_FILE_START) { if (!Update.begin()) Update.printError(Serial); }
         else if (up.status == UPLOAD_FILE_WRITE) { if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial); }
         else if (up.status == UPLOAD_FILE_END) { if (!Update.end(true)) Update.printError(Serial); } }
  );
  server.begin();
  Serial.println("Web server started");

  sampleINA();
  lastSampleMs = millis();
  lastEnergyMs = lastSampleMs;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
  server.handleClient();

  unsigned long nowMs = millis();

  if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = nowMs;
    sampleINA();
    Sample s = getSnapshotCopy();

    float hours = (nowMs - lastEnergyMs) / 3600000.0f;
    if (!isnan(s.power_mW)) energyToday_Wh += (s.power_mW / 1000.0f) * hours;
    lastEnergyMs = nowMs;

    Serial.print("Samp "); Serial.print(nowMs/1000);
    Serial.print("s | Vs="); Serial.print(s.v_source_V, 3);
    Serial.print(" Vl=");   Serial.print(s.v_bus_V, 3);
    Serial.print(" dVsh="); Serial.print(s.v_shunt_mV, 3);
    Serial.print(" I=");    Serial.print(s.current_mA, 3);
    Serial.print(" P=");    Serial.print(s.power_mW, 2);
    Serial.print(" E=");    Serial.println(energyToday_Wh, 3);
  }
}