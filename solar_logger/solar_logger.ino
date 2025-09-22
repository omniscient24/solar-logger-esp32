/************************************************************
  Solar Logger — Feather ESP32 + INA219 (+ optional Adalogger)

  Board pinning (Feather ESP32 silkscreen):
    - I2C: SDA = GPIO23, SCL = GPIO22
    - INA219 @ 0x40
    - (Optional) Adalogger SD: CS = GPIO33  (set Adalogger SDCS pads to 33)
    - RTC: PCF8523 @ 0x68 on I2C

  Features:
    - INA219 V/I/P + Energy Today (Wh) with day-of-year reset
    - CSV logging to /solar_log.csv every 5s when SD present
    - Web dashboard at /
    - JSON endpoints: /data, /health
    - Diagnostics: /ina219 (raw regs; safe reads + retries)
    - OTA: Arduino IDE Network Port (auto-reboot) + browser /update
*************************************************************/

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_INA219.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>

// ========= USER CONFIG =========
const char* WIFI_SSID     = "weenieNet4";
const char* WIFI_PASSWORD = "un-moc-cod-ek-u";
const char* OTA_HOSTNAME  = "solar-logger";
const char* OTA_PASSWORD  = "solar123";   // change me

// External shunt configuration (75mV @ 50A shunt)
const float SHUNT_RESISTANCE_OHMS = 0.0015;  // 75mV/50A = 0.0015 ohms
const float SHUNT_MAX_CURRENT_A = 50.0;      // Maximum current rating
const float SHUNT_CALIBRATION_FACTOR = 6.2;  // Correction factor (measured 0.54A vs reading 0.087A)

// ========= PINS =========
constexpr int I2C_SDA = 23;   // Feather ESP32 silkscreen
constexpr int I2C_SCL = 22;   // Feather ESP32 silkscreen
constexpr int SD_CS   = 33;   // Adalogger SDCS pads must be bridged to 33

// ========= LOGGING =========
constexpr unsigned long LOG_INTERVAL_MS = 5000;

// ========= OBJECTS =========
RTC_PCF8523 rtc;
Adafruit_INA219 ina219(0x40);
WebServer server(80);

// ========= STATE =========
unsigned long lastLogMs       = 0;
unsigned long lastEnergyMs    = 0;
float         energyToday_Wh  = 0.0;
bool          haveRTC         = false;
bool          haveINA         = false;
bool          haveSD          = false;
String        i2cSeen         = "none";
int           lastDayOfYear   = -1;

// ========= HELPERS =========
static inline String safeStr(float v, int digits) {
  if (isnan(v)) return String("NaN");
  return String(v, digits);
}

// leap-safe day-of-year
int dayOfYear(const DateTime& dt) {
  static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int d = dt.day();
  for (int m = 1; m < dt.month(); m++) {
    d += dim[m-1];
    if (m == 2 && ((dt.year()%4==0 && dt.year()%100!=0) || (dt.year()%400==0))) d++;
  }
  return d;
}

void detectI2CDevices() {
  i2cSeen = "";
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (n++) i2cSeen += " ";
      char buf[6]; snprintf(buf, sizeof(buf), "0x%02X", a);
      i2cSeen += String(buf);
    }
  }
  if (i2cSeen.isEmpty()) i2cSeen = "none";
}

// Re-apply calibration (belt & suspenders)
void inaCalibrate32V2A() { ina219.setCalibration_32V_2A(); }

// ----- Safe raw register reads for /ina219 -----
bool inaReadRegSafe(uint8_t addr, uint8_t reg, uint16_t &outVal, uint8_t retries = 4) {
  while (retries--) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) { delay(2); continue; } // STOP = true
    delayMicroseconds(200);
    uint8_t got = Wire.requestFrom(addr, (uint8_t)2, (bool)true);
    if (got == 2) {
      int hi = Wire.read();
      int lo = Wire.read();
      if (hi >= 0 && lo >= 0) { outVal = ((uint16_t)hi << 8) | lo; return true; }
    } else { while (Wire.available()) Wire.read(); }
    delay(2);
  }
  outVal = 0xFFFF;
  return false;
}

// ---------- INIT ----------
void initRTC() {
  if (!rtc.begin()) {
    Serial.println("RTC: begin() failed");
    haveRTC = false;
    return;
  }
  haveRTC = true;
  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC: lost power; setting to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();
  lastDayOfYear = dayOfYear(now);
  Serial.print("RTC time: "); Serial.println(now.timestamp());
}

void initINA() {
  if (!ina219.begin()) {
    Serial.println("INA219: begin() failed (is it at 0x40 on SDA=23/SCL=22?)");
    haveINA = false;
    return;
  }
  haveINA = true;
  inaCalibrate32V2A();
  Serial.println("INA219 initialized + calibrated (32V/2A)");
}

void initSD() {
  // ESP32 Feather SPI pins: SCK=5, MISO=19, MOSI=18, CS=33
  Serial.println("SD: Initializing with CS=33...");

  // Try with explicit SPI configuration
  SPI.begin(5, 19, 18, SD_CS);  // SCK, MISO, MOSI, CS

  if (!SD.begin(SD_CS, SPI, 4000000)) {  // Try 4MHz first
    Serial.println("SD: init at 4MHz FAILED, trying 1MHz...");
    delay(100);

    // Try slower speed
    if (!SD.begin(SD_CS, SPI, 1000000)) {
      Serial.println("SD: init at 1MHz FAILED");
      Serial.println("SD: Check: 1) Card inserted? 2) CS pad bridged to 33? 3) Card formatted FAT32?");
      haveSD = false;
      return;
    }
  }

  haveSD = true;
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.print("SD: initialized! Card size: ");
  Serial.print(cardSize);
  Serial.println(" MB");

  if (!SD.exists("/solar_log.csv")) {
    File f = SD.open("/solar_log.csv", FILE_WRITE);
    if (f) {
      f.println("timestamp,voltage_source_V,voltage_load_V,shunt_mV,current_mA,power_mW,energy_today_Wh");
      f.close();
      Serial.println("SD: created /solar_log.csv");
    }
  } else {
    Serial.println("SD: /solar_log.csv exists");
  }
}

void maybeMidnightReset(const DateTime& now) {
  int doy = dayOfYear(now);
  if (lastDayOfYear == -1) { lastDayOfYear = doy; return; }
  if (doy != lastDayOfYear) {
    lastDayOfYear = doy;
    energyToday_Wh = 0.0f;
    Serial.println("Energy Today reset at midnight");
  }
}

// ---------- RUNTIME ----------
void logSample() {
  // timestamp
  String ts;
  DateTime now;
  if (haveRTC) { now = rtc.now(); ts = now.timestamp(); maybeMidnightReset(now); }
  else         { ts = String(millis()/1000) + "s"; }

  // INA219
  float busV = NAN, shunt_mV = NAN, current_mA = NAN, power_mW = NAN;
  if (haveINA) {
    inaCalibrate32V2A();
    shunt_mV   = ina219.getShuntVoltage_mV();
    busV       = ina219.getBusVoltage_V();
    // Calculate current manually from shunt voltage for external shunt
    // I = V / R = (shunt_mV / 1000) / SHUNT_RESISTANCE_OHMS
    // Use absolute value in case shunt is wired backwards
    // Apply calibration factor to correct for INA219 reading error
    float current_A = (abs(shunt_mV / 1000.0) / SHUNT_RESISTANCE_OHMS) * SHUNT_CALIBRATION_FACTOR;
    current_mA = current_A * 1000.0;  // Convert to mA for consistency
    power_mW = busV * current_A * 1000.0;  // P = V * I, convert to mW
  }
  float sourceV = (!isnan(busV) && !isnan(shunt_mV)) ? (busV + shunt_mV/1000.0f) : NAN;

  // integrate Wh
  unsigned long tNow = millis();
  float hours = (tNow - lastEnergyMs) / 3600000.0f;
  if (!isnan(power_mW)) energyToday_Wh += (power_mW / 1000.0f) * hours;
  lastEnergyMs = tNow;

  // console
  Serial.print("Log "); Serial.print(ts);
  Serial.print(" | Vs="); Serial.print(sourceV, 3); Serial.print(" V");
  Serial.print(" Vl=");   Serial.print(busV, 3);    Serial.print(" V");
  Serial.print(" dVsh="); Serial.print(shunt_mV, 3);Serial.print(" mV");
  Serial.print(" I=");    Serial.print(current_mA, 3); Serial.print(" mA");
  Serial.print(" P=");    Serial.print(power_mW, 2);  Serial.print(" mW");
  Serial.print(" E_today="); Serial.print(energyToday_Wh, 3); Serial.println(" Wh");

  // SD CSV
  if (haveSD) {
    File f = SD.open("/solar_log.csv", FILE_APPEND);
    if (f) {
      f.print(ts);                f.print(",");
      f.print(sourceV, 3);        f.print(",");
      f.print(busV, 3);           f.print(",");
      f.print(shunt_mV, 3);       f.print(",");
      f.print(current_mA, 3);     f.print(",");
      f.print(power_mW, 2);       f.print(",");
      f.println(energyToday_Wh, 3);
      f.close();
    } else {
      Serial.println("SD: append failed");
    }
  }
}

// ---------- WEB ----------
void handleRoot() {
  DateTime now;
  String nowStr = haveRTC ? (now = rtc.now(), String(now.timestamp())) : String(millis() / 1000) + "s";

  inaCalibrate32V2A();
  float shunt_mV  = haveINA ? ina219.getShuntVoltage_mV() : NAN;
  float busV      = haveINA ? ina219.getBusVoltage_V()    : NAN;
  float srcV      = (haveINA && !isnan(busV) && !isnan(shunt_mV)) ? (busV + shunt_mV/1000.0f) : NAN;
  // Calculate current manually from shunt voltage for external shunt
  float current_mA = NAN;
  float power_mW = NAN;
  if (haveINA && !isnan(shunt_mV)) {
    float current_A = (abs(shunt_mV / 1000.0) / SHUNT_RESISTANCE_OHMS) * SHUNT_CALIBRATION_FACTOR;
    current_mA = current_A * 1000.0;
    power_mW = busV * current_A * 1000.0;
  }

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=0'>"
                "<title>Solar Logger</title>"
                "<style>"
                "body{font-family:-apple-system,system-ui,Arial;background:#111;color:#eee;margin:10px;padding:0}"
                ".grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px;margin-top:10px}"
                "@media(min-width:768px){.grid{grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}}"
                ".card{background:#222;border:1px solid #333;border-radius:14px;padding:20px;min-height:100px}"
                ".label{font-size:14px;color:#aaa;text-transform:uppercase;font-weight:600;letter-spacing:0.5px}"
                ".value{font-size:42px;font-weight:800;margin-top:10px;line-height:1}"
                ".unit{font-size:20px;color:#aaa;font-weight:600}"
                "a{color:#7fbfff;font-size:16px}"
                ".aim{margin-top:18px;background:#1a1a2e;border:1px solid #333;border-radius:14px;padding:20px}"
                ".aim h2{margin:0 0 14px 0;font-size:18px;color:#ffa500;font-weight:700}"
                ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}"
                ".btn{border:1px solid #445;background:#223;color:#dbe7ff;border-radius:10px;padding:14px 18px;cursor:pointer;text-decoration:none;font-size:15px;font-weight:600;-webkit-tap-highlight-color:transparent}"
                ".btn:active{transform:translateY(1px);background:#334}"
                ".badge{font-size:14px;background:#2a2a4a;color:#bcd0ff;border:1px solid #3a3964;padding:6px 10px;border-radius:999px;font-weight:500}"
                ".small{font-size:13px;color:#9fb4ff;opacity:.9;margin-top:10px;line-height:1.4}"
                "hr{border:none;border-top:1px solid #445;margin:20px 0}"
                "h1{font-size:24px;margin:10px 0}"
                "@media(max-width:375px){.value{font-size:36px}.unit{font-size:18px}}"
                ".header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px}"
                "</style></head><body>";
  html += "<div class='header'>";
  html += "<h1>Solar Panel Monitor</h1>";
  html += "</div>";
  html += "<div class='grid'>";

  html += "<div class='card'><div class='label'>Voltage (Source)</div>"
          "<div class='value'><span id='v'>" + safeStr(srcV, 3) + "</span><span class='unit'> V</span></div></div>";
  html += "<div class='card'><div class='label'>Voltage (Load)</div>"
          "<div class='value'>" + safeStr(busV, 3) + "<span class='unit'> V</span></div></div>";
  html += "<div class='card'><div class='label'>Shunt ΔV</div>"
          "<div class='value'>" + safeStr(shunt_mV, 3) + "<span class='unit'> mV</span></div></div>";
  html += "<div class='card'><div class='label'>Current</div>"
          "<div class='value'><span id='i'>" + safeStr(current_mA/1000.0f, 3) + "</span><span class='unit'> A</span></div></div>";
  html += "<div class='card'><div class='label'>Power</div>"
          "<div class='value'><span id='p'>" + safeStr(power_mW/1000.0f, 3) + "</span><span class='unit'> W</span></div></div>";
  html += "<div class='card'><div class='label'>Energy Today</div>"
          "<div class='value'><span id='e'>" + safeStr(energyToday_Wh, 3) + "</span><span class='unit'> Wh</span></div></div>";

  html += "</div>";

  html += "<hr>";
  html += "<div class='aim'>";
  html += "<h2>Aim Mode (Audio Feedback)</h2>";
  html += "<div class='row'>";
  html += "<button class='btn' id='aimBtn'>Enable Aim Mode</button>";
  html += "<button class='btn' id='muteBtn'>Mute</button>";
  html += "<span class='badge' id='aimState'>Audio: off</span>";
  html += "<span class='badge' id='peak'>Peak: 0.0 W</span>";
  html += "</div>";
  html += "<div class='small'>Tap 'Enable Aim Mode' first (browser gesture required). Pitch rises with power output - adjust panel for highest pitch!</div>";
  html += "</div>";

  html += "<div style='display:flex;justify-content:space-between;align-items:center;margin-top:20px'>";
  html += "<div><a href='/data' style='color:#7fbfff;font-size:14px'>/data</a> · <a href='/health' style='color:#7fbfff;font-size:14px'>/health</a> · <a href='/ina219' style='color:#7fbfff;font-size:14px'>/ina219</a> · <a href='/update' style='color:#7fbfff;font-size:14px'>/update</a></div>";
  html += "<div style='display:flex;gap:10px'>";
  html += "<a href='/charts' class='btn'>View Charts</a>";
  if (haveSD) {
    html += "<a href='/download' class='btn'>Download CSV</a>";
  }
  html += "</div>";
  html += "</div>";

  html += "<script>"
          "let audioCtx=null, osc=null, gainNode=null, aiming=false, muted=false;"
          "let maxW=0.0;"
          ""
          "function ensureAudio(){"
          "  if(audioCtx) return true;"
          "  try{"
          "    audioCtx=new (window.AudioContext||window.webkitAudioContext)();"
          "    gainNode=audioCtx.createGain(); gainNode.gain.value=0.0;"
          "    osc=audioCtx.createOscillator(); osc.type='sine'; osc.frequency.value=220;"
          "    osc.connect(gainNode).connect(audioCtx.destination); osc.start();"
          "    return true;"
          "  }catch(e){ return false; }"
          "}"
          ""
          "function setToneFromWatts(w){"
          "  if(!aiming || !audioCtx) return;"
          "  if(w>maxW) maxW=w;"
          "  const n=Math.max(0,Math.min(1, w/(maxW||1)));"
          "  const f=220 + Math.pow(n,0.6)*1000;"
          "  const g=muted?0:(0.12 + 0.04*n);"
          "  osc.frequency.setTargetAtTime(f, audioCtx.currentTime, 0.02);"
          "  gainNode.gain.setTargetAtTime(g, audioCtx.currentTime, 0.03);"
          "  document.getElementById('peak').textContent = 'Peak: ' + maxW.toFixed(1) + ' W';"
          "}"
          ""
          "document.getElementById('aimBtn').onclick=async ()=>{"
          "  if(!audioCtx){"
          "    if(!ensureAudio()){ alert('Web Audio not available'); return; }"
          "    if(audioCtx.state==='suspended'){ try{ await audioCtx.resume(); }catch(e){} }"
          "  }"
          "  aiming=!aiming;"
          "  if(!aiming && gainNode){ gainNode.gain.value=0.0; }"
          "  document.getElementById('aimBtn').textContent = aiming?'Disable Aim Mode':'Enable Aim Mode';"
          "  document.getElementById('aimState').textContent = 'Audio: ' + (aiming?'on':'off');"
          "};"
          ""
          "document.getElementById('muteBtn').onclick=()=>{"
          "  muted=!muted;"
          "  document.getElementById('muteBtn').textContent=muted?'Unmute':'Mute';"
          "};"
          ""
          "async function poll(){"
          "  try{"
          "    const r=await fetch('/data',{cache:'no-store'});"
          "    const j=await r.json();"
          "    const v=j.voltage_source||0, a=j.current_mA/1000||0, w=j.power_mW/1000||0, e=j.energy_today_Wh||0;"
          "    document.getElementById('v').textContent = v.toFixed(3);"
          "    document.getElementById('i').textContent = a.toFixed(3);"
          "    document.getElementById('p').textContent = w.toFixed(3);"
          "    document.getElementById('e').textContent = e.toFixed(3);"
          "    setToneFromWatts(w);"
          "  }catch(e){}"
          "  setTimeout(poll, 500);"
          "}"
          "poll();"
          "</script>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleData() {
  inaCalibrate32V2A();
  float shunt_mV  = haveINA ? ina219.getShuntVoltage_mV() : NAN;
  float busV      = haveINA ? ina219.getBusVoltage_V()    : NAN;
  float srcV      = (haveINA && !isnan(busV) && !isnan(shunt_mV)) ? (busV + shunt_mV/1000.0f) : NAN;
  // Calculate current manually from shunt voltage for external shunt
  float current_mA = NAN;
  float power_mW = NAN;
  if (haveINA && !isnan(shunt_mV)) {
    float current_A = (abs(shunt_mV / 1000.0) / SHUNT_RESISTANCE_OHMS) * SHUNT_CALIBRATION_FACTOR;
    current_mA = current_A * 1000.0;
    power_mW = busV * current_A * 1000.0;
  }

  String json = "{";
  json += "\"i2c_seen\":\"" + i2cSeen + "\",";
  json += "\"voltage_source\":" + safeStr(srcV, 3) + ",";
  json += "\"voltage_load\":"   + safeStr(busV, 3) + ",";
  json += "\"shunt_mV\":"       + safeStr(shunt_mV, 3) + ",";
  json += "\"current_mA\":"     + safeStr(current_mA, 3) + ",";
  json += "\"power_mW\":"       + safeStr(power_mW, 2) + ",";
  json += "\"energy_today_Wh\":" + safeStr(energyToday_Wh, 3);
  json += "}";
  server.send(200, "application/json", json);
}

void handleHealth() {
  String out = "{";
  out += "\"i2c_seen\":\"" + i2cSeen + "\",";
  out += "\"rtc\":" + String(haveRTC ? 1 : 0) + ",";
  out += "\"ina219\":" + String(haveINA ? 1 : 0) + ",";
  out += "\"sd\":" + String(haveSD ? 1 : 0) + ",";
  out += "\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0) + ",";
  out += "\"ip\":\"" + (WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\"";
  out += "}";
  server.send(200, "application/json", out);
}

void handleINAregs() {
  uint16_t cfg, shunt, bus, power, curr, cal;
  bool ok_cfg   = inaReadRegSafe(0x40, 0x00, cfg);
  bool ok_shunt = inaReadRegSafe(0x40, 0x01, shunt);
  bool ok_bus   = inaReadRegSafe(0x40, 0x02, bus);
  bool ok_power = inaReadRegSafe(0x40, 0x03, power);
  bool ok_curr  = inaReadRegSafe(0x40, 0x04, curr);
  bool ok_cal   = inaReadRegSafe(0x40, 0x05, cal);

  String out = "{";
  out += "\"cfg\": "   + String(ok_cfg   ? cfg   : 65535) + ",";
  out += "\"shunt\": " + String(ok_shunt ? (int16_t)shunt : -1) + ",";
  out += "\"bus\": "   + String(ok_bus   ? bus   : 65535) + ",";
  out += "\"power\": " + String(ok_power ? power : 65535) + ",";
  out += "\"curr\": "  + String(ok_curr  ? (int16_t)curr  : -1) + ",";
  out += "\"cal\": "   + String(ok_cal   ? cal   : 65535);
  out += "}";
  server.send(200, "application/json", out);
}

// ---------- DOWNLOAD CSV ----------
void handleDownload() {
  if (!haveSD) {
    server.send(503, "text/plain", "SD card not available");
    return;
  }

  File file = SD.open("/solar_log.csv", FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "Log file not found");
    return;
  }

  // Get file size
  size_t fileSize = file.size();

  // Send headers for CSV download
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"solar_log.csv\"");
  server.sendHeader("Content-Length", String(fileSize));
  server.sendHeader("Cache-Control", "no-cache");

  // Stream file in chunks
  server.setContentLength(fileSize);
  server.send(200, "text/csv", "");

  // Send file content in chunks
  uint8_t buffer[512];
  while (file.available()) {
    int bytesRead = file.read(buffer, sizeof(buffer));
    if (bytesRead > 0) {
      server.client().write(buffer, bytesRead);
    }
  }

  file.close();
  Serial.println("CSV download completed");
}

// ---------- CHARTS PAGE (Client-side processing) ----------
void handleChartsClient() {
  // Serve static HTML that processes CSV client-side
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=0">
    <title>Solar Charts</title>
    <style>
        body{font-family:-apple-system,system-ui,Arial;background:#111;color:#eee;margin:10px;padding:0}
        .header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px}
        h1{font-size:24px;margin:10px 0}
        .btn{border:1px solid #445;background:#223;color:#dbe7ff;border-radius:10px;padding:14px 18px;cursor:pointer;text-decoration:none;font-size:15px;font-weight:600}
        .card{margin-top:14px;background:#1a1a2e;border:1px solid #333;border-radius:14px;padding:20px;position:relative}
        .tabs{display:flex;gap:8px;margin:8px 0}
        .tab{padding:10px 14px;border:1px solid #3a3964;border-radius:10px;cursor:pointer;background:#2a2a4a;font-size:14px}
        .tab.active{background:#334;border-color:#556}
        .toggle-container{display:flex;justify-content:flex-end;margin:15px 0;align-items:center;gap:10px;padding:10px;background:#0f1730;border:1px solid #3a3964;border-radius:10px}
        .toggle-label{color:#7fbfff;font-size:14px}
        .toggle-switch{position:relative;width:60px;height:28px;background:#23335e;border-radius:14px;cursor:pointer;transition:background 0.3s}
        .toggle-switch.active{background:#3aa2ff}
        .toggle-slider{position:absolute;top:3px;left:3px;width:22px;height:22px;background:#fff;border-radius:11px;transition:left 0.3s}
        .toggle-switch.active .toggle-slider{left:35px}
        canvas{width:100%;height:350px;background:#0f1730;border:1px solid #23335e;border-radius:10px;display:block}
        .legend{margin-top:8px;color:#bcd0ff;font-size:13px}
        .loading-overlay{position:absolute;top:0;left:0;right:0;bottom:0;background:rgba(26,26,46,0.95);display:flex;flex-direction:column;align-items:center;justify-content:center;border-radius:14px;z-index:1000}
        .spinner{border:3px solid #23335e;border-top:3px solid #3aa2ff;border-radius:50%;width:50px;height:50px;animation:spin 1s linear infinite}
        @keyframes spin{0%{transform:rotate(0deg)}100%{transform:rotate(360deg)}}
        .loading-text{color:#7fbfff;margin-top:20px;font-size:16px}
        .loading-progress{color:#5a8fd8;margin-top:10px;font-size:13px}
        .hidden{display:none}
    </style>
</head>
<body>
    <div class="header">
        <h1>Solar Charts</h1>
        <a href="/" class="btn">Back to Live</a>
    </div>
    <div class="card">
        <div class="loading-overlay" id="loadingOverlay">
            <div class="spinner"></div>
            <div class="loading-text">Downloading data...</div>
            <div class="loading-progress" id="loadingProgress">Please wait</div>
        </div>
        <div class="tabs">
            <div id="tab-hourly" class="tab active" onclick="switchTab('hourly')">Hourly</div>
            <div id="tab-daily" class="tab" onclick="switchTab('daily')">Daily (7d)</div>
            <div id="tab-weekly" class="tab" onclick="switchTab('weekly')">Weekly</div>
            <div id="tab-monthly" class="tab" onclick="switchTab('monthly')">Monthly</div>
        </div>
        <div class="toggle-container" id="toggleContainer" style="display:none;">
            <span class="toggle-label">Show Cumulative</span>
            <div class="toggle-switch" id="cumulativeToggle" onclick="toggleCumulative()">
                <div class="toggle-slider"></div>
            </div>
        </div>
        <canvas id="chart"></canvas>
        <div class="legend" id="legendText">Loading...</div>
    </div>
)rawliteral";

  // Split the large literal to avoid compiler limits
  html += R"rawliteral(
<script>
let csvData=null,cvs,ctx;
let showCumulative=false;
let currentTab='hourly';

function toggleCumulative(){
  showCumulative = !showCumulative;
  const toggle = document.getElementById('cumulativeToggle');
  if(showCumulative){
    toggle.classList.add('active');
  }else{
    toggle.classList.remove('active');
  }
  switchTab(currentTab);
}

async function fetchCSVData(){
  try{
    document.getElementById('loadingProgress').textContent = 'Connecting to ESP32...';

    const r=await fetch('/download',{cache:'no-store'});
    const t=await r.text();

    document.getElementById('loadingProgress').textContent = 'Processing data...';

    const lines=t.trim().split('\n');
    csvData=[];
    const totalLines = lines.length - 1;
    let processedLines = 0;

    for(let i=1;i<lines.length;i++){
      const ln=lines[i].trim();
      if(ln.length===0)continue;
      const p=ln.split(',');
      if(p.length>=7){
        csvData.push({
          timestamp:p[0],
          power_mW:parseFloat(p[5])
        });
      }
      processedLines++;

      // Update progress every 1000 lines
      if(processedLines % 1000 === 0){
        const percent = Math.round((processedLines / totalLines) * 100);
        document.getElementById('loadingProgress').textContent = 'Processing ' + percent + '% (' + processedLines + ' records)';
      }
    }

    document.getElementById('loadingProgress').textContent = 'Rendering chart...';
    return true;
  }catch(e){
    document.getElementById('loadingProgress').textContent = 'Error loading data';
    console.error(e);
    return false;
  }
}

function processHourlyData(){
  const hd=new Array(24).fill(0);
  const hc=new Array(24).fill(0);
  const now=new Date();
  const ch=now.getHours();

  csvData.forEach(r=>{
    const tp=r.timestamp.indexOf('T');
    if(tp>0){
      const h=parseInt(r.timestamp.substring(tp+1,tp+3));
      if(!isNaN(h)&&h>=0&&h<24){
        hd[h]+=r.power_mW;
        hc[h]++;
      }
    }
  });

  const labels=[],values=[];
  for(let h=0;h<=ch&&h<24;h++){
    let l;
    if(h===0)l="12am";
    else if(h<12)l=h+"am";
    else if(h===12)l="12pm";
    else l=(h-12)+"pm";
    labels.push(l);
    values.push(hc[h]>0?(hd[h]/hc[h]/1000):0);
  }
  return{title:'Hourly avg power (W)',unit:'W',labels:labels,values:values};
}

function processDailyData(){
  const dd={};
  csvData.forEach(r=>{
    const tp=r.timestamp.indexOf('T');
    if(tp>0){
      const ds=r.timestamp.substring(0,tp);
      if(!dd[ds]){dd[ds]={totalEnergy:0,count:0};}
      // Each reading represents 5 seconds of generation
      const energyWh = (r.power_mW / 1000) * (5 / 3600); // Convert mW to W, then W * hours
      dd[ds].totalEnergy += energyWh;
      dd[ds].count++;
    }
  });

  const dates=Object.keys(dd).sort().slice(-7);
  const labels=[],values=[];

  let cumulativeTotal = 0;
  dates.forEach(d=>{
    const p=d.split('-');
    if(p.length===3){
      labels.push(parseInt(p[1])+'/'+parseInt(p[2]));
      if(showCumulative){
        cumulativeTotal += dd[d].totalEnergy;
        values.push(cumulativeTotal);
      }else{
        values.push(dd[d].totalEnergy);
      }
    }
  });

  const title = showCumulative ? 'Cumulative energy (Wh, 7d)' : 'Daily energy (Wh, 7d)';
  return{title:title,unit:'Wh',labels:labels,values:values};
}

function processWeeklyData(){
  const wd=new Array(7).fill(0);
  const dayNames=['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];

  csvData.forEach(r=>{
    const tp=r.timestamp.indexOf('T');
    if(tp>0){
      const ds=r.timestamp.substring(0,tp);
      const dt=new Date(ds);
      const dow=dt.getDay();
      if(!isNaN(dow)){
        const energyWh = (r.power_mW / 1000) * (5 / 3600);
        wd[dow] += energyWh;
      }
    }
  });

  const today=new Date().getDay();
  const labels=[],values=[];
  let cumulativeTotal = 0;

  for(let i=0;i<7;i++){
    const idx=(today-6+i+7)%7;
    labels.push(dayNames[idx]);
    if(showCumulative){
      cumulativeTotal += wd[idx];
      values.push(cumulativeTotal);
    }else{
      values.push(wd[idx]);
    }
  }

  const title = showCumulative ? 'Cumulative weekly energy (Wh)' : 'Weekly energy (Wh)';
  return{title:title,unit:'Wh',labels:labels,values:values};
}

function processMonthlyData(){
  const md=new Array(12).fill(0);
  const mc=new Array(12).fill(0);
  const monthNames=['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'];

  csvData.forEach(r=>{
    const tp=r.timestamp.indexOf('T');
    if(tp>0){
      const ds=r.timestamp.substring(0,tp);
      const dt=new Date(ds);
      const m=dt.getMonth();
      if(!isNaN(m)){
        md[m]+=r.power_mW;
        mc[m]++;
      }
    }
  });

  const labels=[],values=[];
  let cumulativeTotal=0;

  for(let m=0;m<12;m++){
    labels.push(monthNames[m]);
    if(mc[m]>0){
      const ap=md[m]/mc[m]/1000;
      const h=(mc[m]*5)/3600;
      const energyWh=ap*h;

      if(showCumulative){
        cumulativeTotal+=energyWh;
        values.push(cumulativeTotal);
      }else{
        values.push(energyWh);
      }
    }else{
      values.push(showCumulative?cumulativeTotal:0);
    }
  }

  const title=showCumulative?'Cumulative monthly energy (Wh)':'Monthly energy (Wh)';
  return{title:title,unit:'Wh',labels:labels,values:values};
}

function drawAxes(pd,yM,yL){
  const W=cvs.clientWidth,H=cvs.clientHeight;
  cvs.width=W*devicePixelRatio;
  cvs.height=H*devicePixelRatio;
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  ctx.clearRect(0,0,W,H);

  ctx.strokeStyle='#2b3b67';
  ctx.lineWidth=1;
  ctx.beginPath();
  ctx.moveTo(pd,10);
  ctx.lineTo(pd,H-pd);
  ctx.lineTo(W-10,H-pd);
  ctx.stroke();

  ctx.fillStyle='#9fb4ff';
  ctx.font='12px system-ui';
  ctx.fillText(yL,10,18);

  for(let i=0;i<=5;i++){
    const y=H-pd-(H-2*pd)*i/5;
    const v=(yM*i/5).toFixed(1);
    ctx.fillText(v,5,y+4);
    if(i>0){
      ctx.strokeStyle='#1f2b4d';
      ctx.beginPath();
      ctx.moveTo(pd,y);
      ctx.lineTo(W-10,y);
      ctx.stroke();
    }
  }
}

function drawLine(lb,vl,cl,yL){
  const W=cvs.clientWidth,H=cvs.clientHeight,pd=40;
  const yM=Math.max(1,Math.max(...vl)*1.15);
  drawAxes(pd,yM,yL);

  const n=vl.length;
  if(n<1)return;
  const xS=(W-pd-20)/(n-1||1);

  ctx.strokeStyle=cl;
  ctx.lineWidth=2.5;
  ctx.lineCap='round';
  ctx.lineJoin='round';
  ctx.beginPath();
  for(let i=0;i<n;i++){
    const x=pd+i*xS;
    const y=H-pd-(H-2*pd)*(vl[i]/yM);
    if(i===0)ctx.moveTo(x,y);
    else ctx.lineTo(x,y);
  }
  ctx.stroke();

  ctx.globalAlpha=0.15;
  ctx.fillStyle=cl;
  ctx.beginPath();
  ctx.moveTo(pd,H-pd);
  for(let i=0;i<n;i++){
    const x=pd+i*xS;
    const y=H-pd-(H-2*pd)*(vl[i]/yM);
    ctx.lineTo(x,y);
  }
  ctx.lineTo(pd+(n-1)*xS,H-pd);
  ctx.closePath();
  ctx.fill();
  ctx.globalAlpha=1;

  ctx.fillStyle='#fff';
  for(let i=0;i<n;i++){
    const x=pd+i*xS;
    const y=H-pd-(H-2*pd)*(vl[i]/yM);
    ctx.beginPath();
    ctx.arc(x,y,3,0,Math.PI*2);
    ctx.fill();
    ctx.strokeStyle=cl;
    ctx.lineWidth=1.5;
    ctx.stroke();
  }

  ctx.fillStyle='#bcd0ff';
  ctx.font='11px system-ui';
  const st=Math.max(1,Math.ceil(n/12));
  for(let i=0;i<n;i+=st){
    const x=pd+i*xS;
    ctx.save();
    ctx.translate(x,H-pd+12);
    ctx.rotate(-0.6);
    ctx.fillText(lb[i],0,0);
    ctx.restore();
  }
}

function switchTab(t){
  currentTab=t;
  ['hourly','daily','weekly','monthly'].forEach(n=>{
    const e=document.getElementById('tab-'+n);
    if(e)e.className='tab';
  });
  const at=document.getElementById('tab-'+t);
  if(at)at.className='tab active';

  // Show/hide toggle based on tab
  const toggleContainer=document.getElementById('toggleContainer');
  if(t==='hourly'){
    toggleContainer.style.display='none';
    console.log('Hiding toggle for hourly tab');
  }else{
    toggleContainer.style.display='flex';
    console.log('Showing toggle for ' + t + ' tab');
  }

  if(!csvData)return;

  let d;
  switch(t){
    case 'hourly':d=processHourlyData();break;
    case 'daily':d=processDailyData();break;
    case 'weekly':d=processWeeklyData();break;
    case 'monthly':d=processMonthlyData();break;
  }

  document.getElementById('legendText').textContent=d.title;
  drawLine(d.labels,d.values,'#3aa2ff',d.unit);
}

// Initialize after DOM is ready
window.addEventListener('DOMContentLoaded', function() {
  cvs=document.getElementById('chart');
  ctx=cvs.getContext('2d');
  // Start fetching data
  fetchCSVData().then(s=>{
    if(s){
      switchTab('hourly');
      // Hide loading overlay after chart is drawn
      setTimeout(()=>{
        document.getElementById('loadingOverlay').classList.add('hidden');
      }, 100);
    }else{
      // Show error but hide spinner
      document.getElementById('loadingOverlay').innerHTML = '<div style="color:#ff6b6b;font-size:18px">Failed to load data</div><div style="margin-top:10px;color:#bcd0ff">Please refresh the page</div>';
    }
  });
});
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html.c_str());
}

// ---------- CHARTS PAGE (Old server-side processing - kept for reference) ----------
void handleCharts() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=0'>"
                "<title>Solar Charts</title>"
                "<style>"
                "body{font-family:-apple-system,system-ui,Arial;background:#111;color:#eee;margin:10px;padding:0}"
                ".header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px}"
                "h1{font-size:24px;margin:10px 0}"
                ".btn{border:1px solid #445;background:#223;color:#dbe7ff;border-radius:10px;padding:14px 18px;"
                "cursor:pointer;text-decoration:none;font-size:15px;font-weight:600;-webkit-tap-highlight-color:transparent}"
                ".btn:active{transform:translateY(1px);background:#334}"
                ".card{margin-top:14px;background:#1a1a2e;border:1px solid #333;border-radius:14px;padding:20px}"
                ".tabs{display:flex;gap:8px;margin:8px 0}"
                ".tab{padding:10px 14px;border:1px solid #3a3964;border-radius:10px;cursor:pointer;background:#2a2a4a;font-size:14px}"
                ".tab.active{background:#334;border-color:#556}"
                "canvas{width:100%;height:350px;background:#0f1730;border:1px solid #23335e;border-radius:10px;touch-action:none;display:block}"
                ".legend{margin-top:8px;color:#bcd0ff;font-size:13px}"
                "</style></head><body>";

  html += "<div class='header'>";
  html += "<h1>Solar Charts</h1>";
  html += "<a href='/' class='btn'>Back to Live</a>";
  html += "</div>";

  html += "<div class='card'>";
  html += "<div class='tabs'>";
  html += "<div id='tab-hourly' class='tab active' onclick=\"switchTab('hourly')\">Hourly (today)</div>";
  html += "<div id='tab-weekly' class='tab' onclick=\"switchTab('weekly')\">Weekly (7d)</div>";
  html += "<div id='tab-daily' class='tab' onclick=\"switchTab('daily')\">Daily (30d)</div>";
  html += "<div id='tab-monthly' class='tab' onclick=\"switchTab('monthly')\">Monthly (12m)</div>";
  html += "</div>";
  html += "<canvas id='chart'></canvas>";
  html += "<div class='legend' id='legendText'>Hourly avg power (W)</div>";
  html += "</div>";

  // Put all JavaScript after the canvas element exists
  html += "<script type='text/javascript'>";
  html += "var cvs = document.getElementById('chart');"
          "var ctx = cvs.getContext('2d');"
          "function switchTab(tabName){"
          "var tabs = ['hourly', 'weekly', 'daily', 'monthly'];"
          "for(var i=0; i<tabs.length; i++){"
          "var el = document.getElementById('tab-' + tabs[i]);"
          "if(el) el.className = 'tab';"
          "}"
          "var activeTab = document.getElementById('tab-' + tabName);"
          "if(activeTab) activeTab.className = 'tab active';"
          "loadAgg(tabName);"
          "}";

  // Combine multiple lines to reduce concatenations
  html += "function drawAxes(pad, yMax, yLabel){"
          "const W=cvs.clientWidth, H=cvs.clientHeight;"
          "cvs.width=W*devicePixelRatio; cvs.height=H*devicePixelRatio;"
          "ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);"
          "ctx.clearRect(0,0,W,H);"
          "ctx.strokeStyle='#2b3b67'; ctx.lineWidth=1;"
          "ctx.beginPath(); ctx.moveTo(pad,10); ctx.lineTo(pad,H-pad); ctx.lineTo(W-10,H-pad); ctx.stroke();"
          "ctx.fillStyle='#9fb4ff'; ctx.font='12px system-ui'; ctx.fillText(yLabel,10,18);"
          "for(let i=0;i<=5;i++){"
          "const y=H-pad - (H-2*pad)*i/5; const v=(yMax*i/5).toFixed(1);"
          "ctx.fillText(v,W-pad+4,y+4);"
          "ctx.strokeStyle='#1f2b4d'; ctx.beginPath(); ctx.moveTo(pad,y); ctx.lineTo(W-10,y); ctx.stroke();"
          "}"
          "}";
  html += "function drawLine(labels, values, color, yLabel){"
          "const W=cvs.clientWidth, H=cvs.clientHeight, pad=40;"
          "const yMax=Math.max(1, Math.max(...values)*1.15);"
          "drawAxes(pad,yMax,yLabel);"
          "const n=values.length;"
          "if(n<1) return;"
          "const xStep=(W-pad-20)/(n-1||1);"
          "ctx.strokeStyle=color; ctx.lineWidth=2.5;"
          "ctx.lineCap='round'; ctx.lineJoin='round';"
          "ctx.beginPath();"
          "for(let i=0;i<n;i++){"
          "const x=pad+i*xStep;"
          "const y=H-pad-(H-2*pad)*(values[i]/yMax);"
          "if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);"
          "}"
          "ctx.stroke();";
  html += "ctx.globalAlpha=0.15; ctx.fillStyle=color;"
          "ctx.beginPath(); ctx.moveTo(pad, H-pad);"
          "for(let i=0;i<n;i++){"
          "const x=pad+i*xStep; const y=H-pad-(H-2*pad)*(values[i]/yMax);"
          "ctx.lineTo(x,y);"
          "}"
          "ctx.lineTo(pad+(n-1)*xStep, H-pad);"
          "ctx.closePath(); ctx.fill(); ctx.globalAlpha=1;";
  html += "ctx.fillStyle='#fff';"
          "for(let i=0;i<n;i++){"
          "const x=pad+i*xStep; const y=H-pad-(H-2*pad)*(values[i]/yMax);"
          "ctx.beginPath(); ctx.arc(x,y,3,0,Math.PI*2); ctx.fill();"
          "ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.stroke();"
          "}";
  html += "ctx.fillStyle='#bcd0ff'; ctx.font='11px system-ui';"
          "const step=Math.max(1,Math.ceil(n/12));"
          "for(let i=0;i<n;i+=step){"
          "const x=pad+i*xStep;"
          "ctx.save(); ctx.translate(x, H-pad+12); ctx.rotate(-0.6); ctx.fillText(labels[i],0,0); ctx.restore();"
          "}"
          "}";

  html += "async function loadAgg(kind){"
          "const res=await fetch('/agg/'+kind,{cache:'no-store'});"
          "const j=await res.json();"
          "const lbl=j.labels||[], val=j.values||[], unit=j.unit||'', title=j.title||'';"
          "document.getElementById('legendText').textContent=title;"
          "drawLine(lbl,val,'#3aa2ff', unit);"
          "}";
  html += "loadAgg('hourly');";
  html += "</script>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Debug CSV endpoint
void handleDebugCSV() {
  String response = "<html><body><pre>";
  response += "SD Card: " + String(haveSD ? "Yes" : "No") + "\n\n";

  if (haveSD) {
    File f = SD.open("/solar_log.csv", FILE_READ);
    if (f) {
      response += "CSV File Size: " + String(f.size()) + " bytes\n\n";
      response += "First 10 lines:\n";
      response += "==============\n";

      int lineCount = 0;
      while (f.available() && lineCount < 10) {
        String line = f.readStringUntil('\n');
        response += line + "\n";
        lineCount++;
      }
      f.close();

      response += "\n\nParsing test on line 2:\n";
      response += "=====================\n";

      // Re-open and test parsing
      f = SD.open("/solar_log.csv", FILE_READ);
      if (f) {
        f.readStringUntil('\n'); // Skip header
        if (f.available()) {
          String testLine = f.readStringUntil('\n');
          response += "Raw: " + testLine + "\n";

          String ts; float v, i, p;
          if (parseLogLine(testLine, ts, v, i, p)) {
            response += "Parsed OK:\n";
            response += "  Timestamp: " + ts + "\n";
            response += "  Power: " + String(p) + " mW\n";

            // Test hour extraction
            int tPos = ts.indexOf('T');
            if (tPos > 0) {
              String timeStr = ts.substring(tPos + 1);
              int colonPos = timeStr.indexOf(':');
              if (colonPos >= 2) {
                int hour = timeStr.substring(0, colonPos).toInt();
                response += "  Hour: " + String(hour) + "\n";
              }
            }
          } else {
            response += "Parse FAILED\n";
          }
        }
        f.close();
      }
    } else {
      response += "Failed to open CSV file\n";
    }
  }

  response += "</pre></body></html>";
  server.send(200, "text/html", response);
}

// CSV parse helper for charts
bool parseLogLine(const String &line, String &timestamp, float &voltage, float &current, float &power) {
  // CSV format: timestamp,voltage_source_V,voltage_load_V,shunt_mV,current_mA,power_mW,energy_today_Wh
  int idx1 = line.indexOf(',');
  if (idx1 < 0) return false;
  int idx2 = line.indexOf(',', idx1+1);
  if (idx2 < 0) return false;
  int idx3 = line.indexOf(',', idx2+1);
  if (idx3 < 0) return false;
  int idx4 = line.indexOf(',', idx3+1);
  if (idx4 < 0) return false;
  int idx5 = line.indexOf(',', idx4+1);
  if (idx5 < 0) return false;
  int idx6 = line.indexOf(',', idx5+1);
  if (idx6 < 0) return false;

  timestamp = line.substring(0, idx1);
  voltage = line.substring(idx1+1, idx2).toFloat();
  current = line.substring(idx4+1, idx5).toFloat() / 1000.0; // mA to A
  power = line.substring(idx5+1, idx6).toFloat();   // Keep in mW, not converting to W here

  // Debug first few lines
  static int debugCount = 0;
  if (debugCount < 3) {
    Serial.println("ParseLine: ts=" + timestamp + " P=" + String(power) + "mW");
    debugCount++;
  }

  return true;
}

// Hourly aggregation
void handleHourly() {
  float sum[24] = {0};
  uint32_t cnt[24] = {0};
  int linesRead = 0;
  int validLines = 0;
  int hoursWithData = 0;

  Serial.println("=== handleHourly called ===");

  if (haveSD) {
    File f = SD.open("/solar_log.csv", FILE_READ);
    if (f) {
      Serial.println("CSV file opened, size: " + String(f.size()));

      // Skip header line
      if (f.available()) {
        String header = f.readStringUntil('\n');
        Serial.println("Header: " + header);
      }

      while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim(); // Remove any trailing whitespace
        if (line.length() == 0) continue; // Skip empty lines

        linesRead++;
        if (linesRead <= 3) {
          Serial.println("Line " + String(linesRead) + ": " + line);
        }

        String ts; float v, i, p;
        if (parseLogLine(line, ts, v, i, p)) {
          validLines++;

          // Extract hour from timestamp format "2025-09-21T10:13:45"
          int tPos = ts.indexOf('T');
          if (tPos > 0 && tPos < ts.length() - 8) {
            // Hour is right after the 'T'
            String timeStr = ts.substring(tPos + 1);
            int colonPos = timeStr.indexOf(':');
            if (colonPos >= 2) {
              int hour = timeStr.substring(0, colonPos).toInt();
              if (hour >= 0 && hour < 24) {
                sum[hour] += p;  // p is already in mW from the CSV
                cnt[hour]++;
                if (validLines <= 3) {
                  Serial.println("Added to hour " + String(hour) + ": " + String(p) + "mW");
                }
              }
            }
          }
        }
      }
      f.close();

      // Count hours with data
      for(int h = 0; h < 24; h++) {
        if (cnt[h] > 0) hoursWithData++;
      }

      Serial.println("Hourly: Read " + String(linesRead) + " lines, " + String(validLines) + " valid, " + String(hoursWithData) + " hours with data");
    } else {
      Serial.println("Hourly: Failed to open CSV");
    }
  } else {
    Serial.println("Hourly: No SD card");
  }

  // Get current hour from RTC if available
  int currentHour = 23;  // Default to full day if no RTC
  if (haveRTC) {
    DateTime now = rtc.now();
    currentHour = now.hour();
  }

  String labels = "[", values = "[";
  bool first = true;
  for(int h = 0; h <= currentHour && h < 24; h++) {
    if (!first) { labels += ","; values += ","; }
    first = false;

    // Convert to 12-hour format with AM/PM
    String timeLabel;
    if (h == 0) {
      timeLabel = "12am";
    } else if (h < 12) {
      timeLabel = String(h) + "am";
    } else if (h == 12) {
      timeLabel = "12pm";
    } else {
      timeLabel = String(h - 12) + "pm";
    }
    labels += "\"" + timeLabel + "\"";
    // Convert from mW to W when displaying (sum is in mW, divide by cnt for avg, then by 1000 for W)
    values += String(cnt[h] > 0 ? ((sum[h] / cnt[h]) / 1000.0) : 0.0, 2);
  }
  labels += "]"; values += "]";

  // Add debug info to the JSON response
  String debugInfo = "Lines:" + String(linesRead) + " Valid:" + String(validLines) + " Hours:" + String(hoursWithData);

  // Also show the first hour with data for debugging
  int firstHourWithData = -1;
  float firstHourValue = 0;
  for(int h = 0; h < 24; h++) {
    if (cnt[h] > 0) {
      firstHourWithData = h;
      firstHourValue = (sum[h] / cnt[h]) / 1000.0;  // Convert mW to W
      break;
    }
  }

  if (firstHourWithData >= 0) {
    debugInfo += " First:" + String(firstHourWithData) + "h=" + String(firstHourValue, 1) + "W";
  }

  String json = "{\"title\":\"Hourly avg power (W) - " + debugInfo + "\",\"unit\":\"W\",\"labels\":" + labels + ",\"values\":" + values + "}";
  server.send(200, "application/json", json);
}

// Weekly aggregation (7 days)
void handleWeekly() {
  const int DAYS = 7;
  float wh[DAYS];
  String dayLabels[DAYS] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};

  for(int i = 0; i < DAYS; i++) wh[i] = 0.0;

  // Get current day of week if RTC is available
  int currentDayOfWeek = 0;
  if (haveRTC) {
    DateTime now = rtc.now();
    currentDayOfWeek = now.dayOfTheWeek(); // 0 = Sunday, 6 = Saturday
  }

  // Build labels based on current day
  String labels = "[";
  for(int i = 0; i < DAYS; i++) {
    if (i > 0) labels += ",";
    int dayIndex = (currentDayOfWeek - (DAYS - 1 - i) + 7) % 7;
    labels += "\"" + dayLabels[dayIndex] + "\"";
  }
  labels += "]";

  // For now, using placeholder values - needs proper implementation
  String values = "[";
  for(int i = 0; i < DAYS; i++) {
    if (i > 0) values += ",";
    values += String(energyToday_Wh * (i == DAYS-1 ? 1.0 : 0.0), 2); // Show today's energy on last day
  }
  values += "]";

  String json = "{\"title\":\"Weekly energy (Wh, 7d)\",\"unit\":\"Wh\",\"labels\":" + labels + ",\"values\":" + values + "}";
  server.send(200, "application/json", json);
}

// Daily aggregation
void handleDaily() {
  const int DAYS = 30;
  float dailySum[DAYS] = {0};
  uint32_t dailyCnt[DAYS] = {0};
  float wh[DAYS] = {0};
  String dayDates[DAYS];

  // Get today's date and generate labels for last 30 days
  if (haveRTC) {
    DateTime now = rtc.now();

    // Generate date labels for the last 30 days
    for(int i = 0; i < DAYS; i++) {
      DateTime targetDate = now - TimeSpan(DAYS - 1 - i, 0, 0, 0);
      dayDates[i] = String(targetDate.month()) + "/" + String(targetDate.day());
    }

    int todayDay = now.day();
    int todayMonth = now.month();
    int todayYear = now.year();

    // Read CSV and aggregate by date
    if (haveSD) {
      File f = SD.open("/solar_log.csv", FILE_READ);
      if (f) {
        // Skip header line
        if (f.available()) f.readStringUntil('\n');

        while (f.available()) {
          String line = f.readStringUntil('\n');
          line.trim();
          if (line.length() == 0) continue;

          String ts; float v, i, p;
          if (parseLogLine(line, ts, v, i, p)) {
            // Extract date from timestamp format "2025-09-21T10:13:45"
            int tPos = ts.indexOf('T');
            if (tPos > 0) {
              String dateStr = ts.substring(0, tPos); // "2025-09-21"
              int dash1 = dateStr.indexOf('-');
              int dash2 = dateStr.indexOf('-', dash1+1);

              if (dash1 > 0 && dash2 > 0) {
                int year = dateStr.substring(0, dash1).toInt();
                int month = dateStr.substring(dash1+1, dash2).toInt();
                int day = dateStr.substring(dash2+1).toInt();

                // Calculate days ago (simple approximation)
                int daysAgo = (todayYear - year) * 365 + (todayMonth - month) * 30 + (todayDay - day);

                if (daysAgo >= 0 && daysAgo < DAYS) {
                  int idx = DAYS - 1 - daysAgo; // Reverse order so today is last
                  dailySum[idx] += p; // p is in mW
                  dailyCnt[idx]++;
                }
              }
            }
          }
        }
        f.close();
      }
    }
  } else {
    // No RTC, use simple day numbering
    for(int i = 0; i < DAYS; i++) {
      dayDates[i] = "Day " + String(DAYS - i);
    }
  }

  // Calculate daily energy (Wh) from average power
  for(int i = 0; i < DAYS; i++) {
    if (dailyCnt[i] > 0) {
      // Convert from mW to W, then multiply by hours
      // Each reading represents 5 seconds, so cnt * 5 seconds = total seconds
      float hours = (dailyCnt[i] * 5.0) / 3600.0;
      wh[i] = (dailySum[i] / dailyCnt[i]) * hours / 1000.0; // avg power in W * hours
    }
  }

  // Build JSON response - only show last 7 days for readability
  const int SHOW_DAYS = 7; // Show last 7 days
  int startIdx = DAYS - SHOW_DAYS;

  String labels = "[", values = "[";
  bool first = true;
  for(int i = startIdx; i < DAYS; i++) {
    if (!first) { labels += ","; values += ","; }
    first = false;

    labels += "\"" + dayDates[i] + "\"";
    values += String(wh[i], 2);
  }
  labels += "]"; values += "]";

  String json = "{\"title\":\"Daily energy (Wh, 7d)\",\"unit\":\"Wh\",\"labels\":" + labels + ",\"values\":" + values + "}";
  server.send(200, "application/json", json);
}

// Monthly aggregation
void handleMonthly() {
  const int MONTHS = 12;
  String monthNames[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};

  String json = "{\"title\":\"Monthly energy (Wh, 12m)\",\"unit\":\"Wh\",\"labels\":[";
  for(int i = 0; i < MONTHS; i++) {
    if (i > 0) json += ",";
    json += "\"" + monthNames[i] + "\"";
  }
  json += "],\"values\":[";
  for(int i = 0; i < MONTHS; i++) {
    if (i > 0) json += ",";
    json += "0"; // Placeholder - needs proper implementation
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// ---------- OTA (Arduino IDE Network Port + Browser OTA) ----------
void setupArduinoOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA
    .onStart([]() { Serial.println("OTA start"); })
    .onEnd([]()   { Serial.println("OTA end → reboot"); delay(100); ESP.restart(); })
    .onError([](ota_error_t err) { Serial.printf("OTA error: %u\n", err); });
  ArduinoOTA.begin();
}

void handleUpdatePage() {
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
}

void handleUpdateUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.printf("HTTP OTA: %s\n", up.filename.c_str());
    if (!Update.begin()) { Update.printError(Serial); }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) { Update.printError(Serial); }
  } else if (up.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) { Update.printError(Serial); }
    else { Serial.printf("HTTP OTA done (%u bytes)\n", up.totalSize); }
  }
}

void handleUpdateFinish() {
  bool ok = !Update.hasError();
  server.send(200, "text/plain", ok ? "Update OK, rebooting..." : "Update FAILED");
  delay(200);
  if (ok) ESP.restart();
}

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println("\n=== Solar Logger Boot (SDA=23, SCL=22) ===");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(1000);

  // I2C probe
  detectI2CDevices();
  Serial.print("I2C seen at boot: "); Serial.println(i2cSeen); // expect "0x40" (+ "0x68" if RTC present)

  // Devices
  initINA();    // with only INA219 present, this is the key piece
  initRTC();    // ok if no RTC present; will fallback
  initSD();     // ok if Adalogger absent

  // Wi-Fi + OTA
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
    setupArduinoOTA();
  } else {
    Serial.println("WiFi not connected (logging-only mode)");
  }

  // Web routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/health", handleHealth);
  server.on("/ina219", handleINAregs);
  server.on("/download", handleDownload);
  server.on("/charts", handleChartsClient);  // Use client-side processing for better performance
  server.on("/agg/hourly", handleHourly);
  server.on("/agg/weekly", handleWeekly);
  server.on("/agg/daily", handleDaily);
  server.on("/agg/monthly", handleMonthly);
  server.on("/debug/csv", handleDebugCSV);
  server.on("/debug/raw", [](){
    handleHourly();  // Call handleHourly but the JSON response will be visible as raw text
  });
  server.on("/debug/html", [](){
    // Return the charts HTML as plain text to debug
    server.send(200, "text/plain", "Check browser view-source instead");
  });
  server.on("/update", HTTP_GET, [](){ handleUpdatePage(); });
  server.on("/update", HTTP_POST, [](){ handleUpdateFinish(); }, [](){ handleUpdateUpload(); });
  server.begin();
  Serial.println("Web server started");

  lastEnergyMs = millis();
  Serial.println("=== Setup complete ===\n");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
  server.handleClient();

  const unsigned long nowMs = millis();
  if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    logSample();
  }
}