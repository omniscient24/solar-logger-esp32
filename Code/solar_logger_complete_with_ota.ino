/************************************************************
  ESP32 Solar Logger with OTA Updates
  - FIXED FOR INA219 (not INA226!)
  - OTA (Over-The-Air) updates enabled
  - HTML moved to global to prevent stack overflow
  - SD logging (CS=5), CSV: epoch,iso,volts,amps,watts
  - Calibration persisted in /cal.cfg on SD
*************************************************************/

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include "esp_http_server.h"
#include <ArduinoOTA.h>

// Wi-Fi Configuration
#define WIFI_SSID      "weenieNet4"
#define WIFI_PASSWORD  "un-moc-cod-ek-u"

// Time/NTP Configuration
#define TZ_STRING      "CST6CDT,M3.2.0,M11.1.0"   // America/Chicago
#define NTP_SERVER     "pool.ntp.org"

// OTA Configuration
#define OTA_HOSTNAME   "solar-logger"
#define OTA_PASSWORD   "solar123"  // CHANGE THIS!

// INA219 Configuration
#define INA219_ADDR 0x40
#define SHUNT_OHMS   0.1f      // INA219 FeatherWing uses 0.1 ohm shunt
#define MAX_CURRENT  3.2f      // Max current with 0.1 ohm shunt

// INA219 Registers
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// INA219 Configuration values
#define INA219_CONFIG_RESET                    0x8000  // Reset bit
#define INA219_CONFIG_BVOLTAGERANGE_32V        0x2000  // 32V range
#define INA219_CONFIG_GAIN_8_320MV             0x1800  // Gain 8, 320mV range
#define INA219_CONFIG_BADCRES_12BIT            0x0180  // 12-bit bus res
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   0x0018  // 12-bit shunt res
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007 // Continuous mode

#define I2C_SDA 21
#define I2C_SCL 22

// SD (VSPI)
#define SD_CS 5

static double energy_Wh     = 0.0;
static unsigned long lastEnergyMs  = 0;
static httpd_handle_t http_server   = nullptr;

static float CAL_I = 1.0f;
static float CAL_V = 1.0f;

// INA219 calibration values
static uint16_t ina219_calValue = 0;
static float current_lsb = 0;

// Forward declarations
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t data_get_handler(httpd_req_t *req);
static esp_err_t cal_show_handler(httpd_req_t *req);
static esp_err_t cal_set_handler (httpd_req_t *req);
static esp_err_t diag_get_handler(httpd_req_t *req);

//============================================
// HTML Pages (Global to prevent stack overflow)
//============================================
static const char ROOT_PAGE_HTML[] = 
  "<!DOCTYPE html>\n"
  "<html>\n"
  "<head>\n"
  "  <title>Solar Logger</title>\n"
  "  <meta name='viewport' content='width=device-width, initial-scale=1'>\n"
  "  <meta http-equiv='refresh' content='5'>\n"
  "  <style>\n"
  "    body { font-family: Arial; background: #1a1a1a; color: #fff; margin: 0; padding: 20px; }\n"
  "    .container { max-width: 1200px; margin: 0 auto; }\n"
  "    .header { text-align: center; margin-bottom: 30px; }\n"
  "    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; }\n"
  "    .tile { background: #2a2a2a; border-radius: 10px; padding: 20px; text-align: center; }\n"
  "    .value { font-size: 3em; font-weight: bold; margin: 10px 0; }\n"
  "    .unit { font-size: 1.2em; color: #888; }\n"
  "    .label { font-size: 1em; color: #aaa; margin-top: 10px; }\n"
  "    .voltage { color: #4CAF50; }\n"
  "    .current { color: #2196F3; }\n"
  "    .power { color: #FF9800; }\n"
  "    .energy { color: #9C27B0; }\n"
  "    button { background: #4CAF50; color: white; border: none; padding: 10px 20px; \n"
  "             border-radius: 5px; cursor: pointer; font-size: 16px; margin: 10px; }\n"
  "    button:hover { background: #45a049; }\n"
  "    .nav { text-align: center; margin: 20px 0; }\n"
  "    .ota-status { position: fixed; top: 10px; right: 10px; padding: 5px 10px; \n"
  "                  background: #333; border-radius: 5px; font-size: 12px; }\n"
  "  </style>\n"
  "</head>\n"
  "<body>\n"
  "  <div class='ota-status'>OTA Ready: 192.168.1.120</div>\n"
  "  <div class='container'>\n"
  "    <div class='header'>\n"
  "      <h1>Solar Panel Monitor</h1>\n"
  "      <div id='time'></div>\n"
  "    </div>\n"
  "    <div class='grid'>\n"
  "      <div class='tile'>\n"
  "        <div class='label'>VOLTAGE</div>\n"
  "        <div class='value voltage' id='voltage'>--</div>\n"
  "        <div class='unit'>V</div>\n"
  "      </div>\n"
  "      <div class='tile'>\n"
  "        <div class='label'>CURRENT</div>\n"
  "        <div class='value current' id='current'>--</div>\n"
  "        <div class='unit'>A</div>\n"
  "      </div>\n"
  "      <div class='tile'>\n"
  "        <div class='label'>POWER</div>\n"
  "        <div class='value power' id='power'>--</div>\n"
  "        <div class='unit'>W</div>\n"
  "      </div>\n"
  "      <div class='tile'>\n"
  "        <div class='label'>ENERGY</div>\n"
  "        <div class='value energy' id='energy'>--</div>\n"
  "        <div class='unit'>Wh</div>\n"
  "      </div>\n"
  "    </div>\n"
  "    <div class='nav'>\n"
  "      <button onclick=\"location.href='/cal/show'\">Calibration</button>\n"
  "      <button onclick=\"location.href='/diag'\">Diagnostics</button>\n"
  "    </div>\n"
  "  </div>\n"
  "  <script>\n"
  "    function update() {\n"
  "      fetch('/data').then(r => r.json()).then(d => {\n"
  "        document.getElementById('voltage').innerText = d.voltage.toFixed(2);\n"
  "        document.getElementById('current').innerText = d.current.toFixed(3);\n"
  "        document.getElementById('power').innerText = d.power.toFixed(2);\n"
  "        document.getElementById('energy').innerText = d.energy_Wh.toFixed(3);\n"
  "        document.getElementById('time').innerText = d.iso;\n"
  "      }).catch(e => console.log('Update error:', e));\n"
  "    }\n"
  "    setInterval(update, 1000);\n"
  "    update();\n"
  "  </script>\n"
  "</body>\n"
  "</html>";

//============================================
// I2C Functions
//============================================
static bool i2c_write16(uint8_t addr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  return (Wire.endTransmission() == 0);
}

static bool i2c_read16(uint8_t addr, uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  
  Wire.requestFrom(addr, (uint8_t)2);
  if (Wire.available() != 2) return false;
  
  value = Wire.read() << 8;
  value |= Wire.read();
  return true;
}

//============================================
// INA219 Functions - FIXED!
//============================================
bool INA219_init() {
  // Reset INA219
  if (!i2c_write16(INA219_ADDR, INA219_REG_CONFIG, INA219_CONFIG_RESET)) {
    Serial.println("Failed to reset INA219");
    return false;
  }
  delay(10);
  
  // Calculate calibration value
  current_lsb = MAX_CURRENT / 32768.0;
  ina219_calValue = (uint16_t)(0.04096 / (current_lsb * SHUNT_OHMS));
  
  // Write calibration register
  if (!i2c_write16(INA219_ADDR, INA219_REG_CALIBRATION, ina219_calValue)) {
    Serial.println("Failed to set calibration");
    return false;
  }
  
  // Configure INA219 for 32V, 320mV range, 12-bit, continuous mode
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  
  if (!i2c_write16(INA219_ADDR, INA219_REG_CONFIG, config)) {
    Serial.println("Failed to configure INA219");
    return false;
  }
  
  Serial.printf("INA219 initialized. Cal value: %u, Current LSB: %.9f\n", 
                ina219_calValue, current_lsb);
  return true;
}

// THIS IS THE CRITICAL FIX - INA219 voltage reading
static bool INA219_read_vbus_V(float &vbusV) {
  uint16_t raw;
  if (!i2c_read16(INA219_ADDR, INA219_REG_BUSVOLTAGE, raw)) return false;
  
  // INA219: voltage is in bits 15-3, LSB = 4mV
  // Must shift right by 3 bits!
  vbusV = ((raw >> 3) * 4.0) / 1000.0;  // Convert to volts
  vbusV *= CAL_V;  // Apply calibration
  return true;
}

// Read current from INA219
static bool INA219_read_current_A(float &currentA) {
  uint16_t raw;
  if (!i2c_read16(INA219_ADDR, INA219_REG_CURRENT, raw)) return false;
  
  // Convert to amps using current LSB
  currentA = (int16_t)raw * current_lsb;  // Cast to signed for negative currents
  currentA *= CAL_I;  // Apply calibration
  return true;
}

//============================================
// HTTP Handlers
//============================================
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, ROOT_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t data_get_handler(httpd_req_t *req) {
  float v = 0, i = 0, p = 0;
  
  // Read values from INA219 (using FIXED functions!)
  bool vOk = INA219_read_vbus_V(v);
  bool iOk = INA219_read_current_A(i);
  
  p = v * i;
  
  // Update energy
  unsigned long now_ms = millis();
  if (lastEnergyMs > 0 && p > 0) {
    double hours = (now_ms - lastEnergyMs) / 3600000.0;
    energy_Wh += p * hours;
  }
  lastEnergyMs = now_ms;
  
  // Get time
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char iso[32];
  strftime(iso, sizeof(iso), "%Y-%m-%d %H:%M:%S", &timeinfo);
  
  // Debug: read raw registers
  uint16_t rawBus = 0, rawConfig = 0, rawCurrent = 0, rawShunt = 0, rawCal = 0;
  bool busReadOk = i2c_read16(INA219_ADDR, INA219_REG_BUSVOLTAGE, rawBus);
  bool configReadOk = i2c_read16(INA219_ADDR, INA219_REG_CONFIG, rawConfig);
  i2c_read16(INA219_ADDR, INA219_REG_CURRENT, rawCurrent);
  i2c_read16(INA219_ADDR, INA219_REG_SHUNTVOLTAGE, rawShunt);
  i2c_read16(INA219_ADDR, INA219_REG_CALIBRATION, rawCal);
  
  // Check if devices exist
  Wire.beginTransmission(0x40);
  bool found40 = (Wire.endTransmission() == 0);
  Wire.beginTransmission(0x41);
  bool found41 = (Wire.endTransmission() == 0);
  
  // Calculate what library would show (for comparison)
  float libVoltage = ((rawBus >> 3) * 4.0) / 1000.0;
  
  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"epoch\":%ld,\"iso\":\"%s\",\"voltage\":%.3f,\"current\":%.3f,\"power\":%.3f,"
    "\"energy_Wh\":%.3f,\"cal_v\":%.3f,\"cal_i\":%.3f,"
    "\"debug\":{\"rawBus\":%u,\"rawShunt\":%d,\"rawCurrent\":%d,\"rawConfig\":\"%04X\","
    "\"rawCal\":%u,\"busReadOk\":%s,\"configReadOk\":%s,"
    "\"found_0x40\":%s,\"found_0x41\":%s,\"libVoltage\":%.3f,"
    "\"calValue\":%u,\"current_lsb\":%.9f,\"overflow\":%s}}",
    (long)now, iso, v, i, p, energy_Wh, CAL_V, CAL_I,
    rawBus, (int16_t)rawShunt, (int16_t)rawCurrent, rawConfig, rawCal,
    busReadOk?"true":"false", configReadOk?"true":"false", 
    found40?"true":"false", found41?"true":"false", libVoltage,
    ina219_calValue, current_lsb,
    (rawBus & 0x01)?"YES!":"No");
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
  
  // Log to SD
  File f = SD.open("/log.csv", FILE_APPEND);
  if (f) {
    f.print((long)now); f.print(',');
    f.print(iso);       f.print(',');
    f.print(v,3);       f.print(',');
    f.print(i,3);       f.print(',');
    f.println(p,3);
    f.close();
  }
  
  return ESP_OK;
}

static esp_err_t diag_get_handler(httpd_req_t *req) {
  uint16_t config = 0, shunt = 0, bus = 0, power = 0, current = 0, cal = 0;
  
  i2c_read16(INA219_ADDR, 0x00, config);
  i2c_read16(INA219_ADDR, 0x01, shunt);
  i2c_read16(INA219_ADDR, 0x02, bus);
  i2c_read16(INA219_ADDR, 0x03, power);
  i2c_read16(INA219_ADDR, 0x04, current);
  i2c_read16(INA219_ADDR, 0x05, cal);
  
  // Parse config bits
  bool is32V = (config & 0x2000) != 0;
  int gain = (config >> 11) & 0x03;
  int mode = config & 0x07;
  
  // Calculate actual values
  float busV = ((bus >> 3) * 4.0) / 1000.0;
  float shuntmV = (int16_t)shunt * 0.01;
  float currentA = (int16_t)current * current_lsb;
  
  char buf[2048];
  snprintf(buf, sizeof(buf),
    "<!DOCTYPE html><html><head><title>INA219 Diagnostics</title>"
    "<style>body{font-family:monospace;background:#1a1a1a;color:#0f0;padding:20px;}"
    "pre{background:#000;padding:20px;border:1px solid #0f0;border-radius:5px;}"
    "h2{color:#4CAF50;}.warn{color:#ff0;}.good{color:#0f0;}</style></head><body>"
    "<h2>INA219 Diagnostics</h2><pre>"
    "==== REGISTERS ====\n"
    "Config:     0x%04X\n"
    "  Bus Range:  %s %s\n"
    "  PGA Gain:   %d (%s)\n"
    "  Bus ADC:    %s\n"
    "  Shunt ADC:  %s\n"
    "  Mode:       %s\n"
    "Shunt:      0x%04X = %.3f mV\n"
    "Bus:        0x%04X = %.3f V %s\n"
    "Power:      0x%04X\n"
    "Current:    0x%04X = %.3f A\n"
    "Calibration: 0x%04X (expected: %u) %s\n\n"
    "==== SETTINGS ====\n"
    "Shunt Resistor: %.3f ohms\n"
    "Max Current:    %.1f A\n"
    "Current LSB:    %.9f A/bit\n"
    "Cal Factor V:   %.3f\n"
    "Cal Factor I:   %.3f\n\n"
    "==== STATUS ====\n"
    "I2C Address 0x40: %s\n"
    "I2C Address 0x41: %s\n"
    "Math Overflow:    %s\n\n"
    "==== OTA STATUS ====\n"
    "Hostname: %s\n"
    "IP Address: %s\n"
    "Password Protected: Yes\n"
    "</pre><p><a href='/'>Back to Dashboard</a></p></body></html>",
    config,
    is32V ? "32V" : "16V", is32V ? "<span class='good'>GOOD</span>" : "<span class='warn'>Should be 32V!</span>",
    gain, gain==3 ? "8 (320mV) GOOD" : "Wrong!",
    ((config >> 7) & 0x0F) == 0x0C ? "12-bit GOOD" : "Not 12-bit",
    ((config >> 3) & 0x0F) == 0x03 ? "12-bit GOOD" : "Not 12-bit",
    mode == 7 ? "Continuous GOOD" : "Not continuous!",
    shunt, shuntmV,
    bus, busV, (bus & 0x01) ? "<span class='warn'>OVERFLOW!</span>" : "",
    power,
    current, currentA,
    cal, ina219_calValue, (cal == ina219_calValue) ? "GOOD" : "MISMATCH!",
    SHUNT_OHMS, MAX_CURRENT, current_lsb,
    CAL_V, CAL_I,
    "Found GOOD", "Not found",
    (bus & 0x01) ? "YES! WARNING" : "No GOOD",
    OTA_HOSTNAME,
    WiFi.localIP().toString().c_str());
  
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t cal_show_handler(httpd_req_t *req) {
  char buf[512];
  snprintf(buf, sizeof(buf),
    "<!DOCTYPE html><html><head><title>Calibration</title>"
    "<style>body{font-family:Arial;background:#1a1a1a;color:#fff;padding:20px;}"
    "input{padding:5px;margin:5px;}</style></head><body>"
    "<h2>Calibration Settings</h2>"
    "<p>Current Calibration: V=%.3f, I=%.3f</p>"
    "<form action='/cal' method='get'>"
    "Voltage Cal: <input type='text' name='v' value='%.3f' step='0.001'><br>"
    "Current Cal: <input type='text' name='i' value='%.3f' step='0.001'><br>"
    "<input type='submit' value='Update Calibration'>"
    "</form>"
    "<p><a href='/'>Back to Dashboard</a></p>"
    "</body></html>",
    CAL_V, CAL_I, CAL_V, CAL_I);
  
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t cal_set_handler(httpd_req_t *req) {
  char buf[100];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char param[32];
    if (httpd_query_key_value(buf, "i", param, sizeof(param)) == ESP_OK) {
      CAL_I = atof(param);
    }
    if (httpd_query_key_value(buf, "v", param, sizeof(param)) == ESP_OK) {
      CAL_V = atof(param);
    }
    
    // Save to SD
    File f = SD.open("/cal.cfg", FILE_WRITE);
    if (f) {
      f.printf("%f\n%f\n", CAL_I, CAL_V);
      f.close();
      Serial.printf("Calibration saved: I=%f V=%f\n", CAL_I, CAL_V);
    }
  }
  
  httpd_resp_set_hdr(req, "Location", "/cal/show");
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

//============================================
// Setup and Loop
//============================================
void register_routes_common(httpd_handle_t server) {
  httpd_uri_t root_uri    = {.uri = "/",         .method = HTTP_GET, .handler = root_get_handler};
  httpd_uri_t data_uri    = {.uri = "/data",     .method = HTTP_GET, .handler = data_get_handler};
  httpd_uri_t cal_show    = {.uri = "/cal/show", .method = HTTP_GET, .handler = cal_show_handler};
  httpd_uri_t cal_set     = {.uri = "/cal",      .method = HTTP_GET, .handler = cal_set_handler};
  httpd_uri_t diag_uri    = {.uri = "/diag",     .method = HTTP_GET, .handler = diag_get_handler};
  
  httpd_register_uri_handler(server, &root_uri);
  httpd_register_uri_handler(server, &data_uri);
  httpd_register_uri_handler(server, &cal_show);
  httpd_register_uri_handler(server, &cal_set);
  httpd_register_uri_handler(server, &diag_uri);
}

void setupOTA() {
  // Set OTA hostname and password
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("OTA Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete!");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPercent = 0;
    unsigned int percent = (progress / (total / 100));
    if (percent != lastPercent) {
      Serial.printf("OTA Progress: %u%%\r", percent);
      lastPercent = percent;
    }
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Update Service Started");
  Serial.printf("  - Hostname: %s\n", OTA_HOSTNAME);
  Serial.printf("  - IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.println("  - Password Protected: Yes");
  Serial.println("  - To upload: Tools->Port->\"solar-logger at 192.168.x.x\"");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 Solar Logger (INA219 + OTA) ===");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100kHz
  Serial.println("I2C initialized");
  
  // Initialize SD Card
  SPI.begin();
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card mount failed!");
  } else {
    Serial.println("SD Card mounted");
    
    // Load calibration
    if (SD.exists("/cal.cfg")) {
      File f = SD.open("/cal.cfg", FILE_READ);
      if (f) {
        CAL_I = f.parseFloat();
        CAL_V = f.parseFloat();
        f.close();
        Serial.printf("Loaded calibration: I=%f V=%f\n", CAL_I, CAL_V);
      }
    }
    
    // Create CSV header if file doesn't exist
    if (!SD.exists("/log.csv")) {
      File f = SD.open("/log.csv", FILE_WRITE);
      if (f) {
        f.println("epoch,iso,voltage,current,power");
        f.close();
      }
    }
  }
  
  // Initialize INA219
  if (!INA219_init()) {
    Serial.println("INA219 initialization failed!");
  } else {
    Serial.println("INA219 initialized successfully");
  }
  
  // Connect WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Setup OTA
  setupOTA();
  
  // Setup time
  configTime(0, 0, NTP_SERVER);
  setenv("TZ", TZ_STRING, 1);
  tzset();
  
  // Start web server
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_len = 512;
  cfg.stack_size = 8192;
  
  if (httpd_start(&http_server, &cfg) == ESP_OK) {
    register_routes_common(http_server);
    Serial.print("HTTP server started. Open: http://");
    Serial.println(WiFi.localIP());
  }
  
  Serial.println("\n=== System Ready ===");
  Serial.println("Web Interface: http://" + WiFi.localIP().toString());
  Serial.println("OTA Updates: Available in Arduino IDE");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Small delay to prevent watchdog issues
  delay(10);
}
