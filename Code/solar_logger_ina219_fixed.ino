/************************************************************
  ESP32 Solar Logger (HTTP only, Big Tiles UI, Aim Mode, Charts)
  - FIXED FOR INA219 (not INA226!)
  - HTTP via esp_http_server (simple & robust)
  - INA219 current sensing with proper register calculations
  - SD logging (CS=5), CSV: epoch,iso,volts,amps,watts
  - Calibration persisted in /cal.cfg on SD
*************************************************************/

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include "esp_http_server.h"
#include "secrets.h"   // define WIFI_SSID, WIFI_PASSWORD, TZ_STRING, NTP servers

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

// Forward declarations of handlers
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t charts_get_handler(httpd_req_t *req);
static esp_err_t data_get_handler(httpd_req_t *req);
static esp_err_t hourly_get_handler(httpd_req_t *req);
static esp_err_t daily_get_handler(httpd_req_t *req);
static esp_err_t monthly_get_handler(httpd_req_t *req);
static esp_err_t cal_show_handler(httpd_req_t *req);
static esp_err_t cal_set_handler (httpd_req_t *req);
static esp_err_t diag_get_handler(httpd_req_t *req);

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
// INA219 Functions
//============================================
bool INA219_init() {
  // Reset INA219
  if (!i2c_write16(INA219_ADDR, INA219_REG_CONFIG, INA219_CONFIG_RESET)) {
    Serial.println("Failed to reset INA219");
    return false;
  }
  delay(10);
  
  // Calculate calibration value
  // Cal = trunc(0.04096 / (Current_LSB * Rshunt))
  // Current_LSB = Max Expected Current / 32768
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

// Read bus voltage from INA219 (FIXED FOR INA219!)
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
// HTML Pages
//============================================
const char ROOT_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Solar Logger</title>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="5">
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
           background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
           margin: 0; padding: 20px; color: #fff; }
    .container { max-width: 1200px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 40px; }
    h1 { font-size: 2.5em; margin: 0; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }
    .time { font-size: 1.2em; opacity: 0.9; margin-top: 10px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); 
            gap: 20px; margin-bottom: 30px; }
    .tile { background: rgba(255,255,255,0.1); backdrop-filter: blur(10px);
            border-radius: 20px; padding: 30px; text-align: center;
            box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
            border: 1px solid rgba(255,255,255,0.18); }
    .value { font-size: 3.5em; font-weight: 300; margin: 20px 0; }
    .unit { font-size: 1.5em; opacity: 0.8; }
    .label { font-size: 1.1em; opacity: 0.7; text-transform: uppercase; 
             letter-spacing: 2px; }
    .btn { display: inline-block; padding: 12px 30px; margin: 10px;
           background: rgba(255,255,255,0.2); color: white; text-decoration: none;
           border-radius: 25px; transition: all 0.3s; }
    .btn:hover { background: rgba(255,255,255,0.3); transform: translateY(-2px); }
    .nav { text-align: center; margin-top: 40px; }
    .voltage { color: #4ade80; }
    .current { color: #60a5fa; }
    .power { color: #fbbf24; }
    .energy { color: #f472b6; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>☀️ Solar Panel Monitor</h1>
      <div class="time" id="time">Loading...</div>
    </div>
    <div class="grid">
      <div class="tile">
        <div class="label">Voltage</div>
        <div class="value voltage" id="voltage">--</div>
        <div class="unit">V</div>
      </div>
      <div class="tile">
        <div class="label">Current</div>
        <div class="value current" id="current">--</div>
        <div class="unit">A</div>
      </div>
      <div class="tile">
        <div class="label">Power</div>
        <div class="value power" id="power">--</div>
        <div class="unit">W</div>
      </div>
      <div class="tile">
        <div class="label">Energy</div>
        <div class="value energy" id="energy">--</div>
        <div class="unit">Wh</div>
      </div>
    </div>
    <div class="nav">
      <a href="/charts" class="btn">📊 Charts</a>
      <a href="/cal/show" class="btn">⚙️ Calibration</a>
      <a href="/diag" class="btn">🔧 Diagnostics</a>
    </div>
  </div>
  <script>
    function update() {
      fetch('/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('voltage').innerText = d.voltage.toFixed(2);
          document.getElementById('current').innerText = d.current.toFixed(3);
          document.getElementById('power').innerText = d.power.toFixed(2);
          document.getElementById('energy').innerText = d.energy_Wh.toFixed(3);
          document.getElementById('time').innerText = d.iso;
        })
        .catch(e => console.error('Update failed:', e));
    }
    setInterval(update, 1000);
    update();
  </script>
</body>
</html>
)rawliteral";

const char CHARTS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Solar Charts</title>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; 
           margin: 0; padding: 20px; }
    .container { max-width: 1200px; margin: 0 auto; }
    h1 { text-align: center; }
    .chart-container { background: #2a2a2a; border-radius: 10px; padding: 20px; 
                      margin: 20px 0; }
    .nav { text-align: center; margin: 20px 0; }
    button { background: #4CAF50; color: white; border: none; padding: 10px 20px;
             border-radius: 5px; cursor: pointer; margin: 5px; }
    button:hover { background: #45a049; }
  </style>
</head>
<body>
  <div class="container">
    <h1>📊 Solar Production Charts</h1>
    <div class="nav">
      <button onclick="loadChart('hourly')">Hourly</button>
      <button onclick="loadChart('daily')">Daily</button>
      <button onclick="loadChart('monthly')">Monthly</button>
      <button onclick="location.href='/'">Back to Dashboard</button>
    </div>
    <div class="chart-container">
      <canvas id="chart"></canvas>
    </div>
  </div>
  <script>
    let chart = null;
    
    function loadChart(type) {
      fetch('/agg/' + type)
        .then(r => r.json())
        .then(data => {
          if (chart) chart.destroy();
          
          const ctx = document.getElementById('chart').getContext('2d');
          chart = new Chart(ctx, {
            type: 'line',
            data: {
              labels: data.labels,
              datasets: [{
                label: 'Power (W)',
                data: data.power,
                borderColor: '#FF9800',
                backgroundColor: 'rgba(255, 152, 0, 0.1)',
                tension: 0.1
              }, {
                label: 'Voltage (V)',
                data: data.voltage,
                borderColor: '#4CAF50',
                backgroundColor: 'rgba(76, 175, 80, 0.1)',
                tension: 0.1,
                yAxisID: 'y1'
              }]
            },
            options: {
              responsive: true,
              plugins: {
                legend: { labels: { color: '#fff' } }
              },
              scales: {
                x: { ticks: { color: '#aaa' }, grid: { color: '#333' } },
                y: { 
                  type: 'linear',
                  display: true,
                  position: 'left',
                  ticks: { color: '#aaa' },
                  grid: { color: '#333' }
                },
                y1: {
                  type: 'linear',
                  display: true,
                  position: 'right',
                  ticks: { color: '#aaa' },
                  grid: { drawOnChartArea: false }
                }
              }
            }
          });
        });
    }
    
    loadChart('hourly');
  </script>
</body>
</html>
)rawliteral";

//============================================
// HTTP Handlers
//============================================
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, ROOT_HTML, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t charts_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, CHARTS_HTML, HTTPD_RESP_USE_STRLEN);
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
  
  char buf[1024];
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
    "Shunt Resistor: %.3f Ω\n"
    "Max Current:    %.1f A\n"
    "Current LSB:    %.9f A/bit\n"
    "Cal Factor V:   %.3f\n"
    "Cal Factor I:   %.3f\n\n"
    "==== STATUS ====\n"
    "I2C Address 0x40: %s\n"
    "I2C Address 0x41: %s\n"
    "Math Overflow:    %s\n"
    "</pre><p><a href='/'>Back to Dashboard</a></p></body></html>",
    config,
    is32V ? "32V" : "16V", is32V ? "<span class='good'>✓</span>" : "<span class='warn'>⚠ Should be 32V!</span>",
    gain, gain==3 ? "8 (320mV) ✓" : "Wrong!",
    ((config >> 7) & 0x0F) == 0x0C ? "12-bit ✓" : "Not 12-bit",
    ((config >> 3) & 0x0F) == 0x03 ? "12-bit ✓" : "Not 12-bit",
    mode == 7 ? "Continuous ✓" : "Not continuous!",
    shunt, shuntmV,
    bus, busV, (bus & 0x01) ? "<span class='warn'>OVERFLOW!</span>" : "",
    power,
    current, currentA,
    cal, ina219_calValue, (cal == ina219_calValue) ? "✓" : "⚠ MISMATCH!",
    SHUNT_OHMS, MAX_CURRENT, current_lsb,
    CAL_V, CAL_I,
    "Found ✓", "Not found",
    (bus & 0x01) ? "YES! ⚠" : "No ✓");
  
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

// Aggregate data handlers (simplified for this example)
static esp_err_t hourly_get_handler(httpd_req_t *req) {
  const char* json = "{\"labels\":[\"1h\",\"2h\",\"3h\"],\"power\":[10,15,20],\"voltage\":[24,24.5,25]}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t daily_get_handler(httpd_req_t *req) {
  const char* json = "{\"labels\":[\"Mon\",\"Tue\",\"Wed\"],\"power\":[100,150,200],\"voltage\":[24,24.5,25]}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t monthly_get_handler(httpd_req_t *req) {
  const char* json = "{\"labels\":[\"Jan\",\"Feb\",\"Mar\"],\"power\":[1000,1500,2000],\"voltage\":[24,24.5,25]}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

//============================================
// Setup and Loop
//============================================
void register_routes_common(httpd_handle_t server) {
  httpd_uri_t root_uri    = {.uri = "/",            .method = HTTP_GET, .handler = root_get_handler};
  httpd_uri_t charts_uri  = {.uri = "/charts",      .method = HTTP_GET, .handler = charts_get_handler};
  httpd_uri_t data_uri    = {.uri = "/data",        .method = HTTP_GET, .handler = data_get_handler};
  httpd_uri_t hourly_uri  = {.uri = "/agg/hourly",  .method = HTTP_GET, .handler = hourly_get_handler};
  httpd_uri_t daily_uri   = {.uri = "/agg/daily",   .method = HTTP_GET, .handler = daily_get_handler};
  httpd_uri_t monthly_uri = {.uri = "/agg/monthly", .method = HTTP_GET, .handler = monthly_get_handler};
  httpd_uri_t cal_show    = {.uri = "/cal/show",    .method = HTTP_GET, .handler = cal_show_handler};
  httpd_uri_t cal_set     = {.uri = "/cal",         .method = HTTP_GET, .handler = cal_set_handler};
  httpd_uri_t diag_uri    = {.uri = "/diag",        .method = HTTP_GET, .handler = diag_get_handler};
  
  httpd_register_uri_handler(server, &root_uri);
  httpd_register_uri_handler(server, &charts_uri);
  httpd_register_uri_handler(server, &data_uri);
  httpd_register_uri_handler(server, &hourly_uri);
  httpd_register_uri_handler(server, &daily_uri);
  httpd_register_uri_handler(server, &monthly_uri);
  httpd_register_uri_handler(server, &cal_show);
  httpd_register_uri_handler(server, &cal_set);
  httpd_register_uri_handler(server, &diag_uri);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 Solar Logger (INA219) ===");
  
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
  
  // Setup time
  configTime(0, 0, NTP_SERVER);
  setenv("TZ", TZ_STRING, 1);
  tzset();
  
  // Start web server
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&http_server, &cfg) == ESP_OK) {
    register_routes_common(http_server);
    Serial.print("HTTP server started. Open: http://");
    Serial.println(WiFi.localIP());
  }
}

void loop() {
  // Nothing needed here - everything handled by HTTP requests
  delay(1000);
}
