/************************************************************
  ESP32 Solar Logger (HTTP only, Big Tiles UI, Aim Mode, Charts)
  - HTTP via esp_http_server (simple & robust)
  - INA226 shunt sensing with RobTillaart INA226 library
  - SD logging (CS=5), CSV: epoch,iso,volts,amps,watts
  - Calibration persisted in /cal.cfg on SD
  - Endpoints:
      /            -> live dashboard (big tiles + Aim Mode)
      /charts      -> charts page (Hourly/Daily/Monthly)
      /data        -> live JSON + appends to /log.csv
      /agg/hourly  -> today average power by hour (W)
      /agg/daily   -> 30-day energy (Wh)
      /agg/monthly -> 12-month energy (Wh)
      /cal/show    -> current calibration
      /cal?i=&v=   -> set calibration
      /diag        -> INA226 raw values
      /debug       -> simple debug page
*************************************************************/

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_INA219.h>  // Adafruit INA219 library
#include <time.h>
#include <string.h>
#include "esp_http_server.h"   // HTTP server

#include "secrets.h"   // Wi-Fi, TZ/NTP. Example:
// #define WIFI_SSID     "your-ssid"
// #define WIFI_PASSWORD "your-pass"
// #define TZ_STRING     "CST6CDT,M3.2.0,M11.1.0"
// #define NTP_SERVER_1  "pool.ntp.org"
// #define NTP_SERVER_2  "time.nist.gov"

// ====== HARDWARE CONFIG ======
#define SHUNT_OHMS   0.0015f   // NOYITO 50A/75mV => 0.0015 Ω
#define MAX_CURRENT  15.0f     // for ina226.setMaxCurrentShunt

#define I2C_SDA 21
#define I2C_SCL 22
// INA219 uses 0x40 address by default
Adafruit_INA219 ina219;  // Adafruit INA219 object

// SD (VSPI)
#define SD_CS 5

// State
static double         energy_Wh     = 0.0;
static unsigned long  lastEnergyMs  = 0;
static httpd_handle_t http_server   = nullptr;

// Calibration (persisted to SD)
static float CAL_I = 1.0f;   // multiply current
static float CAL_V = 1.0f;   // multiply voltage

// Build banner
static const char* BUILD_ID = __DATE__ " " __TIME__;

// ===== Low-level INA226 register read helpers =====
static bool i2c_read16(uint8_t addr, uint8_t reg, uint16_t &val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  val = ((uint16_t)hi << 8) | lo;
  return true;
}

// VBUS register 0x02: 1.25 mV / bit, max ~36.7 V
static bool INA219_read_vbus_V(float &vbusV){
  uint16_t raw;
  if(!i2c_read16(0x40, 0x02, raw)) return false;
  vbusV = (float)raw * 0.00125f;
  return true;
}

// SHUNT register 0x01: 2.5 µV / bit (signed)
static bool INA219_read_vshunt_mV(float &vshunt_mV){
  uint16_t rawU;
  if(!i2c_read16(0x40, 0x01, rawU)) return false;
  int16_t raw = (int16_t)rawU; // signed
  vshunt_mV = (float)raw * 0.0025f;
  return true;
}

static bool INA219_read_current_A(float &amps){
  float vshunt_mV;
  if(!INA219_read_vshunt_mV(vshunt_mV)) return false;
  amps = (vshunt_mV / 1000.0f) / SHUNT_OHMS;
  return true;
}

// ======== MAIN PAGE (Big Tiles + Aim Mode) ========
static const char PAGE_MAIN[] PROGMEM = R"PAGE_MAIN(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Solar Logger</title>
<style>
:root{--bg:#0b1020;--card:#121b33;--grid:#1b2746;--text:#eef3ff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif}
.wrap{max-width:1060px;margin:0 auto;padding:18px}
.header{display:flex;gap:12px;align-items:center;justify-content:space-between;margin-bottom:14px}
.title{font-size:20px;font-weight:700}
.btn{border:1px solid #2e3b64;background:#172349;color:#dbe7ff;border-radius:12px;padding:10px 14px;cursor:pointer;text-decoration:none}
.btn:active{transform:translateY(1px)}
.grid{display:grid;grid-template-columns:repeat(4,minmax(180px,1fr));gap:14px}
@media(max-width:980px){.grid{grid-template-columns:repeat(2,1fr)}}
.tile{background:#121b33;border:1px solid #22305a;border-radius:16px;padding:18px;box-shadow:0 6px 16px rgba(0,0,0,.18)}
.label{font-size:14px;color:#bcd0ff;margin-bottom:8px}
.value{font-size:42px;font-weight:800;letter-spacing:0.5px}
.unit{font-weight:600;font-size:20px;margin-left:6px;opacity:.85}
.meta{margin-top:8px;font-size:12px;color:#a8b7e6}
.aim{margin-top:18px;background:#121b33;border:1px solid #22305a;border-radius:16px;padding:16px}
.aim h2{margin:0 0 10px 0;font-size:16px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
.badge{font-size:12px;background:#1f2b4d;color:#bcd0ff;border:1px solid #2a3964;padding:4px 8px;border-radius:999px}
.footer{margin-top:20px;display:flex;justify-content:space-between;gap:8px;align-items:center}
.small{font-size:12px;color:#9fb4ff;opacity:.9}
hr{border:none;border-top:1px solid #24345e;margin:16px 0}
.volt  {background:linear-gradient(180deg,#1c2a52,#121b33)}
.amp   {background:linear-gradient(180deg,#1d2e52,#121b33)}
.watt  {background:linear-gradient(180deg,#153d3a,#121b33)}
.energy{background:linear-gradient(180deg,#40300f,#121b33)}
</style></head><body>
<div class="wrap">
  <div class="header">
    <div class="title">Solar Logger</div>
    <div class="row"><a class="btn" href="/charts">Open Charts</a></div>
  </div>

  <div id="timestamp" class="small">—</div>

  <div class="grid">
    <div class="tile volt">
      <div class="label">Voltage</div>
      <div class="value"><span id="v">0.00</span><span class="unit">V</span></div>
      <div class="meta">VIN− (load-side +) vs GND</div>
    </div>
    <div class="tile amp">
      <div class="label">Current</div>
      <div class="value"><span id="i">0.00</span><span class="unit">A</span></div>
      <div class="meta">From shunt drop (INA226)</div>
    </div>
    <div class="tile watt">
      <div class="label">Power</div>
      <div class="value"><span id="p">0.00</span><span class="unit">W</span></div>
      <div class="meta">P = V × I (calibrated)</div>
    </div>
    <div class="tile energy">
      <div class="label">Energy</div>
      <div class="value"><span id="e">0.00</span><span class="unit">Wh</span></div>
      <div class="meta">Integrated since boot</div>
    </div>
  </div>

  <hr>

  <div class="aim">
    <h2>Aim Mode (Audio)</h2>
    <div class="row">
      <button class="btn" id="aimBtn">Enable Aim Mode</button>
      <button class="btn" id="muteBtn">Mute</button>
      <span class="badge" id="aimState">Audio: off</span>
      <span class="badge" id="peak">Peak: 0.0 W</span>
    </div>
    <div class="small" style="margin-top:8px">
      Tap "Enable Aim Mode" first (browser gesture required). Pitch rises with watts; tune for the highest pitch.
    </div>
  </div>

  <div class="footer small">
    <div>HTTP served locally — build: %%BUILD%%</div>
    <div><a class="btn" href="/cal/show">Cal</a> <a class="btn" href="/diag">Diag</a></div>
  </div>
</div>

<script>
let audioCtx=null, osc=null, gainNode=null, aiming=false, muted=false;
let maxW=0.0;

function ensureAudio(){
  if(audioCtx) return true;
  try{
    audioCtx=new (window.AudioContext||window.webkitAudioContext)();
    gainNode=audioCtx.createGain(); gainNode.gain.value=0.0;
    osc=audioCtx.createOscillator(); osc.type="sine"; osc.frequency.value=220;
    osc.connect(gainNode).connect(audioCtx.destination); osc.start();
    return true;
  }catch(e){ return false; }
}

function setToneFromWatts(w){
  if(!aiming || !audioCtx) return;
  if(w>maxW) maxW=w;
  const n=Math.max(0,Math.min(1, w/(maxW||1)));
  const f=220 + Math.pow(n,0.6)*1000;
  const g=muted?0:(0.12 + 0.04*n);
  osc.frequency.setTargetAtTime(f, audioCtx.currentTime, 0.02);
  gainNode.gain.setTargetAtTime(g, audioCtx.currentTime, 0.03);
  document.getElementById('peak').textContent = 'Peak: ' + maxW.toFixed(1) + ' W';
}

document.getElementById('aimBtn').onclick=async ()=>{
  if(!audioCtx){
    if(!ensureAudio()){ alert('Web Audio not available'); return; }
    if(audioCtx.state==='suspended'){ try{ await audioCtx.resume(); }catch(e){} }
  }
  aiming=!aiming;
  if(!aiming && gainNode){ gainNode.gain.value=0.0; }
  document.getElementById('aimBtn').textContent = aiming?'Disable Aim Mode':'Enable Aim Mode';
  document.getElementById('aimState').textContent = 'Audio: ' + (aiming?'on':'off');
};
document.getElementById('muteBtn').onclick=()=>{ muted=!muted; document.getElementById('muteBtn').textContent=muted?'Unmute':'Mute'; };

async function poll(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const j=await r.json();
    const v=j.voltage||0, a=j.current||0, w=j.power||0, e=j.energy_Wh||0, t=j.iso||'';
    document.getElementById('timestamp').textContent = t;
    document.getElementById('v').textContent = v.toFixed(2);
    document.getElementById('i').textContent = a.toFixed(2);
    document.getElementById('p').textContent = w.toFixed(2);
    document.getElementById('e').textContent = e.toFixed(2);
    setToneFromWatts(w);
  }catch(e){}
  setTimeout(poll, 500);
}
poll();
</script>
</body></html>
)PAGE_MAIN";

// ======== CHARTS PAGE ========
static const char PAGE_CHARTS[] PROGMEM = R"PAGE_CHARTS(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Solar Charts</title>
<style>
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:0;background:#0b1020;color:#eef3ff}
.wrap{max-width:1080px;margin:0 auto;padding:18px}
a.btn{display:inline-block;border:1px solid #2e3b64;background:#172349;color:#dbe7ff;border-radius:12px;padding:10px 14px;text-decoration:none}
.card{margin-top:14px;background:#121b33;border:1px solid #22305a;border-radius:16px;padding:14px}
h1{font-size:20px;margin:0 0 10px 0}
.tabs{display:flex;gap:8px;margin:8px 0}
.tab{padding:6px 10px;border:1px solid #2a3964;border-radius:10px;cursor:pointer;background:#172349}
.tab.active{background:#1e2d53}
canvas{width:100%;height:300px;background:#0f1730;border:1px solid #23335e;border-radius:10px}
.legend{margin-top:6px;color:#bcd0ff;font-size:12px}
</style></head><body>
<div class="wrap">
  <div style="display:flex;justify-content:space-between;align-items:center">
    <h1>Charts</h1>
    <a class="btn" href="/">Back to Live</a>
  </div>
  <div class="card">
    <div class="tabs">
      <div class="tab active" data-k="hourly">Hourly (today)</div>
      <div class="tab" data-k="daily">Daily (30d)</div>
      <div class="tab" data-k="monthly">Monthly (12m)</div>
    </div>
    <canvas id="chart"></canvas>
    <div class="legend" id="legendText">Hourly avg power (W)</div>
  </div>
</div>
<script>
const cvs=document.getElementById('chart'), ctx=cvs.getContext('2d');
function drawAxes(pad, yMax, yLabel){
  const W=cvs.clientWidth, H=cvs.clientHeight;
  cvs.width=W*devicePixelRatio; cvs.height=H*devicePixelRatio; ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle="#2b3b67"; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(pad,10); ctx.lineTo(pad,H-pad); ctx.lineTo(W-10,H-pad); ctx.stroke();
  ctx.fillStyle="#9fb4ff"; ctx.font="12px system-ui"; ctx.fillText(yLabel,10,18);
  for(let i=0;i<=5;i++){
    const y=H-pad - (H-2*pad)*i/5; const v=(yMax*i/5).toFixed(0);
    ctx.fillText(v,W-pad+4,y+4);
    ctx.strokeStyle="#1f2b4d"; ctx.beginPath(); ctx.moveTo(pad,y); ctx.lineTo(W-10,y); ctx.stroke();
  }
}
function drawBars(labels, values, color, yLabel){
  const W=cvs.clientWidth, H=cvs.clientHeight, pad=40;
  const yMax=Math.max(1, Math.max(...values)*1.15);
  drawAxes(pad,yMax,yLabel);
  const n=values.length, bw=(W- pad - 20)/n;
  ctx.fillStyle=color;
  for(let i=0;i<n;i++){
    const x=pad + i*bw + 2;
    const h=(H-2*pad)*(values[i]/yMax);
    ctx.fillRect(x, H-pad-h, Math.max(2,bw-4), h);
  }
  ctx.fillStyle="#bcd0ff"; ctx.font="11px system-ui";
  const step=Math.ceil(n/12);
  for(let i=0;i<n;i+=step){
    const x=pad + i*bw + bw/2; ctx.save(); ctx.translate(x, H-pad+12); ctx.rotate(-0.6); ctx.fillText(labels[i],0,0); ctx.restore();
  }
}
async function loadAgg(kind){
  const res=await fetch('/agg/'+kind,{cache:'no-store'}); const j=await res.json();
  const lbl=j.labels||[], val=j.values||[], unit=j.unit||"", title=j.title||"";
  document.getElementById('legendText').textContent=title;
  drawBars(lbl,val,"#3aa2ff", unit);
}
document.querySelectorAll('.tab').forEach(el=>{
  el.onclick=()=>{ document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active')); el.classList.add('active'); loadAgg(el.dataset.k); };
});
loadAgg('hourly');
</script>
</body></html>
)PAGE_CHARTS";

// ===== Helpers =====
static String isoTime(time_t ts) {
  struct tm tmLocal; localtime_r(&ts, &tmLocal);
  char buf[32]; strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmLocal);
  return String(buf);
}
static void ensureCsvHeader() {
  if (!SD.exists("/log.csv")) { File f = SD.open("/log.csv", FILE_WRITE); if (f){ f.println("epoch,iso,volts,amps,watts"); f.close(); } }
}
static void sendJson(httpd_req_t *req, const String &s) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, s.c_str(), HTTPD_RESP_USE_STRLEN);
}
static void sendJsonText(httpd_req_t *req, const char* s){
  httpd_resp_set_type(req,"application/json");
  httpd_resp_set_hdr(req,"Cache-Control","no-store");
  httpd_resp_send(req,s,HTTPD_RESP_USE_STRLEN);
}

// ===== HTTP Handlers =====
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  String page = PAGE_MAIN;
  page.replace("%%BUILD%%", BUILD_ID);
  httpd_resp_send(req, page.c_str(), page.length());
  return ESP_OK;
}
static esp_err_t charts_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, PAGE_CHARTS, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// live JSON + logging
static esp_err_t data_get_handler(httpd_req_t *req) {
  float v_raw=0, i_raw=0;

  if(!INA219_read_vbus_V(v_raw)){
    v_raw = ina219.getBusVoltage_V(); // Adafruit method
  }
  if(!INA219_read_current_A(i_raw)){
    i_raw = ina219.getCurrent_mA() / 1000.0f; // Adafruit method
  }

  float v = v_raw * CAL_V;
  float i = i_raw * CAL_I;
  float p = v * i;

  unsigned long nowMs = millis();
  if (lastEnergyMs == 0) lastEnergyMs = nowMs;
  float dt_h = (nowMs - lastEnergyMs) / 3600000.0f;
  lastEnergyMs = nowMs;
  energy_Wh += (p * dt_h);

  time_t now = time(nullptr);
  String iso = isoTime(now);

  // Debug info
  uint16_t rawBus=0, rawConfig=0;
  bool busOk = i2c_read16(0x40, 0x02, rawBus);
  bool configOk = i2c_read16(0x40, 0x00, rawConfig);
  
  // I2C device scan
  bool found40 = false, found41 = false;
  Wire.beginTransmission(0x40); if(Wire.endTransmission() == 0) found40 = true;
  Wire.beginTransmission(0x41); if(Wire.endTransmission() == 0) found41 = true;

  char buf[600];
  snprintf(buf, sizeof(buf),
    "{\"epoch\":%ld,\"iso\":\"%s\",\"voltage\":%.3f,\"current\":%.3f,"
    "\"power\":%.3f,\"energy_Wh\":%.3f,\"cal_v\":%.3f,\"cal_i\":%.3f,"
    "\"debug\":{\"rawBus\":%u,\"rawConfig\":\"%04X\","
    "\"busReadOk\":%s,\"configReadOk\":%s,"
    "\"found_0x40\":%s,\"found_0x41\":%s,\"libVoltage\":%.3f}}",
    (long)now, iso.c_str(), v, i, p, energy_Wh, CAL_V, CAL_I,
    (unsigned)rawBus, (unsigned)rawConfig,
    busOk?"true":"false", configOk?"true":"false", 
    found40?"true":"false", found41?"true":"false", ina219.getBusVoltage_V()); // Adafruit method

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);

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

// CSV parse helper
static bool parseLogLine(const String &line, time_t &ep, float &w) {
  int a = line.indexOf(','); if (a<0) return false;
  int b = line.indexOf(',', a+1); if (b<0) return false;
  int c = line.indexOf(',', b+1); if (c<0) return false;
  int d = line.indexOf(',', c+1); if (d<0) return false;
  String sEpoch = line.substring(0, a);
  String sWatts = line.substring(d+1);
  ep = (time_t)sEpoch.toInt();
  w  = sWatts.toFloat();
  return true;
}

// Aggregations
static esp_err_t hourly_get_handler(httpd_req_t *req) {
  float sum[24]={0}; uint32_t cnt[24]={0};
  time_t now=time(nullptr); struct tm tml; localtime_r(&now,&tml);
  tml.tm_hour=0; tml.tm_min=0; tml.tm_sec=0;
  time_t startDay=mktime(&tml), endDay=startDay+24*3600;

  File f=SD.open("/log.csv", FILE_READ);
  if (f) {
    while (f.available()) {
      String line=f.readStringUntil('\n');
      if (line.length()<10 || line.startsWith("epoch")) continue;
      time_t ep; float w; if (!parseLogLine(line,ep,w)) continue;
      if (ep<startDay || ep>=endDay) continue;
      struct tm t; localtime_r(&ep,&t);
      sum[t.tm_hour]+=w; cnt[t.tm_hour]++;
    }
    f.close();
  }
  String labels="[", values="[";
  for(int h=0;h<24;h++){
    if(h){labels+=",";values+=",";}
    char lb[8]; snprintf(lb,sizeof(lb),"%02d",h);
    labels += "\""; labels += lb; labels += "\"";
    values += String(cnt[h]? (sum[h]/cnt[h]) : 0.0f, 1);
  }
  labels += "]"; values += "]";
  String json = "{\"title\":\"Hourly avg power (W)\",\"unit\":\"W\",\"labels\":"+labels+",\"values\":"+values+"}";
  sendJson(req, json); return ESP_OK;
}

static esp_err_t daily_get_handler(httpd_req_t *req) {
  const int DAYS=30; double wh[DAYS]; for(int i=0;i<DAYS;i++) wh[i]=0.0;
  time_t now=time(nullptr); struct tm tml; localtime_r(&now,&tml);
  tml.tm_hour=0; tml.tm_min=0; tml.tm_sec=0; time_t today0=mktime(&tml);
  File f=SD.open("/log.csv", FILE_READ);
  if (f) {
    time_t epPrev=0; float wPrev=0;
    while (f.available()) {
      String line=f.readStringUntil('\n');
      if (line.length()<10 || line.startsWith("epoch")) continue;
      time_t ep; float w; if(!parseLogLine(line,ep,w)) continue;
      if (epPrev>0 && ep>epPrev) {
        double dt_h = (double)(ep-epPrev)/3600.0;
        double pw = 0.5*(wPrev + w);
        struct tm ta; localtime_r(&epPrev,&ta);
        ta.tm_hour=0; ta.tm_min=0; ta.tm_sec=0; time_t dayStartA=mktime(&ta);
        int idx=(int)((today0 - dayStartA)/86400);
        if (idx>=0 && idx<DAYS) wh[DAYS-1-idx]+=pw*dt_h;
      }
      epPrev=ep; wPrev=w;
    }
    f.close();
  }
  String labels="[", values="[";
  for(int i=0;i<DAYS;i++){
    time_t day = now - (time_t)(DAYS-1-i)*86400;
    struct tm t; localtime_r(&day,&t);
    char lb[12]; strftime(lb,sizeof(lb),"%m-%d",&t);
    if(i){labels+=",";values+=",";}
    labels += "\""; labels += lb; labels += "\"";
    values += String(wh[i],1);
  }
  labels += "]"; values += "]";
  String json = "{\"title\":\"Daily energy (Wh, 30d)\",\"unit\":\"Wh\",\"labels\":"+labels+",\"values\":"+values+"}";
  sendJson(req, json); return ESP_OK;
}

static esp_err_t monthly_get_handler(httpd_req_t *req) {
  const int MOS=12; double wh[MOS]; for(int i=0;i<MOS;i++) wh[i]=0.0;
  time_t now=time(nullptr); struct tm tmNow; localtime_r(&now,&tmNow);
  time_t monthStart[MOS+1];
  { struct tm t=tmNow; t.tm_mday=1; t.tm_hour=0; t.tm_min=0; t.tm_sec=0;
    for(int i=0;i<MOS;i++){ struct tm tt=t; tt.tm_mon-=(MOS-1-i); mktime(&tt); monthStart[i]=mktime(&tt); }
    struct tm tnext=tmNow; tnext.tm_mday=1; tnext.tm_hour=0; tnext.tm_min=0; tnext.tm_sec=0; tnext.tm_mon+=1; monthStart[MOS]=mktime(&tnext);
  }
  File f=SD.open("/log.csv", FILE_READ);
  if (f) {
    time_t epPrev=0; float wPrev=0;
    while (f.available()) {
      String line=f.readStringUntil('\n');
      if (line.length()<10 || line.startsWith("epoch")) continue;
      time_t ep; float w; if(!parseLogLine(line,ep,w)) continue;
      if (epPrev>0 && ep>epPrev) {
        double dt_h = (double)(ep-epPrev)/3600.0;
        double pw = 0.5*(wPrev + w);
        struct tm ta; localtime_r(&epPrev,&ta);
        struct tm tmo=ta; tmo.tm_mday=1; tmo.tm_hour=0; tmo.tm_min=0; tmo.tm_sec=0;
        time_t ms=mktime(&tmo);
        for(int k=0;k<MOS;k++){ if(ms==monthStart[k]){ wh[k]+=pw*dt_h; break; } }
      }
      epPrev=ep; wPrev=w;
    }
    f.close();
  }
  String labels="[", values="[";
  for(int i=0;i<MOS;i++){
    struct tm t; localtime_r(&monthStart[i],&t);
    char lb[12]; strftime(lb,sizeof(lb),"%Y-%m",&t);
    if(i){labels+=",";values+=",";}
    labels += "\""; labels += lb; labels += "\"";
    values += String(wh[i],0);
  }
  labels += "]"; values += "]";
  String json = "{\"title\":\"Monthly energy (Wh, 12m)\",\"unit\":\"Wh\",\"labels\":"+labels+",\"values\":"+values+"}";
  sendJson(req, json); return ESP_OK;
}

// Calibration file I/O
static void saveCal(){ File f=SD.open("/cal.cfg", FILE_WRITE); if(f){ f.printf("CAL_V=%.6f\nCAL_I=%.6f\n",CAL_V,CAL_I); f.close(); } }
static void loadCal(){ File f=SD.open("/cal.cfg", FILE_READ); if(!f) return; while(f.available()){ String line=f.readStringUntil('\n'); line.trim(); if(line.startsWith("CAL_V=")) CAL_V=line.substring(6).toFloat(); else if(line.startsWith("CAL_I=")) CAL_I=line.substring(6).toFloat(); } f.close(); }
static float readQueryParamFloat(httpd_req_t *req, const char* key, bool &present){
  present=false; size_t qlen=httpd_req_get_url_query_len(req)+1; if(qlen<=1) return 0.0f;
  char *qry=(char*)malloc(qlen); if(!qry) return 0.0f;
  if(httpd_req_get_url_query_str(req,qry,qlen)==ESP_OK){ char val[32]; if(httpd_query_key_value(qry,key,val,sizeof(val))==ESP_OK){ present=true; float f=atof(val); free(qry); return f; } }
  free(qry); return 0.0f;
}
static esp_err_t cal_show_handler(httpd_req_t *req){ char buf[96]; snprintf(buf,sizeof(buf),"{\"cal_v\":%.6f,\"cal_i\":%.6f}",CAL_V,CAL_I); sendJsonText(req,buf); return ESP_OK; }
static esp_err_t cal_set_handler (httpd_req_t *req){
  bool hasI=false, hasV=false; float i=readQueryParamFloat(req,"i",hasI); float v=readQueryParamFloat(req,"v",hasV);
  if(hasI) CAL_I=i; if(hasV) CAL_V=v; saveCal();
  char buf[96]; snprintf(buf,sizeof(buf),"{\"ok\":true,\"cal_v\":%.6f,\"cal_i\":%.6f}",CAL_V,CAL_I); sendJsonText(req,buf); return ESP_OK;
}

static esp_err_t debug_get_handler(httpd_req_t *req){
  uint16_t raw=0;
  bool success = i2c_read16(0x40, 0x02, raw);
  float calc = raw * 0.00125f;
  float lib = ina219.getBusVoltage_V(); // Adafruit method
  
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send_chunk(req, "<html><body><h1>Debug</h1>", -1);
  
  char buf[200];
  snprintf(buf, sizeof(buf), "<p>I2C Read: %s</p>", success ? "SUCCESS" : "FAILED");
  httpd_resp_send_chunk(req, buf, -1);
  
  snprintf(buf, sizeof(buf), "<p>Raw Register: 0x%04X (%u)</p>", raw, raw);
  httpd_resp_send_chunk(req, buf, -1);
  
  snprintf(buf, sizeof(buf), "<p>Calculated: %.3fV</p>", calc);
  httpd_resp_send_chunk(req, buf, -1);
  
  snprintf(buf, sizeof(buf), "<p>Library: %.3fV</p>", lib);
  httpd_resp_send_chunk(req, buf, -1);
  
  httpd_resp_send_chunk(req, "<p>Expected: 24V</p><p><a href='/'>Back</a></p></body></html>", -1);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t diag_get_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send_chunk(req, "<html><body><h1>INA226 Diagnostics</h1>", -1);
  
  // I2C scan
  httpd_resp_send_chunk(req, "<h2>I2C Devices:</h2>", -1);
  bool foundAny = false;
  for(byte addr = 0x40; addr <= 0x41; addr++) {
    Wire.beginTransmission(addr);
    if(Wire.endTransmission() == 0) {
      char buf[100];
      snprintf(buf, sizeof(buf), "<p>Found at 0x%02X</p>", addr);
      httpd_resp_send_chunk(req, buf, -1);
      foundAny = true;
    }
  }
  if(!foundAny) httpd_resp_send_chunk(req, "<p>No devices found at 0x40-0x41</p>", -1);
  
  // Key registers
  httpd_resp_send_chunk(req, "<h2>Key Registers:</h2>", -1);
  uint16_t config=0, bus=0, shunt=0;
  bool configOk = i2c_read16(0x40, 0x00, config);
  bool busOk = i2c_read16(0x40, 0x02, bus);  
  bool shuntOk = i2c_read16(0x40, 0x01, shunt);
  
  char buf[200];
  snprintf(buf, sizeof(buf), "<p>Config: 0x%04X (%s)</p>", config, configOk ? "OK" : "FAIL");
  httpd_resp_send_chunk(req, buf, -1);
  
  snprintf(buf, sizeof(buf), "<p>Bus: 0x%04X = %.3fV (%s)</p>", bus, bus*0.00125f, busOk ? "OK" : "FAIL");
  httpd_resp_send_chunk(req, buf, -1);
  
  int16_t shuntSigned = (int16_t)shunt;
  snprintf(buf, sizeof(buf), "<p>Shunt: 0x%04X = %.3fmV (%s)</p>", shunt, shuntSigned*0.0025f, shuntOk ? "OK" : "FAIL");
  httpd_resp_send_chunk(req, buf, -1);
  
  httpd_resp_send_chunk(req, "<p><a href='/'>Back</a></p></body></html>", -1);
  httpd_resp_send_chunk(req, NULL, 0); // End response
  return ESP_OK;
}

static void register_routes_common(httpd_handle_t server){
  httpd_uri_t root     = { "/",            HTTP_GET, root_get_handler,     NULL };
  httpd_uri_t charts   = { "/charts",      HTTP_GET, charts_get_handler,   NULL };
  httpd_uri_t data     = { "/data",        HTTP_GET, data_get_handler,     NULL };
  httpd_uri_t hourly   = { "/agg/hourly",  HTTP_GET, hourly_get_handler,   NULL };
  httpd_uri_t daily    = { "/agg/daily",   HTTP_GET, daily_get_handler,    NULL };
  httpd_uri_t monthly  = { "/agg/monthly", HTTP_GET, monthly_get_handler,  NULL };
  httpd_uri_t calshow  = { "/cal/show",    HTTP_GET, cal_show_handler,     NULL };
  httpd_uri_t calset   = { "/cal",         HTTP_GET, cal_set_handler,      NULL };
  httpd_uri_t diag     = { "/diag",        HTTP_GET, diag_get_handler,     NULL };
  httpd_uri_t debug    = { "/debug",       HTTP_GET, debug_get_handler,    NULL };
  
  httpd_register_uri_handler(server, &root);
  httpd_register_uri_handler(server, &charts);
  httpd_register_uri_handler(server, &data);
  httpd_register_uri_handler(server, &hourly);
  httpd_register_uri_handler(server, &daily);
  httpd_register_uri_handler(server, &monthly);
  httpd_register_uri_handler(server, &calshow);
  httpd_register_uri_handler(server, &calset);
  httpd_register_uri_handler(server, &diag);
  httpd_register_uri_handler(server, &debug);
}

void setup(){
  Serial.begin(115200); delay(200);
  Serial.println("\nBooting Solar Logger (HTTP)…");

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting");
  unsigned long t0=millis(); while(WiFi.status()!=WL_CONNECTED && (millis()-t0)<20000){ Serial.print("."); delay(500); }
  Serial.println();
  if(WiFi.status()!=WL_CONNECTED){ Serial.println("WiFi failed; rebooting in 5s…"); delay(5000); ESP.restart(); }
  Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());

  // Time (NTP)
  configTzTime(TZ_STRING, NTP_SERVER_1, NTP_SERVER_2);
  for(int i=0;i<24;i++){ time_t now=time(nullptr); if(now>1700000000) break; delay(250); }

  // INA219
  Wire.begin(I2C_SDA, I2C_SCL);
  if(!ina219.begin()){  // Adafruit method
    Serial.println("INA219 not found!");
  } else {
    ina219.setCalibration_32V_2A(); // Adafruit preset for 32V range, 2A max
    Serial.println("INA219 initialized with 32V range.");
  }

  // SD
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
  SPI.begin(18,19,23,SD_CS);
  if(!SD.begin(SD_CS, SPI, 10000000)){
    Serial.println("Card Mount Failed (10 MHz). Retrying 4 MHz…");
    if(!SD.begin(SD_CS, SPI, 4000000)){ Serial.println("Card Mount Failed (4 MHz)."); }
    else { Serial.println("SD mounted at 4 MHz."); ensureCsvHeader(); loadCal(); }
  } else { Serial.println("SD mounted at 10 MHz."); ensureCsvHeader(); loadCal(); }

  if (CAL_I<=0 || !isfinite(CAL_I)) CAL_I=1.0f;
  if (CAL_V<=0 || !isfinite(CAL_V)) CAL_V=1.0f;
  Serial.printf("Calibration: CAL_V=%.3f  CAL_I=%.3f\n", CAL_V, CAL_I);

  // Start HTTP server
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.lru_purge_enable = true;
  if (httpd_start(&http_server, &cfg) == ESP_OK){
    register_routes_common(http_server);
    Serial.print("HTTP server started. Open: http://"); Serial.println(WiFi.localIP());
  } else {
    Serial.println("HTTP server failed to start.");
  }
}

void loop(){ /* server task runs in background */ }
