// =============================================================================
// webui.h — WebServer routes: config UI + REST API
// Direct-Sensor Rig Module variant. Pages:
//   /            Config (module info, RS485 baud, CAN enable/bitrate, WiFi)
//   /sensors     Add/edit/remove RS485 Modbus sensors (list, not fixed 8),
//                includes per-sensor "Probe Now" + "Auto-Detect Baud" and
//                bus-wide "Auto-Detect & Enable"
//   /can         Add/edit/remove CAN signals
//   /live        Live values table (sensors + CAN signals)
//   /system      Firmware info, OTA, buffer, reboot/factory-reset
//
// No standalone diagnostics/debugging page in this build — register
// read-back is still available via per-sensor Probe Now (read-only), but
// there is no register-write, raw-traffic-log, bus-scan-report, or CAN
// frame sniffer UI/API here. For deeper debugging (raw sniffing, register
// writes, framing experiments) use the LilyGo sensor-debug tool instead —
// keeps anything that can write to a sensor's own registers off the
// production firmware entirely.
// =============================================================================
#pragma once
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Update.h>
#include <HTTPClient.h>
#include "config.h"

// Forward declarations from main .ino
extern String buildPayload(bool);
extern int countBufferEntries();
extern bool flushNow;
extern int bufferCount;
extern String resolvedPiIp;
extern unsigned long lastPostMs;
extern bool lastPostOk;
extern SensorReading    sensorReadings[MAX_SENSORS];
extern CanSignalReading canReadings[MAX_CAN_SIGNALS];
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t modbusBusMutex;
// modbusAutoDetectBaud(), modbusReadRegs(), modbusRegCount(),
// modbusDecodeValue(), modbusScanSlaves() are all already declared+defined
// in modbus.h, included before this file in the .ino — no forward decls
// here. (Previously duplicated declarations here caused "ambiguous
// overload" compile errors once modbus.h's real signatures grew default
// parameters that these stale copies didn't have.)
bool canStart(int txPin, int rxPin, long bitrate); // can.h
void canStop(); // can.h
bool canIsRunning(); // can.h
int canGetRecentFrames(struct CanFrameLog* out, int maxCount); // can.h
unsigned long canGetFrameTotal(); // can.h
unsigned long canGetLastFrameMs(); // can.h
int canGetRecentFrameRate(); // can.h
extern NTPClient ntpClient;
extern bool apModeActive;
extern String apSSID;
// FW_VERSION is a #define in config.h — no extern needed

// Waveshare ESP32-S3-RS485-CAN pin constants (declared in the main .ino) —
// only needed here for the CAN enable/disable handler to (re)start it.
#define WEBUI_CAN_TXD 15
#define WEBUI_CAN_RXD 16

// Globals set in setupWebRoutes, used by handlers
static WebServer*    _srv    = nullptr;
static ModuleConfig* _cfg    = nullptr;
static Preferences*  _prefs  = nullptr;
static SensorReading* _sReadings = nullptr;
static CanSignalReading* _cReadings = nullptr;
static SemaphoreHandle_t _mtx = nullptr;

// ─── Shared CSS / nav ────────────────────────────────────────────────────────
static const char NAV[] PROGMEM = R"(
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#1a1a2e;color:#e0e0e0;font-family:sans-serif;font-size:14px}
  .nav{background:#16213e;padding:10px 16px;display:flex;flex-wrap:wrap;gap:8px;border-bottom:2px solid #667eea}
  .nav a{color:#667eea;text-decoration:none;padding:4px 10px;border-radius:4px;border:1px solid #667eea}
  .nav a:hover{background:#667eea;color:#fff}
  .page{padding:16px;max-width:900px;margin:0 auto}
  h2{color:#667eea;margin-bottom:16px}
  h3{color:#aaa;margin:16px 0 8px}
  label{display:block;margin-bottom:4px;color:#aaa;font-size:12px}
  input,select{width:100%;padding:8px;background:#0f3460;border:1px solid #667eea;color:#fff;border-radius:4px;margin-bottom:10px}
  input[type=checkbox]{width:auto;margin-right:8px}
  button,.btn{padding:8px 18px;background:#667eea;color:#fff;border:none;border-radius:4px;cursor:pointer}
  button:hover{background:#5568d4}
  .btn-red{background:#c0392b}
  .btn-green{background:#27ae60}
  .card{background:#16213e;border:1px solid #334;border-radius:6px;padding:12px;margin-bottom:12px}
  .ok{color:#2ecc71} .open{color:#e74c3c} .over{color:#e74c3c} .stale{color:#888} .warn{color:#f39c12} .timeout{color:#e74c3c} .crc{color:#e74c3c}
  .row{display:flex;gap:8px;align-items:center}
  .row>div{flex:1}
  .small{font-size:12px;color:#888}
  table{width:100%;border-collapse:collapse}
  td,th{padding:6px 8px;border-bottom:1px solid #334;text-align:left}
  th{color:#667eea;font-size:12px}
  .ap-banner{background:#f39c12;color:#1a1a2e;padding:8px 16px;font-weight:bold;text-align:center}
  .ap-banner a{color:#1a1a2e;text-decoration:underline}
  .mono{font-family:monospace;font-size:12px}
  .grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
  .grid4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:8px}
  .badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:11px;background:#334}
</style>
<div class='nav'>
  <a href='/'>&#9881; Config</a>
  <a href='/sensors'>&#128225; Sensors</a>
  <a href='/can'>&#128225; CAN</a>
  <a href='/live'>&#128202; Live</a>
  <a href='/system'>&#128295; System</a>
</div>
)";

static String apBanner() {
  if (!apModeActive) return "";
  return "<div class='ap-banner'>No WiFi configured yet — connect a device to this AP and "
         "set your network below.</div>";
}

static String _p(const char* name) {
  if (_srv->hasArg(name)) return _srv->arg(name);
  return "";
}
static bool _has(const char* name) { return _srv->hasArg(name); }

// Arduino's String(float) defaults to only 2 decimal places — fine for
// display, but a real bug for round-tripping config values BACK into an
// editable form field: a scale/offset/calibration value smaller than
// 0.01 (e.g. 0.001) would render as "0.00", and if that form is ever
// submitted again without the user manually re-typing the exact value,
// the tiny-but-real number silently gets overwritten with 0. Used for
// every numeric config field embedded as an <input value='...'> below.
static String _f(float v) {
  String s = String(v, 6);
  // Trim trailing zeros (but keep at least one digit after the decimal
  // point) so common whole/2-decimal values still look clean, e.g.
  // "1.000000" -> "1", "0.010000" -> "0.01" -- not just a wall of zeros.
  if (s.indexOf('.') >= 0) {
    while (s.endsWith("0")) s.remove(s.length() - 1);
    if (s.endsWith(".")) s.remove(s.length() - 1);
  }
  return s;
}

static const char* dataTypeName(uint8_t dt) {
  switch (dt) {
    case MB_UINT16: return "uint16";
    case MB_INT16:  return "int16";
    case MB_UINT32: return "uint32";
    case MB_INT32:  return "int32";
    case MB_FLOAT32: return "float32";
    default: return "?";
  }
}

// ─── /  CONFIG PAGE ──────────────────────────────────────────────────────────
static String cfgPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += apBanner();
  h += "<div class='page'><h2>&#9881; Module Configuration</h2>";
  h += "<div class='card'><b>Module ID:</b> " + cfg.moduleId + " <span class='small'>(fixed, derived from MAC)</span></div>";
  h += "<form method='POST' action='/api/config'>";
  h += "<label>Module Name</label><input name='moduleName' value='" + cfg.moduleName + "' placeholder='e.g. Standpipe Pressure Skid'>";
  h += "<label>Module Type</label><input name='moduleType' list='moduleTypeOpts' value='" + cfg.moduleType + "' placeholder='e.g. pressure, drill'>";
  h += "<datalist id='moduleTypeOpts'><option value='generic'><option value='pressure'><option value='tank'><option value='pump'><option value='drill'></datalist>";
  h += "<div class='small'>Shown on the rig dashboard.</div>";
  h += "<label>Description</label><input name='description' value='" + cfg.description + "'>";

  h += "<h3>RS485 / Modbus Bus</h3>";
  h += "<label>RS485 Baud Rate</label><select name='modbusBaud'>";
  {
    struct { long val; const char* label; } bauds[] = {
      { 1200, "1200" }, { 2400, "2400" }, { 4800, "4800" }, { 9600, "9600 (most common)" },
      { 19200, "19200" }, { 38400, "38400" }, { 57600, "57600" }, { 115200, "115200" },
    };
    for (auto& b : bauds) {
      h += "<option value='" + String(b.val) + "'";
      if (cfg.modbusBaud == b.val) h += " selected";
      h += ">" + String(b.label) + "</option>";
    }
  }
  h += "</select>";
  h += "<div class='small'>All sensors on the bus must use this baud rate.</div>";
  if (cfg.baudManuallySet) {
    h += "<div class='small ok'>&#128274; Baud is locked — auto-detect will not change this without you setting it again "
         "or using Auto-Detect Baud on a sensor.</div>";
  } else {
    h += "<div class='small warn'>&#128275; Baud not yet locked — background auto-detect may still adjust this "
         "automatically if no sensor is enabled yet. Save this form (or run Auto-Detect Baud on a sensor) to lock it.</div>";
  }

  h += "<h3>CAN Bus</h3>";
  h += "<label><input type='checkbox' name='canEnabled'";
  if (cfg.canEnabled) h += " checked";
  h += "> Enable CAN</label>";
  h += "<label>CAN Bitrate</label><select name='canBitrate'>";
  {
    struct { long val; const char* label; } bauds[] = {
      { 125000, "125 kbit/s" }, { 250000, "250 kbit/s (J1939 / most drill CAN)" },
      { 500000, "500 kbit/s" }, { 1000000, "1 Mbit/s" },
    };
    for (auto& b : bauds) {
      h += "<option value='" + String(b.val) + "'";
      if (cfg.canBitrate == b.val) h += " selected";
      h += ">" + String(b.label) + "</option>";
    }
  }
  h += "</select>";
  h += "<div class='small'>Listen-only.</div>";

  h += "<h3>Pi Logger</h3>";
  h += "<label>Poll Interval (1-30 s)</label><input name='pollIntervalS' type='number' min='1' max='30' value='" + String(cfg.pollIntervalS) + "'>";
  h += "<label>Pi Host (blank = auto)</label><input name='piHost' value='" + cfg.piHost + "' placeholder='192.168.x.x or rig-logger.local'>";
  h += "<div class='small'>Blank = auto-discover.</div>";
  h += "<label>X-Rig-Token</label><input name='rigToken' type='password' value='" + cfg.rigToken + "'>";

  h += "<h3>WiFi</h3>";
  if (apModeActive) {
    h += "<div class='card'>Currently broadcasting setup AP: <b>" + apSSID + "</b><br>Not connected to any site network yet.</div>";
  } else {
    h += "<div class='card'>Connected to: <b>" + cfg.wifiSSID + "</b><br>IP: " + WiFi.localIP().toString() + "  RSSI: " + String(WiFi.RSSI()) + " dBm</div>";
  }
  h += "<div class='row' style='margin-bottom:10px'><button type='button' onclick='doScan()' id='scanBtn'>&#128269; Scan for Networks</button></div>";
  h += "<div id='scanResults'></div>";
  h += "<label>SSID</label><input name='wifiSSID' id='wifiSSID' value='" + cfg.wifiSSID + "' placeholder='site wifi network name'>";
  h += "<label>Password</label><input name='wifiPass' id='wifiPass' type='password' value='" + cfg.wifiPass + "' placeholder='site wifi password'>";
  h += "<div class='small'>Changing SSID/password reboots the unit.</div>";
  h += "<br><button type='submit'>Save</button>";
  h += "</form>";
  h += "<div class='card' style='margin-top:12px'><b>Forget WiFi</b><br><span class='small'>Clears saved network, reboots into setup-AP mode.</span><br><br>";
  h += "<button class='btn-red' onclick=\"if(confirm('Forget saved WiFi and reboot into setup mode?'))fetch('/api/wifi/forget',{method:'POST'}).then(()=>alert('Forgotten. Rebooting...'))\">Forget WiFi</button></div>";
  h += R"JS(
<script>
function doScan(){
  let btn=document.getElementById('scanBtn'); let box=document.getElementById('scanResults');
  btn.disabled=true; btn.textContent='Scanning...';
  box.innerHTML='<p class="small">Scanning (a few seconds)...</p>';
  fetch('/api/wifi/scan').then(r=>r.json()).then(d=>{
    btn.disabled=false; btn.innerHTML='&#128269; Scan for Networks';
    let nets = d.networks || [];
    if(nets.length===0){ box.innerHTML='<p class="small">No networks found. Try again.</p>'; return; }
    let s = '<div class="card">';
    nets.forEach(function(n, idx){
      let bars = n.rssi>-60?'####':n.rssi>-70?'###.':n.rssi>-80?'##..':'#...';
      let lock = n.secure ? '&#128274;' : '';
      s += '<div class="row" style="justify-content:space-between;padding:4px 0;border-bottom:1px solid #334;cursor:pointer" onclick="pickNet(scanNets['+idx+'])">'+
           '<span>'+lock+' '+n.ssid+'</span><span class="small">'+bars+' '+n.rssi+'dBm</span></div>';
    });
    s += '</div>';
    window.scanNets = nets.map(function(n){ return n.ssid; });
    box.innerHTML = s;
  }).catch(function(){ btn.disabled=false; btn.innerHTML='&#128269; Scan for Networks'; box.innerHTML='<p class="small">Scan failed.</p>'; });
}
function pickNet(ssid){ document.getElementById('wifiSSID').value = ssid; document.getElementById('wifiPass').value=''; document.getElementById('wifiPass').focus(); }
</script>)JS";
  h += "</div>";
  return h;
}

// ─── /sensors  RS485 SENSOR LIST ────────────────────────────────────────────
static String sensorsPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128225; RS485 Sensors</h2>";
  h += "<p class='small'>Any Modbus RTU sensor on the RS485 bus. Don't know its slave ID/register? Use "
       "\"Probe Now\" below to check, or \"Auto-Detect &amp; Enable\" to find new sensors.</p>";
  h += "<div class='card' style='border-color:#27ae60'><b>&#9889; Auto-Detect &amp; Enable</b><br>"
       "<span class='small'>Scans addresses 1-16, auto-enables new sensors with starter defaults. Also runs "
       "on boot and every few minutes.</span><br><br>"
       "<button type='button' class='btn-green' onclick='autoDetectEnable()' id='adeBtn'>&#9889; Auto-Detect &amp; Enable Now</button>"
       "<div id='adeResult' class='small' style='margin-top:8px'></div></div>";
  h += "<form method='POST' action='/api/sensors/save'>";
  h += "<button type='submit' style='margin-bottom:14px'>&#128190; Save All Sensors</button>";

  for (int i = 0; i < MAX_SENSORS; i++) {
    SensorConfig& s = cfg.sensors[i];
    h += "<div class='card'><div class='row' style='justify-content:space-between'>";
    h += "<b>Sensor " + String(i + 1) + "</b><span class='small mono' id='live" + String(i) + "'></span></div>";
    h += "<label><input type='checkbox' name='s" + String(i) + "en'";
    if (s.enabled) h += " checked";
    h += "> Enabled</label>";
    h += "<div class='row'><div><label>Name</label><input name='s" + String(i) + "nm' value='" + s.name + "' placeholder='e.g. Standpipe Pressure'></div>";
    h += "<div><label>Kind</label><input name='s" + String(i) + "kd' value='" + s.kind + "' placeholder='e.g. pressure'></div>";
    h += "<div><label>Unit</label><input name='s" + String(i) + "ut' value='" + s.unit + "' placeholder='e.g. psi'></div></div>";
    h += "<div class='grid4'>";
    h += "<div><label>Slave ID (1-247)</label><input name='s" + String(i) + "sid' type='number' min='1' max='247' value='" + String(s.slaveId) + "'></div>";
    h += "<div><label>Function Code</label><select name='s" + String(i) + "fc'>";
    h += "<option value='4'" + String(s.funcCode == 4 ? " selected" : "") + ">04 - Read Input Regs</option>";
    h += "<option value='3'" + String(s.funcCode == 3 ? " selected" : "") + ">03 - Read Holding Regs</option>";
    h += "</select></div>";
    h += "<div><label>Register Addr (hex or dec)</label><input name='s" + String(i) + "reg' value='" + String(s.regAddr) + "'></div>";
    h += "<div><label>Data Type</label><select name='s" + String(i) + "dt'>";
    {
      struct { uint8_t val; const char* label; } types[] = {
        { MB_UINT16, "uint16 (1 reg)" }, { MB_INT16, "int16 (1 reg)" },
        { MB_UINT32, "uint32 (2 regs)" }, { MB_INT32, "int32 (2 regs)" }, { MB_FLOAT32, "float32 (2 regs)" },
      };
      for (auto& t : types) {
        h += "<option value='" + String(t.val) + "'";
        if (s.dataType == t.val) h += " selected";
        h += ">" + String(t.label) + "</option>";
      }
    }
    h += "</select></div></div>";
    h += "<div class='row'><div><label>Word Order (32-bit types only)</label><select name='s" + String(i) + "wo'>";
    h += "<option value='0'" + String(s.wordOrder == 0 ? " selected" : "") + ">High word first</option>";
    h += "<option value='1'" + String(s.wordOrder == 1 ? " selected" : "") + ">Low word first</option>";
    h += "</select></div>";
    h += "<div><label>Scale (value = raw * scale + offset)</label><input name='s" + String(i) + "sc' type='number' step='any' value='" + _f(s.scale) + "'></div>";
    h += "<div><label>Offset</label><input name='s" + String(i) + "of' type='number' step='any' value='" + _f(s.offset) + "'></div></div>";

    h += "<label style='margin-top:8px'><input type='checkbox' name='s" + String(i) + "volEn' id='volEn" + String(i) + "' onchange='toggleVol(" + String(i) + ")'";
    if (s.volumeEnabled) h += " checked";
    h += "> Compute Tank Volume from this sensor</label>";
    h += "<div id='volFields" + String(i) + "' style='display:" + String(s.volumeEnabled ? "block" : "none") + "'>";
    h += "<div class='small'>Empty = 0 volume, Full = Capacity.</div>";
    h += "<div class='row'><div><label>Capacity</label><input name='s" + String(i) + "cap' type='number' step='any' min='0' value='" + _f(s.capacity) + "'></div>";
    h += "<div><label>Unit</label><select name='s" + String(i) + "cu'>";
    h += "<option value='m3'" + String(s.capacityUnit == "m3" ? " selected" : "") + ">m&#179;</option>";
    h += "<option value='gal'" + String(s.capacityUnit == "gal" ? " selected" : "") + ">gal</option></select></div></div>";
    h += "<div class='row'><div><label>Value @ Empty</label><input name='s" + String(i) + "vz' type='number' step='any' value='" + _f(s.volZeroLevel) + "'></div>";
    h += "<div><label>Value @ Full</label><input name='s" + String(i) + "vm' type='number' step='any' value='" + _f(s.volMaxLevel) + "'></div></div>";
    h += "</div>";

    h += "<div class='row' style='margin-top:8px'><button type='button' onclick='probeSensor(" + String(i) + ")'>&#128269; Probe Now</button>";
    h += "<button type='button' onclick='autoBaud(" + String(i) + ")'>&#128260; Auto-Detect Baud</button></div>";
    h += "</div>";
  }

  h += "<button type='submit' style='margin-top:6px'>&#128190; Save All Sensors</button>";
  h += "</form>";
  h += R"(
<script>
function fetchLive(){
  fetch('/api/sensors/live').then(r=>r.json()).then(d=>{
    d.sensors.forEach(s=>{
      let el=document.getElementById('live'+s.idx);
      if(!el) return;
      let ds = s.displayStatus || s.status;
      if (s.hasValue) el.textContent = s.value.toFixed(2)+' ('+ds+')';
      else el.textContent = '-- ('+ds+')';
      el.className = 'small mono ' + ds;
    });
  });
}
setInterval(fetchLive,2000); fetchLive();
function toggleVol(i){
  var el=document.getElementById('volFields'+i);
  var cb=document.getElementById('volEn'+i);
  el.style.display = cb.checked ? 'block' : 'none';
}
function probeSensor(i){
  let sid = document.querySelector('[name=s'+i+'sid]').value;
  let fc  = document.querySelector('[name=s'+i+'fc]').value;
  let reg = document.querySelector('[name=s'+i+'reg]').value;
  let dt  = document.querySelector('[name=s'+i+'dt]').value;
  let wo  = document.querySelector('[name=s'+i+'wo]').value;
  let el = document.getElementById('live'+i);
  probeWithRetry('/api/modbus/probe?slaveId='+sid+'&funcCode='+fc+'&reg='+reg+'&dataType='+dt+'&wordOrder='+wo, el,
    (d)=>'Probe OK: raw='+d.raw+' decoded='+d.decoded.toFixed(3),
    (d)=>'Probe FAILED: '+d.error);
}
// Many sensors (e.g. slow-cycle radar/ultrasonic level sensors) only have
// fresh data ready on some fraction of polls and time out the rest — that's
// normal sensor behavior, not a comms fault (see modbus.h notes). Rather
// than making the user mash the probe button until one lands, retry
// automatically a handful of times with a short gap and show whichever
// result comes back first (ok wins immediately; only shows FAILED after
// exhausting all attempts). renderOk/renderFail build the final message.
function probeWithRetry(url, el, renderOk, renderFail, attempt){
  attempt = attempt || 1;
  const maxAttempts = 8;     // ~8 tries...
  const gapMs = 900;         // ...spaced out ~900ms apart => up to ~7s total,
                              // enough to catch a sensor mid-cycle without
                              // the page feeling like it's hung.
  el.textContent = attempt === 1 ? 'Probing...' : ('Probing... (retry '+attempt+'/'+maxAttempts+')');
  fetch(url).then(r=>r.json()).then(d=>{
    if (d.ok) { el.innerHTML = renderOk(d); return; }
    if (attempt < maxAttempts) {
      setTimeout(()=>probeWithRetry(url, el, renderOk, renderFail, attempt+1), gapMs);
    } else {
      el.innerHTML = renderFail(d) + ' (after '+maxAttempts+' attempts)';
    }
  }).catch(e=>{
    if (attempt < maxAttempts) {
      setTimeout(()=>probeWithRetry(url, el, renderOk, renderFail, attempt+1), gapMs);
    } else {
      el.textContent = 'Request failed: '+e;
    }
  });
}
function autoBaud(i){
  let sid = document.querySelector('[name=s'+i+'sid]').value;
  let el = document.getElementById('live'+i);
  el.textContent = 'Scanning bauds (a few seconds)...';
  fetch('/api/modbus/autodetect?slaveId='+sid,{method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.detected){ el.textContent = 'Found '+d.baud+' baud. Saved — reloading...'; setTimeout(()=>location.reload(),1200); }
    else el.textContent = 'No response at any baud for slave '+sid+'.';
  });
}
function autoDetectEnable(){
  let btn=document.getElementById('adeBtn'); let box=document.getElementById('adeResult');
  btn.disabled=true; btn.textContent='Scanning...';
  box.textContent='Scanning addresses 1-16...';
  fetch('/api/modbus/autodetect-enable?max=16',{method:'POST'}).then(r=>r.json()).then(d=>{
    btn.disabled=false; btn.innerHTML='&#9889; Auto-Detect &amp; Enable Now';
    if(d.newCount>0){ box.innerHTML = '<span class="ok">Found and enabled '+d.newCount+' new sensor(s).</span> Reloading...'; setTimeout(()=>location.reload(),1200); }
    else box.textContent = 'No new sensors found (either nothing new on the bus, or all sensor slots are full).';
  }).catch(e=>{ btn.disabled=false; btn.innerHTML='&#9889; Auto-Detect &amp; Enable Now'; box.textContent='Request failed: '+e; });
}

</script>)";
  h += "</div>";
  return h;
}

// ─── /can  CAN SIGNAL LIST ───────────────────────────────────────────────────
static String canPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128225; CAN Signals</h2>";
  if (!cfg.canEnabled) {
    h += "<div class='card' style='border-color:#f39c12'>CAN is currently <b>disabled</b>. Enable it on the "
         "<a href='/'>Config</a> page first.</div>";
  } else {
    h += "<div class='card'>CAN running at " + String(cfg.canBitrate) + " bit/s, listen-only. "
         "Total frames seen: <span id='canTotal'>...</span>, recent rate: <span id='canRate'>...</span> fps.</div>";
  }
  h += "<p class='small'>Decodes a byte range from a specific CAN ID into a value.</p>";
  h += "<form method='POST' action='/api/can/save'>";
  h += "<button type='submit' style='margin-bottom:14px'>&#128190; Save All Signals</button>";

  for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
    CanSignalConfig& sg = cfg.canSignals[i];
    h += "<div class='card'><div class='row' style='justify-content:space-between'>";
    h += "<b>Signal " + String(i + 1) + "</b><span class='small mono' id='canlive" + String(i) + "'></span></div>";
    h += "<label><input type='checkbox' name='c" + String(i) + "en'";
    if (sg.enabled) h += " checked";
    h += "> Enabled</label>";
    h += "<div class='row'><div><label>Name</label><input name='c" + String(i) + "nm' value='" + sg.name + "' placeholder='e.g. Engine RPM'></div>";
    h += "<div><label>Kind</label><input name='c" + String(i) + "kd' value='" + sg.kind + "' placeholder='e.g. rpm'></div>";
    h += "<div><label>Unit</label><input name='c" + String(i) + "ut' value='" + sg.unit + "' placeholder='e.g. rpm'></div></div>";
    h += "<div class='grid4'>";
    h += "<div><label>CAN ID (hex, e.g. 18FEF200)</label><input name='c" + String(i) + "id' value='" + String(sg.canId, HEX) + "'></div>";
    h += "<div><label>ID Type</label><select name='c" + String(i) + "ext'>";
    h += "<option value='0'" + String(!sg.extended ? " selected" : "") + ">Standard (11-bit)</option>";
    h += "<option value='1'" + String(sg.extended ? " selected" : "") + ">Extended (29-bit)</option></select></div>";
    h += "<div><label>Byte Offset (0-7)</label><input name='c" + String(i) + "bo' type='number' min='0' max='7' value='" + String(sg.byteOffset) + "'></div>";
    h += "<div><label>Byte Length</label><select name='c" + String(i) + "bl'>";
    h += "<option value='1'" + String(sg.byteLen == 1 ? " selected" : "") + ">1 byte</option>";
    h += "<option value='2'" + String(sg.byteLen == 2 ? " selected" : "") + ">2 bytes</option>";
    h += "<option value='4'" + String(sg.byteLen == 4 ? " selected" : "") + ">4 bytes</option></select></div>";
    h += "</div>";
    h += "<div class='row'>";
    h += "<div><label>Byte Order</label><select name='c" + String(i) + "be'>";
    h += "<option value='1'" + String(sg.bigEndian ? " selected" : "") + ">Big-endian (most CAN/J1939)</option>";
    h += "<option value='0'" + String(!sg.bigEndian ? " selected" : "") + ">Little-endian</option></select></div>";
    h += "<div><label>Signed</label><select name='c" + String(i) + "sv'>";
    h += "<option value='0'" + String(!sg.signedVal ? " selected" : "") + ">Unsigned</option>";
    h += "<option value='1'" + String(sg.signedVal ? " selected" : "") + ">Signed</option></select></div>";
    h += "<div><label>Scale</label><input name='c" + String(i) + "sc' type='number' step='any' value='" + _f(sg.scale) + "'></div>";
    h += "<div><label>Offset</label><input name='c" + String(i) + "of' type='number' step='any' value='" + _f(sg.offset) + "'></div>";
    h += "</div>";
    h += "</div>";
  }

  h += "<button type='submit' style='margin-top:6px'>&#128190; Save All Signals</button>";
  h += "</form>";
  h += R"(
<script>
function fetchCanLive(){
  fetch('/api/can/live').then(r=>r.json()).then(d=>{
    document.getElementById('canTotal') && (document.getElementById('canTotal').textContent = d.frameTotal);
    document.getElementById('canRate') && (document.getElementById('canRate').textContent = d.frameRate);
    d.signals.forEach(s=>{
      let el=document.getElementById('canlive'+s.idx);
      if(!el) return;
      if (s.hasValue) el.textContent = s.value.toFixed(2)+' ('+s.status+')';
      else el.textContent = '-- ('+s.status+')';
      el.className = 'small mono ' + s.status;
    });
  });
}
setInterval(fetchCanLive,1500); fetchCanLive();
</script>)";
  h += "</div>";
  return h;
}

// ─── /live  LIVE VALUES TABLE ────────────────────────────────────────────────
static String livePage() {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128202; Live Status</h2><div id='liveData'>Loading...</div>";
  h += R"(
<script>
function fetchLive(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    let s = '';
    s += '<h3>RS485 Sensors</h3><table><tr><th>Name</th><th>Kind</th><th>Slave</th><th>Value</th><th>Volume</th><th>Status</th></tr>';
    (d.channels||[]).forEach(c=>{
      let ds = c.displayStatus || c.status;
      let cls = ds==='ok'?'ok':ds;
      s += '<tr><td>'+(c.name||'')+'</td><td>'+(c.kind||'')+'</td><td>'+c.ch+'</td>';
      s += '<td>'+(c.value!=null?c.value+' '+c.unit:'--')+'</td>';
      if (c.volume) {
        let v = c.volume;
        let vcls = v.status==='ok'?'ok':'open';
        let vtxt = (v.value!=null ? v.value : '--') + ' ' + v.unit;
        if (c.capacity!=null) vtxt += ' <span class="small">/ '+c.capacity+' '+v.unit+'</span>';
        s += '<td class="'+vcls+'">'+vtxt+'</td>';
      } else { s += '<td class="small">--</td>'; }
      s += '<td class="'+cls+'">'+ds+'</td></tr>';
    });
    s += '</table>';
    if (d.canEnabled) {
      s += '<h3>CAN Signals</h3><table><tr><th>Name</th><th>Kind</th><th>CAN ID</th><th>Value</th><th>Status</th></tr>';
      (d.canSignals||[]).forEach(c=>{
        let cls = c.status==='ok'?'ok':'stale';
        s += '<tr><td>'+(c.name||'')+'</td><td>'+(c.kind||'')+'</td><td>0x'+Number(c.canId).toString(16).toUpperCase()+'</td>';
        s += '<td>'+(c.value!=null?c.value+' '+c.unit:'--')+'</td><td class="'+cls+'">'+c.status+'</td></tr>';
      });
      s += '</table>';
      s += '<p class="small">CAN frame rate: '+(d.canFrameRate||0)+' fps, total: '+(d.canFrameTotal||0)+'</p>';
    }
    let sys = d.system||{};
    s += '<h3>System</h3><table>';
    s += '<tr><td>Module ID</td><td>'+d.moduleId+'</td></tr>';
    s += '<tr><td>Pi</td><td>'+(sys.piIp||'unresolved')+'</td></tr>';
    s += '<tr><td>Last post</td><td class="'+(sys.lastPostOk?'ok':'open')+'">'+((sys.lastPostMs>0)?(sys.lastPostMs/1000).toFixed(0)+'s ago':'never')+' '+(sys.lastPostOk?'ok':'fail')+'</td></tr>';
    s += '<tr><td>Buffer</td><td>'+sys.bufCount+' entries</td></tr>';
    s += '<tr><td>WiFi RSSI</td><td>'+(sys.rssi||'?')+' dBm</td></tr>';
    s += '<tr><td>Uptime</td><td>'+(d.uptimeS||0)+'s</td></tr>';
    s += '<tr><td>Free Heap</td><td>'+(sys.freeHeap||'?')+'</td></tr>';
    s += '<tr><td>NTP</td><td class="'+(sys.ntpOk?'ok':'warn')+'">'+(sys.ntpOk?d.ts:'no sync')+'</td></tr>';
    s += '</table>';
    document.getElementById('liveData').innerHTML = s;
  });
}
setInterval(fetchLive,3000); fetchLive();
</script>)";
  h += "</div>";
  return h;
}

// ─── /system  SYSTEM PAGE ────────────────────────────────────────────────────
static String sysPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128295; System</h2>";
  h += "<div class='card'><b>Firmware:</b> " + String(FW_VERSION);
  h += "<br><b>Module ID:</b> " + cfg.moduleId;
  h += "<br><b>MAC:</b> " + WiFi.macAddress();
  h += "<br><b>Chip:</b> " + String(ESP.getChipModel()) + " @ " + String(ESP.getCpuFreqMHz()) + "MHz";
  h += "<br><b>CAN:</b> " + String(cfg.canEnabled ? ("enabled, " + String(cfg.canBitrate) + " bit/s") : "disabled") + "</div>";
  {
    NvsStats st = getNvsStats();
    if (st.ok) {
      bool low = st.freeEntries < 20;
      h += "<div class='card'><b>NVS Storage:</b> " + String(st.usedEntries) + " used / " +
           String(st.totalEntries) + " total entries (" + String(st.freeEntries) + " free)";
      if (low) {
        h += "<br><span class='warn'>&#9888; Running low — new config keys may silently fail to save. "
             "If settings (e.g. baud rate) aren't sticking, this is likely why.</span>";
      }
      h += "</div>";
    }
  }
  {
    // Live ground-truth readback: read mbBaud/mbBaudSet straight from
    // flash right now (not from cfg, which is just RAM) and compare
    // against what's currently in RAM. If these two ever disagree, it's
    // definitive proof of a save that didn't actually persist — no
    // guessing, no reboot-and-check-later.
    Preferences verify;
    verify.begin("rigmod", true);
    long flashBaud = verify.getLong("mbBaud", -1);
    bool flashBaudSet = verify.getBool("mbBaudSet", false);
    verify.end();
    bool mismatch = (flashBaud != cfg.modbusBaud) || (flashBaudSet != cfg.baudManuallySet);
    h += "<div class='card'><b>Baud Persistence Check:</b>";
    h += "<br>In RAM right now: baud=" + String(cfg.modbusBaud) + " manuallySet=" + String(cfg.baudManuallySet ? "true" : "false");
    h += "<br>On flash right now: baud=" + String(flashBaud) + " manuallySet=" + String(flashBaudSet ? "true" : "false");
    if (mismatch) {
      h += "<br><span class='warn'>&#9888; MISMATCH — flash does not match what's running. "
           "This confirms the save isn't persisting; the values shown above as \"on flash\" are what "
           "will come back after a reboot.</span>";
    } else {
      h += "<br><span class='ok'>&#10003; Match — flash agrees with what's running.</span>";
    }
    h += "</div>";
  }
  h += "<h3>OTA Update</h3><div class='card'>";
  h += "<label>OTA from URL:</label><div class='row'><input id='otaUrl' placeholder='http://...'>&nbsp;";
  h += "<button onclick='doOTA()'>Update</button></div></div>";
  h += "<h3>Buffer</h3><div class='card'><div id='bufInfo'>...</div><br>";
  h += "<button onclick='fetch(\"/api/buffer/flush\",{method:\"POST\"}).then(()=>alert(\"Flushing\"))'>Flush Now</button>&nbsp;";
  h += "<button class='btn-red' onclick='if(confirm(\"Clear all buffered data?\"))fetch(\"/api/buffer/clear\",{method:\"POST\"}).then(()=>location.reload())'>Clear Buffer</button></div>";
  h += "<h3>Danger Zone</h3><div class='card'>";
  h += "<button onclick='if(confirm(\"Reboot?\"))fetch(\"/api/reboot\",{method:\"POST\"})'>Reboot</button>&nbsp;";
  h += "<button class='btn-red' onclick='if(confirm(\"Factory reset? ALL config will be lost.\"))fetch(\"/api/factory-reset\",{method:\"POST\"})'>Factory Reset</button></div>";
  h += R"(
<script>
fetch('/api/status').then(r=>r.json()).then(d=>{
  let sys=d.system||{};
  document.getElementById('bufInfo').textContent = 'Buffered entries: '+(sys.bufCount||0);
});
function doOTA(){
  let url=document.getElementById('otaUrl').value;
  if(!url)return alert('Enter URL');
  fetch('/api/ota?url='+encodeURIComponent(url)).then(r=>r.text()).then(t=>alert(t));
}
</script>)";
  h += "</div>";
  return h;
}

// ─── API handler helpers ──────────────────────────────────────────────────────
static void handleApiStatus() {
  DynamicJsonDocument doc(8192);
  String payload = buildPayload(false);
  deserializeJson(doc, payload);
  doc["system"]["uptime"]     = millis() / 1000;
  doc["system"]["freeHeap"]   = ESP.getFreeHeap();
  doc["system"]["rssi"]       = WiFi.RSSI();
  doc["system"]["piIp"]       = resolvedPiIp;
  doc["system"]["lastPostMs"] = lastPostMs ? (millis() - lastPostMs) : -1;
  doc["system"]["lastPostOk"] = lastPostOk;
  doc["system"]["bufCount"]   = bufferCount;
  doc["system"]["ntpOk"]      = ntpClient.isTimeSet();
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Live sensor readings for the /sensors page — richer than the payload's
// "channels" array (includes disabled slots, error counters, slave IDs)
// since this is an editing view, not the wire format.
static void handleSensorsLive() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.createNestedArray("sensors");
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(300)) == pdTRUE) {
    for (int i = 0; i < MAX_SENSORS; i++) {
      SensorConfig& s = _cfg->sensors[i];
      SensorReading& r = _sReadings[i];
      JsonObject o = arr.createNestedObject();
      o["idx"]        = i;
      o["enabled"]    = s.enabled;
      o["name"]       = s.name;
      o["slaveId"]    = s.slaveId;
      o["hasValue"]   = r.hasValue;
      o["value"]      = r.hasValue ? r.value : (float)0;
      if (!r.hasValue) o["value"] = nullptr;
      // "status" = raw per-poll result (every real timeout/CRC error,
      // unfiltered). "displayStatus" = debounced version for the /sensors page's own
      // live indicator next to each sensor card (suppresses a lone
      // timeout on a sensor with a slow measurement cycle that's
      // otherwise reporting fine).
      o["status"]        = s.enabled ? r.status : "disabled";
      o["displayStatus"] = s.enabled ? r.displayStatus : "disabled";
      o["pollCount"]  = r.pollCount;
      o["errorCount"] = r.errorCount;
      if (r.lastOkMs) o["lastOkAgoMs"] = (long)(millis() - r.lastOkMs);
      else o["lastOkAgoMs"] = nullptr;
    }
    xSemaphoreGive(_mtx);
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

static void handleCanLive() {
  DynamicJsonDocument doc(4096);
  doc["frameTotal"] = canGetFrameTotal();
  doc["frameRate"]  = canGetRecentFrameRate();
  JsonArray arr = doc.createNestedArray("signals");
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(300)) == pdTRUE) {
    for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
      CanSignalConfig& sg = _cfg->canSignals[i];
      if (!sg.enabled) continue;
      CanSignalReading& r = _cReadings[i];
      JsonObject o = arr.createNestedObject();
      o["idx"]      = i;
      o["hasValue"] = r.hasValue;
      o["value"]    = r.hasValue ? r.value : (float)0;
      if (!r.hasValue) o["value"] = nullptr;
      o["status"]   = r.status;
    }
    xSemaphoreGive(_mtx);
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Reads a specific register combo from a specific slave right now —
// backs the per-sensor "Probe Now" button on /sensors. Doesn't touch
// saved config at all.
static void handleModbusProbe() {
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy\"}");
    return;
  }
  uint8_t slaveId = _p("slaveId").toInt();
  uint8_t funcCode = _p("funcCode").toInt();
  // Register may be entered as hex ("0x1000"/"1000" typed as hex-looking)
  // or decimal — accept both: strtol with base 0 auto-detects "0x" prefix,
  // otherwise falls back to decimal.
  String regStr = _p("reg");
  uint16_t regAddr = (uint16_t)strtol(regStr.c_str(), nullptr, 0);
  uint8_t dataType = _p("dataType").toInt();
  uint8_t wordOrder = _p("wordOrder").toInt();

  uint8_t n = modbusRegCount(dataType);
  uint16_t regs[2] = {0, 0};
  uint8_t actualSlaveId = 0;
  int rc = modbusReadRegs(slaveId, funcCode, regAddr, n, regs, true, 600, &actualSlaveId);
  xSemaphoreGive(modbusBusMutex);

  DynamicJsonDocument doc(512);
  if (rc == MB_OK) {
    float decoded = modbusDecodeValue(regs, dataType, wordOrder);
    doc["ok"] = true;
    doc["raw"] = regs[0];
    JsonArray regsArr = doc.createNestedArray("regs");
    for (int i = 0; i < n; i++) regsArr.add(regs[i]);
    doc["decoded"] = decoded;
    // If we queried a broadcast address, report which real address the
    // sensor answered as — that's the whole point of a broadcast probe.
    if (modbusIsBroadcastAddr(slaveId)) doc["actualSlaveId"] = actualSlaveId;
  } else {
    doc["ok"] = false;
    doc["error"] = (rc == MB_TIMEOUT) ? "timeout — no response" :
                   (rc == MB_CRC_ERROR) ? "CRC error" : "bad response";
    // rc==MB_BAD_RESPONSE can mean "CRC-valid reply from a DIFFERENT
    // address than queried" — surface that address, it's diagnostic gold
    // (means the sensor is alive and talking, just not at the address
    // you thought).
    if (rc == MB_BAD_RESPONSE && actualSlaveId != 0 && actualSlaveId != slaveId) {
      doc["error"] = String("bad response — but got a reply FROM address ") + actualSlaveId +
                     " instead of the address you queried (" + slaveId + ")";
      doc["actualSlaveId"] = actualSlaveId;
    }
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Auto-Detect & Enable — scans the bus and fills in/enables sensor slots
// for anything new found, saving config if anything changed. Backs both
// the boot-time/periodic background pass and the manual button on
// /sensors. Synchronous/blocking — a few hundred ms per address scanned.
// modbusAutoDetectAndEnable() is already declared+defined in modbus.h,
// included before this file — no forward decl needed here.
static void handleAutoDetectEnable() {
  int maxAddr = _p("max").isEmpty() ? 16 : _p("max").toInt();
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;

  int newCount = 0;
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(60000)) == pdTRUE) {
    newCount = modbusAutoDetectAndEnable(*_cfg, maxAddr);
    xSemaphoreGive(modbusBusMutex);
  }
  if (newCount > 0) {
    saveConfig(*_prefs, *_cfg);
  }

  DynamicJsonDocument doc(256);
  doc["ok"] = true;
  doc["newCount"] = newCount;
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

static void handleConfig() {
  auto applyParam = [&](const char* name, std::function<void(String)> fn){
    if (_srv->hasArg(name)) fn(_srv->arg(name));
  };

  applyParam("moduleName",  [](String v){ _cfg->moduleName = v; });
  applyParam("moduleType",  [](String v){ _cfg->moduleType = v.isEmpty() ? "generic" : v; });
  applyParam("description", [](String v){ _cfg->description = v; });

  bool baudChanged = false;
  applyParam("modbusBaud", [&](String v){
    long nb = v.toInt();
    if (nb != _cfg->modbusBaud) { _cfg->modbusBaud = nb; baudChanged = true; }
    // Any explicit save from this form counts as "user has decided the
    // baud" — even re-selecting the same value locks it in, since the
    // form always submits a modbusBaud value. Prevents the background
    // auto-detect from ever silently changing it again after this.
    _cfg->baudManuallySet = true;
  });

  bool canChanged = false;
  bool newCanEnabled = _srv->hasArg("canEnabled");
  if (newCanEnabled != _cfg->canEnabled) { _cfg->canEnabled = newCanEnabled; canChanged = true; }
  applyParam("canBitrate", [&](String v){ long nb = v.toInt(); if (nb != _cfg->canBitrate) { _cfg->canBitrate = nb; canChanged = true; } });

  applyParam("pollIntervalS", [](String v){ _cfg->pollIntervalS = constrain(v.toInt(), 1, 30); });
  applyParam("piHost",        [](String v){ _cfg->piHost = v; });
  applyParam("rigToken",      [](String v){ _cfg->rigToken = v.isEmpty() ? "7804991970" : v; });

  bool wifiChanged = false;
  applyParam("wifiSSID", [&](String v){ if (v != _cfg->wifiSSID) { _cfg->wifiSSID = v; wifiChanged = true; } });
  applyParam("wifiPass", [](String v){ _cfg->wifiPass = v; });

  saveConfig(*_prefs, *_cfg);

  // Read the two baud keys straight back from NVS right now, in a fresh
  // Preferences handle — not from the in-RAM _cfg struct, which would
  // "verify" nothing (it's just proving RAM still has what we put there
  // 2 lines ago). This is the actual ground truth of what's on flash at
  // this exact moment, shown on the save confirmation itself so a silent
  // NVS write failure would be visible immediately instead of only
  // discovered after a reboot+guess cycle.
  Preferences verify;
  verify.begin("rigmod", true); // read-only
  long verifiedBaud = verify.getLong("mbBaud", -1);
  bool verifiedBaudSet = verify.getBool("mbBaudSet", false);
  verify.end();
  String verifyMsg = "<p class='small'>Verified on flash right now: mbBaud=" + String(verifiedBaud) +
    " mbBaudSet=" + String(verifiedBaudSet ? "true" : "false") + "</p>";
  if (verifiedBaud != _cfg->modbusBaud || verifiedBaudSet != _cfg->baudManuallySet) {
    verifyMsg = "<p class='small warn'><b>WARNING: readback mismatch!</b> Wrote baud=" + String(_cfg->modbusBaud) +
      " manuallySet=" + String(_cfg->baudManuallySet ? "true" : "false") +
      " but flash shows baud=" + String(verifiedBaud) + " manuallySet=" + String(verifiedBaudSet ? "true" : "false") +
      " — the NVS write did not stick.</p>";
  }

  if (wifiChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to connect to new WiFi...</p>" + verifyMsg);
    delay(1000);
    ESP.restart();
  } else if (baudChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to apply new RS485 baud rate...</p>" + verifyMsg);
    delay(1000);
    ESP.restart();
  } else if (canChanged) {
    // CAN driver install/uninstall needs a clean restart context (same
    // reasoning as RS485 baud) rather than trying to hot-swap it out from
    // under canPoll() running in loop().
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to apply CAN settings...</p>");
    delay(1000);
    ESP.restart();
  } else {
    _srv->sendHeader("Location", "/");
    _srv->send(302, "text/plain", "");
  }
}

// Saves the whole RS485 sensor list from /sensors' single shared form.
static void handleSensorsSave() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    String pre = "s" + String(i);
    SensorConfig& s = _cfg->sensors[i];
    s.enabled = _srv->hasArg((pre + "en").c_str());
    if (_srv->hasArg((pre + "nm").c_str()))  s.name = _srv->arg((pre + "nm").c_str());
    if (_srv->hasArg((pre + "kd").c_str()))  s.kind = _srv->arg((pre + "kd").c_str());
    if (_srv->hasArg((pre + "ut").c_str()))  s.unit = _srv->arg((pre + "ut").c_str());
    if (_srv->hasArg((pre + "sid").c_str())) s.slaveId = (uint8_t)constrain(_srv->arg((pre + "sid").c_str()).toInt(), 1, 247);
    if (_srv->hasArg((pre + "fc").c_str()))  s.funcCode = (uint8_t)_srv->arg((pre + "fc").c_str()).toInt();
    if (_srv->hasArg((pre + "reg").c_str())) s.regAddr = (uint16_t)strtol(_srv->arg((pre + "reg").c_str()).c_str(), nullptr, 0);
    if (_srv->hasArg((pre + "dt").c_str()))  s.dataType = (uint8_t)_srv->arg((pre + "dt").c_str()).toInt();
    if (_srv->hasArg((pre + "wo").c_str()))  s.wordOrder = (uint8_t)_srv->arg((pre + "wo").c_str()).toInt();
    if (_srv->hasArg((pre + "sc").c_str()))  s.scale = _srv->arg((pre + "sc").c_str()).toFloat();
    if (_srv->hasArg((pre + "of").c_str()))  s.offset = _srv->arg((pre + "of").c_str()).toFloat();
    s.volumeEnabled = _srv->hasArg((pre + "volEn").c_str());
    if (_srv->hasArg((pre + "cap").c_str())) s.capacity = _srv->arg((pre + "cap").c_str()).toFloat();
    if (_srv->hasArg((pre + "cu").c_str()))  s.capacityUnit = _srv->arg((pre + "cu").c_str());
    if (_srv->hasArg((pre + "vz").c_str()))  s.volZeroLevel = _srv->arg((pre + "vz").c_str()).toFloat();
    if (_srv->hasArg((pre + "vm").c_str()))  s.volMaxLevel = _srv->arg((pre + "vm").c_str()).toFloat();
  }
  saveConfig(*_prefs, *_cfg);
  _srv->sendHeader("Location", "/sensors");
  _srv->send(302, "text/plain", "");
}

// Saves the whole CAN signal list from /can's single shared form.
static void handleCanSave() {
  for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
    String pre = "c" + String(i);
    CanSignalConfig& sg = _cfg->canSignals[i];
    sg.enabled = _srv->hasArg((pre + "en").c_str());
    if (_srv->hasArg((pre + "nm").c_str())) sg.name = _srv->arg((pre + "nm").c_str());
    if (_srv->hasArg((pre + "kd").c_str())) sg.kind = _srv->arg((pre + "kd").c_str());
    if (_srv->hasArg((pre + "ut").c_str())) sg.unit = _srv->arg((pre + "ut").c_str());
    if (_srv->hasArg((pre + "id").c_str())) sg.canId = (uint32_t)strtoul(_srv->arg((pre + "id").c_str()).c_str(), nullptr, 16);
    if (_srv->hasArg((pre + "ext").c_str())) sg.extended = _srv->arg((pre + "ext").c_str()).toInt() != 0;
    if (_srv->hasArg((pre + "bo").c_str())) sg.byteOffset = (uint8_t)constrain(_srv->arg((pre + "bo").c_str()).toInt(), 0, 7);
    if (_srv->hasArg((pre + "bl").c_str())) sg.byteLen = (uint8_t)_srv->arg((pre + "bl").c_str()).toInt();
    if (_srv->hasArg((pre + "be").c_str())) sg.bigEndian = _srv->arg((pre + "be").c_str()).toInt() != 0;
    if (_srv->hasArg((pre + "sv").c_str())) sg.signedVal = _srv->arg((pre + "sv").c_str()).toInt() != 0;
    if (_srv->hasArg((pre + "sc").c_str())) sg.scale = _srv->arg((pre + "sc").c_str()).toFloat();
    if (_srv->hasArg((pre + "of").c_str())) sg.offset = _srv->arg((pre + "of").c_str()).toFloat();
  }
  saveConfig(*_prefs, *_cfg);
  _srv->sendHeader("Location", "/can");
  _srv->send(302, "text/plain", "");
}

static void handleWifiScan() {
  int found = WiFi.scanNetworks();
  DynamicJsonDocument doc(2048);
  JsonArray arr = doc.createNestedArray("networks");
  if (found > 0) {
    const int MAXN = 32;
    String seenSsid[MAXN];
    int seenRssi[MAXN];
    bool seenSecure[MAXN];
    int nSeen = 0;
    for (int i = 0; i < found; i++) {
      String ssid = WiFi.SSID(i);
      if (ssid.isEmpty()) continue;
      int rssi = WiFi.RSSI(i);
      bool secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
      int existing = -1;
      for (int j = 0; j < nSeen; j++) if (seenSsid[j] == ssid) { existing = j; break; }
      if (existing >= 0) {
        if (rssi > seenRssi[existing]) seenRssi[existing] = rssi;
      } else if (nSeen < MAXN) {
        seenSsid[nSeen] = ssid; seenRssi[nSeen] = rssi; seenSecure[nSeen] = secure; nSeen++;
      }
    }
    for (int i = 1; i < nSeen; i++) {
      String kS = seenSsid[i]; int kR = seenRssi[i]; bool kSec = seenSecure[i];
      int j = i - 1;
      while (j >= 0 && seenRssi[j] < kR) {
        seenSsid[j+1] = seenSsid[j]; seenRssi[j+1] = seenRssi[j]; seenSecure[j+1] = seenSecure[j];
        j--;
      }
      seenSsid[j+1] = kS; seenRssi[j+1] = kR; seenSecure[j+1] = kSec;
    }
    for (int i = 0; i < nSeen; i++) {
      JsonObject o = arr.createNestedObject();
      o["ssid"] = seenSsid[i]; o["rssi"] = seenRssi[i]; o["secure"] = seenSecure[i];
    }
  }
  WiFi.scanDelete();
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

static void handleWifiForget() {
  _cfg->wifiSSID = "";
  _cfg->wifiPass = "";
  saveConfig(*_prefs, *_cfg);
  _srv->send(200, "application/json", "{\"ok\":true}");
  delay(500);
  ESP.restart();
}

// Per-sensor "Auto-Detect Baud" button — probes all standard bauds against
// one specific slave ID (passed as a query param, doesn't need the full
// saved sensor config) and reboots on success so Serial2 picks it up clean.
static void handleModbusAutoDetect() {
  uint8_t slaveId = _p("slaveId").isEmpty() ? 1 : _p("slaveId").toInt();
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy, try again\"}");
    return;
  }
  long found = modbusAutoDetectBaud(slaveId, (uint32_t)_cfg->modbusBaud);
  xSemaphoreGive(modbusBusMutex);

  if (found > 0) {
    _cfg->modbusBaud = found;
    _cfg->baudManuallySet = true; // confirmed baud via active probe — lock it in
    saveConfig(*_prefs, *_cfg);
    String r = "{\"ok\":true,\"detected\":true,\"baud\":" + String(found) + "}";
    _srv->send(200, "application/json", r);
    delay(500);
    ESP.restart();
  } else {
    _srv->send(200, "application/json", "{\"ok\":true,\"detected\":false}");
  }
}

static void handleOTA() {
  String url = _srv->hasArg("url") ? _srv->arg("url") : "";
  if (url.isEmpty()) { _srv->send(400,"text/plain","No URL"); return; }
  _srv->send(200,"text/plain","OTA starting from: "+url+"\nCheck serial for progress.");
  delay(500);
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  if (code == 200) {
    int len = http.getSize();
    WiFiClient* stream = http.getStreamPtr();
    if (!Update.begin(len)) { Serial.println("[OTA] Not enough space"); http.end(); return; }
    size_t written = Update.writeStream(*stream);
    if (written == (size_t)len && Update.end()) {
      Serial.println("[OTA] Success, rebooting");
      ESP.restart();
    } else {
      Serial.printf("[OTA] Failed: wrote %d of %d\n", written, len);
      Update.printError(Serial);
    }
  } else {
    Serial.printf("[OTA] HTTP %d from %s\n", code, url.c_str());
  }
  http.end();
}

// ─── Route setup ─────────────────────────────────────────────────────────────
void setupWebRoutes(WebServer& srv, ModuleConfig& cfg, Preferences& prefs,
                    SensorReading* sReadings, CanSignalReading* cReadings,
                    SemaphoreHandle_t mtx) {
  _srv       = &srv;
  _cfg       = &cfg;
  _prefs     = &prefs;
  _sReadings = sReadings;
  _cReadings = cReadings;
  _mtx       = mtx;

  auto noCacheHtml = [](int code, const String& body){
    _srv->sendHeader("Cache-Control", "no-store");
    _srv->send(code, "text/html; charset=utf-8", body);
  };
  srv.on("/",         HTTP_GET, [noCacheHtml](){ noCacheHtml(200, cfgPage(*_cfg)); });
  srv.on("/sensors",  HTTP_GET, [noCacheHtml](){ noCacheHtml(200, sensorsPage(*_cfg)); });
  srv.on("/can",      HTTP_GET, [noCacheHtml](){ noCacheHtml(200, canPage(*_cfg)); });
  srv.on("/live",     HTTP_GET, [noCacheHtml](){ noCacheHtml(200, livePage()); });
  srv.on("/system",   HTTP_GET, [noCacheHtml](){ noCacheHtml(200, sysPage(*_cfg)); });

  // GET APIs
  srv.on("/api/status",       HTTP_GET, handleApiStatus);
  srv.on("/api/sensors/live", HTTP_GET, handleSensorsLive);
  srv.on("/api/can/live",     HTTP_GET, handleCanLive);
  srv.on("/api/modbus/probe", HTTP_GET, handleModbusProbe);
  srv.on("/api/wifi/scan",    HTTP_GET, handleWifiScan);
  srv.on("/api/ota",          HTTP_GET, handleOTA);

  // POST APIs
  srv.on("/api/config",         HTTP_POST, handleConfig);
  srv.on("/api/sensors/save",   HTTP_POST, handleSensorsSave);
  srv.on("/api/can/save",       HTTP_POST, handleCanSave);
  srv.on("/api/wifi/forget",    HTTP_POST, handleWifiForget);
  srv.on("/api/modbus/autodetect", HTTP_POST, handleModbusAutoDetect);
  srv.on("/api/modbus/autodetect-enable", HTTP_POST, handleAutoDetectEnable);

  srv.on("/api/buffer/flush", HTTP_POST, [](){
    flushNow = true;
    _srv->send(200,"application/json","{\"ok\":true}");
  });
  srv.on("/api/buffer/clear", HTTP_POST, [](){
    LittleFS.remove("/buffer.jsonl");
    bufferCount = 0;
    _srv->send(200,"application/json","{\"ok\":true}");
  });
  srv.on("/api/reboot", HTTP_POST, [](){
    _srv->send(200,"application/json","{\"ok\":true}");
    delay(500); ESP.restart();
  });
  srv.on("/api/factory-reset", HTTP_POST, [](){
    _srv->send(200,"application/json","{\"ok\":true}");
    delay(200);
    _prefs->begin("rigmod",false); _prefs->clear(); _prefs->end();
    LittleFS.format();
    delay(500); ESP.restart();
  });
}
