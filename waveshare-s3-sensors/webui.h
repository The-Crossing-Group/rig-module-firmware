// =============================================================================
// webui.h — WebServer routes: config + diagnostics UI + REST API
// Direct-Sensor Rig Module variant. Pages:
//   /            Config (module info, RS485 baud, CAN enable/bitrate, WiFi)
//   /sensors     Add/edit/remove RS485 Modbus sensors (list, not fixed 8)
//   /can         Add/edit/remove CAN signals + live raw frame sniffer
//   /live        Live values table (sensors + CAN signals)
//   /diag        Debugging diagnostics: bus scan, register probe, CAN
//                frame log, comms error counters — the "figure out what's
//                actually on the bus" toolbox
//   /system      Firmware info, OTA, buffer, reboot/factory-reset
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
// modbusDecodeValue(), modbusScanSlaves(), modbusGetRecentLog() are all
// already declared+defined in modbus.h, included before this file in the
// .ino — no forward decls here. (Previously duplicated declarations here
// caused "ambiguous overload" compile errors once modbus.h's real
// signatures grew default parameters that these stale copies didn't have.)
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
  <a href='/diag'>&#128269; Diagnostics</a>
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
  h += "<div class='small'>Shown on the rig dashboard's module card. Type anything, or pick a suggestion.</div>";
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
  h += "<div class='small'>Shared by every sensor on the bus — all your RS485 sensors must use the same baud rate "
       "(that's a Modbus RTU / RS485 wiring requirement, not something this firmware can work around). Use the "
       "<a href='/diag'>Diagnostics</a> page's bus scan to find sensors and confirm this is right.</div>";

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
  h += "<div class='small'>Runs in <b>listen-only</b> mode — this module never transmits on the CAN bus, only "
       "reads. Not sure what's on the bus yet? Enable it and check the <a href='/diag'>Diagnostics</a> page's "
       "live raw frame sniffer — see actual traffic before defining any signals on the <a href='/can'>CAN</a> page.</div>";

  h += "<h3>Pi Logger</h3>";
  h += "<label>Poll Interval (1-30 s)</label><input name='pollIntervalS' type='number' min='1' max='30' value='" + String(cfg.pollIntervalS) + "'>";
  h += "<label>Pi Host (blank = auto)</label><input name='piHost' value='" + cfg.piHost + "' placeholder='192.168.x.x or rig-logger.local'>";
  h += "<div class='small'>Leave blank for auto-discovery via the standard \"rigNNN\" convention or mDNS.</div>";
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
  h += "<div class='small'>Saving with a changed SSID/password reboots the unit. No saved network? This unit "
       "auto-scans for a standard rig router (SSID like rig132) first, before falling back to its own setup AP.</div>";
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
  h += "<p class='small'>Each sensor here is an independent Modbus RTU slave on the shared RS485 bus — its own "
       "slave ID, register, data type, and scaling. Works with any Modbus sensor (pressure, temp, flow, level...). "
       "Don't know a sensor's slave ID or register yet? Use the <a href='/diag'>Diagnostics</a> page's bus scan "
       "and register probe first.</p>";
  h += "<div class='card' style='border-color:#27ae60'><b>&#9889; Auto-Detect &amp; Enable</b><br>"
       "<span class='small'>Scans the bus (addresses 1-16) and automatically enables any sensor found that isn't "
       "already configured — fills in slave ID + func code, register 0, uint16, scale 1 as a starting point. "
       "If nothing's configured yet and nothing answers at the current baud, it also sweeps the other standard "
       "baud rates and adopts whichever one a sensor actually responds at (saved as the module's RS485 baud). "
       "Once at least one sensor is enabled, the baud is locked in — every sensor on this bus has to share it "
       "anyway. You'll still want to dial in the real register/data type/scale for a meaningful reading, but "
       "this gets a freshly-wired sensor reporting <i>something</i> immediately without touching this page. Also "
       "runs automatically on boot and every few minutes in the background.</span><br><br>"
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
    h += "<div><label>Scale (value = raw * scale + offset)</label><input name='s" + String(i) + "sc' type='number' step='any' value='" + String(s.scale) + "'></div>";
    h += "<div><label>Offset</label><input name='s" + String(i) + "of' type='number' step='any' value='" + String(s.offset) + "'></div></div>";

    h += "<label style='margin-top:8px'><input type='checkbox' name='s" + String(i) + "volEn' id='volEn" + String(i) + "' onchange='toggleVol(" + String(i) + ")'";
    if (s.volumeEnabled) h += " checked";
    h += "> Compute Tank Volume from this sensor</label>";
    h += "<div id='volFields" + String(i) + "' style='display:" + String(s.volumeEnabled ? "block" : "none") + "'>";
    h += "<div class='small'>This sensor's value at \"Empty\" = 0 volume, value at \"Full\" = Capacity.</div>";
    h += "<div class='row'><div><label>Capacity</label><input name='s" + String(i) + "cap' type='number' step='any' min='0' value='" + String(s.capacity) + "'></div>";
    h += "<div><label>Unit</label><select name='s" + String(i) + "cu'>";
    h += "<option value='m3'" + String(s.capacityUnit == "m3" ? " selected" : "") + ">m&#179;</option>";
    h += "<option value='gal'" + String(s.capacityUnit == "gal" ? " selected" : "") + ">gal</option></select></div></div>";
    h += "<div class='row'><div><label>Value @ Empty</label><input name='s" + String(i) + "vz' type='number' step='any' value='" + String(s.volZeroLevel) + "'></div>";
    h += "<div><label>Value @ Full</label><input name='s" + String(i) + "vm' type='number' step='any' value='" + String(s.volMaxLevel) + "'></div></div>";
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
         "<a href='/'>Config</a> page first, then use the <a href='/diag'>Diagnostics</a> page's raw frame "
         "sniffer to see what's actually on the bus before defining signals below.</div>";
  } else {
    h += "<div class='card'>CAN running at " + String(cfg.canBitrate) + " bit/s, listen-only. "
         "Total frames seen: <span id='canTotal'>...</span>, recent rate: <span id='canRate'>...</span> fps. "
         "See raw frames on the <a href='/diag'>Diagnostics</a> page.</div>";
  }
  h += "<p class='small'>Each signal pulls a byte range out of frames matching a specific CAN ID and decodes it "
       "into a value — the same idea as a Modbus sensor, just sourced from CAN. Define these once you've "
       "identified a pattern in the raw frames (Diagnostics page).</p>";
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
    h += "<div><label>Scale</label><input name='c" + String(i) + "sc' type='number' step='any' value='" + String(sg.scale) + "'></div>";
    h += "<div><label>Offset</label><input name='c" + String(i) + "of' type='number' step='any' value='" + String(sg.offset) + "'></div>";
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

// ─── /diag  DEBUGGING DIAGNOSTICS ───────────────────────────────────────────
static String diagPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128269; Diagnostics</h2>";

  h += "<h3>RS485 Bus Scan</h3><div class='card'>";
  h += "<p class='small'>Probes every Modbus slave address 1-247 at the current baud rate (" + String(cfg.modbusBaud) +
       ") and reports which ones respond. Takes up to ~30s depending on how many addresses time out (no response "
       "is the slow case — a wired sensor answers almost instantly). Use this to find a new sensor's slave ID "
       "before adding it on the <a href='/sensors'>Sensors</a> page. (This just reports what's out there — it "
       "doesn't configure anything. For auto-enabling new sensors automatically, see \"Auto-Detect &amp; Enable\" "
       "on the <a href='/sensors'>Sensors</a> page, which also runs on its own on boot and every few minutes.)</p>";
  h += "<div class='row'><div><label>Scan up to address</label><input id='scanMax' type='number' min='1' max='247' value='32'></div>";
  h += "<button type='button' onclick='runScan()' id='scanBusBtn' style='margin-top:18px'>&#128269; Scan Bus</button></div>";
  h += "<div id='scanBusResult' class='small'></div></div>";

  h += "<h3>Register Probe</h3><div class='card'>";
  h += "<p class='small'>Read a specific register from a specific slave right now, without saving a sensor "
       "config — the fastest way to confirm a slave ID/register/data type combo before committing to it.</p>";
  h += "<div class='grid4'>";
  h += "<div><label>Slave ID</label><input id='probeSid' type='number' min='1' max='247' value='1'></div>";
  h += "<div><label>Function Code</label><select id='probeFc'><option value='4'>04 - Input Regs</option><option value='3'>03 - Holding Regs</option></select></div>";
  h += "<div><label>Register (hex or dec)</label><input id='probeReg' value='0'></div>";
  h += "<div><label>Data Type</label><select id='probeDt'><option value='0'>uint16</option><option value='1'>int16</option><option value='2'>uint32</option><option value='3'>int32</option><option value='4'>float32</option></select></div>";
  h += "</div>";
  h += "<button type='button' onclick='runProbe()'>&#128269; Probe Register</button>";
  h += "<div id='probeResult' class='small' style='margin-top:8px'></div></div>";

  h += "<h3>Register Write</h3><div class='card'>";
  h += "<p class='small'><b>Caution:</b> writes a value directly to a sensor's config register (Modbus FC06 - "
       "Write Single Register). Only use this if a sensor's datasheet/manual documents a specific register "
       "address and value — e.g. switching a radar/ultrasonic level sensor from a slow/filtered measurement "
       "mode into a fast mode, setting a response-time register, or programming range/blind-zone. Writing the "
       "wrong register or value on a sensor that doesn't expect it can put it into an unexpected state — double "
       "check against the manual first.</p>";
  h += "<p class='small'>Known presets below are confirmed from the SONBEST/Sonbust manufacturer manual (this "
       "sensor family) — they auto-fill the register for you, but you still need to work out the right value. "
       "There is currently no known/documented register for a \"fast measurement mode\" or acquisition-time "
       "setting on this sensor — the manual doesn't cover it. If you get that from the seller/manufacturer, "
       "use \"Custom / other\" below with the register + value they give you.</p>";
  h += "<div class='row'><div><label>Preset</label><select id='wrPreset' onchange='wrPresetChange()'>";
  h += "<option value='custom'>Custom / other (enter register manually)</option>";
  h += "<option value='0x006B'>Correction/calibration offset (reg 0x006B) — add or subtract from every reading. "
       "Value 0-1000 = add that many raw units; 64535-65535 = negative (e.g. 65535-100+1=65436 subtracts 100)</option>";
  h += "<option value='0x0066'>Device (slave) address (reg 0x0066) — value = new address 1-249</option>";
  h += "<option value='0x0067'>Baud rate (reg 0x0067) — value: 1=2400 2=4800 3=9600 4=19200 5=38400 6=115200. "
       "Sensor goes silent at old baud immediately after — update your sensor config's baud to match</option>";
  h += "</select></div></div>";
  h += "<div class='grid4'>";
  h += "<div><label>Slave ID</label><input id='wrSid' type='number' min='1' max='247' value='1'></div>";
  h += "<div><label>Register (hex or dec)</label><input id='wrReg' value='0'></div>";
  h += "<div><label>Value (hex or dec)</label><input id='wrVal' value='0'></div>";
  h += "<div></div>";
  h += "</div>";
  h += "<button type='button' onclick='runWrite()'>&#9888; Write Register</button>";
  h += "<div id='wrResult' class='small' style='margin-top:8px'></div></div>";

  h += "<h3>RS485 Sensor Comms Health</h3><div class='card'><div id='commsHealth'>Loading...</div></div>";

  h += "<h3>RS485 Raw Traffic</h3><div class='card'>";
  h += "<p class='small'>Every Modbus request/response, byte-for-byte, as it actually happens — background polling "
       "AND manual probes/scans. Newest first. <span class='ok'>ok</span> = valid CRC + expected length; "
       "<span class='timeout'>timeout</span> = no response at all (check wiring/baud/slave ID); "
       "<span class='crc'>crc</span> = got bytes back but they don't check out (noise, wrong baud, wiring issue).</p>";
  h += "<label><input type='checkbox' id='rawPause'> Pause</label>";
  h += "<div style='max-height:400px;overflow-y:auto;margin-top:8px'><table class='mono'><thead><tr>"
       "<th>Age (s)</th><th>Slave</th><th>FC</th><th>TX (hex)</th><th>RX (hex)</th><th>Result</th></tr></thead>"
       "<tbody id='mbLogBody'></tbody></table></div></div>";

  h += "<h3>CAN Raw Frame Sniffer</h3><div class='card'>";
  if (!cfg.canEnabled) {
    h += "<p class='small'>CAN is disabled — enable it on the <a href='/'>Config</a> page to see live traffic here.</p>";
  } else {
    h += "<p class='small'>Live capture of the most recent frames seen on the bus (newest first). Look for a "
         "consistent ID reporting a value you recognize (e.g. matches a known pressure/RPM reading), note its "
         "byte offset, then define it as a signal on the <a href='/can'>CAN</a> page.</p>";
    h += "<div>Total frames: <span id='diagCanTotal'>...</span> &nbsp; Rate: <span id='diagCanRate'>...</span> fps &nbsp; "
         "Last frame: <span id='diagCanLast'>...</span></div>";
    h += "<div style='max-height:400px;overflow-y:auto;margin-top:8px'><table class='mono'><thead><tr>"
         "<th>Age (s)</th><th>ID</th><th>Ext</th><th>DLC</th><th>Data (hex)</th></tr></thead><tbody id='canFrameBody'></tbody></table></div>";
  }
  h += "</div>";

  h += R"(
<script>
function runScan(){
  let btn=document.getElementById('scanBusBtn'); let box=document.getElementById('scanBusResult');
  let max=document.getElementById('scanMax').value;
  btn.disabled=true; btn.textContent='Scanning...';
  box.textContent='Scanning addresses 1-'+max+'... this can take a while for a mostly-empty range.';
  fetch('/api/modbus/scan?max='+max).then(r=>r.json()).then(d=>{
    btn.disabled=false; btn.innerHTML='&#128269; Scan Bus';
    if(d.found.length===0){ box.textContent='No slaves responded. Check wiring, baud rate, and DE pin.'; return; }
    box.innerHTML = 'Found ' + d.found.length + ' slave(s): <b>' + d.found.join(', ') + '</b>';
  }).catch(e=>{ btn.disabled=false; btn.innerHTML='&#128269; Scan Bus'; box.textContent='Scan failed: '+e; });
}
function runProbe(){
  let sid=document.getElementById('probeSid').value;
  let fc=document.getElementById('probeFc').value;
  let reg=document.getElementById('probeReg').value;
  let dt=document.getElementById('probeDt').value;
  let box=document.getElementById('probeResult');
  probeWithRetry('/api/modbus/probe?slaveId='+sid+'&funcCode='+fc+'&reg='+reg+'&dataType='+dt+'&wordOrder=0', box,
    (d)=>'<span class="ok">OK</span> — raw registers: ['+d.regs.join(', ')+']  decoded: <b>'+d.decoded.toFixed(4)+'</b>',
    (d)=>'<span class="timeout">FAILED</span> — '+d.error);
}
function wrPresetChange(){
  let p=document.getElementById('wrPreset').value;
  if(p==='custom') return;
  document.getElementById('wrReg').value = p;
}
function runWrite(){
  let sid=document.getElementById('wrSid').value;
  let reg=document.getElementById('wrReg').value;
  let val=document.getElementById('wrVal').value;
  let box=document.getElementById('wrResult');
  if(!confirm('Write value '+val+' to register '+reg+' on slave '+sid+'? Make sure this matches the sensor\'s documented register map.')) return;
  box.textContent='Writing...';
  fetch('/api/modbus/write?slaveId='+sid+'&reg='+reg+'&value='+val,{method:'POST'})
    .then(r=>r.json()).then(d=>{
      if(d.ok) box.innerHTML = '<span class="ok">OK</span> — sensor confirmed register '+d.reg+' = '+d.value;
      else box.innerHTML = '<span class="timeout">FAILED</span> — '+d.error;
    }).catch(e=>{ box.textContent='Request failed: '+e; });
}
function fetchCommsHealth(){
  fetch('/api/sensors/live').then(r=>r.json()).then(d=>{
    let s='<table><tr><th>Sensor</th><th>Slave</th><th>Polls</th><th>Errors</th><th>Last OK</th><th>Status</th></tr>';
    d.sensors.forEach(x=>{
      if(!x.enabled) return;
      let lastOk = x.lastOkAgoMs!=null ? (x.lastOkAgoMs/1000).toFixed(0)+'s ago' : 'never';
      s += '<tr><td>'+(x.name||'Sensor '+(x.idx+1))+'</td><td>'+x.slaveId+'</td><td>'+x.pollCount+'</td>'+
           '<td class="'+(x.errorCount>0?'warn':'')+'">'+x.errorCount+'</td><td>'+lastOk+'</td>'+
           '<td class="'+x.status+'">'+x.status+'</td></tr>';
    });
    s+='</table>';
    document.getElementById('commsHealth').innerHTML = s || 'No sensors configured yet.';
  });
}
setInterval(fetchCommsHealth, 3000); fetchCommsHealth();
function fetchCanFrames(){
  fetch('/api/can/frames').then(r=>r.json()).then(d=>{
    document.getElementById('diagCanTotal') && (document.getElementById('diagCanTotal').textContent = d.frameTotal);
    document.getElementById('diagCanRate') && (document.getElementById('diagCanRate').textContent = d.frameRate);
    document.getElementById('diagCanLast') && (document.getElementById('diagCanLast').textContent = d.lastFrameAgoMs!=null ? (d.lastFrameAgoMs/1000).toFixed(1)+'s ago' : 'never');
    let body = document.getElementById('canFrameBody');
    if(!body) return;
    let s = '';
    d.frames.forEach(f=>{
      s += '<tr><td>'+(f.ageMs/1000).toFixed(1)+'</td><td>'+f.id+'</td><td>'+(f.ext?'Y':'N')+'</td><td>'+f.dlc+'</td><td>'+f.data+'</td></tr>';
    });
    body.innerHTML = s;
  });
}
if(document.getElementById('canFrameBody')) setInterval(fetchCanFrames, 1000);
fetchCanFrames();
function fetchModbusLog(){
  if(document.getElementById('rawPause') && document.getElementById('rawPause').checked) return;
  fetch('/api/modbus/log').then(r=>r.json()).then(d=>{
    let body = document.getElementById('mbLogBody');
    if(!body) return;
    let s = '';
    d.log.forEach(e=>{
      let cls = e.result==='ok'?'ok':e.result;
      s += '<tr><td>'+(e.ageMs/1000).toFixed(1)+'</td><td>'+e.slaveId+'</td><td>'+e.funcCode+'</td>'+
           '<td>'+e.tx+'</td><td>'+(e.rx||'<span class="stale">(none)</span>')+'</td>'+
           '<td class="'+cls+'">'+e.result+'</td></tr>';
    });
    body.innerHTML = s || '<tr><td colspan="6" class="small">No traffic yet — add a sensor on /sensors or run a probe/scan above.</td></tr>';
  });
}
if(document.getElementById('mbLogBody')) setInterval(fetchModbusLog, 1000);
fetchModbusLog();
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

// Live sensor readings for the /sensors + /diag pages — richer than the
// payload's "channels" array (includes disabled slots, error counters,
// slave IDs) since this is a diagnostics/editing view, not the wire format.
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
      // "status" = raw per-poll result, used by /diag's comms health
      // table (shows every real timeout/CRC error, unfiltered — that
      // page exists specifically to see what's actually happening).
      // "displayStatus" = debounced version for the /sensors page's own
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

// Raw CAN frame log for the Diagnostics page sniffer.
static void handleCanFrames() {
  DynamicJsonDocument doc(6144);
  doc["frameTotal"] = canGetFrameTotal();
  doc["frameRate"]  = canGetRecentFrameRate();
  unsigned long lastMs = canGetLastFrameMs();
  if (lastMs) doc["lastFrameAgoMs"] = (long)(millis() - lastMs);
  else doc["lastFrameAgoMs"] = nullptr;
  doc["serverNowMs"] = (long)millis();

  JsonArray arr = doc.createNestedArray("frames");
  CanFrameLog frames[CAN_LOG_SIZE];
  int n = canGetRecentFrames(frames, CAN_LOG_SIZE);
  unsigned long now = millis();
  for (int i = 0; i < n; i++) {
    JsonObject o = arr.createNestedObject();
    char idHex[12];
    snprintf(idHex, sizeof(idHex), "0x%lX", (unsigned long)frames[i].id);
    o["id"]  = idHex;
    o["ext"] = frames[i].extended;
    o["dlc"] = frames[i].dlc;
    o["ageMs"] = (long)(now - frames[i].ms);
    char dataHex[24] = {0};
    int p = 0;
    for (int b = 0; b < frames[i].dlc && b < 8; b++) {
      p += snprintf(dataHex + p, sizeof(dataHex) - p, "%02X ", frames[i].data[b]);
    }
    o["data"] = String(dataHex);
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Raw Modbus TX/RX log for the Diagnostics page — every actual
// transaction (background polling AND manual probes), byte-for-byte, no
// USB cable required. This is the direct answer to "let me see what the
// sensor is actually saying."
static void handleModbusLog() {
  DynamicJsonDocument doc(8192);
  doc["serverNowMs"] = (long)millis();
  JsonArray arr = doc.createNestedArray("log");
  ModbusRawLogEntry entries[MODBUS_LOG_SIZE];
  int n;
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    n = modbusGetRecentLog(entries, MODBUS_LOG_SIZE);
    xSemaphoreGive(modbusBusMutex);
  } else {
    n = 0;
  }
  unsigned long now = millis();
  for (int i = 0; i < n; i++) {
    JsonObject o = arr.createNestedObject();
    o["ageMs"]    = (long)(now - entries[i].ms);
    o["slaveId"]  = entries[i].slaveId;
    o["funcCode"] = entries[i].funcCode;
    o["result"]   = (entries[i].result == MB_OK) ? "ok" :
                     (entries[i].result == MB_TIMEOUT) ? "timeout" :
                     (entries[i].result == MB_CRC_ERROR) ? "crc" : "bad";
    char txHex[32] = {0}; int p = 0;
    for (int b = 0; b < entries[i].txLen; b++) p += snprintf(txHex + p, sizeof(txHex) - p, "%02X ", entries[i].tx[b]);
    o["tx"] = String(txHex);
    char rxHex[128] = {0}; p = 0;
    for (int b = 0; b < entries[i].rxLen && p < (int)sizeof(rxHex) - 4; b++) p += snprintf(rxHex + p, sizeof(rxHex) - p, "%02X ", entries[i].rx[b]);
    o["rx"] = String(rxHex);
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Reads a specific register combo from a specific slave right now —
// backs both the /diag Register Probe tool and the per-sensor "Probe Now"
// button on /sensors. Doesn't touch saved config at all.
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
  int rc = modbusReadRegs(slaveId, funcCode, regAddr, n, regs, true);
  xSemaphoreGive(modbusBusMutex);

  DynamicJsonDocument doc(512);
  if (rc == MB_OK) {
    float decoded = modbusDecodeValue(regs, dataType, wordOrder);
    doc["ok"] = true;
    doc["raw"] = regs[0];
    JsonArray regsArr = doc.createNestedArray("regs");
    for (int i = 0; i < n; i++) regsArr.add(regs[i]);
    doc["decoded"] = decoded;
  } else {
    doc["ok"] = false;
    doc["error"] = (rc == MB_TIMEOUT) ? "timeout — no response" :
                   (rc == MB_CRC_ERROR) ? "CRC error" : "bad response";
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Writes a single register to a slave right now (Modbus FC06). Backs
// the /diag "Register Write" tool — for sensor-side config registers
// (measurement mode, response time, range/blind-zone, etc.) that a
// datasheet documents but this firmware has no dedicated field for.
// Deliberately manual/one-shot — nothing here is saved to our own NVS
// config; it's a direct write to the SENSOR's own memory.
static void handleModbusWrite() {
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy\"}");
    return;
  }
  uint8_t slaveId = _p("slaveId").toInt();
  String regStr = _p("reg");
  String valStr = _p("value");
  uint16_t regAddr = (uint16_t)strtol(regStr.c_str(), nullptr, 0);
  uint16_t value = (uint16_t)strtol(valStr.c_str(), nullptr, 0);

  int rc = modbusWriteReg(slaveId, regAddr, value, true);
  xSemaphoreGive(modbusBusMutex);

  DynamicJsonDocument doc(256);
  if (rc == MB_OK) {
    doc["ok"] = true;
    doc["reg"] = regAddr;
    doc["value"] = value;
  } else {
    doc["ok"] = false;
    doc["error"] = (rc == MB_TIMEOUT) ? "timeout — no response" :
                   (rc == MB_CRC_ERROR) ? "CRC error" :
                   (rc == MB_BAD_RESPONSE) ? "sensor rejected write (exception response)" : "bad response";
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Bus-wide slave scan for the Diagnostics page — synchronous/blocking,
// holds modbusBusMutex for the whole scan so the poll task can't
// interleave. Deliberately capped by the caller's "max" param since a
// full 1-247 scan is slow when most addresses are empty (each miss is a
// ~400ms*2 timeout — FC04 then FC03 both get tried per address).
static void handleModbusScan() {
  int maxAddr = _p("max").isEmpty() ? 32 : _p("max").toInt();
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;

  DynamicJsonDocument doc(1024);
  JsonArray found = doc.createNestedArray("found");

  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(60000)) == pdTRUE) {
    modbusScanSlaves(maxAddr, [&](int addr, int fc) { found.add(addr); });
    xSemaphoreGive(modbusBusMutex);
  }

  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Auto-Detect & Enable — scans the bus and fills in/enables sensor slots
// for anything new found, saving config if anything changed. Backs both
// the boot-time/periodic background pass and the manual button on
// /sensors. Synchronous — same blocking-scan caveat as handleModbusScan.
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
  applyParam("modbusBaud", [&](String v){ long nb = v.toInt(); if (nb != _cfg->modbusBaud) { _cfg->modbusBaud = nb; baudChanged = true; } });

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

  if (wifiChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to connect to new WiFi...</p>");
    delay(1000);
    ESP.restart();
  } else if (baudChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to apply new RS485 baud rate...</p>");
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
    if (!Update.begin(len)) { Serial.println("[OTA] Not enough space"); return; }
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
  srv.on("/diag",     HTTP_GET, [noCacheHtml](){ noCacheHtml(200, diagPage(*_cfg)); });
  srv.on("/system",   HTTP_GET, [noCacheHtml](){ noCacheHtml(200, sysPage(*_cfg)); });

  // GET APIs
  srv.on("/api/status",       HTTP_GET, handleApiStatus);
  srv.on("/api/sensors/live", HTTP_GET, handleSensorsLive);
  srv.on("/api/can/live",     HTTP_GET, handleCanLive);
  srv.on("/api/can/frames",   HTTP_GET, handleCanFrames);
  srv.on("/api/modbus/probe", HTTP_GET, handleModbusProbe);
  srv.on("/api/modbus/write", HTTP_POST, handleModbusWrite);
  srv.on("/api/modbus/scan",  HTTP_GET, handleModbusScan);
  srv.on("/api/modbus/log",   HTTP_GET, handleModbusLog);
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
