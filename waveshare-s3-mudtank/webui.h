// =============================================================================
// webui.h — WebServer routes: merged config UI + REST API for the
// combined mudtank variant.
//
// Pages:
//   /            Config (module info, RS485 baud, board override, CAN,
//                WiFi) — now also links Config Export/Import at the
//                bottom (see /api/config/export, /api/config/import).
//   /channels    Fixed adapter board's 8 (or 6) per-channel config
//   /digital     Fixed adapter board's AMIDJ14 digital I/O + Pulse Counter
//   /sensors     Independent RS485 sensor list (add/edit/remove, Probe
//                Now, Auto-Detect Baud, bus-wide Auto-Detect & Enable)
//   /can         CAN signal list
//   /advanced    Hidden power-user page: extra fixed boards + slave ID
//                bus scan
//   /live        Live values (fixed board channels + independent sensors
//                + CAN signals, all in one table)
//   /system      Firmware info, OTA, buffer, reboot/factory-reset, NVS
//                diagnostics, config export/import
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
extern uint16_t rawModbus[8];
extern ChannelReading readings[8];
extern DigitalReading dinReadings[4];
extern DigitalReading doutReadings[4];
extern SensorReading    sensorReadings[MAX_SENSORS];
extern CanSignalReading canReadings[MAX_CAN_SIGNALS];
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t modbusBusMutex;
extern BoardProfile boardProfile; // .ino — detected once at poll-task startup

extern BoardProfile   extraBoardProfile[MAX_EXTRA_BOARDS];
extern ChannelReading extraReadings[MAX_EXTRA_BOARDS][8];
extern DigitalReading extraDinReadings[MAX_EXTRA_BOARDS][4];
extern DigitalReading extraDoutReadings[MAX_EXTRA_BOARDS][4];
extern NTPClient ntpClient;
extern bool apModeActive;
extern String apSSID;
// FW_VERSION is a #define in config.h — no extern needed

// modbus.h already declares/defines: modbusAutoDetectBaud(),
// modbusDetectBoard(), modbusWriteDO(), boardProfileForType(),
// modbusScanSlaves(), modbusScanSlavesWithBoardId(), modbusReadRegs(),
// modbusRegCount(), modbusDecodeValue(), modbusAutoDetectAndEnable() — all
// included before this file, no forward decls needed here.
bool canStart(int txPin, int rxPin, long bitrate); // can.h
void canStop(); // can.h
bool canIsRunning(); // can.h
int canGetRecentFrames(struct CanFrameLog* out, int maxCount); // can.h
unsigned long canGetFrameTotal(); // can.h
unsigned long canGetLastFrameMs(); // can.h
int canGetRecentFrameRate(); // can.h

// Waveshare ESP32-S3-RS485-CAN pin constants (declared in the main .ino) —
// only needed here for the CAN enable/disable handler to (re)start it.
#define WEBUI_CAN_TXD 15
#define WEBUI_CAN_RXD 16

// Globals set in setupWebRoutes, used by handlers
static WebServer*        _srv    = nullptr;
static ModuleConfig*      _cfg    = nullptr;
static Preferences*       _prefs  = nullptr;
static ChannelReading*    _readings = nullptr;
static uint16_t*          _raw    = nullptr;
static SensorReading*     _sReadings = nullptr;
static CanSignalReading*  _cReadings = nullptr;
static SemaphoreHandle_t  _mtx = nullptr;

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
  input[type=file]{padding:6px}
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
  <a href='/channels'>&#128208; Channels</a>
  <a href='/digital'>&#128268; Digital I/O</a>
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

// Arduino's String(float) defaults to only 2 decimal places — real bug
// for round-tripping config values BACK into an editable form field (a
// value smaller than 0.01 would render as "0.00" and get silently
// overwritten with 0 if the form is resubmitted without manual re-entry).
static String _f(float v) {
  String s = String(v, 6);
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
  h += "<div class='card'><b>Module ID:</b> " + cfg.moduleId + " <span class='small'>(fixed, derived from MAC — not editable)</span></div>";
  h += "<form method='POST' action='/api/config'>";
  h += "<label>Module Name</label><input name='moduleName' value='" + cfg.moduleName + "' placeholder='e.g. Mud Pump Skid, Water Tank 2'>";
  h += "<label>Module Type</label><input name='moduleType' list='moduleTypeOpts' value='" + cfg.moduleType + "' placeholder='e.g. tank, pump, pressure, drill'>";
  h += "<datalist id='moduleTypeOpts'><option value='generic'><option value='tank'><option value='pump'><option value='pressure'><option value='drill'></datalist>";
  h += "<div class='small'>Shown on the rig dashboard's module card. \"tank\" and \"pump\" get their "
       "own dedicated layout; anything else just shows as a plain labeled tile view.</div>";
  h += "<label>Description</label><input name='description' value='" + cfg.description + "'>";

  h += "<h3>RS485 / Modbus Bus (shared by fixed board + independent sensors)</h3>";
  h += "<label>RS485 Baud Rate</label><select name='modbusBaud'>";
  {
    struct { long val; const char* label; } bauds[] = {
      { 1200, "1200" }, { 2400, "2400" }, { 4800, "4800 (SDSIN)" }, { 9600, "9600 (Waveshare / most common)" },
      { 19200, "19200" }, { 38400, "38400" }, { 57600, "57600" }, { 115200, "115200" },
    };
    for (auto& b : bauds) {
      h += "<option value='" + String(b.val) + "'";
      if (cfg.modbusBaud == b.val) h += " selected";
      h += ">" + String(b.label) + "</option>";
    }
  }
  h += "</select>";
  h += "<div class='small'>ALL devices on this bus — the fixed adapter board and every independent "
       "sensor — must use this same baud rate.</div>";
  if (cfg.baudManuallySet) {
    h += "<div class='small ok'>&#128274; Baud is locked — auto-detect will not change this without you setting it again.</div>";
  } else {
    h += "<div class='small warn'>&#128275; Baud not yet locked — background auto-detect may still adjust this if nothing is "
         "configured yet. Save this form (or run any Auto-Detect Baud button) to lock it in.</div>";
  }

  h += "<h3>Fixed Adapter Board</h3>";
  h += "<label>Modbus Slave ID</label><input name='modbusSlaveId' type='number' min='1' max='247' value='" + String(cfg.modbusSlaveId) + "'>";
  h += "<label>Board Type</label><select name='boardOverride'>";
  {
    struct { const char* val; const char* label; } boards[] = {
      { "auto",     "Auto-Detect (default)" },
      { "waveshare", "Force: Waveshare 8AI (B)" },
      { "amidj14",   "Force: Eletechsup AMIDJ14" },
    };
    for (auto& b : boards) {
      h += "<option value='" + String(b.val) + "'";
      if (cfg.boardOverride == b.val) h += " selected";
      h += ">" + String(b.label) + "</option>";
    }
  }
  h += "</select>";
  h += "<div class='small' style='margin:6px 0 10px'>Leave on Auto-Detect normally — it reads the "
       "board's Product ID register at boot and picks the right channel count/scaling automatically. "
       "Only force a specific board if auto-detect is picking the wrong one. Takes effect on next reboot.</div>";
  h += "<button type='button' id='autoBaudBtn' onclick='autoDetectBaud()'>&#128269; Auto-Detect Baud Rate</button>"
       "<div class='small' id='autoBaudStatus' style='margin:6px 0 10px'>Probes the connected board at every "
       "standard rate and picks whichever gets a real response.</div>"
       "<script>"
       "function autoDetectBaud(){"
       "var btn=document.getElementById('autoBaudBtn');var st=document.getElementById('autoBaudStatus');"
       "btn.disabled=true;btn.textContent='Probing bus...';st.textContent='Trying each baud rate against the wired board — a few seconds...';"
       "fetch('/api/modbus/autodetect',{method:'POST'}).then(r=>r.json()).then(d=>{"
       "btn.disabled=false;btn.textContent='\\u{1F50D} Auto-Detect Baud Rate';"
       "if(d.detected){st.textContent='Found it: '+d.baud+' baud. Saved \\u2014 reloading...';setTimeout(()=>location.reload(),1200);}"
       "else{st.textContent='No response at any standard baud. Check wiring/DE pin/board power, then try again.';}"
       "}).catch(e=>{btn.disabled=false;btn.textContent='\\u{1F50D} Auto-Detect Baud Rate';st.textContent='Request failed: '+e;});"
       "}"
       "</script>";

  h += "<h3>CAN Bus (independent, listen-only)</h3>";
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
  h += "<div class='small'>Listen-only — this firmware never transmits on the CAN bus.</div>";

  h += "<h3>Pi Logger</h3>";
  h += "<label>Poll Interval (1-30 s)</label><input name='pollIntervalS' type='number' min='1' max='30' value='" + String(cfg.pollIntervalS) + "'>";
  h += "<label>Pi Host (blank = auto)</label><input name='piHost' value='" + cfg.piHost + "' placeholder='192.168.x.x or rig-logger.local'>";
  h += "<div class='small'>Leave blank for auto-discovery: on a standard \"rigNNN\" WiFi network, "
       "this derives 192.168.NNN.10 automatically; otherwise falls back to mDNS then rig-logger.local.</div>";
  h += "<label>X-Rig-Token</label><input name='rigToken' type='password' value='" + cfg.rigToken + "'>";
  h += "<div class='small'>Tank volume lives on the <a href='/channels'>&#128208; Channels</a> page "
       "(fixed board) or the <a href='/sensors'>&#128225; Sensors</a> page (independent sensors) — "
       "check \"Compute Tank Volume\" on whichever channel/sensor is your level input.</div>";

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
  h += "<div class='small'>Saving with a changed SSID/password reboots the unit. If no network is saved at all, this unit "
       "first auto-scans for a standard rig router (SSID starting with \"rig\" followed by numbers, e.g. rig132) before "
       "falling back to its own setup AP.</div>";
  h += "<br><button type='submit'>Save</button>";
  h += "</form>";
  h += "<div class='card' style='margin-top:12px'><b>Forget WiFi</b><br><span class='small'>Clears the saved network and "
       "reboots straight into setup-AP mode.</span><br><br>";
  h += "<button class='btn-red' onclick=\"if(confirm('Forget saved WiFi and reboot into setup mode?'))"
       "fetch('/api/wifi/forget',{method:'POST'}).then(()=>alert('Forgotten. Rebooting...'))\">Forget WiFi</button></div>";
  h += "<div class='small' style='margin-top:16px'>Full config export/import (backup/clone all settings) lives on the "
       "<a href='/system'>&#128295; System</a> page.</div>";
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

// ─── /channels  FIXED BOARD CHANNEL CONFIG ──────────────────────────────────
static String calPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128208; Fixed Board Channel Configuration</h2>";
  h += "<p class='small'>Each channel of the fixed adapter board is fully independent — set its own name, kind (free text), "
       "unit, and scaling. Live mA auto-refreshes every 2s. Set Zero/Max at known engineering values for the most accurate "
       "reading, or rely on the mA linear map below if not calibrated.</p>";
  h += "<p class='small'>Detected board: <b>" + String(boardProfile.name) + "</b> (" +
       String(boardProfile.numChannels) + " channels)</p>";

  h += "<form method='POST' action='/api/config'>";
  h += "<input type='hidden' name='fromChannelsPage' value='1'>";
  h += "<button type='submit' style='margin-bottom:14px'>&#128190; Save All Channels</button>";

  for (int i = 0; i < 8; i++) {
    bool exists = (i < boardProfile.numChannels);
    h += "<div class='card'";
    if (!exists) h += " style='opacity:.4'";
    h += "><b>Channel " + String(i+1) + "</b>&nbsp;<span class='small' id='ma" + String(i) + "'>";
    if (!exists) h += "(not present on " + String(boardProfile.name) + ")";
    h += "</span>";
    h += "<label><input type='checkbox' name='ch" + String(i) + "en'";
    h += (cfg.ch[i].enabled ? " checked" : "");
    h += "> Enabled</label>";
    h += "<label>Name</label><input name='ch" + String(i) + "nm' value='" + cfg.ch[i].name + "' placeholder='e.g. Suction Pressure'>";
    h += "<label>Kind (free text — anything: level, pressure, temp, flow, rpm...)</label><input name='ch" + String(i) + "kd' value='" + cfg.ch[i].kind + "' placeholder='e.g. pressure'>";
    h += "<label>Unit</label><input name='ch" + String(i) + "ut' value='" + cfg.ch[i].unit + "' placeholder='e.g. psi, m, degC, rpm'>";
    h += "<div class='row'><div><label>mA Min</label><input name='ch" + String(i) + "maLo' type='number' step='any' value='" + String(cfg.ch[i].maMin) + "'></div>";
    h += "<div><label>mA Max</label><input name='ch" + String(i) + "maHi' type='number' step='any' value='" + String(cfg.ch[i].maMax) + "'></div></div>";
    h += "<div class='row'><div><label>Eng Min</label><input name='ch" + String(i) + "eLo' type='number' step='any' value='" + String(cfg.ch[i].engMin) + "'></div>";
    h += "<div><label>Eng Max</label><input name='ch" + String(i) + "eHi' type='number' step='any' value='" + String(cfg.ch[i].engMax) + "'></div></div>";
    h += "<div class='small'>Cal: zeroRaw=" + String(cfg.ch[i].zeroRaw) + " maxRaw=" + String(cfg.ch[i].maxRaw) +
         " (if both set, overrides the mA Min/Max map above)</div>";

    h += "<label style='margin-top:10px'><input type='checkbox' name='ch" + String(i) + "volEn' id='volEn" + String(i) +
         "' onchange='toggleVol(" + String(i) + ")'";
    h += (cfg.ch[i].volumeEnabled ? " checked" : "");
    h += "> Compute Tank Volume from this channel</label>";
    h += "<div id='volFields" + String(i) + "' style='display:" + String(cfg.ch[i].volumeEnabled ? "block" : "none") + "'>";
    h += "<div class='small'>Straight-line map: this channel's value at \"Empty\" = 0 volume, value at \"Full\" = Capacity.</div>";
    h += "<div class='row'><div><label>Capacity</label><input name='ch" + String(i) + "cap' type='number' step='any' min='0' value='" + String(cfg.ch[i].capacity) + "'></div>";
    h += "<div><label>Capacity Unit</label><select name='ch" + String(i) + "capUt'>";
    h += "<option value='m3'" + String(cfg.ch[i].capacityUnit == "m3" ? " selected" : "") + ">m&#179; (cubic meters)</option>";
    h += "<option value='gal'" + String(cfg.ch[i].capacityUnit == "gal" ? " selected" : "") + ">gal (US gallons)</option></select></div></div>";
    h += "<div class='row'><div><label>Value @ Empty</label><input name='ch" + String(i) + "vZLvl' type='number' step='any' value='" + String(cfg.ch[i].volZeroLevel) + "'></div>";
    h += "<div><label>Value @ Full</label><input name='ch" + String(i) + "vMLvl' type='number' step='any' value='" + String(cfg.ch[i].volMaxLevel) + "'></div></div>";
    h += "</div>";

    h += "<div class='row' style='margin-top:8px'>";
    h += "<button type='button' onclick='setZero(" + String(i) + ")'>Set Zero</button>&nbsp;";
    h += "<button type='button' onclick='setMax(" + String(i) + ")'>Set Max</button></div>";
    h += "</div>"; // closes .card
  }

  h += "<button type='submit' style='margin-top:6px'>&#128190; Save All Channels</button>";
  h += "</form>";

  h += "<script>var _numRealChannels = " + String(boardProfile.numChannels) + ";</script>";
  h += R"(
<script>
function fetchRaw(){
  fetch('/api/channel-raw').then(r=>r.json()).then(d=>{
    d.channels.forEach(c=>{
      if (c.ch >= _numRealChannels) return;
      let el=document.getElementById('ma'+c.ch);
      if(el) el.textContent = c.ma.toFixed(3)+' mA (raw '+c.raw+')';
    });
  });
}
setInterval(fetchRaw,2000); fetchRaw();
function setZero(ch){ fetch('/api/cal/zero?ch='+ch,{method:'POST'}).then(()=>fetchRaw()); }
function setMax(ch){  fetch('/api/cal/max?ch='+ch, {method:'POST'}).then(()=>fetchRaw()); }
function toggleVol(ch){
  var el=document.getElementById('volFields'+ch);
  var cb=document.getElementById('volEn'+ch);
  el.style.display = cb.checked ? 'block' : 'none';
}
</script>)";
  h += "</div>";
  return h;
}

// ─── /digital  FIXED BOARD DIGITAL I/O ──────────────────────────────────────
static String digitalPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128268; Digital I/O</h2>";

  if (!boardProfile.hasDigitalIO) {
    h += "<p class='small'>Detected board: <b>" + String(boardProfile.name) +
         "</b> — no digital I/O on this board. Digital inputs/outputs are only "
         "available on the Eletechsup AMIDJ14 (6AI-4DI-4DO).</p></div>";
    return h;
  }

  h += "<p class='small'>Detected board: <b>" + String(boardProfile.name) +
       "</b> — 4 digital inputs (dry contact, DIx&harr;GND), 4 digital outputs "
       "(open-collector, DOx&harr;VCC &mdash; use an intermediate relay to drive anything "
       "beyond a small load). State auto-refreshes every 2s.</p>";

  h += "<form method='POST' action='/api/config'>";
  h += "<input type='hidden' name='fromDigitalPage' value='1'>";
  h += "<button type='submit' style='margin-bottom:14px'>&#128190; Save Names/Enabled</button>";

  h += "<h3>Digital Inputs</h3>";
  {
    float pollHz = pulsePollRateHz();
    if (pollHz > 0.1f) {
      h += "<p class='small'>Pulse Counter Mode fast-poll rate: <b>" + String(pollHz, 1) +
           " Hz</b> — that's the accuracy ceiling for RPM below; readings alias (silently wrong, "
           "not just slow) well past roughly half that rate in pulses/sec.</p>";
    }
  }
  for (int i = 0; i < 4; i++) {
    h += "<div class='card'><b>DI" + String(i+1) + "</b>&nbsp;<span class='small' id='diState" + String(i) + "'></span>";
    h += "<label><input type='checkbox' name='di" + String(i) + "en'";
    h += (cfg.din[i].enabled ? " checked" : "");
    h += "> Enabled</label>";
    h += "<label>Name</label><input name='di" + String(i) + "nm' value='" + cfg.din[i].name + "' placeholder='e.g. Low Level Switch'>";
    h += "<label><input type='checkbox' name='di" + String(i) + "pmEn'";
    h += (cfg.din[i].pulseModeEnabled ? " checked" : "");
    h += "> Pulse Counter Mode (report RPM from this DI's transitions) &nbsp;<span class='small' id='diRpm" + String(i) + "'></span></label>";
    h += "<div style='margin-left:20px'>";
    h += "<label>Pulses per Revolution</label><input name='di" + String(i) + "ppr' type='number' min='1' value='" + String(cfg.din[i].pulsesPerRev) + "'>";
    h += "<div class='small' style='margin-top:-6px;margin-bottom:10px'>1 if there's a single trigger point on the shaft; higher if the "
         "sensor sees multiple points per revolution (e.g. gear teeth).</div>";
    h += "<label>Stopped Timeout (s)</label><input name='di" + String(i) + "pto' type='number' min='0.1' step='0.1' value='" + String(cfg.din[i].timeoutS) + "'>";
    h += "<div class='small' style='margin-top:-6px'>No transition for this long &rarr; reports 0 RPM (stopped) instead of holding the last reading.</div>";
    h += "<label>Fast-Poll Read Timeout (ms)</label><input name='di" + String(i) + "pms' type='number' min='5' max='300' value='" + String(cfg.din[i].pulseTimeoutMs) + "'>";
    h += "<div class='small' style='margin-top:-6px'>Per-read timeout for the RPM fast-poll loop. Lower this (try 20-30) after raising RS485 "
         "Baud Rate above 9600 on the <a href='/'>Config</a> page. Don't go below ~15-20 on 9600 baud.</div>";
    h += "</div>";
    h += "</div>";
  }

  h += "<h3>Digital Outputs</h3>";
  for (int i = 0; i < 4; i++) {
    h += "<div class='card'><b>DO" + String(i+1) + "</b>&nbsp;<span class='small' id='doState" + String(i) + "'></span>";
    h += "<label><input type='checkbox' name='do" + String(i) + "en'";
    h += (cfg.dout[i].enabled ? " checked" : "");
    h += "> Enabled</label>";
    h += "<label>Name</label><input name='do" + String(i) + "nm' value='" + cfg.dout[i].name + "' placeholder='e.g. Alarm Beacon'>";
    h += "<div class='row' style='margin-top:8px'>";
    h += "<button type='button' onclick='writeDO(" + String(i) + ",true)'>ON</button>&nbsp;";
    h += "<button type='button' onclick='writeDO(" + String(i) + ",false)'>OFF</button></div>";
    h += "</div>";
  }

  h += "<button type='submit' style='margin-top:6px'>&#128190; Save Names/Enabled</button>";
  h += "</form>";

  h += R"(
<script>
function fetchDigital(){
  fetch('/api/digital').then(r=>r.json()).then(d=>{
    d.din.forEach(c=>{
      let el=document.getElementById('diState'+c.ch);
      if(el) el.textContent = c.status!=='ok' ? '(stale)' : (c.state ? 'ON' : 'OFF');
      let rpmEl=document.getElementById('diRpm'+c.ch);
      if(rpmEl){
        if(!c.rpm) rpmEl.textContent='';
        else if(!c.rpm.valid) rpmEl.textContent='(no pulses seen yet)';
        else rpmEl.textContent = c.rpm.status==='stopped' ? '0 RPM (stopped)' : c.rpm.value.toFixed(1)+' RPM';
      }
    });
    d.dout.forEach(c=>{
      let el=document.getElementById('doState'+c.ch);
      if(el) el.textContent = c.status!=='ok' ? '(stale)' : (c.state ? 'ON' : 'OFF');
    });
  });
}
setInterval(fetchDigital,2000); fetchDigital();
function writeDO(ch,val){
  fetch('/api/digital/write?ch='+ch+'&value='+(val?1:0),{method:'POST'}).then(()=>fetchDigital());
}
</script>)";
  h += "</div>";
  return h;
}

// ─── /sensors  INDEPENDENT RS485 SENSOR LIST ────────────────────────────────
static String sensorsPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128225; Independent RS485 Sensors</h2>";
  h += "<p class='small'>Any Modbus RTU sensor on the shared RS485 bus, separate from the fixed adapter board. Don't know "
       "its slave ID/register? Use \"Probe Now\" below to check, or \"Auto-Detect &amp; Enable\" to find new sensors "
       "(automatically skips the fixed board's own slave ID and any extra board's slave ID).</p>";
  h += "<div class='card' style='border-color:#27ae60'><b>&#9889; Auto-Detect &amp; Enable</b><br>"
       "<span class='small'>Scans addresses 1-16, auto-enables new sensors with starter defaults (fc=03). "
       "Manual only — press the button below.</span><br><br>"
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
    h += "<option value='3'" + String(s.funcCode == 3 ? " selected" : "") + ">03 - Read Holding Regs</option>";
    h += "<option value='4'" + String(s.funcCode == 4 ? " selected" : "") + ">04 - Read Input Regs</option>";
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
    h += "<div><label>Reply Register Offset</label><input name='s" + String(i) + "ro' type='number' min='0' max='15' value='" + String(s.respRegOffset) + "'></div>";
    h += "<div><label>Scale (value = raw * scale + offset)</label><input name='s" + String(i) + "sc' type='number' step='any' value='" + _f(s.scale) + "'></div>";
    h += "<div><label>Offset</label><input name='s" + String(i) + "of' type='number' step='any' value='" + _f(s.offset) + "'></div></div>";
    h += "<div class='small'>Reply Register Offset: for sensors that always answer with the same fixed multi-register "
         "block regardless of Register Addr (e.g. SM7779 radar always replies [distance, level, status] from its own "
         "reg 0) &mdash; set this to pick which register OF THE REPLY to use (0=1st, 1=2nd...), leave Register Addr at 0. "
         "Normal sensors: leave at 0.</div>";

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
  let ro  = document.querySelector('[name=s'+i+'ro]').value;
  let el = document.getElementById('live'+i);
  probeWithRetry('/api/modbus/probe?slaveId='+sid+'&funcCode='+fc+'&reg='+reg+'&dataType='+dt+'&wordOrder='+wo+'&respOffset='+ro, el,
    (d)=>'Probe OK: raw='+d.raw+' decoded='+d.decoded.toFixed(3),
    (d)=>'Probe FAILED: '+d.error);
}
function probeWithRetry(url, el, renderOk, renderFail, attempt){
  attempt = attempt || 1;
  const maxAttempts = 8;
  const gapMs = 900;
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
  fetch('/api/modbus/autodetect-sensor?slaveId='+sid,{method:'POST'}).then(r=>r.json()).then(d=>{
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

// =============================================================================
// /advanced — hidden power-user page (only linked quietly from /system).
// Multi-board support: wire up to MAX_EXTRA_BOARDS additional analog-to-
// Modbus boards on the SAME RS485 bus as the primary board, each at its
// own unique slave address. Also hosts the Slave ID Bus Scan tool.
// =============================================================================
static String advancedPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#9881;&#65039; Advanced</h2>";
  h += "<p class='small'>Power-user settings — most setups never need this page. "
       "Changes to slave ID or board type reboot the module to take effect.</p>";

  h += "<h3>Slave ID Bus Scan</h3><div class='card'>";
  h += "<p class='small'>Probes addresses 1-N at the current baud rate and reports which ones "
       "answer, plus their Product ID if it identifies as a known fixed board. Use this to double-check "
       "for typos or address collisions before/after editing the extra boards below, or the independent "
       "sensors on <a href='/sensors'>/sensors</a>.</p>";
  h += "<div class='row'><div><label>Scan up to address</label><input id='scanMax' type='number' min='1' max='247' value='16'></div>";
  h += "<button type='button' onclick='runScan()' id='scanBusBtn' style='margin-top:18px'>&#128269; Scan Bus</button></div>";
  h += "<div id='scanBusResult' class='small' style='margin-top:8px'></div></div>";

  h += "<h3>Extra Boards (Multi-Board RS485)</h3>";
  h += "<p class='small' style='margin-bottom:10px'>Add up to " + String(MAX_EXTRA_BOARDS) +
       " more fixed analog-to-Modbus boards on this same RS485 bus. The primary board above (see "
       "<a href='/'>Config</a>) keeps its own Slave ID / Board Type — these are IN ADDITION to it. "
       "Every slave ID on the bus (primary + every extra board + every independent sensor) must be "
       "unique. Extra boards report generic channel names and a standard 4-20mA linear map — no "
       "per-channel calibration here, unlike the primary board's <a href='/channels'>Channels</a> page.</p>";

  h += "<form method='POST' action='/api/config'>";
  h += "<input type='hidden' name='fromAdvancedPage' value='1'>";
  h += "<button type='submit' style='margin-bottom:14px'>&#128190; Save Extra Boards</button>";

  for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
    ExtraBoardConfig& xb = cfg.extraBoards[i];
    String pre = "xb" + String(i);
    h += "<div class='card'><b>Extra Board " + String(i + 1) + "</b>";
    if (xb.enabled) {
      h += " &nbsp;<span class='small'>(" + String(extraBoardProfile[i].name) + ", " +
           String(extraBoardProfile[i].numChannels) + " channels)</span>";
    }
    h += "<label><input type='checkbox' name='" + pre + "en'";
    if (xb.enabled) h += " checked";
    h += "> Enabled</label>";
    h += "<label>Name</label><input name='" + pre + "nm' value='" + xb.name + "' placeholder='e.g. Board " + String(i + 2) + "'>";
    h += "<div class='row'><div><label>Slave ID (1-247, unique)</label><input name='" + pre + "sid' type='number' min='1' max='247' value='" + String(xb.slaveId) + "'></div>";
    h += "<div><label>Board Type</label><select name='" + pre + "bt'>";
    h += "<option value='amidj14'"; if (xb.boardType == "amidj14") h += " selected"; h += ">Eletechsup AMIDJ14</option>";
    h += "<option value='waveshare'"; if (xb.boardType == "waveshare") h += " selected"; h += ">Waveshare 8AI (B)</option>";
    h += "</select></div></div>";
    h += "</div>";
  }

  h += "<button type='submit' style='margin-top:6px'>&#128190; Save Extra Boards</button>";
  h += "</form>";

  h += R"(
<script>
function runScan(){
  let btn=document.getElementById('scanBusBtn'); let box=document.getElementById('scanBusResult');
  let max=document.getElementById('scanMax').value;
  btn.disabled=true; btn.textContent='Scanning...';
  box.textContent='Scanning addresses 1-'+max+'... this can take a while.';
  fetch('/api/modbus/scan?max='+max).then(r=>r.json()).then(d=>{
    btn.disabled=false; btn.innerHTML='&#128269; Scan Bus';
    if(d.found.length===0){ box.textContent='No slaves responded. Check wiring, baud rate, and DE pin.'; return; }
    let lines = d.found.map(f => 'Address ' + f.addr + (f.productId>0 ? ' — ' + (f.productId===2308?'Waveshare 8AI (B)':(f.productId===2814?'Eletechsup AMIDJ14':f.productId)) : ' — unidentified board'));
    box.innerHTML = 'Found ' + d.found.length + ' device(s):<br>' + lines.join('<br>');
  }).catch(e=>{ btn.disabled=false; btn.innerHTML='&#128269; Scan Bus'; box.textContent='Scan failed: '+e; });
}
</script>)";
  h += "</div>";
  return h;
}

// ─── /live  LIVE VALUES TABLE (fixed board + independent sensors + CAN) ────
static String livePage() {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128202; Live Status</h2><div id='liveData'>Loading...</div>";
  h += R"(
<script>
function fetchLive(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    let s = '';
    s += '<h3>Channels (fixed board + independent sensors)</h3><table><tr><th>Ch</th><th>Name</th><th>Kind</th><th>mA</th><th>Value</th><th>Volume</th><th>Status</th></tr>';
    (d.channels||[]).forEach(c=>{
      let ds = c.displayStatus || c.status;
      let cls = ds==='ok'?'ok':ds;
      s += '<tr><td>'+(c.ch+1)+'</td><td>'+(c.name||'')+'</td><td>'+(c.kind||'')+'</td><td>'+(c.ma!=null?c.ma.toFixed(2):'--')+'</td>';
      s += '<td>'+(c.value!=null?c.value+' '+c.unit:'--')+'</td>';
      if (c.volume) {
        let v = c.volume;
        let vcls = v.status==='ok'?'ok':'open';
        let vtxt = (v.value!=null ? v.value : '--') + ' ' + v.unit;
        if (c.capacity!=null) vtxt += ' <span class="small">/ '+c.capacity+' '+v.unit+'</span>';
        s += '<td class="'+vcls+'">'+vtxt+'</td>';
      } else {
        s += '<td class="small">--</td>';
      }
      s += '<td class="'+cls+'">'+ds+'</td></tr>';
    });
    s += '</table>';
    if ((d.digitalInputs && d.digitalInputs.length) || (d.digitalOutputs && d.digitalOutputs.length)) {
      s += '<h3>Digital I/O</h3><table><tr><th>Ch</th><th>Name</th><th>Direction</th><th>State</th><th>Status</th></tr>';
      (d.digitalInputs||[]).forEach(c=>{
        let cls = c.status==='ok'?'ok':'open';
        let stxt = c.state==null ? '--' : (c.state ? 'ON' : 'OFF');
        s += '<tr><td>DI'+(c.ch+1)+'</td><td>'+(c.name||'')+'</td><td>Input</td><td>'+stxt+'</td><td class="'+cls+'">'+c.status+'</td></tr>';
      });
      (d.digitalOutputs||[]).forEach(c=>{
        let cls = c.status==='ok'?'ok':'open';
        let stxt = c.state==null ? '--' : (c.state ? 'ON' : 'OFF');
        s += '<tr><td>DO'+(c.ch+1)+'</td><td>'+(c.name||'')+'</td><td>Output</td><td>'+stxt+'</td><td class="'+cls+'">'+c.status+'</td></tr>';
      });
      s += '</table>';
    }
    let rpmChs = (d.digitalInputs||[]).filter(c=>c.rpm);
    if (rpmChs.length) {
      s += '<h3>RPM (Pulse Counter Mode)</h3><table><tr><th>Ch</th><th>Name</th><th>RPM</th><th>Status</th></tr>';
      rpmChs.forEach(c=>{
        let r = c.rpm;
        let cls = r.status==='ok'?'ok':(r.status==='stopped'?'warn':'open');
        let vtxt = r.value==null ? '--' : (r.status==='stopped' ? '0 (stopped)' : r.value.toFixed(1));
        s += '<tr><td>DI'+(c.ch+1)+'</td><td>'+(c.name||'')+'</td><td>'+vtxt+'</td><td class="'+cls+'">'+r.status+'</td></tr>';
      });
      s += '</table>';
    }
    if (d.extraBoards && d.extraBoards.length) {
      s += '<h3>Extra Boards</h3>';
      d.extraBoards.forEach(b=>{
        s += '<p class="small"><b>'+b.name+'</b> (slave '+b.slaveId+', '+b.boardType+')</p>';
        s += '<table><tr><th>Ch</th><th>mA</th><th>Value</th><th>Status</th></tr>';
        (b.channels||[]).forEach(c=>{
          let cls = c.status==='ok'?'ok':'open';
          s += '<tr><td>'+c.name+'</td><td>'+(c.ma!=null?c.ma.toFixed(2):'--')+'</td><td>'+(c.value!=null?c.value:'--')+'</td><td class="'+cls+'">'+c.status+'</td></tr>';
        });
        s += '</table>';
      });
    }
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

// ─── /system  SYSTEM PAGE (+ Config Export/Import) ─────────────────────────
static String sysPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128295; System</h2>";
  h += "<div class='card'><b>Firmware:</b> " + String(FW_VERSION);
  h += "<br><b>Detected Board:</b> <span id='detBoardName'>" + String(boardProfile.name) + "</span> (<span id='detBoardCh'>" +
       String(boardProfile.numChannels) + "</span> channels, auto-detected via Product ID register) ";
  h += "<button type='button' onclick='redetectBoard()' style='margin-left:6px;padding:2px 8px'>&#128260; Re-detect</button>";
  h += "<div id='redetectResult' class='small'></div>";
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
           "This confirms the save isn't persisting.</span>";
    } else {
      h += "<br><span class='ok'>&#10003; Match — flash agrees with what's running.</span>";
    }
    h += "</div>";
  }

  h += "<h3>Config Export / Import</h3><div class='card'>";
  h += "<p class='small'>Download every setting on this device as a single JSON file (fixed board channels, digital I/O, "
       "extra boards, independent sensors, CAN signals, module/Pi settings, WiFi). Restore it later, or upload it onto a "
       "different unit to clone this one's configuration. <b>Module ID is NOT included</b> — it's always derived from "
       "each device's own MAC address. The exported file DOES include your WiFi password in plain text — handle it with "
       "the same care as any written-down password.</p>";
  h += "<button type='button' onclick=\"window.location.href='/api/config/export'\">&#128190; Download Config (.json)</button>";
  h += "<div style='margin-top:14px'>";
  h += "<label>Restore from file:</label>";
  h += "<input type='file' id='importFile' accept='application/json,.json'>";
  h += "<button type='button' onclick='doImport()' style='margin-top:8px'>&#128228; Upload &amp; Restore</button>";
  h += "<div id='importResult' class='small' style='margin-top:8px'></div>";
  h += "</div></div>";

  h += "<h3>OTA Update</h3><div class='card'>";
  h += "<label>OTA from URL:</label><div class='row'><input id='otaUrl' placeholder='http://...'>&nbsp;";
  h += "<button onclick='doOTA()'>Update</button></div></div>";
  h += "<h3>Buffer</h3><div class='card'><div id='bufInfo'>...</div><br>";
  h += "<button onclick='fetch(\"/api/buffer/flush\",{method:\"POST\"}).then(()=>alert(\"Flushing\"))'>Flush Now</button>&nbsp;";
  h += "<button class='btn-red' onclick='if(confirm(\"Clear all buffered data?\"))fetch(\"/api/buffer/clear\",{method:\"POST\"}).then(()=>location.reload())'>Clear Buffer</button></div>";
  h += "<h3>Danger Zone</h3><div class='card'>";
  h += "<button onclick='if(confirm(\"Reboot?\"))fetch(\"/api/reboot\",{method:\"POST\"})'>Reboot</button>&nbsp;";
  h += "<button class='btn-red' onclick='if(confirm(\"Factory reset? ALL config will be lost.\"))fetch(\"/api/factory-reset\",{method:\"POST\"})'>Factory Reset</button></div>";
  h += "<div class='small' style='margin-top:20px'><a href='/advanced' style='color:#556'>Advanced settings</a></div>";
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
function redetectBoard(){
  let box=document.getElementById('redetectResult');
  box.textContent='Probing bus...';
  fetch('/api/modbus/redetect-board',{method:'POST'}).then(r=>r.json()).then(d=>{
    document.getElementById('detBoardName').textContent = d.boardName;
    document.getElementById('detBoardCh').textContent = d.numChannels;
    box.textContent = 'Re-detected: ' + d.boardName + '. Any "Force" override on Config has been cleared back to Auto-Detect.';
  }).catch(e=>{ box.textContent='Re-detect failed: '+e; });
}
function doImport(){
  let f = document.getElementById('importFile').files[0];
  let box = document.getElementById('importResult');
  if(!f){ box.textContent='Choose a file first.'; return; }
  box.textContent='Reading file...';
  let reader = new FileReader();
  reader.onload = function(){
    box.textContent='Uploading and restoring...';
    fetch('/api/config/import', {method:'POST', headers:{'Content-Type':'application/json'}, body: reader.result})
      .then(r=>r.json()).then(d=>{
        if(d.ok){ box.innerHTML = '<span class="ok">Restored. Rebooting to apply...</span>'; }
        else { box.innerHTML = '<span class="warn">Import failed: '+(d.error||'unknown error')+'</span>'; }
      }).catch(e=>{ box.textContent='Request failed: '+e; });
  };
  reader.readAsText(f);
}
</script>)";
  h += "</div>";
  return h;
}

// ─── API handler helpers ──────────────────────────────────────────────────────
static void handleApiStatus() {
  DynamicJsonDocument doc(20480); // matches buildPayload()'s size
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

static void handleChannelRaw() {
  DynamicJsonDocument doc(512);
  JsonArray arr = doc.createNestedArray("channels");
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(200)) == pdTRUE) {
    for (int i = 0; i < 8; i++) {
      JsonObject o = arr.createNestedObject();
      o["ch"]  = i;
      o["raw"] = _raw[i];
      o["ma"]  = round(_raw[i] / boardProfile.rawDivisor * 100.0f) / 100.0f;
    }
    xSemaphoreGive(_mtx);
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Live sensor readings for the /sensors page.
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
// backs the per-sensor "Probe Now" button on /sensors.
static void handleModbusProbe() {
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy\"}");
    return;
  }
  uint8_t slaveId = _p("slaveId").toInt();
  uint8_t funcCode = _p("funcCode").toInt();
  String regStr = _p("reg");
  uint16_t regAddr = (uint16_t)strtol(regStr.c_str(), nullptr, 0);
  uint8_t dataType = _p("dataType").toInt();
  uint8_t wordOrder = _p("wordOrder").toInt();
  uint8_t respOffset = _p("respOffset").isEmpty() ? 0 : (uint8_t)_p("respOffset").toInt();

  uint8_t n = modbusRegCount(dataType);
  uint16_t regs[2] = {0, 0};
  uint8_t actualSlaveId = 0;
  int rc = modbusReadRegs(slaveId, funcCode, regAddr, n, regs, true, 600, &actualSlaveId, respOffset);
  xSemaphoreGive(modbusBusMutex);

  DynamicJsonDocument doc(512);
  if (rc == MB_OK) {
    float decoded = modbusDecodeValue(regs, dataType, wordOrder);
    doc["ok"] = true;
    doc["raw"] = regs[0];
    JsonArray regsArr = doc.createNestedArray("regs");
    for (int i = 0; i < n; i++) regsArr.add(regs[i]);
    doc["decoded"] = decoded;
    if (modbusIsBroadcastAddr(slaveId)) doc["actualSlaveId"] = actualSlaveId;
  } else {
    doc["ok"] = false;
    doc["error"] = (rc == MB_TIMEOUT) ? "timeout — no response" :
                   (rc == MB_CRC_ERROR) ? "CRC error" : "bad response";
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

  applyParam("moduleName",     [](String v){ _cfg->moduleName    = v; });
  applyParam("moduleType",     [](String v){ _cfg->moduleType    = v.isEmpty() ? "generic" : v; });
  applyParam("description",    [](String v){ _cfg->description   = v; });
  applyParam("modbusSlaveId",  [](String v){ _cfg->modbusSlaveId = v.toInt(); });

  bool baudChanged = false;
  applyParam("modbusBaud", [&](String v){
    long nb = v.toInt();
    if (nb != _cfg->modbusBaud) { _cfg->modbusBaud = nb; baudChanged = true; }
    // Any explicit save from this form counts as "user has decided the
    // baud" — locks it in so background auto-detect never silently
    // changes it again (see config.h ModuleConfig.baudManuallySet).
    _cfg->baudManuallySet = true;
  });
  applyParam("boardOverride",  [&](String v){ if (v != _cfg->boardOverride) { _cfg->boardOverride = v; baudChanged = true; } });

  bool canChanged = false;
  bool newCanEnabled = _srv->hasArg("canEnabled");
  if (newCanEnabled != _cfg->canEnabled) { _cfg->canEnabled = newCanEnabled; canChanged = true; }
  applyParam("canBitrate", [&](String v){ long nb = v.toInt(); if (nb != _cfg->canBitrate) { _cfg->canBitrate = nb; canChanged = true; } });

  applyParam("pollIntervalS",  [](String v){ _cfg->pollIntervalS = constrain(v.toInt(),1,30); });
  applyParam("piHost",         [](String v){ _cfg->piHost        = v; });
  applyParam("rigToken",       [](String v){ _cfg->rigToken = v.isEmpty() ? "7804991970" : v; });

  bool wifiChanged = false;
  applyParam("wifiSSID", [&](String v){ if(v!=_cfg->wifiSSID){_cfg->wifiSSID=v;wifiChanged=true;} });
  applyParam("wifiPass", [](String v){ _cfg->wifiPass = v; });

  // /channels submits all 8 channel cards through ONE shared form
  // ("fromChannelsPage=1"), so every channel's enabled/volumeEnabled
  // checkbox is present in the same POST and can be applied
  // unconditionally via hasArg().
  bool allChannels = _srv->hasArg("fromChannelsPage");
  int which = _srv->hasArg("ch") ? _srv->arg("ch").toInt() : -1;
  for (int i = 0; i < 8; i++) {
    String pre = "ch" + String(i);
    if (allChannels || i == which) {
      _cfg->ch[i].enabled       = _srv->hasArg((pre+"en").c_str());
      _cfg->ch[i].volumeEnabled = _srv->hasArg((pre+"volEn").c_str());
    }
    applyParam((pre+"nm").c_str(),   [i](String v){ _cfg->ch[i].name  = v; });
    applyParam((pre+"kd").c_str(),   [i](String v){ _cfg->ch[i].kind  = v; });
    applyParam((pre+"ut").c_str(),   [i](String v){ _cfg->ch[i].unit  = v; });
    applyParam((pre+"maLo").c_str(), [i](String v){ _cfg->ch[i].maMin = v.toFloat(); });
    applyParam((pre+"maHi").c_str(), [i](String v){ _cfg->ch[i].maMax = v.toFloat(); });
    applyParam((pre+"eLo").c_str(),  [i](String v){ _cfg->ch[i].engMin = v.toFloat(); });
    applyParam((pre+"eHi").c_str(),  [i](String v){ _cfg->ch[i].engMax = v.toFloat(); });
    applyParam((pre+"cap").c_str(),   [i](String v){ _cfg->ch[i].capacity     = v.toFloat(); });
    applyParam((pre+"capUt").c_str(), [i](String v){ _cfg->ch[i].capacityUnit = v.isEmpty() ? "m3" : v; });
    applyParam((pre+"vZLvl").c_str(), [i](String v){ _cfg->ch[i].volZeroLevel = v.toFloat(); });
    applyParam((pre+"vMLvl").c_str(), [i](String v){ _cfg->ch[i].volMaxLevel  = v.toFloat(); });
  }

  if (_srv->hasArg("fromDigitalPage")) {
    for (int i = 0; i < 4; i++) {
      String preIn  = "di" + String(i);
      String preOut = "do" + String(i);
      _cfg->din[i].enabled  = _srv->hasArg((preIn+"en").c_str());
      _cfg->dout[i].enabled = _srv->hasArg((preOut+"en").c_str());
      applyParam((preIn+"nm").c_str(),  [i](String v){ _cfg->din[i].name  = v; });
      applyParam((preOut+"nm").c_str(), [i](String v){ _cfg->dout[i].name = v; });
      _cfg->din[i].pulseModeEnabled = _srv->hasArg((preIn+"pmEn").c_str());
      applyParam((preIn+"ppr").c_str(), [i](String v){ _cfg->din[i].pulsesPerRev = max(1, (int)v.toInt()); });
      applyParam((preIn+"pto").c_str(), [i](String v){ _cfg->din[i].timeoutS = max(0.1f, v.toFloat()); });
      applyParam((preIn+"pms").c_str(), [i](String v){ _cfg->din[i].pulseTimeoutMs = constrain((int)v.toInt(), 5, 300); });
    }
    pulseConfigChanged(*_cfg);
  }

  bool boardsChanged = false;
  if (_srv->hasArg("fromAdvancedPage")) {
    for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
      String pre = "xb" + String(i);
      bool nowEnabled = _srv->hasArg((pre+"en").c_str());
      if (nowEnabled != _cfg->extraBoards[i].enabled) boardsChanged = true;
      _cfg->extraBoards[i].enabled = nowEnabled;
      applyParam((pre+"sid").c_str(), [i,&boardsChanged](String v){
        int nv = v.toInt();
        if (nv != _cfg->extraBoards[i].slaveId) boardsChanged = true;
        _cfg->extraBoards[i].slaveId = nv;
      });
      applyParam((pre+"bt").c_str(), [i,&boardsChanged](String v){
        if (v != _cfg->extraBoards[i].boardType) boardsChanged = true;
        _cfg->extraBoards[i].boardType = v;
      });
      applyParam((pre+"nm").c_str(), [i](String v){ _cfg->extraBoards[i].name = v; });
    }
  }

  saveConfig(*_prefs, *_cfg);

  if (boardsChanged && !wifiChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to apply board changes...</p>");
    delay(1000);
    ESP.restart();
  } else if (wifiChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to connect to new WiFi...</p>");
    delay(1000);
    ESP.restart();
  } else if (baudChanged || canChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to apply RS485/board/CAN settings...</p>");
    delay(1000);
    ESP.restart();
  } else {
    String backTo = "/";
    if (allChannels || which >= 0) backTo = "/channels";
    else if (_srv->hasArg("fromDigitalPage")) backTo = "/digital";
    else if (_srv->hasArg("fromAdvancedPage")) backTo = "/advanced";
    _srv->sendHeader("Location", backTo);
    _srv->send(302, "text/plain", "");
  }
}

// Saves the whole independent-sensor list from /sensors' shared form.
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
    if (_srv->hasArg((pre + "ro").c_str()))  s.respRegOffset = (uint8_t)constrain(_srv->arg((pre + "ro").c_str()).toInt(), 0, 15);
    if (_srv->hasArg((pre + "sc").c_str()))  s.scale = _srv->arg((pre + "sc").c_str()).toFloat();
    if (_srv->hasArg((pre + "of").c_str()))  s.offset = _srv->arg((pre + "of").c_str()).toFloat();
    s.volumeEnabled = _srv->hasArg((pre + "volEn").c_str());
    if (_srv->hasArg((pre + "cap").c_str())) s.capacity = _srv->arg((pre + "cap").c_str()).toFloat();
    if (_srv->hasArg((pre + "cu").c_str()))  s.capacityUnit = _srv->arg((pre + "cu").c_str());
    if (_srv->hasArg((pre + "vz").c_str()))  s.volZeroLevel = _srv->arg((pre + "vz").c_str()).toFloat();
    if (_srv->hasArg((pre + "vm").c_str()))  s.volMaxLevel = _srv->arg((pre + "vm").c_str()).toFloat();
  }
  saveConfig(*_prefs, *_cfg);

  // Readback verification — confirm what's ACTUALLY in flash for each
  // enabled slot's function code right after writing it.
  _prefs->begin("rigmod", false);
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (!_cfg->sensors[i].enabled) continue;
    String pre = "s" + String(i) + "_";
    int readBack = _prefs->getInt((pre + "fc").c_str(), -1);
    if (readBack != (int)_cfg->sensors[i].funcCode) {
      Serial.printf("[Sensors] MISMATCH slot %d: wrote fc=%d, flash readback=%d — NVS write may have failed!\n",
        i, _cfg->sensors[i].funcCode, readBack);
    }
  }
  _prefs->end();

  _srv->sendHeader("Location", "/sensors");
  _srv->send(302, "text/plain", "");
}

// Saves the whole CAN signal list from /can's shared form.
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

// Fixed-board "Auto-Detect Baud Rate" button on /config — probes at the
// fixed board's own slave ID.
static void handleModbusAutoDetect() {
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy, try again\"}");
    return;
  }
  long found = modbusAutoDetectBaud(_cfg->modbusSlaveId, (uint32_t)_cfg->modbusBaud);
  xSemaphoreGive(modbusBusMutex);

  if (found > 0) {
    _cfg->modbusBaud = found;
    _cfg->baudManuallySet = true;
    saveConfig(*_prefs, *_cfg);
    String r = "{\"ok\":true,\"detected\":true,\"baud\":" + String(found) + "}";
    _srv->send(200, "application/json", r);
    delay(500);
    ESP.restart();
  } else {
    _srv->send(200, "application/json", "{\"ok\":true,\"detected\":false}");
  }
}

// Per-sensor "Auto-Detect Baud" button on /sensors — probes at a specific
// slave ID passed as a query param, separate route from the fixed
// board's version above since the two pages pass different params
// (fixed board always uses its own configured slave ID; a sensor's Auto-
// Detect Baud button needs to pass whichever slave ID is in that specific
// sensor's form field, which may not be saved yet).
static void handleModbusAutoDetectSensor() {
  uint8_t slaveId = _p("slaveId").isEmpty() ? 1 : _p("slaveId").toInt();
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy, try again\"}");
    return;
  }
  long found = modbusAutoDetectBaud(slaveId, (uint32_t)_cfg->modbusBaud);
  xSemaphoreGive(modbusBusMutex);

  if (found > 0) {
    _cfg->modbusBaud = found;
    _cfg->baudManuallySet = true;
    saveConfig(*_prefs, *_cfg);
    String r = "{\"ok\":true,\"detected\":true,\"baud\":" + String(found) + "}";
    _srv->send(200, "application/json", r);
    delay(500);
    ESP.restart();
  } else {
    _srv->send(200, "application/json", "{\"ok\":true,\"detected\":false}");
  }
}

// Live board re-detect for the /system page's "Re-detect Board" button.
static void handleBoardRedetect() {
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy, try again\"}");
    return;
  }
  BoardProfile detected = modbusDetectBoard(_cfg->modbusSlaveId);
  xSemaphoreGive(modbusBusMutex);

  boardProfile = detected;
  if (_cfg->boardOverride != "auto") {
    Serial.println("[Modbus] Re-detect: clearing stuck boardOverride back to auto");
    _cfg->boardOverride = "auto";
  }
  saveConfig(*_prefs, *_cfg);

  DynamicJsonDocument doc(256);
  doc["ok"]           = true;
  doc["boardName"]    = detected.name;
  doc["numChannels"]  = detected.numChannels;
  doc["hasDigitalIO"] = detected.hasDigitalIO;
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// Slave ID bus scan for the /advanced page — includes Product ID
// identification for fixed boards.
static void handleModbusScan() {
  int maxAddr = _p("max").isEmpty() ? 16 : _p("max").toInt();
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;

  DynamicJsonDocument doc(2048);
  JsonArray found = doc.createNestedArray("found");

  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(90000)) == pdTRUE) {
    modbusScanSlavesWithBoardId(maxAddr, [&](int addr, int productId) {
      JsonObject o = found.createNestedObject();
      o["addr"]      = addr;
      o["productId"] = productId;
    }, 300);
    xSemaphoreGive(modbusBusMutex);
  } else {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy, try again\"}");
    return;
  }

  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

static void handleApiDigital() {
  DynamicJsonDocument doc(1536);
  JsonArray din = doc.createNestedArray("din");
  JsonArray dout = doc.createNestedArray("dout");
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(200)) == pdTRUE) {
    for (int i = 0; i < 4; i++) {
      JsonObject di = din.createNestedObject();
      di["ch"] = i;
      if (dinReadings[i].valid) di["state"] = dinReadings[i].state;
      else                      di["state"] = nullptr;
      di["status"] = dinReadings[i].status;
      if (_cfg->din[i].pulseModeEnabled) {
        PulseReading pr;
        pulseRpmCompute(i, *_cfg, pr);
        JsonObject rpmObj = di.createNestedObject("rpm");
        rpmObj["valid"]  = pr.valid;
        if (pr.valid) rpmObj["value"] = pr.rpm;
        else          rpmObj["value"] = nullptr;
        rpmObj["status"] = pr.status;
      }
      JsonObject dop = dout.createNestedObject();
      dop["ch"] = i;
      if (doutReadings[i].valid) dop["state"] = doutReadings[i].state;
      else                       dop["state"] = nullptr;
      dop["status"] = doutReadings[i].status;
    }
    xSemaphoreGive(_mtx);
  }
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

static void handleDigitalWrite() {
  if (!boardProfile.hasDigitalIO) {
    _srv->send(400, "application/json", "{\"ok\":false,\"error\":\"board has no digital I/O\"}");
    return;
  }
  int ch = _srv->hasArg("ch") ? _srv->arg("ch").toInt() : -1;
  bool value = _srv->hasArg("value") && _srv->arg("value").toInt() != 0;
  if (ch < 0 || ch > 3) {
    _srv->send(400, "application/json", "{\"ok\":false,\"error\":\"ch must be 0-3\"}");
    return;
  }
  bool ok = false;
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    ok = modbusWriteDO(_cfg->modbusSlaveId, ch, value);
    xSemaphoreGive(modbusBusMutex);
  }
  if (ok && xSemaphoreTake(_mtx, pdMS_TO_TICKS(200)) == pdTRUE) {
    doutReadings[ch].valid = true;
    doutReadings[ch].state = value;
    doutReadings[ch].status = "ok";
    xSemaphoreGive(_mtx);
  }
  String r = "{\"ok\":";
  r += (ok ? "true" : "false");
  r += "}";
  _srv->send(ok ? 200 : 500, "application/json", r);
}

static void handleCalZero() {
  int ch = _srv->hasArg("ch") ? _srv->arg("ch").toInt() : -1;
  if (ch < 0 || ch > 7) { _srv->send(400,"application/json","{\"ok\":false}"); return; }
  _cfg->ch[ch].zeroRaw = _raw[ch];
  saveConfig(*_prefs, *_cfg);
  String r = "{\"ok\":true,\"zeroRaw\":" + String(_raw[ch]) + "}";
  _srv->send(200, "application/json", r);
}

static void handleCalMax() {
  int ch = _srv->hasArg("ch") ? _srv->arg("ch").toInt() : -1;
  if (ch < 0 || ch > 7) { _srv->send(400,"application/json","{\"ok\":false}"); return; }
  _cfg->ch[ch].maxRaw = _raw[ch];
  saveConfig(*_prefs, *_cfg);
  String r = "{\"ok\":true,\"maxRaw\":" + String(_raw[ch]) + "}";
  _srv->send(200, "application/json", r);
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

// =============================================================================
// CONFIG EXPORT / IMPORT HANDLERS
// =============================================================================

// GET /api/config/export — downloads the full config as a JSON file. Sent
// with Content-Disposition so the browser prompts a save-as with a
// sensible filename instead of trying to render it inline.
static void handleConfigExport() {
  DynamicJsonDocument doc(16384); // generous — full config, all slots
  doc["exportedFw"] = FW_VERSION;
  doc["exportedFromModuleId"] = _cfg->moduleId; // informational only — NOT re-imported as moduleId
  configToJson(*_cfg, doc);
  String out;
  serializeJson(doc, out);
  String filename = "rig-module-config-" + _cfg->moduleId + ".json";
  _srv->sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
  _srv->send(200, "application/json", out);
}

// POST /api/config/import — body is a raw JSON config (as produced by
// /api/config/export, or hand-edited). Parses it, merges into the
// in-memory config (configFromJson() only touches fields actually present
// in the JSON), saves to NVS, and reboots so every subsystem picks up the
// restored settings cleanly.
static void handleConfigImport() {
  if (!_srv->hasArg("plain")) {
    _srv->send(400, "application/json", "{\"ok\":false,\"error\":\"no request body\"}");
    return;
  }
  String body = _srv->arg("plain");
  DynamicJsonDocument doc(16384);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    String r = "{\"ok\":false,\"error\":\"JSON parse error: " + String(err.c_str()) + "\"}";
    _srv->send(400, "application/json", r);
    return;
  }
  bool ok = configFromJson(doc, *_cfg);
  if (!ok) {
    _srv->send(400, "application/json", "{\"ok\":false,\"error\":\"not a valid config export (empty or not an object)\"}");
    return;
  }
  saveConfig(*_prefs, *_cfg);
  _srv->send(200, "application/json", "{\"ok\":true}");
  delay(800);
  ESP.restart();
}

// ─── Route setup ─────────────────────────────────────────────────────────────
void setupWebRoutes(WebServer& srv, ModuleConfig& cfg, Preferences& prefs,
                    ChannelReading* chReadings, uint16_t* rawModbus,
                    SensorReading* sReadings, CanSignalReading* cReadings,
                    SemaphoreHandle_t mtx) {
  _srv       = &srv;
  _cfg       = &cfg;
  _prefs     = &prefs;
  _readings  = chReadings;
  _raw       = rawModbus;
  _sReadings = sReadings;
  _cReadings = cReadings;
  _mtx       = mtx;

  auto noCacheHtml = [](int code, const String& body){
    _srv->sendHeader("Cache-Control", "no-store");
    _srv->send(code, "text/html; charset=utf-8", body);
  };
  srv.on("/",          HTTP_GET, [noCacheHtml](){ noCacheHtml(200, cfgPage(*_cfg)); });
  srv.on("/channels",  HTTP_GET, [noCacheHtml](){ noCacheHtml(200, calPage(*_cfg)); });
  srv.on("/digital",   HTTP_GET, [noCacheHtml](){ noCacheHtml(200, digitalPage(*_cfg)); });
  srv.on("/sensors",   HTTP_GET, [noCacheHtml](){ noCacheHtml(200, sensorsPage(*_cfg)); });
  srv.on("/can",       HTTP_GET, [noCacheHtml](){ noCacheHtml(200, canPage(*_cfg)); });
  srv.on("/live",      HTTP_GET, [noCacheHtml](){ noCacheHtml(200, livePage()); });
  srv.on("/system",    HTTP_GET, [noCacheHtml](){ noCacheHtml(200, sysPage(*_cfg)); });
  // /advanced — hidden power-user page, only linked quietly from /system.
  srv.on("/advanced",  HTTP_GET, [noCacheHtml](){ noCacheHtml(200, advancedPage(*_cfg)); });

  // GET APIs
  srv.on("/api/status",       HTTP_GET, handleApiStatus);
  srv.on("/api/channel-raw",  HTTP_GET, handleChannelRaw);
  srv.on("/api/sensors/live", HTTP_GET, handleSensorsLive);
  srv.on("/api/can/live",     HTTP_GET, handleCanLive);
  srv.on("/api/modbus/probe", HTTP_GET, handleModbusProbe);
  srv.on("/api/modbus/scan",  HTTP_GET, handleModbusScan);
  srv.on("/api/wifi/scan",    HTTP_GET, handleWifiScan);
  srv.on("/api/ota",          HTTP_GET, handleOTA);
  srv.on("/api/digital",      HTTP_GET, handleApiDigital);
  srv.on("/api/config/export", HTTP_GET, handleConfigExport);

  // POST APIs
  srv.on("/api/config",         HTTP_POST, handleConfig);
  srv.on("/api/sensors/save",   HTTP_POST, handleSensorsSave);
  srv.on("/api/can/save",       HTTP_POST, handleCanSave);
  srv.on("/api/wifi/forget",    HTTP_POST, handleWifiForget);
  srv.on("/api/cal/zero",       HTTP_POST, handleCalZero);
  srv.on("/api/cal/max",        HTTP_POST, handleCalMax);
  srv.on("/api/modbus/autodetect",        HTTP_POST, handleModbusAutoDetect);
  srv.on("/api/modbus/autodetect-sensor", HTTP_POST, handleModbusAutoDetectSensor);
  srv.on("/api/modbus/autodetect-enable", HTTP_POST, handleAutoDetectEnable);
  srv.on("/api/modbus/redetect-board",    HTTP_POST, handleBoardRedetect);
  srv.on("/api/digital/write",  HTTP_POST, handleDigitalWrite);
  srv.on("/api/config/import",  HTTP_POST, handleConfigImport);

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