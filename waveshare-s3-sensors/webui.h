// FIRMWARE VERSION: rig-module-sensors-1.19.23 (see FW_VERSION in config.h)
// =============================================================================
// webui.h — WebServer routes: config UI + REST API
// Direct-Sensor Rig Module variant. Pages:
//   /            Config (module info, RS485 baud, CAN bitrate (always on), WiFi)
//   /sensors     Add/edit/remove RS485 Modbus sensors (list, not fixed 8),
//                includes per-sensor "Probe Now" + "Auto-Detect Baud" and
//                bus-wide "Auto-Detect & Enable"
//   /can         Live raw CAN feed (v1.19.14: replaced the signal-setup form)
//   /live        Live values table (sensors + CAN signals)
//   /system      Firmware info, OTA, buffer, reboot/factory-reset
// (Debug page removed 2026-08-18 — root-caused sensor issue was FC03 vs
//  FC04, not a hardware/wiring problem. No longer needed day-to-day; the
//  standalone sensor-debug/sensor-debug-lilygo tools still exist separately
//  if deep RS485 debugging is ever needed again.)
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
extern ModbusChannelReading modbusReadings[MAX_MODBUS_CHANNELS];
extern ModbusDigitalReading modbusDinReadings[4];
extern ModbusDigitalReading modbusDoutReadings[4];
extern String modbusDetectedType;
// modbusWriteBoardDO() is declared+defined in modbus.h, included before
// this file in the .ino.
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t modbusBusMutex;
// modbusAutoDetectBaud(), modbusReadRegs(), modbusRegCount(),
// modbusDecodeValue(), modbusScanSlaves(), canStart(), canStop(),
// canIsRunning(), canGetRecentFrames(), canGetFrameTotal(),
// canGetLastFrameMs(), canGetRecentFrameRate() are all already
// declared+defined in modbus.h/can.h, included before this file in the
// .ino — no forward decls here. (2026-09-09: removed a stale
// canStart(int,int,long) forward decl left over from before the
// CANopen bridge listenOnly param was added to the real one in can.h —
// exactly the "ambiguous overload from a stale duplicate declaration"
// trap this comment already warned about for modbus.h.)
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
static const char NAV[] PROGMEM = R"(<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
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
  /* v1.15.34: visual dot-masking WITHOUT type='password'. Chrome/Firefox
     key their "save this password?" prompt off the mere presence of a
     type='password' field with a real value in it once you navigate
     away from the page — happens on the token/WiFi-password fields here
     even though this is a LAN-only bench config page with no login flow
     at all, and every field's value is stored/shown in plaintext on
     THIS same page regardless (see wifiSSID's card, /system, etc). A
     real type='password' field bought no actual secrecy, just an
     unwanted browser prompt. -webkit-text-security gets the same dot-
     masked look on a type='text' input, which browsers do not treat as
     a credential field at all. (Standard-track CSS text-security isn't
     shipped cross-browser yet — this -webkit- prefixed property is,
     confirmed, in every Chromium and Firefox release currently in use.)
  */
  .masked-text{-webkit-text-security:disc}
  .netlist{max-height:260px;overflow-y:auto;border:1px solid #334;border-radius:6px;margin-bottom:8px}
  .netrow{padding:11px 12px;border-bottom:1px solid #334;cursor:pointer;-webkit-tap-highlight-color:rgba(102,126,234,.25)}
  .netrow:last-child{border-bottom:none}
  .netrow.sel{background:#0f3460;box-shadow:inset 3px 0 0 #667eea}
  .manualrow{color:#667eea}
  /* ── Mobile (v1.19.12, Sarah: "current interface does not scale well on
     phones"). One breakpoint at 640px: bigger touch targets (44px — the
     rig-config.html kiosk standard), 16px inputs (stops iOS auto-zooming
     the whole page when a field gets focus), grids collapse to 2-up/1-up,
     wide tables scroll sideways inside the card instead of blowing out
     the page width. Desktop rendering is unchanged. */
  @media (max-width:640px){
    body{font-size:15px}
    .nav{padding:8px 10px;gap:6px}
    .nav a{padding:9px 12px;font-size:14px}
    .page{padding:10px}
    input,select{font-size:16px;padding:10px;margin-bottom:12px}
    button,.btn{font-size:15px;padding:11px 18px;min-height:44px}
    .card{padding:10px}
    .grid3,.grid4{grid-template-columns:1fr 1fr}
    .row{flex-direction:column;align-items:stretch}
    table{display:block;overflow-x:auto;white-space:nowrap;-webkit-overflow-scrolling:touch}
    td,th{padding:9px 8px}
    label{margin-bottom:6px}
    input[type=checkbox]{width:20px;height:20px;margin-right:10px;vertical-align:middle}
  }
</style>
</head><body>
<div class='nav'>
  <a href='/'>&#9881; Config</a>
  <a href='/sensors'>&#128225; RS485</a>
  <a href='/modbus'>&#9881; Modbus</a>
  <a href='/can'>&#128225; CAN</a>
  <a href='/live'>&#128202; Live</a>
  <a href='/system'>&#128295; System</a>
</div>
)";

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

// BUG FOUND 2026-09-10 (Sarah asked for a "spot sneaky bugs" pass): every
// user-editable string field (module name/type/description, Pi host,
// WiFi SSID/password, every sensor/CAN-signal name/kind/unit) was
// interpolated RAW into single-quoted HTML attributes across every page
// (cfgPage/sensorsPage/canPage) with zero escaping -- present on every
// rig-module-firmware variant, not something introduced today. A value
// containing a single quote (e.g. a sensor named "Driller's Pressure")
// breaks out of the attribute early, silently corrupting the rest of
// that page's HTML/form -- every field after it in the DOM renders
// wrong or vanishes, with no error anywhere. Values containing '<' or
// '>' can inject arbitrary markup into the page too (stored, since it's
// round-tripped through saved config) -- low real-world severity given
// this is a LAN-only device with no auth on the config page regardless,
// but a genuinely silent failure mode worth closing since the fix is
// cheap and mechanical. Used on every user-string field embedded as an
// <input value='...'> (or any other single-quoted attribute) below.
static String _esc(const String& v) {
  String s = v;
  // Order matters: & first, so escaping the other four doesn't double-
  // escape the & this function itself just inserted.
  s.replace("&", "&amp;");
  s.replace("'", "&#39;");
  s.replace("\"", "&quot;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  return s;
}

// Moved here 2026-09-17 (was above _p(), before _esc() existed in the file
// yet -- unlike the .ino, this is a plain header parsed top-to-bottom with
// no auto-prototyping, so calling _esc() before its definition would not
// have compiled). Takes cfg explicitly (not the bare global) since this
// file never extern-declares ModuleConfig cfg — its one caller (cfgPage())
// already has it in scope as a parameter.
static String apBanner(ModuleConfig& cfg) {
  if (!apModeActive) return "";
  // CHANGED 2026-09-17: apModeActive can now also mean "was connected, just
  // dropped, the runtime watchdog (see .ino's wifiWatchdog()) is retrying in
  // the background" -- not just "never configured/failed at boot". Wording
  // updated so it's still accurate for a saved-but-currently-unreachable
  // network, not just the never-configured case.
  if (!cfg.wifiSSID.isEmpty()) {
    return "<div class='ap-banner'>Not currently connected to \"" + _esc(cfg.wifiSSID) +
           "\" — retrying automatically in the background. This setup AP stays "
           "reachable the whole time if you need to change anything below.</div>";
  }
  return "<div class='ap-banner'>No WiFi configured yet — connect a device to this AP and "
         "set your network below.</div>";
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
  h += apBanner(cfg);
  h += "<div class='page'><h2>&#9881; Module Configuration</h2>";
  h += "<div class='card'><b>Module ID:</b> " + cfg.moduleId + " <span class='small'>(fixed, derived from MAC)</span></div>";
  h += "<form method='POST' action='/api/config'>";
  h += "<label>Module Name</label><input name='moduleName' value='" + _esc(cfg.moduleName) + "' placeholder='e.g. Standpipe Pressure Skid'>";
  h += "<label>Module Type</label><input name='moduleType' list='moduleTypeOpts' value='" + _esc(cfg.moduleType) + "' placeholder='e.g. pressure, drill'>";
  h += "<datalist id='moduleTypeOpts'><option value='generic'><option value='pressure'><option value='tank'><option value='pump'><option value='drill'></datalist>";
  h += "<div class='small'>Shown on the rig dashboard.</div>";
  h += "<label>Description</label><input name='description' value='" + _esc(cfg.description) + "'>";

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
    h += "<div class='small warn'>&#128275; Baud not yet locked — running Auto-Detect & Enable Now on /sensors may still "
         "adjust this automatically if no sensor is enabled yet. Save this form (or run Auto-Detect Baud on a sensor) to lock it.</div>";
  }

  h += "<h3>CAN Bus</h3>";
  h += "<div class='small'>Always on — plug-and-play, no enable step. Listen-only unless Carriage "
       "Position Sensor bridge below is on.</div>";
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

  h += "<h3>Carriage Position Sensor (CANopen Encoder Bridge)</h3>";
  h += "<div class='small'>This board's CAN link exists for one job right now: waking the "
       "ditchwitch-logger carriage-position CANopen encoder and relaying its frames to the Host PC "
       "(decode happens there — see that project's <code>web/sensors.html</code>).</div>";
  h += "<label><input type='checkbox' name='canopenBridge'";
  if (cfg.canopenBridge) h += " checked";
  h += "> Enable bridge (transmit bring-up, wake the encoder)</label>";
  h += "<div class='small'>Only for a bus you own — sends real frames.</div>";
  h += "<label>Encoder Node ID (hex)</label><input name='canopenNodeId' value='" +
       String(cfg.canopenNodeId, HEX) + "'>";
  h += "<div class='small'>Confirmed: EPC CANopen manual factory default = 0x7F — the only node "
       "ever seen answering on this bus.</div>";
  h += "<label><input type='checkbox' name='canopenTargetSpecific'";
  if (cfg.canopenTargetSpecific) h += " checked";
  h += "> Target specific node (unchecked = broadcast, confirmed working)</label>";
  h += "<div class='small'>Confirmed: encoder counts/rev = 16384 (EPC object 6001h factory "
       "default 0x4000). Position-per-foot calibration is unconfirmed — set on the Host PC's Sensor "
       "Configuration page once the sensor is jogged a known distance, not here.</div>";

  h += "<h3>Raw CAN Stream</h3>";
  h += "<label><input type='checkbox' name='canRawStream'";
  if (cfg.canRawStream) h += " checked";
  h += "> Stream ALL received CAN frames to Host PC (~1/s)</label>";
  h += "<div class='small'>No per-signal setup needed — the logger decodes with its rig profile. Works listen-only.</div>";

  h += "<h3>Host PC Logger</h3>";
  h += "<label>Send Interval (1-30 s)</label><input name='pollIntervalS' type='number' min='1' max='30' value='" + String(cfg.pollIntervalS) + "'>";
  h += "<label>Host PC Address (blank = auto)</label><input name='piHost' value='" + _esc(cfg.piHost) + "' placeholder='192.168.5.194 or tcc-rig6.local'>";
  h += "<div class='small'>Blank = auto-discover. RS485 polls continuously; this only paces sends.</div>";
  // v1.15.34: type='text' + masked-text (dot-masked via CSS), not
  // type='password' — see that class's comment above. Chrome/Firefox
  // don't treat this as a credential field at all now, so no more
  // "save this password?" prompt on leaving the page.
  h += "<label>X-Rig-Token</label><input name='rigToken' type='text' class='masked-text' autocomplete='off' value='" + _esc(cfg.rigToken) + "'>";

  h += "<h3>WiFi</h3>";
  // v1.15.14: AP-mode safety net. While the unit is in setup-AP mode it has
  // no route to the configured network, so a Save that changes the SSID must
  // reboot to join it. If the SSID was NOT changed (e.g. user scanned, gave
  // up, and just hit Save), rebooting would land right back in the AP —
  // exactly the "it keeps using the old network / never changes" symptom seen
  // in the field 2026-09-11. In AP mode with no SSID change: stay put, no
  // reboot, and say so loudly on the page.
  h += "<input type='hidden' name='apModeNow' value='" + String(apModeActive ? 1 : 0) + "'>";
  h += "<input type='hidden' name='ssidAtPageLoad' value='" + _esc(cfg.wifiSSID) + "'>";
  if (apModeActive) {
    h += "<div class='card'>Currently broadcasting setup AP: <b>" + _esc(apSSID) + "</b><br>Not connected to any site network yet.</div>";
  } else {
    h += "<div class='card'>Connected to: <b>" + _esc(cfg.wifiSSID) + "</b><br>IP: " + WiFi.localIP().toString() + "  RSSI: " + String(WiFi.RSSI()) + " dBm</div>";
  }
  // v1.19.13 (Sarah: "make it use the same interface like in rig logger"):
  // scan results are now TAP-TO-SELECT rows (rig-config.html's WiFi panel
  // pattern) instead of an info-only list + separate <select> dropdown you
  // had to re-pick the network in. The chosen SSID lives in a HIDDEN input
  // (name=wifiSSID) that JS fills — hidden inputs are not password-manager
  // autofill targets, so this keeps v1.15.13's autofill-overwrite fix intact
  // while dropping the dropdown entirely. "/api/config" save contract
  // unchanged: wifiSSID = picked name or "__manual__" + wifiSSIDManual.
  h += "<div class='row' style='margin-bottom:10px'><button type='button' onclick='doScan()' id='scanBtn'>&#128269; Scan for Networks</button></div>";
  h += "<div id='scanResults' class='small'>Tap Scan, then tap your network.</div>";
  h += "<input type='hidden' name='wifiSSID' id='wifiSSID' value='" + _esc(cfg.wifiSSID) + "'>";
  h += "<div id='selRow' class='card' style='display:none'></div>";
  h += "<div id='manualWrap' style='display:none'><label>Network name</label><input name='wifiSSIDManual' id='wifiSSIDManual' autocomplete='off' placeholder='site wifi network name'></div>";
  // v1.15.34: type='text' + masked-text, same reasoning as X-Rig-Token
  // above — this was the field actually causing the save-password
  // prompt Sarah reported, since it's the one that most looks like a
  // real login password field to the browser's heuristics.
  h += "<label>Password</label><input name='wifiPass' id='wifiPass' autocomplete='off' type='text' class='masked-text' placeholder='tap a network above, then type its password'>";
  h += "<div class='small'>Changing SSID/password reboots the unit. Password is never pre-filled — type it fresh after picking a network.</div>";
  h += "<br><button type='submit'>Save &amp; Connect</button>";
  h += "</form>";
  h += "<div class='card' style='margin-top:12px'><b>Forget WiFi</b><br><span class='small'>Clears saved network, reboots into setup-AP mode.</span><br><br>";
  h += "<button class='btn-red' onclick=\"if(confirm('Forget saved WiFi and reboot into setup mode?'))fetch('/api/wifi/forget',{method:'POST'}).then(()=>alert('Forgotten. Rebooting...'))\">Forget WiFi</button></div>";
  h += R"JS(
<script>
var CUR_SSID = "__CURSSID__";
function esc(s){ return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }
function setSsid(name){ document.getElementById('wifiSSID').value = name; }
function pickNet(ssid, row){
  // tap-to-select: highlight row, hide manual box, stash SSID in hidden input
  var rows=document.querySelectorAll('#scanResults .netrow');
  for(var i=0;i<rows.length;i++) rows[i].classList.remove('sel');
  if(row) row.classList.add('sel');
  setSsid(ssid);
  document.getElementById('manualWrap').style.display='none';
  document.getElementById('wifiSSIDManual').value='';
  var sel=document.getElementById('selRow');
  sel.style.display=''; sel.innerHTML='Selected: <b>'+esc(ssid)+'</b>';
  document.getElementById('wifiPass').focus();
}
function pickManual(row){
  var rows=document.querySelectorAll('#scanResults .netrow');
  for(var i=0;i<rows.length;i++) rows[i].classList.remove('sel');
  if(row) row.classList.add('sel');
  setSsid('__manual__');
  var sel=document.getElementById('selRow');
  sel.style.display=''; sel.innerHTML='Typing network name manually:';
  var m=document.getElementById('manualWrap');
  m.style.display=''; document.getElementById('wifiSSIDManual').focus();
}
function doScan(){
  let btn=document.getElementById('scanBtn'); let box=document.getElementById('scanResults');
  btn.disabled=true; btn.textContent='Scanning...';
  box.innerHTML='<p class="small">Scanning (a few seconds)...</p>';
  fetch('/api/wifi/scan').then(r=>r.json()).then(d=>{
    btn.disabled=false; btn.innerHTML='&#128269; Scan for Networks';
    let nets = d.networks || [];
    if(nets.length===0){ box.innerHTML='<p class="small">No networks found. Try again.</p>'; return; }
    let s = '<div class="netlist">';
    nets.forEach(function(n){
      let bars = n.rssi>-60?'####':n.rssi>-70?'###.':n.rssi>-80?'##..':'#...';
      let lock = n.secure ? '&#128274;' : '';
      let cur = (n.ssid===CUR_SSID) ? ' <span class="small">(current)</span>' : '';
      s += '<div class="netrow" data-ssid="'+esc(n.ssid)+'">'+lock+' '+esc(n.ssid)+cur
         + ' <span class="small" style="float:right">'+bars+' '+n.rssi+'dBm</span></div>';
    });
    s += '<div class="netrow manualrow">+ Type network name manually...</div>';
    s += '</div><div class="small" style="margin-top:6px">Tap your network, type the password, then Save &amp; Connect.</div>';
    box.innerHTML = s;
    let rows=box.querySelectorAll('.netrow');
    rows.forEach(function(r){
      if(r.classList.contains('manualrow')){ r.onclick=function(){ pickManual(r); }; }
      else { r.onclick=function(){ pickNet(r.getAttribute('data-ssid'), r); }; }
    });
    // pre-highlight the currently-saved network so the state is obvious
    rows.forEach(function(r){
      if(!r.classList.contains('manualrow') && r.getAttribute('data-ssid')===CUR_SSID) r.classList.add('sel');
    });
  }).catch(function(){ btn.disabled=false; btn.innerHTML='&#128269; Scan for Networks'; box.innerHTML='<p class="small">Scan failed.</p>'; });
}
</script>)JS";
  // Inject the current SSID into the JS constant AFTER the raw-string block
  // (raw string literals can't interpolate). _esc first, then escape
  // backslash/quote for JS string context.
  {
    String jsSsid = _esc(cfg.wifiSSID);
    jsSsid.replace("\\", "\\\\");
    jsSsid.replace("'", "\\'");
    h.replace("__CURSSID__", jsSsid);
  }
  h += "</div>";
  return h;
}

// ─── /modbus 4-20mA ADAPTER BOARD (RESERVED SLAVE 1) ─────────────────────
static String modbusPage(ModuleConfig& cfg) {
  String h=FPSTR(NAV); h += "<div class='page'><h2>&#9881; Modbus 4-20mA</h2>";
  h += "<p class='small'>Eletechsup/Waveshare analog input board. This board is always slave ID <b>1</b>; direct RS485 sensors use IDs 2-247.</p>";
  h += "<form method='POST' action='/api/modbus/save'>";
  h += "<button type='submit' style='margin-bottom:14px'>&#128190; Save Modbus Settings</button>";
  h += "<div class='card'><label><input type='checkbox' name='enabled'" + String(cfg.modbusBoard.enabled?" checked":"") + "> Enable Modbus board at slave ID 1</label>";
  h += "<label>Board type</label><select id='mbBoardType' name='boardType'>";
  h += "<option value='auto'"+String(cfg.modbusBoard.boardType=="auto"?" selected":"")+">Auto-detect</option>";
  for (auto& p : MODBUS_BOARD_PRESETS) {
    h += "<option value='"+String(p.type)+"'"+String(cfg.modbusBoard.boardType==p.type?" selected":"")+">"+String(p.label)+" — "+String(p.baud)+" baud, "+String(p.numChannels)+"ch</option>";
  }
  h += "</select>";
  h += "<button type='button' id='mbApplyPresetBtn' onclick='mbApplyPreset()' style='margin-top:6px'>&#9889; Apply Board Preset</button>";
  h += "<div class='small' id='mbApplyPresetStatus' style='margin:6px 0 4px'>Sets the RS485 baud rate + channel count to that board's factory "
       "defaults and saves immediately (reboots if the baud actually changes). <b>Baud is shared with any direct RS485 sensors on /sensors — "
       "changing it here changes it for the whole bus.</b> Has no effect while \"Auto-detect\" is selected.</div>";
  h += "<div class='small'>Raw scaling: Waveshare uses microamps; AMIDJ14 uses hundredths of a milliamp. The board is read using the shared RS485 baud setting on Config.</div></div>";
  h += "<h3>Analog Inputs</h3>";
  for(int i=0;i<MAX_MODBUS_CHANNELS;i++){ auto& c=cfg.modbusBoard.channels[i]; h += "<div class='card'><b>AI"+String(i+1)+"</b><span class='small' id='mbLive"+String(i)+"'> loading...</span>";
    h += "<label><input type='checkbox' name='mb"+String(i)+"en'"+String(c.enabled?" checked":"")+"> Enabled</label><div class='row'><div><label>Name</label><input name='mb"+String(i)+"nm' value='"+_esc(c.name)+"' placeholder='e.g. Mud Pressure'></div><div><label>Kind</label><input name='mb"+String(i)+"kd' value='"+_esc(c.kind)+"' placeholder='pressure'></div><div><label>Unit</label><input name='mb"+String(i)+"ut' value='"+_esc(c.unit)+"' placeholder='psi'></div></div>";
    h += "<div class='row'><div><label>Engineering Min @ 4mA</label><input name='mb"+String(i)+"lo' type='number' step='any' value='"+String(c.engMin,3)+"'></div><div><label>Engineering Max @ 20mA</label><input name='mb"+String(i)+"hi' type='number' step='any' value='"+String(c.engMax,3)+"'></div></div></div>";
  }

  // Digital I/O only exists on the AMIDJ14 — shown regardless of the
  // currently-detected type (so it can be configured ahead of time / even
  // if "auto" hasn't run yet), but with a plain note when the board isn't
  // an AMIDJ14. Same "still render, just say so" convention as the
  // analog-board variant's /digital page for a board with no DI/DO at all.
  h += "<h3>Digital I/O <span class='small'>(Eletechsup AMIDJ14 only)</span></h3>";
  if (cfg.modbusBoard.boardType != "amidj14" && modbusDetectedType != "amidj14") {
    h += "<p class='small'>Detected/selected board type is not AMIDJ14 — digital channels below "
         "won't be polled unless the board type is (or auto-detects as) AMIDJ14.</p>";
  }
  h += "<div class='row'>";
  h += "<div><h4>Digital Inputs</h4>";
  for (int i=0;i<4;i++) {
    auto& d = cfg.modbusBoard.din[i];
    h += "<div class='card'><b>DI"+String(i+1)+"</b>&nbsp;<span class='small' id='mbDiState"+String(i)+"'></span>";
    h += "<label><input type='checkbox' name='mbdi"+String(i)+"en'"+String(d.enabled?" checked":"")+"> Enabled</label>";
    h += "<label>Name</label><input name='mbdi"+String(i)+"nm' value='"+_esc(d.name)+"' placeholder='e.g. Low Level Switch'></div>";
  }
  h += "</div><div><h4>Digital Outputs</h4>";
  for (int i=0;i<4;i++) {
    auto& d = cfg.modbusBoard.dout[i];
    h += "<div class='card'><b>DO"+String(i+1)+"</b>&nbsp;<span class='small' id='mbDoState"+String(i)+"'></span>";
    h += "<label><input type='checkbox' name='mbdo"+String(i)+"en'"+String(d.enabled?" checked":"")+"> Enabled</label>";
    h += "<label>Name</label><input name='mbdo"+String(i)+"nm' value='"+_esc(d.name)+"' placeholder='e.g. Alarm Beacon'>";
    h += "<div class='row' style='margin-top:8px'><button type='button' onclick='mbWriteDO("+String(i)+",true)'>ON</button>&nbsp;";
    h += "<button type='button' onclick='mbWriteDO("+String(i)+",false)'>OFF</button></div></div>";
  }
  h += "</div></div>";

  h += "<button type='submit'>&#128190; Save Modbus Settings</button></form><script>"
       "function mbLive(){fetch('/api/modbus/live').then(r=>r.json()).then(d=>{"
       "(d.channels||[]).forEach(c=>{let e=document.getElementById('mbLive'+c.ch);if(e)e.textContent=c.value==null?' -- ('+c.status+')':' '+c.value+' '+(c.unit||'')+' / '+c.ma+'mA ('+c.status+')';});"
       "(d.din||[]).forEach(c=>{let e=document.getElementById('mbDiState'+c.ch);if(e)e.textContent=c.status!=='ok'?'(stale)':(c.state?'ON':'OFF');});"
       "(d.dout||[]).forEach(c=>{let e=document.getElementById('mbDoState'+c.ch);if(e)e.textContent=c.status!=='ok'?'(stale)':(c.state?'ON':'OFF');});"
       "});}"
       "setInterval(mbLive,2000);mbLive();"
       "function mbWriteDO(ch,val){fetch('/api/modbus/digital/write?ch='+ch+'&value='+(val?1:0),{method:'POST'}).then(()=>mbLive());}"
       "function mbApplyPreset(){"
       "var bt=document.getElementById('mbBoardType').value;var st=document.getElementById('mbApplyPresetStatus');var btn=document.getElementById('mbApplyPresetBtn');"
       "if(bt==='auto'){st.textContent='Pick a specific board (not Auto-detect) first.';return;}"
       "if(!confirm('This sets the RS485 baud rate for the WHOLE bus (including any direct RS485 sensors on /sensors) to this board\\'s factory default, and disables analog channels beyond its real count. Continue?'))return;"
       "btn.disabled=true;st.textContent='Applying...';"
       "fetch('/api/modbus/preset?boardType='+bt,{method:'POST'}).then(r=>r.json()).then(d=>{"
       "if(d.rebooting){st.textContent='Applied: '+d.baud+' baud, '+d.numChannels+' channels. Rebooting to apply the new baud rate...';setTimeout(()=>location.reload(),2000);}"
       "else{st.textContent='Applied: '+d.baud+' baud, '+d.numChannels+' channels (baud unchanged, no reboot needed). Reloading...';setTimeout(()=>location.reload(),800);}"
       "}).catch(e=>{btn.disabled=false;st.textContent='Request failed: '+e;});"
       "}"
       "</script></div>";
  return h;
}

// ─── /sensors  RS485 SENSOR LIST ────────────────────────────────────────────
static String sensorsPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128225; RS485 Sensors</h2>";
  h += "<p class='small'>Any Modbus RTU sensor on the RS485 bus. Don't know its slave ID/register? Use "
       "\"Probe Now\" below to check, or \"Auto-Detect &amp; Enable\" to find new sensors.</p>";
  h += "<div class='card' style='border-color:#27ae60'><b>&#9889; Auto-Detect &amp; Enable</b><br>"
       "<span class='small'>Scans addresses 2-16, auto-enables new sensors with starter defaults (fc=03). "
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
    h += "<div class='row'><div><label>Name</label><input name='s" + String(i) + "nm' value='" + _esc(s.name) + "' placeholder='e.g. Standpipe Pressure'></div>";
    h += "<div><label>Kind</label><input name='s" + String(i) + "kd' value='" + _esc(s.kind) + "' placeholder='e.g. pressure'></div>";
    h += "<div><label>Unit</label><input name='s" + String(i) + "ut' value='" + _esc(s.unit) + "' placeholder='e.g. psi'></div></div>";
    h += "<div class='grid4'>";
    h += "<div><label>Slave ID (2-247)</label><input name='s" + String(i) + "sid' type='number' min='2' max='247' value='" + String(s.slaveId) + "'></div>";
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
    // CHANGED 2026-09-17 (Sarah/Gerald, two passes same day): 1st pass
    // replaced a single "Capacity" number with tank footprint (length x
    // width). 2nd pass removed the Value @ Empty / Value @ Full
    // calibration entirely -- Sarah didn't understand it and didn't need
    // it. Now just a third physical measurement (Tank Height) plus the
    // assumption baked into the wording below: this sensor is a
    // top-mounted radar/distance sensor, reading = air gap down to the
    // water surface. See scaling.h's computeSensorVolume() for the math.
    h += "<div class='small'>For a top-mounted distance sensor (e.g. radar): closer reading = "
         "more water. Volume = Length &times; Width &times; (Tank Height &minus; current reading).</div>";
    h += "<div class='row'><div><label>Tank Length (m)</label><input name='s" + String(i) + "len' type='number' step='any' min='0' value='" + _f(s.tankLengthM) + "'></div>";
    h += "<div><label>Tank Width (m)</label><input name='s" + String(i) + "wid' type='number' step='any' min='0' value='" + _f(s.tankWidthM) + "'></div></div>";
    h += "<div class='row'><div><label>Tank Height (m)</label><input name='s" + String(i) + "th' type='number' step='any' min='0' value='" + _f(s.tankHeightM) + "'></div>";
    h += "<div><label>Output Unit</label><select name='s" + String(i) + "cu'>";
    h += "<option value='m3'" + String(s.capacityUnit == "m3" ? " selected" : "") + ">m&#179;</option>";
    h += "<option value='gal'" + String(s.capacityUnit == "gal" ? " selected" : "") + ">gal</option></select></div></div>";
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
  box.textContent='Scanning addresses 2-16...';
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

// ─── /can  LIVE CAN FEED ─────────────────────────────────────────────────────
// v1.19.14 (Sarah: "replace the can sensor setup with a live can feed — we
// aren't gonna be hooking up sensors to the can bus, it reads the pump").
// The per-signal byte-offset decode form is GONE from the UI: the module's
// CAN job is to relay the pump's own traffic (raw stream POST + this feed),
// and decoding lives on the Host PC's rig profile. The canSignals[] backend
// (config.h NVS fields, /api/can/save, payload block) stays for board
// compatibility — old boards with signals configured keep POSTing them —
// but there is nothing to configure here anymore.
static String canPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128225; Live CAN Feed</h2>";
  h += "<div class='card'>CAN running at " + String(cfg.canBitrate) + " bit/s, " +
       (cfg.canopenBridge ? "Carriage Position Sensor bridge (transmitting bring-up)"
                           : "listen-only") + ". "
       "Total frames seen: <span id='canTotal'>...</span>, recent rate: <span id='canRate'>...</span> fps."
       "<br><span class='small'>Raw frames below — values are decoded on the Host PC's rig profile, not here.</span></div>";
  // v1.19.15 (Sarah, Vermeer SA400 mud pump): placeholder decode card ABOVE
  // the raw feed. Decodes only standard J1939 EEC1 (PGN 61484) — engine speed
  // + % torque — which virtually every diesel ECU broadcasts. Explicitly
  // labelled unverified until we confirm against real SA400 traffic; extend
  // the J1939 table in the script below once we have a capture.
  h += "<div class='card'><b>&#128167; J1939 data</b> "
       "<span class='small'>field-verified decode (j1939-logger table)</span>"
       "<div id='pumpVals' class='small' style='margin-top:6px'>No J1939 data seen yet — is the ECU running?</div></div>";
  h += "<div class='row' style='margin-bottom:8px'>"
       "<button type='button' id='capBtn' onclick='toggleCap()'>&#9210; Start capture</button>"
       "<button type='button' id='dlBtn' style='display:none' onclick='dlCsv()'>&#11015; Download CSV</button>"
       "<label style='margin:0 0 0 12px'><input type='checkbox' id='pauseFeed'> Pause</label>"
       "<span class='small' style='margin-left:auto' id='feedCount'></span></div>";
  h += "<div class='small' id='capNote' style='margin-bottom:6px'>Capture records every frame while running (browser-side, no length limit) — stop, then download as CSV for analysis.</div>";
  h += "<div class='card' style='padding:0;overflow-x:auto'><table style='width:100%;margin:0'>"
       "<tr><th style='text-align:left'>Age</th><th style='text-align:left'>ID</th><th>DLC</th><th style='text-align:left'>Data (hex)</th></tr>"
       "<tbody id='feedBody'><tr><td colspan='4' class='small' style='padding:10px'>Waiting for frames...</td></tr></tbody>"
       "</table></div>";
  h += R"JS(
<script>
var lastMs = 0, total = 0, capOn = false, cap = [], pump = {};
// Placeholder J1939 signal table (SA400 verification pending). off = byte
// offset in the 8-byte data field, big-endian, J1939 convention. 0xFF/0xFFFF
// means "not available" on a real ECU and is skipped.
// Table ported from waveshare-s3-j1939-logger/j1939.h — field-verified against
// a real engine (crank/idle/rev/shutdown capture 2026-10-27). NOTE: this ECU's
// EEC1 is NOT the textbook layout — RPM is bytes 3-4 (0.125 rpm/bit), torque
// byte 2, driver-demand byte 1 (both -125 offset). Scales below bake in the
// imperial conversion (F/psi/gal) so display needs no extra math.
var J1939 = [
  {pgn:0xF004, name:'RPM',            off:3, len:2, scale:0.125,               unit:'rpm'},
  {pgn:0xF004, name:'Torque',         off:2, len:1, scale:1, ofs:-125,         unit:'%'},
  {pgn:0xF004, name:'Demand torque',  off:1, len:1, scale:1, ofs:-125,         unit:'%'},
  {pgn:0xF003, name:'Pedal',          off:1, len:1, scale:0.4,                 unit:'%'},
  {pgn:0xF003, name:'Load',           off:2, len:1, scale:1,                   unit:'%'},
  {pgn:0xFEEE, name:'Coolant temp',   off:0, len:1, scale:1.8, ofs:-40,        unit:'\u00b0F'},
  {pgn:0xFEEE, name:'Fuel temp',      off:1, len:1, scale:1.8, ofs:-40,        unit:'\u00b0F'},
  {pgn:0xFEEE, name:'Oil temp',       off:2, len:2, scale:0.05625, ofs:-459.4, unit:'\u00b0F'},
  {pgn:0xFEEE, name:'Intercooler',    off:6, len:1, scale:1.8, ofs:-40,        unit:'\u00b0F'},
  {pgn:0xFEEF, name:'Fuel pressure',  off:0, len:1, scale:0.5801508,           unit:'psi'},
  {pgn:0xFEEF, name:'Oil pressure',   off:3, len:1, scale:0.5801508,           unit:'psi'},
  {pgn:0xFEEF, name:'Coolant press.', off:6, len:1, scale:0.2900754,           unit:'psi'},
  {pgn:0xFEEF, name:'Oil level',      off:2, len:1, scale:0.4,                 unit:'%'},
  {pgn:0xFEEF, name:'Coolant level',  off:7, len:1, scale:0.4,                 unit:'%'},
  {pgn:0xFEF2, name:'Fuel rate',      off:0, len:2, scale:0.0132086,           unit:'gal/h'},
  {pgn:0xFEF2, name:'Throttle',       off:6, len:1, scale:0.4,                 unit:'%'},
  {pgn:0xFEE5, name:'Engine hours',   off:0, len:4, scale:0.05,                unit:'h'},
  {pgn:0xFEE9, name:'Fuel used',      off:4, len:4, scale:0.132086,            unit:'gal'},
  {pgn:0xFEF6, name:'Boost',          off:1, len:1, scale:0.2900754,           unit:'psi'},
  {pgn:0xFEF6, name:'Intake temp',    off:2, len:1, scale:1.8, ofs:-40,        unit:'\u00b0F'},
  {pgn:0xFEF6, name:'Exhaust temp',   off:5, len:2, scale:0.05625, ofs:-459.4, unit:'\u00b0F'},
  {pgn:0xFEF7, name:'Alternator',     off:2, len:2, scale:0.05,                unit:'V'},
  {pgn:0xFEF7, name:'Battery',        off:4, len:2, scale:0.05,                unit:'V'},
  {pgn:0xFEFC, name:'Fuel level',     off:1, len:1, scale:0.4,                 unit:'%'},
  {pgn:0xFEF5, name:'Baro',           off:0, len:1, scale:0.07251885,          unit:'psi'},
  {pgn:0xFEF5, name:'Ambient temp',   off:3, len:2, scale:0.05625, ofs:-459.4, unit:'\u00b0F'}
];
function esc2(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;'); }
function decodeJ1939(f){
  if(!f.extended) return;
  // PGN incl. low byte, same formula as the field-verified j1939-logger:
  // (dp<<16)|(pf<<8)|ps. NOTE EEC1 is 0xF004 with pf=0xF0>=240 — the low byte
  // IS part of the PGN here, so do NOT mask ps away (earlier versions did and
  // silently matched nothing).
  var pgn=(f.id>>>8)&0x3FFFF;
  var hex=f.data_hex.trim().split(' ');
  for(var i=0;i<J1939.length;i++){
    var sg=J1939[i]; if(sg.pgn!==pgn) continue;
    var v=0, ok=true;
    for(var b=0;b<sg.len;b++){
      var idx=sg.off+b;
      if(idx>=hex.length){ok=false;break;}
      v|=parseInt(hex[idx],16)<<(8*b);   // J1939 multi-byte is little-endian
    }
    if(!ok) continue;
    if(v>=(sg.len===4?0xFB000000:(sg.len===2?0xFB00:0xFB))) continue;  // J1939 error/not-available
    pump[sg.name]={val:v*sg.scale+(sg.ofs||0),unit:sg.unit,ms:f.ms};
  }
}
function renderPump(newest){
  var el=document.getElementById('pumpVals'); if(!el) return;
  var s='';
  for(var k in pump){var p=pump[k];
    var age=newest&&p.ms?(((newest-p.ms)/1000).toFixed(0)+'s ago'):'now';
    var stale=newest&&p.ms&&(newest-p.ms>5000)?' stale':'';
    s+='<span class="'+(stale?'stale':'ok')+'"><b>'+k+'</b>: '+p.val.toFixed(1)+' '+p.unit
      +' <span class="small">('+age+')</span></span><br>';}
  if(s) el.innerHTML=s;
}
function toggleCap(){
  capOn=!capOn;
  var b=document.getElementById('capBtn');
  if(capOn){cap=[];b.innerHTML='&#9209; Stop capture ('+cap.length+')';
    document.getElementById('dlBtn').style.display='none';}
  else{b.innerHTML='&#9210; Start capture';
    document.getElementById('capNote').textContent=cap.length+' frames captured.';
    if(cap.length) document.getElementById('dlBtn').style.display='';}
}
function dlCsv(){
  var s='ms,id_hex,extended,dlc,data_hex\n';
  for(var i=0;i<cap.length;i++){var f=cap[i];
    var idh=(f.id>>>0).toString(16).toUpperCase();
    while(idh.length<8)idh='0'+idh;
    s+=f.ms+','+idh+','+(f.extended?1:0)+','+f.dlc+',"'+f.data_hex.trim().replace(/\s+/g,' ')+'"\n';}
  var blob=new Blob([s],{type:'text/csv'});
  var a=document.createElement('a');
  a.href=URL.createObjectURL(blob);
  a.download='can-capture-'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';
  a.click();
}
var feedInflight = false;
function fetchFeed(){
  if (document.getElementById('pauseFeed').checked) return;
  if (feedInflight) return;   // don't overlap polls: two in-flight fetches
  feedInflight = true;        // share one cursor and both append the same batch
  fetch('/api/can/live?since=' + lastMs).then(r=>r.json()).then(d=>{
    feedInflight = false;
    document.getElementById('canTotal').textContent = d.frameTotal;
    document.getElementById('canRate').textContent  = d.frameRate;
    var fr = d.frames || [];
    if (d.newestMs) lastMs = d.newestMs;
    if (!fr.length) return;
    var body = document.getElementById('feedBody');
    if (body.querySelector('td[colspan]')) body.innerHTML = '';
    var newest = d.newestMs || 0;
    for (var k = 0; k < fr.length; k++) {
      decodeJ1939(fr[k]);
      if (capOn && cap.length < 500000) cap.push(fr[k]);
    }
    if (capOn) document.getElementById('capBtn').innerHTML = '&#9209; Stop capture (' + cap.length + ')';
    renderPump(newest);
    var rows = '';
    // frames arrive oldest->newest; prepend newest first so the feed reads
    // top-down like a tail -f
    for (var i = fr.length - 1; i >= 0; i--) {
      var f = fr[i];
      var age = newest && f.ms ? ((newest - f.ms)/1000).toFixed(1) + 's' : 'now';
      var idHex = (f.id >>> 0).toString(16).toUpperCase();
      while (idHex.length < 8) idHex = '0' + idHex;
      rows += '<tr><td class="small">' + age + '</td><td class="mono">' + idHex +
              (f.extended ? 'x' : '') + '</td><td>' + f.dlc + '</td><td class="mono">' + esc2(f.data_hex) + '</td></tr>';
    }
    total += fr.length;
    document.getElementById('feedCount').textContent = total + ' frames shown';
    body.insertAdjacentHTML('afterbegin', rows);
    while (body.children.length > 300) body.removeChild(body.lastChild);
  }).catch(function(){ feedInflight = false; });
}
setInterval(fetchFeed, 1000); fetchFeed();
</script>)JS";
  h += "</div>";
  return h;
}

// ─── /live  LIVE VALUES TABLE ────────────────────────────────────────────────
static String livePage() {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128202; Live Status</h2>"
       "<div class='small' id='liveNet' style='margin:-6px 0 8px'>—</div>"
       "<div id='liveData'>Loading...</div>";
  h += R"(
<script>
function fetchLive(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    let s = '';
    const net = document.getElementById('liveNet');
    if (net) net.textContent = d.ip
      ? ('On "' + (d.ssid||'?') + '" @ ' + d.ip + '  (setup AP: 192.168.4.1)')
      : 'No network — setup AP only (192.168.4.1)';
    s += '<h3>RS485 Sensors</h3><table><tr><th>Name</th><th>Kind</th><th>Slave</th><th>Value</th><th>Volume</th><th>Status</th></tr>';
    (d.channels||[]).forEach(c=>{
      if (c.kind==='j1939') return;  // engine signals live in the CAN table below
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
    if (d.modbusChannels && d.modbusChannels.length) {
      s += '<h3>Modbus 4-20mA Board</h3><table><tr><th>Name</th><th>Kind</th><th>Channel</th><th>Value</th><th>mA</th><th>Status</th></tr>';
      d.modbusChannels.forEach(c=>{
        let cls = c.status==='ok'?'ok':(c.status==='stale'?'stale':'timeout');
        s += '<tr><td>'+(c.name||'')+'</td><td>'+(c.kind||'')+'</td><td>AI'+(c.ch+1)+'</td>';
        s += '<td>'+(c.value!=null?c.value+' '+(c.unit||''):'--')+'</td>';
        s += '<td>'+(c.ma!=null?c.ma+'mA':'--')+'</td><td class="'+cls+'">'+c.status+'</td></tr>';
      });
      s += '</table>';
    }
    // v1.19.30: digital I/O (AMIDJ14) was in the payload but never drawn here.
    const dig = function(title, arr){
      if (!arr || !arr.length) return '';
      let t = '<h3>'+title+'</h3><table><tr><th>Name</th><th>Channel</th><th>State</th><th>Status</th></tr>';
      arr.forEach(c=>{
        let ok = c.status==='ok' && c.state!=null;
        let st = ok ? (c.state ? 'ON' : 'OFF') : '--';
        let cls = c.status==='ok' ? 'ok' : (c.status==='stale' ? 'stale' : 'timeout');
        t += '<tr><td>'+(c.name||'')+'</td><td>'+(title.indexOf('Input')>=0?'DI':'DO')+(c.ch+1)+'</td>';
        t += '<td class="'+(ok&&c.state?'ok':'')+'"><b>'+st+'</b></td><td class="'+cls+'">'+c.status+'</td></tr>';
      });
      return t + '</table>';
    };
    s += dig('Digital Inputs', d.modbusDigitalInputs);
    s += dig('Digital Outputs', d.modbusDigitalOutputs);
    if (d.canEnabled) {
      s += '<h3>CAN Signals</h3><table><tr><th>Name</th><th>Kind</th><th>CAN ID</th><th>Value</th><th>Status</th></tr>';
      (d.canSignals||[]).forEach(c=>{
        let cls = c.status==='ok'?'ok':'stale';
        s += '<tr><td>'+(c.name||'')+'</td><td>'+(c.kind||'')+'</td><td>0x'+Number(c.canId).toString(16).toUpperCase()+'</td>';
        s += '<td>'+(c.value!=null?c.value+' '+c.unit:'--')+'</td><td class="'+cls+'">'+c.status+'</td></tr>';
      });
      // Onboard-decoded J1939 engine channels (kind j1939 in the channels
      // array) belong in THIS table, not the RS485 one (Sarah 2026-10-27).
      (d.channels||[]).filter(c=>c.kind==='j1939').forEach(c=>{
        let cls = c.status==='ok'?'ok':'stale';
        s += '<tr><td>'+(c.name||'')+'</td><td>j1939</td><td>'+(c.pgn!=null?'PGN 0x'+Number(c.pgn).toString(16).toUpperCase():'J1939')+'</td>';
        s += '<td>'+(c.value!=null?c.value+' '+(c.unit||''):'--')+'</td><td class="'+cls+'">'+c.status+'</td></tr>';
      });
      s += '</table>';
      s += '<p class="small">CAN frame rate: '+(d.canFrameRate||0)+' fps, total: '+(d.canFrameTotal||0)+'</p>';
    }
    let sys = d.system||{};
    s += '<h3>System</h3><table>';
    s += '<tr><td>Module ID</td><td>'+d.moduleId+'</td></tr>';
    s += '<tr><td>Host PC</td><td>'+(sys.piIp||'unresolved')+(sys.piMethod?' ('+sys.piMethod+')':'')+'</td></tr>';
    s += '<tr><td>Last post</td><td class="'+(sys.lastPostOk?'ok':'open')+'">'+((sys.lastPostMs>0)?(sys.lastPostMs/1000).toFixed(0)+'s ago':'never')+' '+(sys.lastPostOk?'ok':'fail')+'</td></tr>';
    s += '<tr><td>Buffer</td><td>'+sys.bufCount+' entries</td></tr>';
    s += '<tr><td>WiFi RSSI</td><td>'+(sys.rssi||'?')+' dBm</td></tr>';
    s += '<tr><td>Uptime</td><td>'+(d.uptimeS||0)+'s</td></tr>';
    s += '<tr><td>Free Heap</td><td>'+(sys.freeHeap||'?')+'</td></tr>';
    s += '<tr><td>NTP</td><td class="'+(sys.ntpOk?'ok':'warn')+'">'+(sys.ntpOk?d.ts:'no sync')+'</td></tr>';
    s += '</table>';
    document.getElementById('liveData').innerHTML = s;
  }).catch(function(e){
    // v1.19.26: was silently swallowing every failure — page sat on
    // "Loading..." forever during discovery with no explanation.
    var el=document.getElementById('liveData');
    if(el && el.textContent==='Loading...') el.textContent='Waiting for module... (retrying every 3s)';
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
  // v1.19.23 (Sarah: "not seeing its new ip, still see 192.168.4.1"): the
  // STA IP was only visible on the config page's WiFi card — nowhere else.
  // System page now shows BOTH addresses explicitly so "which IP is it on
  // now" is answerable at a glance.
  if (WiFi.status() == WL_CONNECTED) {
    h += "<br><b>WiFi:</b> " + _esc(cfg.wifiSSID) + " @ <b>" + WiFi.localIP().toString() +
         "</b> (" + String(WiFi.RSSI()) + " dBm)";
  } else {
    h += "<br><b>WiFi:</b> " + (cfg.wifiSSID.isEmpty()
         ? "not configured" : "NOT connected to " + _esc(cfg.wifiSSID));
  }
  h += "<br><b>Setup AP:</b> " + _esc(apSSID) + " @ " + WiFi.softAPIP().toString();
  h += "<br><b>Chip:</b> " + String(ESP.getChipModel()) + " @ " + String(ESP.getCpuFreqMHz()) + "MHz";
  h += "<br><b>CAN:</b> always on, " + String(cfg.canBitrate) + " bit/s" +
    (cfg.canopenBridge ? ", bridge active" : ", listen-only") +
    "</div>";
  // v1.15.32: the NVS-erase self-heal that used to bump this counter was
  // REMOVED (it was the actual root cause of a boot-loop bug — see
  // nvsEraseSelfHealCount's comment in config.h). A non-zero reading here
  // is now only possible on an OLDER firmware still running on this board.
  bool selfHealFired = cfg.nvsEraseSelfHealCount > 0;
  if (selfHealFired) {
    h += "<div class='card' style='border-color:#e74c3c'><b>&#9888; NVS self-heal fired " +
         String(cfg.nvsEraseSelfHealCount) + "x on old firmware.</b> Fixed in v1.15.32+, historical only.</div>";
  }
  NvsStats st = getNvsStats();
  bool nvsLow = false;
  if (st.ok) {
    nvsLow = st.freeEntries < 20;
    size_t nvsPartBytes = getNvsPartitionSizeBytes();
    h += "<div class='card'><b>NVS Storage:</b> " + String(st.usedEntries) + " used / " +
         String(st.totalEntries) + " total entries (" + String(st.freeEntries) + " free)";
    h += "<br>Partition size: " + String(nvsPartBytes) + " bytes";
    if (nvsPartBytes == 0x5000) {
      // Stock 20K partition — expected size, no warning.
    } else if (nvsPartBytes > 0 && nvsPartBytes < 0x5000) {
      h += " <span class='warn'>&#9888; Smaller than stock 20K — reflash recommended.</span>";
    } else if (nvsPartBytes >= 0x10000) {
      h += " <span class='warn'>&#9888; Non-stock partition table (" + String(nvsPartBytes) + " bytes).</span>";
    }
    if (nvsLow) {
      h += "<br><span class='warn'>&#9888; Running low — new settings may silently fail to save.</span>";
    }
    h += "</div>";
  }
  {
    // Live ground-truth readback: read mbBaud/mbBaudSet straight from
    // flash right now (not from cfg, which is just RAM) and compare
    // against what's currently in RAM. If these two ever disagree, it's
    // definitive proof of a save that didn't actually persist — no
    // guessing, no reboot-and-check-later.
    //
    // isKey() checked SEPARATELY from the value readback (2026-09-10 fix):
    // a device that's simply never had its Config page saved yet will
    // legitimately show flashBaud=-1 (the sentinel default, key absent)
    // while RAM shows the loadConfig() default of 9600 -- that is NOT
    // evidence of a broken save, it's evidence no save has ever been
    // attempted. Only "key EXISTS but has the wrong value" is real proof
    // of a save that silently failed.
    Preferences verify;
    verify.begin("rigmod", true);
    bool baudKeyExists = verify.isKey("mbBaud");
    bool baudSetKeyExists = verify.isKey("mbBaudSet");
    long flashBaud = verify.getLong("mbBaud", -1);
    bool flashBaudSet = verify.getBool("mbBaudSet", false);
    verify.end();
    bool keysAbsent = !baudKeyExists && !baudSetKeyExists;
    // NOTE: keysAbsent is genuinely AMBIGUOUS on its own -- it means either
    // "Config page has never been saved on this device" OR (only possible
    // on firmware OLDER than v1.15.32, since the self-heal path was removed
    // then) "the self-heal wipe fired and its own restore-save also failed
    // to recreate these keys". Do NOT assert "not a bug" here without
    // checking the self-heal counter -- that was the mistake to avoid
    // (2026-09-10).
    bool mismatch = !keysAbsent && ((flashBaud != cfg.modbusBaud) || (flashBaudSet != cfg.baudManuallySet));
    h += "<div class='card'><b>Baud Persistence Check:</b>";
    h += "<br>RAM: baud=" + String(cfg.modbusBaud) + " manuallySet=" + String(cfg.baudManuallySet ? "true" : "false");
    h += "<br>Flash: baud=" + String(flashBaud) + " manuallySet=" + String(flashBaudSet ? "true" : "false");
    // Terse, one line per state (2026-09-16, Sarah: no walls of text) —
    // full root-cause reasoning that used to live here moved to MEMORY.md.
    if (keysAbsent) {
      h += "<br><span class='warn'>&#9888; Not saved yet — click Save on /config.</span>";
    } else if (mismatch) {
      h += "<br><span class='warn'>&#9888; Mismatch — save didn't persist. Check serial log.</span>";
    } else {
      h += "<br><span class='ok'>&#10003; Match.</span>";
    }
    h += "</div>";
  }
  h += "<h3>Firmware Update</h3><div class='card'>";
  h += "<label>Upload firmware file (.bin):</label><div class='row'>";
  h += "<input type='file' id='fwFile' accept='.bin,application/octet-stream'>";
  h += "<button type='button' onclick='doFwUpload()'>Upload &amp; Update</button></div>";
  h += "<div id='fwStatus' class='small' style='margin-top:6px'></div>";
  h += "<details style='margin-top:8px'><summary class='small'>Update from URL</summary>";
  h += "<div class='row' style='margin-top:6px'><input id='otaUrl' placeholder='http://...'>&nbsp;";
  h += "<button onclick='doOTA()'>Update</button></div></details></div>";
  h += "<h3>Buffer</h3><div class='card'><div id='bufInfo'>...</div>";
  h += "<div class='small'>File line count vs. tracked count are cross-checked live below — if they "
       "ever disagree, the tracked number (used everywhere else, including this page) is wrong, not "
       "the file. See countBufferEntries()/bufferCount comment in the .ino.</div><br>";
  h += "<button onclick='fetch(\"/api/buffer/flush\",{method:\"POST\"}).then(()=>alert(\"Flushing\"))'>Flush Now</button>&nbsp;";
  h += "<button class='btn-red' onclick='if(confirm(\"Clear all buffered data?\"))fetch(\"/api/buffer/clear\",{method:\"POST\"}).then(()=>location.reload())'>Clear Buffer</button></div>";

  h += "<h3>Config Export / Import</h3><div class='card'>";
  h += "<p class='small'>Download every setting on this device as a single JSON file (RS485 sensors, Modbus board + "
       "digital I/O, CAN signals, module/Pi/WiFi settings). Restore it later, or upload it onto a different unit to "
       "clone this one's configuration. <b>Module ID is NOT included</b> — it's always derived from each device's own "
       "MAC address. The exported file DOES include your WiFi password in plain text — handle it with the same care "
       "as any written-down password.</p>";
  h += "<button type='button' onclick=\"window.location.href='/api/config/export'\">&#128190; Download Config (.json)</button>";
  h += "<div style='margin-top:14px'>";
  h += "<label>Restore from file:</label>";
  h += "<input type='file' id='importFile' accept='application/json,.json'>";
  h += "<button type='button' onclick='doImport()' style='margin-top:8px'>&#128228; Upload &amp; Restore</button>";
  h += "<div id='importResult' class='small' style='margin-top:8px'></div>";
  h += "</div></div>";

  h += "<h3>Danger Zone</h3><div class='card'>";
  h += "<button onclick='if(confirm(\"Reboot?\"))fetch(\"/api/reboot\",{method:\"POST\"})'>Reboot</button>&nbsp;";
  h += "<button class='btn-red' id='factoryResetBtn' onclick='doFactoryReset()'>Factory Reset</button>"
       "<span id='factoryResetResult' class='small'></span></div>";
  h += R"(
<script>
fetch('/api/status').then(r=>r.json()).then(d=>{
  let sys=d.system||{};
  let tracked=sys.bufCount||0, real=sys.bufCountReal||0;
  let txt='Buffered entries (tracked): '+tracked+' | on disk right now: '+real;
  if(tracked!==real) txt='<span style="color:#e74c3c">&#9888; MISMATCH — '+txt+' (tracked count is wrong, disk is the truth)</span>';
  document.getElementById('bufInfo').innerHTML = txt;
});
function doOTA(){
  let url=document.getElementById('otaUrl').value;
  if(!url)return alert('Enter URL');
  fetch('/api/ota?url='+encodeURIComponent(url)).then(r=>r.text()).then(t=>alert(t));
}
function doFwUpload(){
  let f=document.getElementById('fwFile').files[0];
  let box=document.getElementById('fwStatus');
  if(!f){box.textContent='Choose a .bin file first.';return;}
  if(!confirm('Flash '+f.name+'? Board reboots when done.'))return;
  box.textContent='Uploading...';
  let x=new XMLHttpRequest();
  x.open('POST','/update');
  x.upload.onprogress=function(e){if(e.lengthComputable)box.textContent='Uploading '+Math.round(e.loaded*100/e.total)+'%';};
  x.onload=function(){box.textContent=x.responseText;};
  x.onerror=function(){box.textContent='Upload failed — check the file is a .bin firmware image.';};
  // MUST be multipart/form-data — ESP32 WebServer only fires the upload
  // handler for multipart bodies; a raw x.send(f) never reaches it and the
  // route answers "Update FAILED" without the handler ever running.
  let fd=new FormData(); fd.append('bin', f);
  x.send(fd);
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
        if(d.ok){ box.innerHTML = '<span class="ok">Restored. Rebooting to apply...</span>'; setTimeout(()=>location.reload(), 2000); }
        else { box.innerHTML = '<span class="warn">Import failed: '+(d.error||'unknown error')+'</span>'; }
      }).catch(e=>{ box.textContent='Request failed: '+e; });
  };
  reader.readAsText(f);
}
function doFactoryReset(){
  if(!confirm('Factory reset? ALL config will be lost.'))return;
  let btn=document.getElementById('factoryResetBtn'), res=document.getElementById('factoryResetResult');
  btn.disabled=true; res.textContent=' resetting...';
  fetch('/api/factory-reset',{method:'POST'}).then(r=>r.json()).then(d=>{
    if(!d.ok){
      res.innerHTML=' <span style="color:#e74c3c">&#9888; reset did NOT fully succeed (clearOk='+d.clearOk+
        ' fsOk='+d.fsOk+' anyKeyStillPresent='+d.anyKeyStillPresent+') — check Serial log</span>';
      btn.disabled=false;
    } else {
      res.textContent=' done, rebooting...';
      setTimeout(()=>location.reload(), 4000);
    }
  }).catch(e=>{ res.textContent=' request failed ('+e+'), device may still be rebooting'; setTimeout(()=>location.reload(), 4000); });
}
</script>)";
  h += "</div>";
  return h;
}

// ─── API handler helpers ──────────────────────────────────────────────────────
static void handleApiStatus() {
  // Must be >= buildPayload()'s own doc size (16384) — buildPayload can
  // include up to MAX_CAN_SIGNALS entries plus, in CANopen Bridge mode,
  // up to 40 raw CAN frames. A smaller buffer here used to silently
  // truncate deserializeJson() partway through (no error checked), which
  // dropped whatever came after in the JSON — in practice the CAN
  // section, since it's built after "channels". Bumped to match so /live
  // (and anything else hitting this endpoint) actually sees CAN data.
  DynamicJsonDocument doc(16384);
  String payload = buildPayload(false);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("[API] /api/status: buildPayload() JSON did not parse (%s) — "
      "check MAX_SENSORS/MAX_CAN_SIGNALS growth vs buffer sizes\n", err.c_str());
  }
  doc["system"]["uptime"]     = millis() / 1000;
  doc["system"]["freeHeap"]   = ESP.getFreeHeap();
  doc["system"]["rssi"]       = WiFi.RSSI();
  doc["system"]["piIp"]       = resolvedPiIp;
  doc["system"]["piMethod"]   = discoveryMethod();  // static|name|rig-ssid|mdns|subnet-sweep|none
  doc["system"]["rigName"]    = _cfg->rigHostname;    // v1.19.24 learned <name>.local
  doc["system"]["lastPostMs"] = lastPostMs ? (millis() - lastPostMs) : -1;
  doc["system"]["lastPostOk"] = lastPostOk;
  doc["system"]["bufCount"]     = bufferCount; // in-RAM tracked count
  doc["system"]["bufCountReal"] = countBufferEntries(); // actual /buffer.jsonl line count right now — should always match bufCount; see .ino comment on the 2026-09-09 accounting bug this line exists to catch
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
  DynamicJsonDocument doc(12288);
  doc["frameTotal"] = canGetFrameTotal();
  doc["frameRate"]  = canGetRecentFrameRate();
  // v1.19.14: raw frame ring for the /can live feed. ?since=<ms> returns only
  // frames newer than the cursor so the browser can tail -f without resending
  // the whole ring every second. Ring is written by canPoll() in loop() and
  // read here (also loop()) — same task, no locking needed.
  unsigned long since = 0;
  if (_srv->hasArg("since")) since = strtoul(_srv->arg("since").c_str(), nullptr, 10);
  JsonArray frames = doc.createNestedArray("frames");
  doc["newestMs"] = canSerializeRecentFrames(frames, since, 120);
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
  if (maxAddr < SENSOR_MIN_SLAVE_ID) maxAddr = SENSOR_MIN_SLAVE_ID;
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

  // No canEnabled toggle -- CAN is unconditional (see waveshare-s3-sensors.ino
  // setup()). Only canBitrate/canopenBridge/node settings remain configurable.
  bool canChanged = false;
  applyParam("canBitrate", [&](String v){ long nb = v.toInt(); if (nb != _cfg->canBitrate) { _cfg->canBitrate = nb; canChanged = true; } });
  bool newCanopenBridge = _srv->hasArg("canopenBridge");
  if (newCanopenBridge != _cfg->canopenBridge) { _cfg->canopenBridge = newCanopenBridge; canChanged = true; }
  applyParam("canopenNodeId", [&](String v){
    uint8_t nid = (uint8_t)strtol(v.c_str(), nullptr, 16);
    if (nid != _cfg->canopenNodeId) { _cfg->canopenNodeId = nid; canChanged = true; }
  });
  bool newCanopenTargetSpecific = _srv->hasArg("canopenTargetSpecific");
  if (newCanopenTargetSpecific != _cfg->canopenTargetSpecific) {
    _cfg->canopenTargetSpecific = newCanopenTargetSpecific; canChanged = true;
  }
  // Raw stream is pure POST-side behavior — no TWAI restart needed, so it
  // doesn't participate in canChanged.
  bool newCanRawStream = _srv->hasArg("canRawStream");
  if (newCanRawStream != _cfg->canRawStream) { _cfg->canRawStream = newCanRawStream; }

  applyParam("pollIntervalS", [](String v){ _cfg->pollIntervalS = constrain(v.toInt(), 1, 30); });
  // NOTE: String::trim() mutates in place and returns void — it can't be used
  // inline in an assignment. (v1.15.17 had `_cfg->piHost = v.trim()`, which
  // compiled as `String = void` and killed the build.)
  applyParam("piHost",        [&](String v){ v.trim(); _cfg->piHost = v; });
  applyParam("rigToken",      [](String v){ _cfg->rigToken = v.isEmpty() ? "7804991970" : v; });

  // BUG FOUND 2026-09-10 (Sarah asked for a "spot sneaky bugs" pass):
  // wifiPass used to be applied with NO wifiChanged=true check at all --
  // only wifiSSID set the flag. Changing JUST the password (same SSID,
  // e.g. the site WiFi's password got rotated) saved the new password to
  // NVS correctly, but handleConfig()'s dispatch below only reboots
  // (which is what actually applies a new WiFi credential -- WiFi.begin()
  // is only called from setup()/connectToWiFi(), never mid-session) when
  // wifiChanged is true. Net effect: the page said "Saved" and redirected
  // back to /, the OLD password kept running until some UNRELATED reboot
  // happened to pick up the new one from flash -- a password change that
  // silently doesn't take effect, with no error and no indication why.
  bool wifiChanged = false;
  // v1.15.13: SSID arrives from a <select> (name=wifiSSID). The special
  // value __manual__ means the user chose "Type manually..." and the real
  // value is in wifiSSIDManual. A literal "__manual__" must NEVER be saved
  // as an SSID (that would brick WiFi config until Forget WiFi).
  String submittedSsid = "";  // what the form actually resolved to
  applyParam("wifiSSID", [&](String v){
    if (v == "__manual__") {
      String mv = _srv->hasArg("wifiSSIDManual") ? _srv->arg("wifiSSIDManual") : String("");
      mv.trim();
      if (mv.isEmpty()) { submittedSsid = _cfg->wifiSSID; return; }  // blank manual: keep existing
      v = mv;
    }
    submittedSsid = v;
    if (v != _cfg->wifiSSID) { _cfg->wifiSSID = v; wifiChanged = true; }
  });
  // BUG FOUND 2026-09-15 (Sarah: "waveshare now is totally resetting the
  // wifi settings when it reboots when i try to set it"): wifiPass is
  // NEVER pre-filled into the form on purpose (see the label below this
  // form -- "Password is never pre-filled, type it fresh"). That means
  // EVERY /config save, for ANY field, submits wifiPass="" (blank) unless
  // the user actually retyped it that visit. The old check `v !=
  // _cfg->wifiPass` treated that blank submission as a real change
  // (blank almost never equals the actual saved password), silently
  // WIPED the saved password to empty, and set wifiChanged=true --
  // forcing an immediate reboot that then fails to join with a blank
  // password. This bug already existed but was masked as long as
  // wifiPass was type='password': Chrome/Firefox's own saved-credential
  // autofill (the exact prompt Sarah asked to remove in v1.15.34) was
  // silently re-filling the real password back into the field on every
  // page load, so `v` was never actually blank in practice. Switching to
  // type='text' (v1.15.34) removed that accidental safety net and fully
  // exposed this. Fix: a blank submitted password means "user didn't
  // type a new one" -- leave the saved password untouched. Only a
  // genuinely non-empty, different value counts as a real change.
  bool passActuallyChanged = false;
  applyParam("wifiPass", [&](String v){
    if (!v.isEmpty() && v != _cfg->wifiPass) { _cfg->wifiPass = v; wifiChanged = true; passActuallyChanged = true; }
  });

  // v1.15.14 serial diagnostic (field report: "SSID won't change" twice on
  // 2026-09-11 and we've been guessing). Print exactly what the browser
  // submitted vs what's in NVS, so the next report is settled by one log line
  // instead of another firmwares.
  // v1.15.35: pass_changed now reflects passActuallyChanged (the real
  // decision made above), not a raw string comparison against a field
  // that's blank on every single save by design -- that comparison would
  // print "yes" on every save regardless of whether the password was
  // actually touched, which is exactly the kind of misleading diagnostic
  // that would send a future debugging session down the wrong path.
  Serial.printf("[CFG] POST wifi: submitted_ssid=\"%s\" saved_ssid=\"%s\" pass_changed=%s ap_mode=%s\n",
    submittedSsid.c_str(), _cfg->wifiSSID.c_str(),
    passActuallyChanged ? "yes" : "no",
    apModeActive ? "yes" : "no");

  saveConfig(*_prefs, *_cfg);

  // v1.15.21: give loopTask a moment to finish any LittleFS write it's in the
  // middle of before we reboot out from under it. This WiFi-save path reboots
  // from webTask while loopTask can be inside appendBufferEntry()/
  // removeBufferHead() — LittleFS has no cross-task locking, and cutting a
  // write mid-flight is a good way to leave the partition unmountable, which
  // is exactly the "LittleFS mount failed — formatting" line Sarah kept seeing
  // after every WiFi change (and it silently ate the outage buffer each time).
  // This doesn't make the two tasks safe against each other in general; it
  // just removes the guaranteed collision at the reboot.
  delay(1500);

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

  // BUG FIXED 2026-09-17 (Sarah's "spot sneaky bugs" pass): apModeActive
  // used to be checked BEFORE baudChanged/canChanged. modbusInit()/
  // canStart() only ever run once, from setup() — a baud or CAN/CANopen
  // change only takes effect after a reboot, same as WiFi. But if you're
  // on the setup AP (totally normal during first-time bench setup, before
  // WiFi is even configured) and change ONLY the baud rate or CAN
  // settings — no WiFi field touched — the old ordering hit the
  // apModeActive branch first and told you "Saved. No SSID change
  // detected, so NOT rebooting", implying nothing needed a reboot. That
  // was wrong for exactly this case: the change WAS saved to NVS, but
  // silently never applied to the running board until some unrelated
  // future reboot happened to pick it up. baudChanged/canChanged now
  // come first — they need a reboot regardless of AP/STA state, and
  // rebooting while on the AP is safe (v1.15.32+: the AP is unconditional
  // and comes right back up either way). apModeActive now only fires
  // when it's true AND genuinely nothing needs a reboot.
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
  } else if (apModeActive) {
    // v1.15.14: don't reboot-loop in the AP when nothing WiFi-related
    // (or, as of this fix, baud/CAN-related) actually changed.
    _srv->send(200, "text/html; charset=utf-8",
      "<p>Saved. <b>No SSID change detected, so NOT rebooting</b> — the unit is still in setup-AP mode"
      " and would just come back here. Pick a network from the scan list (or Type manually...) and"
      " press Save again to join it. Current saved SSID: <b>" + _esc(_cfg->wifiSSID) + "</b></p>" + verifyMsg);
  } else {
    _srv->sendHeader("Location", "/");
    _srv->send(302, "text/plain", "");
  }
}

// Saves the whole RS485 sensor list from /sensors' single shared form.
static void handleModbusLive() {
  DynamicJsonDocument doc(2560); JsonArray a=doc.createNestedArray("channels");
  for(int i=0;i<MAX_MODBUS_CHANNELS;i++){if(!_cfg->modbusBoard.channels[i].enabled)continue;JsonObject o=a.createNestedObject();o["ch"]=i;o["name"]=_cfg->modbusBoard.channels[i].name;o["kind"]=_cfg->modbusBoard.channels[i].kind;o["unit"]=_cfg->modbusBoard.channels[i].unit;o["ma"]=modbusReadings[i].hasValue?modbusReadings[i].mA:(float)0;o["value"]=modbusReadings[i].hasValue?modbusReadings[i].value:(float)0;if(!modbusReadings[i].hasValue){o["ma"]=nullptr;o["value"]=nullptr;}o["status"]=modbusReadings[i].status;}
  JsonArray din=doc.createNestedArray("din");
  for(int i=0;i<4;i++){if(!_cfg->modbusBoard.din[i].enabled)continue;JsonObject o=din.createNestedObject();o["ch"]=i;if(modbusDinReadings[i].valid)o["state"]=modbusDinReadings[i].state;else o["state"]=nullptr;o["status"]=modbusDinReadings[i].status;}
  JsonArray dout=doc.createNestedArray("dout");
  for(int i=0;i<4;i++){if(!_cfg->modbusBoard.dout[i].enabled)continue;JsonObject o=dout.createNestedObject();o["ch"]=i;if(modbusDoutReadings[i].valid)o["state"]=modbusDoutReadings[i].state;else o["state"]=nullptr;o["status"]=modbusDoutReadings[i].status;}
  String out;serializeJson(doc,out);_srv->send(200,"application/json",out);
}
static void handleModbusSave() {
  _cfg->modbusBoard.enabled=_srv->hasArg("enabled"); if(_srv->hasArg("boardType"))_cfg->modbusBoard.boardType=_srv->arg("boardType");
  for(int i=0;i<MAX_MODBUS_CHANNELS;i++){String p="mb"+String(i);auto& c=_cfg->modbusBoard.channels[i];c.enabled=_srv->hasArg((p+"en").c_str());if(_srv->hasArg((p+"nm").c_str()))c.name=_srv->arg((p+"nm").c_str());if(_srv->hasArg((p+"kd").c_str()))c.kind=_srv->arg((p+"kd").c_str());if(_srv->hasArg((p+"ut").c_str()))c.unit=_srv->arg((p+"ut").c_str());if(_srv->hasArg((p+"lo").c_str()))c.engMin=_srv->arg((p+"lo").c_str()).toFloat();if(_srv->hasArg((p+"hi").c_str()))c.engMax=_srv->arg((p+"hi").c_str()).toFloat();}
  for(int i=0;i<4;i++){String p="mbdi"+String(i);auto& d=_cfg->modbusBoard.din[i];d.enabled=_srv->hasArg((p+"en").c_str());if(_srv->hasArg((p+"nm").c_str()))d.name=_srv->arg((p+"nm").c_str());}
  for(int i=0;i<4;i++){String p="mbdo"+String(i);auto& d=_cfg->modbusBoard.dout[i];d.enabled=_srv->hasArg((p+"en").c_str());if(_srv->hasArg((p+"nm").c_str()))d.name=_srv->arg((p+"nm").c_str());}
  saveConfig(*_prefs,*_cfg);_srv->sendHeader("Location","/modbus");_srv->send(302,"text/plain","");
}

// Applies a known board's factory-default settings in one step (2026-09-18,
// Sarah: "put out the proper settings for each board and set the correct
// baud rate"). Sets: boardType (explicit, not "auto"), the shared RS485
// baud (cfg.modbusBaud — same field /config's baud selector writes, since
// this variant's whole bus, board + direct sensors, shares one Serial2),
// and disables analog channels beyond the board's real channel count
// (doesn't touch names/kind/unit/engMin/engMax on channels that stay
// enabled, or DI/DO — those are left as the user configured them).
// Reboots only if the baud actually changed, same "only reboot when it's
// genuinely needed" rule as handleConfig()'s baudChanged branch — Serial2
// only re-inits from setup(), so a baud change can't take effect any other
// way, but changing just boardType/channel-enables doesn't need one since
// pollTask() reads cfg.modbusBoard fresh every cycle.
static void handleModbusPreset() {
  String bt = _srv->hasArg("boardType") ? _srv->arg("boardType") : "";
  const ModbusBoardPreset* preset = modbusFindPreset(bt);
  if (!preset) {
    _srv->send(400, "application/json", "{\"ok\":false,\"error\":\"unknown board type\"}");
    return;
  }
  _cfg->modbusBoard.boardType = bt;
  for (int i = 0; i < MAX_MODBUS_CHANNELS; i++) {
    if (i >= preset->numChannels) _cfg->modbusBoard.channels[i].enabled = false;
  }
  bool baudChanged = (_cfg->modbusBaud != preset->baud);
  _cfg->modbusBaud = preset->baud;
  _cfg->baudManuallySet = true; // explicit board choice counts as "user decided", same as /config's baud form
  saveConfig(*_prefs, *_cfg);

  DynamicJsonDocument doc(256);
  doc["ok"] = true;
  doc["baud"] = preset->baud;
  doc["numChannels"] = preset->numChannels;
  doc["rebooting"] = baudChanged;
  String out; serializeJson(doc, out);
  _srv->send(200, "application/json", out);
  if (baudChanged) { delay(300); ESP.restart(); }
}

// Writes a single digital output on the adapter board (FC05) — an
// immediate, non-persisted bus write, same "instant action button"
// pattern as the analog-board variant's /api/digital/write. Only valid
// when the detected board is AMIDJ14 (the Waveshare 8AI has no DO coils
// at all); holds modbusBusMutex since this is a genuine extra wire
// transaction outside the poll task's own cycle.
static void handleModbusDigitalWrite() {
  if (modbusDetectedType != "amidj14") {
    _srv->send(400, "application/json", "{\"ok\":false,\"error\":\"detected board has no digital outputs\"}");
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
    ok = modbusWriteBoardDO(ch, value);
    xSemaphoreGive(modbusBusMutex);
  }
  if (ok) {
    modbusDoutReadings[ch].valid = true;
    modbusDoutReadings[ch].state = value;
    modbusDoutReadings[ch].status = "ok";
  }
  _srv->send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

static void handleSensorsSave() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    String pre = "s" + String(i);
    SensorConfig& s = _cfg->sensors[i];
    s.enabled = _srv->hasArg((pre + "en").c_str());
    if (_srv->hasArg((pre + "nm").c_str()))  s.name = _srv->arg((pre + "nm").c_str());
    if (_srv->hasArg((pre + "kd").c_str()))  s.kind = _srv->arg((pre + "kd").c_str());
    if (_srv->hasArg((pre + "ut").c_str()))  s.unit = _srv->arg((pre + "ut").c_str());
    if (_srv->hasArg((pre + "sid").c_str())) s.slaveId = (uint8_t)constrain(_srv->arg((pre + "sid").c_str()).toInt(), SENSOR_MIN_SLAVE_ID, 247);
    if (_srv->hasArg((pre + "fc").c_str()))  s.funcCode = (uint8_t)_srv->arg((pre + "fc").c_str()).toInt();
    if (_srv->hasArg((pre + "reg").c_str())) s.regAddr = (uint16_t)strtol(_srv->arg((pre + "reg").c_str()).c_str(), nullptr, 0);
    if (_srv->hasArg((pre + "dt").c_str()))  s.dataType = (uint8_t)_srv->arg((pre + "dt").c_str()).toInt();
    if (_srv->hasArg((pre + "wo").c_str()))  s.wordOrder = (uint8_t)_srv->arg((pre + "wo").c_str()).toInt();
    if (_srv->hasArg((pre + "ro").c_str()))  s.respRegOffset = (uint8_t)constrain(_srv->arg((pre + "ro").c_str()).toInt(), 0, 15);
    if (_srv->hasArg((pre + "sc").c_str()))  s.scale = _srv->arg((pre + "sc").c_str()).toFloat();
    if (_srv->hasArg((pre + "of").c_str()))  s.offset = _srv->arg((pre + "of").c_str()).toFloat();
    s.volumeEnabled = _srv->hasArg((pre + "volEn").c_str());
    // CHANGED 2026-09-17 (two passes same day): "cap" (single capacity
    // number) -> "len"/"wid" (tank footprint, meters) -> "vz"/"vm"
    // (Value @ Empty/Full calibration) removed entirely, replaced by a
    // single "th" (tank height, meters) -- see SensorConfig's
    // volumeEnabled comment in config.h for the current model.
    if (_srv->hasArg((pre + "len").c_str())) s.tankLengthM = _srv->arg((pre + "len").c_str()).toFloat();
    if (_srv->hasArg((pre + "wid").c_str())) s.tankWidthM = _srv->arg((pre + "wid").c_str()).toFloat();
    if (_srv->hasArg((pre + "th").c_str()))  s.tankHeightM = _srv->arg((pre + "th").c_str()).toFloat();
    if (_srv->hasArg((pre + "cu").c_str()))  s.capacityUnit = _srv->arg((pre + "cu").c_str());
  }
  saveConfig(*_prefs, *_cfg);

  // Readback verification — same idea as the mbBaud check in saveConfig():
  // confirm what's ACTUALLY in flash right after writing it, not just
  // what's in the RAM struct we just wrote from. Added 2026-08-18 to
  // chase down a report of fc reverting to 4 after a save — if this
  // fires, the write itself is failing (NVS full/corrupt), not a logic
  // bug in the save/load code.
  //
  // UPDATED 2026-09-16: reads back the packed "sensorsBlob" (see
  // SensorConfigPacked in config.h) instead of individual per-key
  // "sX_fc" values — those old keys are no longer written.
  //
  // STACK OVERFLOW BUG FIXED (2026-09-18): `packed` here is 2688 bytes
  // (168 x MAX_SENSORS) — as a plain local, stacked directly on top of
  // saveConfig()'s OWN 2688-byte local of the same shape (just called,
  // above) in the same request-handler call chain. Confirmed via a real
  // -fstack-usage compile: this function alone uses 2912 bytes of stack,
  // saveConfig() another 3904 — 6816+ bytes combined against the
  // default 8192-byte loop task stack, with WebServer's own frames and
  // everything the flash/NVS driver needs UNDER this eating the rest.
  // That's what was actually crashing/rebooting the board on "Save All
  // Sensors" and losing the settings — see the long comment on
  // saveConfig()'s copy of this exact pattern in config.h for the full
  // writeup. `static` moves this out of the stack frame into .bss;
  // safe because this handler only ever runs synchronously, one request
  // at a time, on a single task.
  {
    static SensorConfigPacked packed[MAX_SENSORS];
    _prefs->begin("rigmod", true); // read-only
    size_t got = _prefs->getBytes("sensorsBlob", packed, sizeof(packed));
    _prefs->end();
    if (got != sizeof(packed)) {
      Serial.printf("[Sensors] MISMATCH: sensorsBlob readback got %u bytes, expected %u — NVS write may have failed!\n",
        (unsigned)got, (unsigned)sizeof(packed));
    } else {
      for (int i = 0; i < MAX_SENSORS; i++) {
        if (!_cfg->sensors[i].enabled) continue;
        if (packed[i].funcCode != _cfg->sensors[i].funcCode) {
          Serial.printf("[Sensors] MISMATCH slot %d: wrote fc=%d, flash readback=%d — NVS write may have failed!\n",
            i, _cfg->sensors[i].funcCode, packed[i].funcCode);
        } else {
          Serial.printf("[Sensors] slot %d saved OK: fc=%d (slaveId=%d)\n",
            i, packed[i].funcCode, packed[i].slaveId);
        }
      }
    }
  }

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
  uint8_t slaveId = _p("slaveId").isEmpty() ? SENSOR_MIN_SLAVE_ID : constrain(_p("slaveId").toInt(), SENSOR_MIN_SLAVE_ID, 247);
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

// POST /update — multipart file upload of a compiled .bin firmware image.
// Flash-write progress goes to serial; the browser gets one final
// pass/fail line (XHR progress covers the upload half). Update.begin()
// automatically targets the inactive OTA slot when the partition scheme
// has one; on single-app schemes it rewrites the running app in place —
// a power cut mid-flash then needs a USB reflash, same as any update.
static bool _fwUploadOk = false;
static void handleUpdateUpload() {
  // Core 3.3.x: WebServer::upload is a method, not a public member (2.x).
  HTTPUpload& upload = _srv->upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[FwUpdate] Start: %s\n", upload.filename.c_str());
    _fwUploadOk = Update.begin(UPDATE_SIZE_UNKNOWN);
    if (!_fwUploadOk) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (_fwUploadOk && Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      _fwUploadOk = false;
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (_fwUploadOk && Update.end(true)) {
      Serial.printf("[FwUpdate] OK: %u bytes written\n", upload.totalSize);
    } else {
      _fwUploadOk = false;
      Update.printError(Serial);
    }
  }
}
static void handleUpdateDone() {
  if (_fwUploadOk) {
    _srv->send(200, "text/plain", "Update OK — rebooting, reload this page in ~20s.");
    delay(1000);
    ESP.restart();
  } else {
    _srv->send(500, "text/plain", "Update FAILED — old firmware still running. Check serial log.");
  }
}

// =============================================================================
// CONFIG EXPORT / IMPORT HANDLERS (2026-09-18, Sarah's request)
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
// in the JSON), saves to NVS, and reboots so every subsystem (Serial2
// baud, CAN driver) picks up the restored settings cleanly.
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

  // v1.19.29 CAPTIVE PORTAL. The DNS catch-all (dnsServer in the .ino) sends
  // every hostname a phone on the setup AP looks up to us; phones then probe a
  // well-known URL expecting a specific "internet works" answer. We answer
  // with a 302 to the setup page instead, which makes the OS pop its
  // "sign in to network" browser on that page. Only honoured for requests that
  // arrived via the AP interface — anything via the site-WiFi side gets a
  // plain 404 so this can't redirect real LAN traffic.
  auto captive = [](){
    if (_srv->client().localIP() != WiFi.softAPIP()) { _srv->send(404, "text/plain", "Not found"); return; }
    _srv->sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    _srv->sendHeader("Cache-Control", "no-store");
    _srv->send(302, "text/plain", "");
  };
  static const char* const kProbePaths[] = {
    "/generate_204", "/gen_204",                       // Android / Chrome
    "/hotspot-detect.html", "/library/test/success.html", "/success.txt", // Apple
    "/connecttest.txt", "/ncsi.txt", "/redirect", "/fwlink", // Windows
    "/canonical.html"                                  // Firefox
  };
  for (size_t i = 0; i < sizeof(kProbePaths)/sizeof(kProbePaths[0]); i++) srv.on(kProbePaths[i], HTTP_ANY, captive);
  srv.onNotFound(captive);
  srv.on("/sensors",  HTTP_GET, [noCacheHtml](){ noCacheHtml(200, sensorsPage(*_cfg)); });
  srv.on("/modbus", HTTP_GET, [noCacheHtml](){ noCacheHtml(200, modbusPage(*_cfg)); });
  srv.on("/can",      HTTP_GET, [noCacheHtml](){ noCacheHtml(200, canPage(*_cfg)); });
  srv.on("/live",     HTTP_GET, [noCacheHtml](){ noCacheHtml(200, livePage()); });
  srv.on("/system",   HTTP_GET, [noCacheHtml](){ noCacheHtml(200, sysPage(*_cfg)); });

  // GET APIs
  srv.on("/api/status",       HTTP_GET, handleApiStatus);
  srv.on("/api/sensors/live", HTTP_GET, handleSensorsLive);
  srv.on("/api/modbus/live", HTTP_GET, handleModbusLive);
  srv.on("/api/can/live",     HTTP_GET, handleCanLive);
  srv.on("/api/modbus/probe", HTTP_GET, handleModbusProbe);
  srv.on("/api/wifi/scan",    HTTP_GET, handleWifiScan);
  srv.on("/api/ota",          HTTP_GET, handleOTA);
  srv.on("/update",           HTTP_POST, handleUpdateDone, handleUpdateUpload);

  // POST APIs
  srv.on("/api/config",         HTTP_POST, handleConfig);
  srv.on("/api/sensors/save",   HTTP_POST, handleSensorsSave);
  srv.on("/api/modbus/save", HTTP_POST, handleModbusSave);
  srv.on("/api/modbus/preset", HTTP_POST, handleModbusPreset);
  srv.on("/api/can/save",       HTTP_POST, handleCanSave);
  srv.on("/api/wifi/forget",    HTTP_POST, handleWifiForget);
  srv.on("/api/modbus/autodetect", HTTP_POST, handleModbusAutoDetect);
  srv.on("/api/modbus/autodetect-enable", HTTP_POST, handleAutoDetectEnable);
  srv.on("/api/modbus/digital/write", HTTP_POST, handleModbusDigitalWrite);


  srv.on("/api/buffer/flush", HTTP_POST, [](){
    flushNow = true;
    _srv->send(200,"application/json","{\"ok\":true}");
  });
  srv.on("/api/buffer/clear", HTTP_POST, [](){
    LittleFS.remove("/buffer.jsonl");
    bufferCount = 0;
    _srv->send(200,"application/json","{\"ok\":true}");
  });
  srv.on("/api/config/export", HTTP_GET, handleConfigExport);
  srv.on("/api/config/import", HTTP_POST, handleConfigImport);
  srv.on("/api/reboot", HTTP_POST, [](){
    _srv->send(200,"application/json","{\"ok\":true}");
    delay(500); ESP.restart();
  });
  srv.on("/api/factory-reset", HTTP_POST, [](){
    // BUG FIXED 2026-09-09: clear()'s return value was never checked —
    // same class of silent-failure blind spot this file already guards
    // against for mbBaud writes in saveConfig(). If clear() ever
    // silently fails/partially fails, every setting would read back
    // whatever was left over instead of truly-unset, and Factory Reset
    // would appear to do nothing. Verified with a fresh read-only
    // Preferences handle + isKey() — NOT the same handle we just
    // cleared, and NOT getBool()-with-a-default (which can't
    // distinguish "key gone, using default" from "key still there,
    // happens to equal the default").
    _prefs->begin("rigmod",false);
    bool clearOk = _prefs->clear();
    _prefs->end();

    Preferences verify;
    verify.begin("rigmod", true);
    bool copBrStillPresent = verify.isKey("copBr");
    bool mbBaudStillPresent = verify.isKey("mbBaud");
    bool anyKeyStillPresent = verify.isKey("modName") || copBrStillPresent || mbBaudStillPresent;
    verify.end();

    Serial.printf("[FactoryReset] clear()=%s copBr still present=%s mbBaud still present=%s "
      "any key still present=%s\n",
      clearOk ? "true" : "FALSE",
      copBrStillPresent ? "TRUE (BUG)" : "false",
      mbBaudStillPresent ? "TRUE (BUG)" : "false",
      anyKeyStillPresent ? "TRUE (BUG — clear() did not actually clear)" : "false");

    // formatLittleFSWithWatchdog() (waveshare-s3-sensors.ino) forces a
    // reboot if format() hangs instead of leaving the board dead — see
    // that function's header comment for the field incident this fixes.
    bool fsOk = formatLittleFSWithWatchdog();
    Serial.printf("[FactoryReset] LittleFS.format()=%s\n", fsOk ? "true" : "FALSE");

    DynamicJsonDocument doc(256);
    doc["ok"] = clearOk && fsOk && !anyKeyStillPresent;
    doc["clearOk"] = clearOk;
    doc["fsOk"] = fsOk;
    doc["anyKeyStillPresent"] = anyKeyStillPresent;
    String out;
    serializeJson(doc, out);
    _srv->send(200, "application/json", out);

    delay(500); ESP.restart();
  });
}
