// =============================================================================
// webui.h — WebServer routes: 4-page UI + REST API
// Uses built-in ESP32 WebServer (no ESPAsyncWebServer needed)
// Generic Rig Module: no tank/mud-specific pages. Each of the 8 channels is
// independently configured (enable/name/kind/unit/scaling/calibration) and
// reported as a raw engineering value — any higher-level meaning (volume,
// mud weight, etc.) is applied elsewhere, not on this device.
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
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t modbusBusMutex;
long modbusAutoDetectBaud(uint8_t slaveId, uint32_t originalBaud); // modbus.h
BoardProfile modbusDetectBoard(uint8_t slaveId); // modbus.h
extern BoardProfile boardProfile; // rig-module-firmware.ino — detected once at boot
extern NTPClient ntpClient;
extern bool apModeActive;
extern String apSSID;
// FW_VERSION is a #define in rig-module-firmware.ino — no extern needed

// Globals set in setupWebRoutes, used by handlers
static WebServer*    _srv    = nullptr;
static ModuleConfig* _cfg    = nullptr;
static Preferences*  _prefs  = nullptr;
static ChannelReading* _readings = nullptr;
static uint16_t*     _raw    = nullptr;
static SemaphoreHandle_t _mtx = nullptr;

// ─── Shared CSS / nav ────────────────────────────────────────────────────────
static const char NAV[] PROGMEM = R"(
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#1a1a2e;color:#e0e0e0;font-family:sans-serif;font-size:14px}
  .nav{background:#16213e;padding:10px 16px;display:flex;flex-wrap:wrap;gap:8px;border-bottom:2px solid #667eea}
  .nav a{color:#667eea;text-decoration:none;padding:4px 10px;border-radius:4px;border:1px solid #667eea}
  .nav a:hover{background:#667eea;color:#fff}
  .page{padding:16px;max-width:700px;margin:0 auto}
  h2{color:#667eea;margin-bottom:16px}
  h3{color:#aaa;margin:16px 0 8px}
  label{display:block;margin-bottom:4px;color:#aaa;font-size:12px}
  input,select{width:100%;padding:8px;background:#0f3460;border:1px solid #667eea;color:#fff;border-radius:4px;margin-bottom:10px}
  input[type=checkbox]{width:auto;margin-right:8px}
  button,.btn{padding:8px 18px;background:#667eea;color:#fff;border:none;border-radius:4px;cursor:pointer}
  button:hover{background:#5568d4}
  .btn-red{background:#c0392b}
  .card{background:#16213e;border:1px solid #334;border-radius:6px;padding:12px;margin-bottom:12px}
  .ok{color:#2ecc71} .open{color:#e74c3c} .over{color:#e74c3c} .stale{color:#888} .warn{color:#f39c12}
  .row{display:flex;gap:8px;align-items:center}
  .small{font-size:12px;color:#888}
  table{width:100%;border-collapse:collapse}
  td,th{padding:6px 8px;border-bottom:1px solid #334;text-align:left}
  th{color:#667eea;font-size:12px}
  .ap-banner{background:#f39c12;color:#1a1a2e;padding:8px 16px;font-weight:bold;text-align:center}
  .ap-banner a{color:#1a1a2e;text-decoration:underline}
</style>
<div class='nav'>
  <a href='/'>&#9881; Config</a>
  <a href='/channels'>&#128208; Channels</a>
  <a href='/live'>&#128202; Live</a>
  <a href='/system'>&#128295; System</a>
</div>
)";

// AP-mode warning banner — prepended to pages when broadcasting the setup AP.
// WiFi is configured right on this page (the Config page, "/") now — the
// old standalone /wifi page was a redundant duplicate of the same SSID/
// password fields and has been removed.
static String apBanner() {
  if (!apModeActive) return "";
  return "<div class='ap-banner'>No WiFi configured yet — connect a device to this AP and "
         "set your network below.</div>";
}

// ─── Helper: get param from WebServer ────────────────────────────────────────
static String _p(const char* name) {
  if (_srv->hasArg(name)) return _srv->arg(name);
  return "";
}
static bool _has(const char* name) { return _srv->hasArg(name); }

// ─── HTML pages ──────────────────────────────────────────────────────────────
static String cfgPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += apBanner();
  h += "<div class='page'><h2>&#9881; Module Configuration</h2>";
  h += "<div class='card'><b>Module ID:</b> " + cfg.moduleId + " <span class='small'>(fixed, derived from MAC — not editable)</span></div>";
  h += "<form method='POST' action='/api/config'>";
  h += "<label>Module Name</label><input name='moduleName' value='";
  h += cfg.moduleName;
  h += "' placeholder='e.g. Mud Pump Skid, Water Tank 2'>";
  h += "<label>Module Type</label><input name='moduleType' list='moduleTypeOpts' value='";
  h += cfg.moduleType;
  h += "' placeholder='e.g. tank, pump, pressure, drill'>";
  h += "<datalist id='moduleTypeOpts'>"
       "<option value='generic'>"
       "<option value='tank'>"
       "<option value='pump'>"
       "<option value='pressure'>"
       "<option value='drill'>"
       "</datalist>";
  h += "<div class='small'>Shown on the rig dashboard's module card. \"tank\" and \"pump\" get their "
       "own dedicated layout (fill bar / SPM gauge); anything else — including free text like "
       "\"drill\" — just shows as a plain labeled tile view. Type anything, or pick a suggestion.</div>";
  h += "<label>Description</label><input name='description' value='";
  h += cfg.description;
  h += "'>";
  h += "<h3>Modbus / Pi</h3>";
  h += "<label>Modbus Slave ID</label><input name='modbusSlaveId' type='number' min='1' max='247' value='";
  h += String(cfg.modbusSlaveId);
  h += "'>";
  h += "<label>RS485 Baud Rate</label><select name='modbusBaud'>";
  {
    // value, label — labeled by which board ships with it as factory default,
    // so it's obvious which to pick when swapping analog-to-Modbus boards
    // without needing to remember datasheet defaults.
    struct { long val; const char* label; } bauds[] = {
      { 1200,   "1200" },
      { 2400,   "2400" },
      { 4800,   "4800 (SDSIN)" },
      { 9600,   "9600 (Waveshare)" },
      { 19200,  "19200" },
      { 38400,  "38400" },
      { 57600,  "57600" },
      { 115200, "115200" },
    };
    for (auto& b : bauds) {
      h += "<option value='" + String(b.val) + "'";
      if (cfg.modbusBaud == b.val) h += " selected";
      h += ">" + String(b.label) + "</option>";
    }
  }
  h += "</select>";
  h += "<button type='button' id='autoBaudBtn' onclick='autoDetectBaud()'>&#128269; Auto-Detect Baud Rate</button>"
       "<div class='small' id='autoBaudStatus' style='margin:6px 0 10px'>Probes the connected board at every "
       "standard rate and picks whichever gets a real response — no need to know the board's factory default.</div>"
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
  h += "<label>Poll Interval (1-30 s)</label><input name='pollIntervalS' type='number' min='1' max='30' value='";
  h += String(cfg.pollIntervalS);
  h += "'>";
  h += "<label>Pi Host (blank = auto)</label><input name='piHost' value='";
  h += cfg.piHost;
  h += "' placeholder='192.168.x.x or rig-logger.local'>";
  h += "<div class='small'>Leave blank for auto-discovery: on a standard \"rigNNN\" WiFi network, "
       "this derives 192.168.NNN.10 automatically (no typing needed); otherwise falls back to mDNS "
       "(_rig-logger._tcp.local) then rig-logger.local. Only set this manually for a non-standard "
       "network or a Pi host that doesn't follow the convention.</div>";
  h += "<label>X-Rig-Token</label><input name='rigToken' type='password' value='";
  h += cfg.rigToken;
  h += "'>";
  h += "<div class='small'>Tank volume (computed from a level channel) now lives on the "
       "<a href='/channels'>&#128208; Channels</a> page — check \"Compute Tank Volume\" on "
       "whichever channel is your level sensor.</div>";

  // --- WiFi (merged in from the old standalone /wifi page — was a
  // redundant duplicate of these same two fields, so it's gone now and
  // everything WiFi-related, including network scan, lives right here). ---
  h += "<h3>WiFi</h3>";
  if (apModeActive) {
    h += "<div class='card'>Currently broadcasting setup AP: <b>" + apSSID + "</b><br>"
         "Not connected to any site network yet.</div>";
  } else {
    h += "<div class='card'>Connected to: <b>" + cfg.wifiSSID + "</b><br>"
         "IP: " + WiFi.localIP().toString() + "  RSSI: " + String(WiFi.RSSI()) + " dBm</div>";
  }
  h += "<div class='row' style='margin-bottom:10px'>";
  h += "<button type='button' onclick='doScan()' id='scanBtn'>&#128269; Scan for Networks</button></div>";
  h += "<div id='scanResults'></div>";
  h += "<label>SSID</label><input name='wifiSSID' id='wifiSSID' value='";
  h += cfg.wifiSSID;
  h += "' placeholder='site wifi network name'>";
  h += "<label>Password</label><input name='wifiPass' id='wifiPass' type='password' value='";
  h += cfg.wifiPass;
  h += "' placeholder='site wifi password'>";
  h += "<div class='small'>Saving with a changed SSID/password reboots the unit to connect to the "
       "new network. If it can't connect within about a minute, it automatically falls back to "
       "its own setup AP so you can try again — no need to reflash or recompile anything. If no "
       "network is saved at all, this unit first auto-scans for a standard rig router (SSID "
       "starting with \"rig\" followed by numbers, e.g. rig132) using the shared rig password, "
       "before falling back to the setup AP.</div>";
  h += "<br><button type='submit'>Save</button>";
  h += "</form>"; // closes the single <form action='/api/config'> opened above
  h += "<div class='card' style='margin-top:12px'><b>Forget WiFi</b><br><span class='small'>Clears the "
       "saved network and reboots straight into setup-AP mode. Use this before moving the unit to a "
       "different site, or if it's stuck trying (and failing) to reconnect to a saved network.</span><br><br>";
  h += "<button class='btn-red' onclick=\"if(confirm('Forget saved WiFi and reboot into setup mode?'))"
       "fetch('/api/wifi/forget',{method:'POST'}).then(()=>alert('Forgotten. Rebooting...'))\">Forget WiFi</button></div>";
  h += R"JS(
<script>
function doScan(){
  let btn=document.getElementById('scanBtn');
  let box=document.getElementById('scanResults');
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
      s += '<div class="row" style="justify-content:space-between;padding:4px 0;border-bottom:1px solid #334;cursor:pointer" '+
           'data-ssid="'+idx+'" onclick="pickNet(scanNets['+idx+'])">'+
           '<span>'+lock+' '+n.ssid+'</span><span class="small">'+bars+' '+n.rssi+'dBm</span></div>';
    });
    s += '</div>';
    window.scanNets = nets.map(function(n){ return n.ssid; });
    box.innerHTML = s;
  }).catch(function(){
    btn.disabled=false; btn.innerHTML='&#128269; Scan for Networks';
    box.innerHTML='<p class="small">Scan failed. Try again.</p>';
  });
}
function pickNet(ssid){
  document.getElementById('wifiSSID').value = ssid;
  document.getElementById('wifiPass').value = '';
  document.getElementById('wifiPass').focus();
}
</script>)JS";
  h += "</div>";
  return h;
}

static String calPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128208; Channel Configuration</h2>";
  h += "<p class='small'>Each channel is fully independent — set its own name, kind (free text: level, pressure, temp, "
       "flow, rpm... anything), unit, and scaling. Live mA auto-refreshes every 2s. "
       "Set Zero/Max at known engineering values for the most accurate reading, or rely on the mA linear map below if not calibrated.</p>";
  // Detected analog-to-Modbus board (modbus.h modbusDetectBoard(), run once
  // at boot) — no manual selection needed, Waveshare and Eletechsup AMIDJ14
  // are both auto-identified via their Product ID register.
  h += "<p class='small'>Detected board: <b>" + String(boardProfile.name) + "</b> (" +
       String(boardProfile.numChannels) + " channels)</p>";

  // All 8 channel cards share ONE form now, so configuring several channels
  // at once (e.g. naming/enabling a whole board) is a single Save instead of
  // eight separate per-channel saves. "fromChannelsPage" tells handleConfig()
  // this POST covers every channel's enabled/volumeEnabled checkbox at once
  // (see handleConfig() for why that distinction matters). Set Zero/Set Max
  // stay as their own instant AJAX buttons (unchanged) — only the field save
  // moved to a page-level Save.
  h += "<form method='POST' action='/api/config'>";
  h += "<input type='hidden' name='fromChannelsPage' value='1'>";
  h += "<button type='submit' style='margin-bottom:14px'>&#128190; Save All Channels</button>";

  for (int i = 0; i < 8; i++) {
    bool exists = (i < boardProfile.numChannels);
    h += "<div class='card'";
    if (!exists) h += " style='opacity:.4'";
    h += "><b>Channel ";
    h += String(i+1);
    h += "</b>&nbsp;<span class='small' id='ma";
    h += String(i);
    h += "'>";
    if (!exists) h += "(not present on " + String(boardProfile.name) + ")";
    h += "</span>";
    h += "<label><input type='checkbox' name='ch";
    h += String(i);
    h += "en'";
    h += (cfg.ch[i].enabled ? " checked" : "");
    h += "> Enabled</label>";
    h += "<label>Name</label><input name='ch";
    h += String(i);
    h += "nm' value='";
    h += cfg.ch[i].name;
    h += "' placeholder='e.g. Suction Pressure'>";
    h += "<label>Kind (free text — anything: level, pressure, temp, flow, rpm...)</label><input name='ch";
    h += String(i);
    h += "kd' value='";
    h += cfg.ch[i].kind;
    h += "' placeholder='e.g. pressure'>";
    h += "<label>Unit</label><input name='ch";
    h += String(i);
    h += "ut' value='";
    h += cfg.ch[i].unit;
    h += "' placeholder='e.g. psi, m, degC, rpm'>";
    h += "<div class='row'><div><label>mA Min</label><input name='ch";
    h += String(i);
    h += "maLo' type='number' step='any' value='";
    h += String(cfg.ch[i].maMin);
    h += "'></div><div><label>mA Max</label><input name='ch";
    h += String(i);
    h += "maHi' type='number' step='any' value='";
    h += String(cfg.ch[i].maMax);
    h += "'></div></div>";
    h += "<div class='row'><div><label>Eng Min</label><input name='ch";
    h += String(i);
    h += "eLo' type='number' step='any' value='";
    h += String(cfg.ch[i].engMin);
    h += "'></div><div><label>Eng Max</label><input name='ch";
    h += String(i);
    h += "eHi' type='number' step='any' value='";
    h += String(cfg.ch[i].engMax);
    h += "'></div></div>";
    h += "<div class='small'>Cal: zeroRaw=";
    h += String(cfg.ch[i].zeroRaw);
    h += " maxRaw=";
    h += String(cfg.ch[i].maxRaw);
    h += " (if both set, overrides the mA Min/Max map above)</div>";

    // --- Tank Volume (optional, per-channel) --------------------------------
    // Checking this treats THIS channel's scaled value as a tank level and
    // computes a derived volume from it (spec-tank-modules.md §4b). Fields
    // only make sense once checked, so they're hidden until then.
    h += "<label style='margin-top:10px'><input type='checkbox' name='ch";
    h += String(i);
    h += "volEn' id='volEn";
    h += String(i);
    h += "' onchange='toggleVol(";
    h += String(i);
    h += ")'";
    h += (cfg.ch[i].volumeEnabled ? " checked" : "");
    h += "> Compute Tank Volume from this channel</label>";
    h += "<div id='volFields";
    h += String(i);
    h += "' style='display:";
    h += (cfg.ch[i].volumeEnabled ? "block" : "none");
    h += "'>";
    h += "<div class='small'>Straight-line map: this channel's value at \"Empty\" = 0 volume, "
         "value at \"Full\" = Capacity. Sent as <code>derived.volume</code> + top-level "
         "<code>capacity</code> for the rig dashboard's tank fill-bar card.</div>";
    h += "<div class='row'><div><label>Capacity</label><input name='ch";
    h += String(i);
    h += "cap' type='number' step='any' min='0' value='";
    h += String(cfg.ch[i].capacity);
    h += "'></div><div><label>Capacity Unit</label><select name='ch";
    h += String(i);
    h += "capUt'>";
    h += "<option value='m3'";
    if (cfg.ch[i].capacityUnit == "m3") h += " selected";
    h += ">m&#179; (cubic meters)</option>";
    h += "<option value='gal'";
    if (cfg.ch[i].capacityUnit == "gal") h += " selected";
    h += ">gal (US gallons)</option>";
    h += "</select></div></div>";
    h += "<div class='row'><div><label>Value @ Empty</label><input name='ch";
    h += String(i);
    h += "vZLvl' type='number' step='any' value='";
    h += String(cfg.ch[i].volZeroLevel);
    h += "'></div><div><label>Value @ Full</label><input name='ch";
    h += String(i);
    h += "vMLvl' type='number' step='any' value='";
    h += String(cfg.ch[i].volMaxLevel);
    h += "'></div></div>";
    h += "</div>";

    h += "<div class='row' style='margin-top:8px'>";
    h += "<button type='button' onclick='setZero(";
    h += String(i);
    h += ")'>Set Zero</button>&nbsp;";
    h += "<button type='button' onclick='setMax(";
    h += String(i);
    h += ")'>Set Max</button></div>";
    h += "</div>"; // closes .card (no per-channel </form> — one shared form wraps all 8 cards)
  }

  h += "<button type='submit' style='margin-top:6px'>&#128190; Save All Channels</button>";
  h += "</form>"; // closes the single form opened before the channel loop

  h += "<script>var _numRealChannels = " + String(boardProfile.numChannels) + ";</script>";
  h += R"(
<script>
function fetchRaw(){
  fetch('/api/channel-raw').then(r=>r.json()).then(d=>{
    d.channels.forEach(c=>{
      // Don't overwrite the not-present label for channels beyond what
      // the detected board actually has.
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

static String livePage() {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128202; Live Status</h2><div id='liveData'>Loading...</div>";
  h += R"(
<script>
function fetchLive(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    let s = '';
    // Volume is a column on the Channels table below (not a single card up
    // top) — a module can have MULTIPLE tanks, one per channel with
    // "Compute Tank Volume" checked (see /channels), and every one of them
    // needs to show up here, not just the first.
    s += '<h3>Channels</h3><table><tr><th>Ch</th><th>Name</th><th>Kind</th><th>mA</th><th>Value</th><th>Volume</th><th>Status</th></tr>';
    (d.channels||[]).forEach(c=>{
      let cls = c.status==='ok'?'ok':'open';
      s += '<tr><td>'+(c.ch+1)+'</td><td>'+c.name+'</td><td>'+(c.kind||'')+'</td><td>'+(c.ma!=null?c.ma.toFixed(2):'--')+'</td>';
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
      s += '<td class="'+cls+'">'+c.status+'</td></tr>';
    });
    s += '</table>';
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

static String sysPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128295; System</h2>";
  h += "<div class='card'><b>Firmware:</b> ";
  h += String(FW_VERSION);
  h += "<br><b>Detected Board:</b> ";
  h += String(boardProfile.name);
  h += " (" + String(boardProfile.numChannels) + " channels, auto-detected via Product ID register)";
  h += "<br><b>Module ID:</b> ";
  h += cfg.moduleId;
  h += "<br><b>MAC:</b> ";
  h += WiFi.macAddress();
  h += "<br><b>Chip:</b> ";
  h += String(ESP.getChipModel());
  h += " @ ";
  h += String(ESP.getCpuFreqMHz());
  h += "MHz</div>";
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
  DynamicJsonDocument doc(6144); // matches buildPayload()'s bumped size (multi-tank volume fields)
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
    // Use the auto-detected board's actual raw divisor (modbus.h) — this
    // used to hardcode Waveshare's /1000 (as raw/10/100), which showed a
    // wrong (10x too low) live mA reading on the Channels page for any
    // other board, e.g. the Eletechsup AMIDJ14 (/100).
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

static void handleConfig() {
  // Helper lambda-equivalent via local function
  auto applyParam = [&](const char* name, std::function<void(String)> fn){
    if (_srv->hasArg(name)) fn(_srv->arg(name));
  };

  applyParam("moduleName",     [](String v){ _cfg->moduleName    = v; });
  applyParam("moduleType",     [](String v){ _cfg->moduleType    = v.isEmpty() ? "generic" : v; });
  applyParam("description",    [](String v){ _cfg->description   = v; });
  applyParam("modbusSlaveId",  [](String v){ _cfg->modbusSlaveId = v.toInt(); });
  bool baudChanged = false;
  applyParam("modbusBaud",     [&](String v){ long nb = v.toInt(); if (nb != _cfg->modbusBaud) { _cfg->modbusBaud = nb; baudChanged = true; } });
  applyParam("pollIntervalS",  [](String v){ _cfg->pollIntervalS = constrain(v.toInt(),1,30); });
  applyParam("piHost",         [](String v){ _cfg->piHost        = v; });
  // Never save a blank token — this field is required for the Pi to accept
  // the module's posts at all (X-Rig-Token header), and an accidentally
  // cleared field (autofill, stray select-all+delete, etc.) would
  // otherwise silently break posting with no obvious symptom on this end.
  // Still fully editable to any OTHER value; only empty is rejected,
  // falling back to the shared default instead.
  applyParam("rigToken",       [](String v){ _cfg->rigToken = v.isEmpty() ? "7804991970" : v; });

  bool wifiChanged = false;
  String oldSSID = _cfg->wifiSSID;
  applyParam("wifiSSID", [&](String v){ if(v!=_cfg->wifiSSID){_cfg->wifiSSID=v;wifiChanged=true;} });
  applyParam("wifiPass", [](String v){ _cfg->wifiPass = v; });

  // /channels now submits all 8 channel cards through ONE shared form
  // ("fromChannelsPage=1" marks this), so every channel's enabled/
  // volumeEnabled checkbox is present in the same POST and can be applied
  // unconditionally via hasArg() — a genuinely unchecked box is correctly
  // absent either way. This replaced the old one-form-per-channel layout
  // (each card POSTed independently, tagged by a hidden "ch" field) where
  // only the single submitted channel's checkboxes could be touched
  // without silently unchecking every other channel; that per-channel
  // "ch" field/gate is no longer needed now that every channel arrives
  // together, but is still accepted harmlessly if some older client posts
  // it (fromChannelsPage absent, which >= 0 still gates to that one
  // channel, matching the previous behavior exactly).
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

  saveConfig(*_prefs, *_cfg);

  if (wifiChanged) {
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to connect to new WiFi...</p>");
    delay(1000);
    ESP.restart();
  } else if (baudChanged) {
    // Serial2 (RS485) is only configured once at boot via modbusInit() —
    // reboot so the poll task picks up the new baud cleanly instead of
    // trying to reinit HardwareSerial out from under a running task.
    _srv->send(200, "text/html; charset=utf-8", "<p>Saved. Rebooting to apply new RS485 baud rate...</p>");
    delay(1000);
    ESP.restart();
  } else {
    // Redirect back to whichever page the form actually came from, not
    // always the index — fromChannelsPage (all-channel save) or the
    // legacy "ch" hidden field (single-channel save) both mean this came
    // from /channels, so send the user back there instead of bouncing
    // them to / every time.
    _srv->sendHeader("Location", (allChannels || which >= 0) ? "/channels" : "/");
    _srv->send(302, "text/plain", "");
  }
}

// Scans for nearby WiFi networks and returns them for the /wifi page's
// "Scan for Networks" button. Works whether we're currently sitting in
// the setup AP (WIFI_AP_STA — STA radio idle but scannable) or already
// connected to a site network (WIFI_STA) — a passive scan while
// WL_CONNECTED does not drop the existing link (only scanning while
// mid-connect, i.e. between WiFi.begin() and WL_CONNECTED, would).
static void handleWifiScan() {
  int found = WiFi.scanNetworks();
  DynamicJsonDocument doc(2048);
  JsonArray arr = doc.createNestedArray("networks");
  if (found > 0) {
    // De-dupe by SSID (mesh/multi-AP networks show once per radio), keep strongest
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
        seenSsid[nSeen] = ssid;
        seenRssi[nSeen] = rssi;
        seenSecure[nSeen] = secure;
        nSeen++;
      }
    }
    // sort strongest first
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
      o["ssid"]   = seenSsid[i];
      o["rssi"]   = seenRssi[i];
      o["secure"] = seenSecure[i];
    }
  }
  WiFi.scanDelete();
  String out;
  serializeJson(doc, out);
  _srv->send(200, "application/json", out);
}

// NOTE: WiFi save used to have its own dedicated handler + form (from the
// now-removed standalone /wifi page, POSTing to /api/wifi). WiFi SSID/
// password fields moved onto the main Config page's single form, so
// saving them now goes through handleConfig() (/api/config) like every
// other setting — it already reboots on SSID/password change (see
// wifiChanged in handleConfig). No separate handler needed anymore.

// Clears saved WiFi creds and reboots straight into setup-AP mode —
// use when relocating a unit to a different site network.
static void handleWifiForget() {
  _cfg->wifiSSID = "";
  _cfg->wifiPass = "";
  saveConfig(*_prefs, *_cfg);
  _srv->send(200, "application/json", "{\"ok\":true}");
  delay(500);
  ESP.restart();
}

// Probes the bus at every standard baud rate to find the one the connected
// board actually speaks, and saves+reboots on success so the poll task
// picks it up cleanly (Serial2 is only fully reconfigured at boot).
// Holds modbusBusMutex for the whole scan so the running poll task can't
// interleave a read attempt on top of us mid-probe.
static void handleModbusAutoDetect() {
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    _srv->send(503, "application/json", "{\"ok\":false,\"error\":\"bus busy, try again\"}");
    return;
  }
  long found = modbusAutoDetectBaud(_cfg->modbusSlaveId, (uint32_t)_cfg->modbusBaud);
  xSemaphoreGive(modbusBusMutex);

  if (found > 0) {
    _cfg->modbusBaud = found;
    saveConfig(*_prefs, *_cfg);
    String r = "{\"ok\":true,\"detected\":true,\"baud\":";
    r += String(found);
    r += "}";
    _srv->send(200, "application/json", r);
    delay(500);
    ESP.restart(); // same as a manual baud change — Serial2 needs a clean re-init
  } else {
    _srv->send(200, "application/json", "{\"ok\":true,\"detected\":false}");
  }
}

static void handleCalZero() {
  int ch = _srv->hasArg("ch") ? _srv->arg("ch").toInt() : -1;
  if (ch < 0 || ch > 7) { _srv->send(400,"application/json","{\"ok\":false}"); return; }
  _cfg->ch[ch].zeroRaw = _raw[ch];
  saveConfig(*_prefs, *_cfg);
  String r = "{\"ok\":true,\"zeroRaw\":";
  r += String(_raw[ch]);
  r += "}";
  _srv->send(200, "application/json", r);
}

static void handleCalMax() {
  int ch = _srv->hasArg("ch") ? _srv->arg("ch").toInt() : -1;
  if (ch < 0 || ch > 7) { _srv->send(400,"application/json","{\"ok\":false}"); return; }
  _cfg->ch[ch].maxRaw = _raw[ch];
  saveConfig(*_prefs, *_cfg);
  String r = "{\"ok\":true,\"maxRaw\":";
  r += String(_raw[ch]);
  r += "}";
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
                    ChannelReading* readings, uint16_t* rawModbus,
                    SemaphoreHandle_t mtx) {
  _srv      = &srv;
  _cfg      = &cfg;
  _prefs    = &prefs;
  _readings = readings;
  _raw      = rawModbus;
  _mtx      = mtx;

  // Pages
  // charset=utf-8 explicitly, because the pages contain UTF-8 multi-byte
  // characters (em dashes, degree/section symbols) in string literals, and
  // without a charset browsers fall back to guessing (usually Latin-1),
  // which renders those bytes as garbled "â€”" mojibake instead of "—".
  //
  // Also send Cache-Control: no-store. The WebServer library sends no
  // cache headers of its own, and these pages are built fresh from live
  // config on every request (cheap on an ESP32 with no other traffic) —
  // there's no reason for a browser to ever cache them, and doing so
  // is exactly how a stale nav bar (e.g. an old "/calibration" link)
  // can keep showing up in a browser even after reflashing newer
  // firmware that changed the route.
  auto noCacheHtml = [](int code, const String& body){
    _srv->sendHeader("Cache-Control", "no-store");
    _srv->send(code, "text/html; charset=utf-8", body);
  };
  srv.on("/",            HTTP_GET,  [noCacheHtml](){ noCacheHtml(200, cfgPage(*_cfg)); });
  srv.on("/channels",    HTTP_GET,  [noCacheHtml](){ noCacheHtml(200, calPage(*_cfg)); });
  srv.on("/live",        HTTP_GET,  [noCacheHtml](){ noCacheHtml(200, livePage()); });
  srv.on("/system",      HTTP_GET,  [noCacheHtml](){ noCacheHtml(200, sysPage(*_cfg)); });
  // /wifi removed — WiFi settings now live on the main Config page ("/"),
  // no more redundant standalone page duplicating the same SSID/password
  // fields.

  // GET APIs
  srv.on("/api/status",      HTTP_GET,  handleApiStatus);
  srv.on("/api/channel-raw", HTTP_GET,  handleChannelRaw);
  srv.on("/api/wifi/scan",   HTTP_GET,  handleWifiScan);
  srv.on("/api/ota",         HTTP_GET,  handleOTA);

  // POST APIs
  srv.on("/api/config",        HTTP_POST, handleConfig);
  srv.on("/api/wifi/forget",   HTTP_POST, handleWifiForget);
  srv.on("/api/cal/zero",      HTTP_POST, handleCalZero);
  srv.on("/api/cal/max",       HTTP_POST, handleCalMax);
  srv.on("/api/modbus/autodetect", HTTP_POST, handleModbusAutoDetect);

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