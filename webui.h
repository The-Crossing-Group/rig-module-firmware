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
  <a href='/calibration'>&#128208; Channels</a>
  <a href='/live'>&#128202; Live</a>
  <a href='/system'>&#128295; System</a>
  <a href='/wifi'>&#128246; WiFi</a>
</div>
)";

// AP-mode warning banner — prepended to pages when broadcasting the setup AP
static String apBanner() {
  if (!apModeActive) return "";
  return "<div class='ap-banner'>No WiFi configured yet — connect a device to this AP and "
         "<a href='/wifi'>set your network here</a>.</div>";
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
  h += "<label>Description</label><input name='description' value='";
  h += cfg.description;
  h += "'>";
  h += "<h3>Modbus / Pi</h3>";
  h += "<label>Modbus Slave ID</label><input name='modbusSlaveId' type='number' min='1' max='247' value='";
  h += String(cfg.modbusSlaveId);
  h += "'>";
  h += "<label>Poll Interval (1-30 s)</label><input name='pollIntervalS' type='number' min='1' max='30' value='";
  h += String(cfg.pollIntervalS);
  h += "'>";
  h += "<label>Pi Host (blank = mDNS auto)</label><input name='piHost' value='";
  h += cfg.piHost;
  h += "' placeholder='192.168.x.x or rig-logger.local'>";
  h += "<label>X-Rig-Token</label><input name='rigToken' type='password' value='";
  h += cfg.rigToken;
  h += "'>";
  h += "<h3>WiFi</h3>";
  h += "<label>SSID</label><input name='wifiSSID' value='";
  h += cfg.wifiSSID;
  h += "'>";
  h += "<label>Password</label><input name='wifiPass' type='password' value='";
  h += cfg.wifiPass;
  h += "'>";
  h += "<br><button type='submit'>Save</button>";
  h += "</form>";
  h += "</div>";
  return h;
}

static String calPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += "<div class='page'><h2>&#128208; Channel Configuration</h2>";
  h += "<p class='small'>Each channel is fully independent — set its own name, kind (free text: level, pressure, temp, "
       "flow, rpm... anything), unit, and scaling. Live mA auto-refreshes every 2s. "
       "Set Zero/Max at known engineering values for the most accurate reading, or rely on the mA linear map below if not calibrated.</p>";

  for (int i = 0; i < 8; i++) {
    h += "<div class='card'><b>Channel ";
    h += String(i+1);
    h += "</b>&nbsp;<span class='small' id='ma";
    h += String(i);
    h += "'></span>";
    h += "<form method='POST' action='/api/config'>";
    h += "<input type='hidden' name='ch' value='";
    h += String(i);
    h += "'>";
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
    h += "<div class='row' style='margin-top:8px'>";
    h += "<button type='button' onclick='setZero(";
    h += String(i);
    h += ")'>Set Zero</button>&nbsp;";
    h += "<button type='button' onclick='setMax(";
    h += String(i);
    h += ")'>Set Max</button>&nbsp;";
    h += "<button type='submit'>Save</button></div>";
    h += "</form></div>";
  }

  h += R"(
<script>
function fetchRaw(){
  fetch('/api/channel-raw').then(r=>r.json()).then(d=>{
    d.channels.forEach(c=>{
      let el=document.getElementById('ma'+c.ch);
      if(el) el.textContent = c.ma.toFixed(3)+' mA (raw '+c.raw+')';
    });
  });
}
setInterval(fetchRaw,2000); fetchRaw();
function setZero(ch){ fetch('/api/cal/zero?ch='+ch,{method:'POST'}).then(()=>fetchRaw()); }
function setMax(ch){  fetch('/api/cal/max?ch='+ch, {method:'POST'}).then(()=>fetchRaw()); }
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
    let s = '<h3>Channels</h3><table><tr><th>Ch</th><th>Name</th><th>Kind</th><th>mA</th><th>Value</th><th>Status</th></tr>';
    (d.channels||[]).forEach(c=>{
      let cls = c.status==='ok'?'ok':'open';
      s += '<tr><td>'+(c.ch+1)+'</td><td>'+c.name+'</td><td>'+(c.kind||'')+'</td><td>'+(c.ma!=null?c.ma.toFixed(2):'--')+'</td>';
      s += '<td>'+(c.value!=null?c.value+' '+c.unit:'--')+'</td>';
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

static String wifiPage(ModuleConfig& cfg) {
  String h = FPSTR(NAV);
  h += apBanner();
  h += "<div class='page'><h2>&#128246; WiFi Setup</h2>";
  if (apModeActive) {
    h += "<div class='card'>Currently broadcasting setup AP: <b>" + apSSID + "</b><br>";
    h += "Not connected to any site network yet.</div>";
  } else {
    h += "<div class='card'>Connected to: <b>" + cfg.wifiSSID + "</b><br>";
    h += "IP: " + WiFi.localIP().toString() + "  RSSI: " + String(WiFi.RSSI()) + " dBm</div>";
  }
  h += "<div class='row' style='margin-bottom:10px'>";
  h += "<button type='button' onclick='doScan()' id='scanBtn'>&#128269; Scan for Networks</button></div>";
  h += "<div id='scanResults'></div>";
  h += "<form method='POST' action='/api/wifi'>";
  h += "<label>Network Name (SSID)</label><input name='wifiSSID' id='wifiSSID' value='";
  h += cfg.wifiSSID;
  h += "' placeholder='site wifi network name' required>";
  h += "<label>Password</label><input name='wifiPass' id='wifiPass' type='password' value='";
  h += cfg.wifiPass;
  h += "' placeholder='site wifi password'>";
  h += "<button type='submit'>Save &amp; Connect</button>";
  h += "</form>";
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
  h += "<p class='small'>Saving reboots the unit and attempts to join this network. "
       "If it can't connect within about a minute, it automatically falls back to "
       "this setup AP so you can try again — no need to reflash or recompile anything.</p>";
  h += "<p class='small'>Note: if no network is saved at all, this unit first auto-scans "
       "for a standard rig router (SSID starting with \"rig\" followed by numbers, e.g. "
       "rig132) using the shared rig password, before falling back to this setup AP. "
       "You only need this page manually for non-standard networks.</p>";
  if (!apModeActive) {
    h += "<div class='card'><b>Forget WiFi</b><br><span class='small'>Clears the saved network and reboots "
         "straight into setup-AP mode. Use this before moving the unit to a different site.</span><br><br>";
    h += "<button class='btn-red' onclick=\"if(confirm('Forget saved WiFi and reboot into setup mode?'))"
         "fetch('/api/wifi/forget',{method:'POST'}).then(()=>alert('Forgotten. Rebooting...'))\">Forget WiFi</button></div>";
  }
  h += "</div>";
  return h;
}

// ─── API handler helpers ──────────────────────────────────────────────────────
static void handleApiStatus() {
  DynamicJsonDocument doc(4096);
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
      o["ma"]  = round(_raw[i] / 10.0f) / 100.0f;
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
  applyParam("description",    [](String v){ _cfg->description   = v; });
  applyParam("modbusSlaveId",  [](String v){ _cfg->modbusSlaveId = v.toInt(); });
  applyParam("pollIntervalS",  [](String v){ _cfg->pollIntervalS = constrain(v.toInt(),1,30); });
  applyParam("piHost",         [](String v){ _cfg->piHost        = v; });
  applyParam("rigToken",       [](String v){ _cfg->rigToken      = v; });

  bool wifiChanged = false;
  String oldSSID = _cfg->wifiSSID;
  applyParam("wifiSSID", [&](String v){ if(v!=_cfg->wifiSSID){_cfg->wifiSSID=v;wifiChanged=true;} });
  applyParam("wifiPass", [](String v){ _cfg->wifiPass = v; });

  for (int i = 0; i < 8; i++) {
    String pre = "ch" + String(i);
    _cfg->ch[i].enabled = _srv->hasArg((pre+"en").c_str());
    applyParam((pre+"nm").c_str(),   [i](String v){ _cfg->ch[i].name  = v; });
    applyParam((pre+"kd").c_str(),   [i](String v){ _cfg->ch[i].kind  = v; });
    applyParam((pre+"ut").c_str(),   [i](String v){ _cfg->ch[i].unit  = v; });
    applyParam((pre+"maLo").c_str(), [i](String v){ _cfg->ch[i].maMin = v.toFloat(); });
    applyParam((pre+"maHi").c_str(), [i](String v){ _cfg->ch[i].maMax = v.toFloat(); });
    applyParam((pre+"eLo").c_str(),  [i](String v){ _cfg->ch[i].engMin = v.toFloat(); });
    applyParam((pre+"eHi").c_str(),  [i](String v){ _cfg->ch[i].engMax = v.toFloat(); });
  }

  saveConfig(*_prefs, *_cfg);

  if (wifiChanged) {
    _srv->send(200, "text/html", "<p>Saved. Rebooting to connect to new WiFi...</p>");
    delay(1000);
    ESP.restart();
  } else {
    _srv->sendHeader("Location", "/");
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

// Dedicated WiFi setup handler (from /wifi page). Always reboots on save
// so the device re-runs connectWifi() with the fresh credentials — this
// is what lets a unit move between site networks with zero recompiling.
static void handleWifiSave() {
  if (!_srv->hasArg("wifiSSID") || _srv->arg("wifiSSID").isEmpty()) {
    _srv->send(400, "text/html", "<p>SSID is required. <a href='/wifi'>Go back</a></p>");
    return;
  }
  _cfg->wifiSSID = _srv->arg("wifiSSID");
  _cfg->wifiPass = _srv->hasArg("wifiPass") ? _srv->arg("wifiPass") : "";
  saveConfig(*_prefs, *_cfg);

  _srv->send(200, "text/html",
    "<p>Saved. Rebooting and attempting to join \"" + _cfg->wifiSSID + "\"...<br>"
    "If it can't connect, this unit will fall back to its setup AP automatically.</p>");
  delay(1000);
  ESP.restart();
}

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
  srv.on("/",            HTTP_GET,  [](){ _srv->send(200,"text/html",cfgPage(*_cfg)); });
  srv.on("/calibration", HTTP_GET,  [](){ _srv->send(200,"text/html",calPage(*_cfg)); });
  srv.on("/live",        HTTP_GET,  [](){ _srv->send(200,"text/html",livePage()); });
  srv.on("/system",      HTTP_GET,  [](){ _srv->send(200,"text/html",sysPage(*_cfg)); });
  srv.on("/wifi",        HTTP_GET,  [](){ _srv->send(200,"text/html",wifiPage(*_cfg)); });

  // GET APIs
  srv.on("/api/status",      HTTP_GET,  handleApiStatus);
  srv.on("/api/channel-raw", HTTP_GET,  handleChannelRaw);
  srv.on("/api/wifi/scan",   HTTP_GET,  handleWifiScan);
  srv.on("/api/ota",         HTTP_GET,  handleOTA);

  // POST APIs
  srv.on("/api/config",        HTTP_POST, handleConfig);
  srv.on("/api/wifi",          HTTP_POST, handleWifiSave);
  srv.on("/api/wifi/forget",   HTTP_POST, handleWifiForget);
  srv.on("/api/cal/zero",      HTTP_POST, handleCalZero);
  srv.on("/api/cal/max",       HTTP_POST, handleCalMax);

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