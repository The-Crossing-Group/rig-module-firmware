// =============================================================================
// sensor-debug.ino — Standalone RS485/Modbus debugging tool
//
// Built after the SM7779 radar sensor got stuck outputting garbage
// following experimental register writes. Rather than keep iterating on
// the "real" firmware (waveshare-s3-sensors/) blind, this is a minimal,
// separate sketch focused entirely on raw visibility into what's
// actually happening on the RS485 bus — no config persistence, no
// per-sensor scaling/polling loop, no telemetry POST. Just: connect to
// WiFi, serve a web page with live traffic log + every debugging tool
// you'd want, and get out of the way.
//
// Does NOT touch or depend on waveshare-s3-sensors/ at all — separate
// sketch, separate folder, flash it to the SAME board (or a spare) when
// you need to dig into a sensor that's misbehaving. Flash the real
// firmware back when you're done.
//
// Hardware: Waveshare ESP32-S3-RS485-CAN
//   RS485: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21 (SP3485, HIGH=transmit)
//   (CAN not wired up in this tool — RS485/Modbus debugging only)
//
// Arduino IDE board settings: same as waveshare-s3-sensors/
//   Board: "ESP32S3 Dev Module", USB CDC On Boot: Enabled,
//   Flash Size: 16MB, PSRAM: OPI PSRAM,
//   Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
//
// WiFi: on first boot (or if it can't connect), starts a setup AP named
// "sensor-debug-setup" (password "debug1234") serving a captive-ish
// config page at 192.168.4.1 to enter your WiFi SSID/password. Saved to
// NVS (namespace "dbgwifi") so it reconnects automatically after that.
// =============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "modbus_debug.h"

#define RS485_TXD 17
#define RS485_RXD 18
#define RS485_DE  21

WebServer server(80);
Preferences wifiPrefs;

String wifiSSID = "";
String wifiPass = "";
bool apMode = false;

// -----------------------------------------------------------------------------
// WIFI SETUP (minimal — this tool doesn't need the full self-heal logic
// the production firmware has, just needs to reliably get online or fall
// back to a setup AP)
// -----------------------------------------------------------------------------
void loadWifiCreds() {
  wifiPrefs.begin("dbgwifi", true);
  wifiSSID = wifiPrefs.getString("ssid", "");
  wifiPass = wifiPrefs.getString("pass", "");
  wifiPrefs.end();
}

void saveWifiCreds(const String& ssid, const String& pass) {
  wifiPrefs.begin("dbgwifi", false);
  wifiPrefs.putString("ssid", ssid);
  wifiPrefs.putString("pass", pass);
  wifiPrefs.end();
}

void startSetupAP() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("sensor-debug-setup", "debug1234");
  Serial.println("[WiFi] Setup AP started: sensor-debug-setup / debug1234 @ 192.168.4.1");
}

bool tryConnectWifi(int timeoutMs = 10000) {
  if (wifiSSID.length() == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  unsigned long deadline = millis() + timeoutMs;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay250();
  }
  return WiFi.status() == WL_CONNECTED;
}

void delay250() { delay(250); Serial.print("."); }

// -----------------------------------------------------------------------------
// WEB UI — single page, everything inline. Deliberately plain (no theme
// switcher, no shared nav — this is a standalone tool, not part of the
// dashboard family).
// -----------------------------------------------------------------------------
String htmlHeader(const String& title) {
  String h = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + "</title><style>";
  h += "body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#eee;margin:0;padding:16px;max-width:900px;margin:0 auto}";
  h += "h1{font-size:20px}h3{border-bottom:2px solid #667eea;padding-bottom:4px;margin-top:24px}";
  h += ".card{background:#252540;border-radius:8px;padding:12px;margin:8px 0;box-shadow:0 2px 10px rgba(0,0,0,0.3)}";
  h += "input,select{background:#1a1a2e;color:#eee;border:1px solid #444;border-radius:4px;padding:6px;margin:2px}";
  h += "button{background:#667eea;color:#fff;border:none;border-radius:4px;padding:8px 14px;margin:4px 2px;cursor:pointer}";
  h += "button:hover{background:#5568d3}";
  h += "table{width:100%;border-collapse:collapse;font-size:13px}";
  h += "th,td{text-align:left;padding:4px 8px;border-bottom:1px solid #333}";
  h += ".mono{font-family:monospace;font-size:12px}";
  h += ".ok{color:#4ade80}.err{color:#f87171}.small{font-size:12px;color:#aaa}";
  h += "label{display:inline-block;min-width:110px;font-size:13px}";
  h += "</style></head><body>";
  h += "<h1>&#128295; RS485/Modbus Sensor Debug Tool</h1>";
  h += "<p class='small'>Standalone diagnostics — not connected to any rig telemetry. IP: " + WiFi.localIP().toString() + "</p>";
  return h;
}
String htmlFooter() { return "</body></html>"; }

void handleRoot() {
  String h = htmlHeader("Sensor Debug");

  SerialCfg sc = dbgGetSerialCfg();
  h += "<h3>Serial Config</h3><div class='card'>";
  h += "<div><label>Baud</label><select id='baud'>";
  for (uint32_t b : {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200}) {
    h += "<option value='" + String(b) + "'" + (b == sc.baud ? " selected" : "") + ">" + String(b) + "</option>";
  }
  h += "</select></div>";
  h += "<div><label>Parity</label><select id='parity'>";
  for (char p : {'N', 'E', 'O'}) {
    h += "<option value='" + String(p) + "'" + (p == sc.parity ? " selected" : "") + ">" + String(p) +
         (p == 'N' ? " (none)" : p == 'E' ? " (even)" : " (odd)") + "</option>";
  }
  h += "</select></div>";
  h += "<div><label>Stop bits</label><select id='stopbits'>";
  for (int s : {1, 2}) {
    h += "<option value='" + String(s) + "'" + (s == sc.stopBits ? " selected" : "") + ">" + String(s) + "</option>";
  }
  h += "</select></div>";
  h += "<button onclick='applySerial()'>Apply (live, not saved)</button>";
  h += "<p class='small'>Current: " + String(sc.baud) + " 8" + String(sc.parity) + String(sc.stopBits) +
       ". If a sensor went silent after register writes, try every parity/stopbit combo here at a few bauds — "
       "an accidental framing change (not just baud) is a real possibility with undocumented config registers.</p>";
  h += "<div id='serialResult' class='small'></div></div>";

  h += "<h3>Bus Scan</h3><div class='card'>";
  h += "<div><label>Max address</label><input id='scanMax' type='number' value='20' min='1' max='247' style='width:70px'></div>";
  h += "<button onclick='runScan()'>Scan Bus</button>";
  h += "<div id='scanResult' class='small' style='margin-top:8px'></div></div>";

  h += "<h3>Register Read (FC03/FC04)</h3><div class='card'>";
  h += "<div class='row'>";
  h += "<label>Slave ID</label><input id='rdSid' type='number' value='1' min='0' max='250' style='width:60px'> ";
  h += "<label>Func code</label><select id='rdFc'><option value='3'>03 (holding)</option><option value='4'>04 (input)</option></select> ";
  h += "<label>Register</label><input id='rdReg' value='0x0000' style='width:80px'> ";
  h += "<label>Count</label><input id='rdCount' type='number' value='1' min='1' max='16' style='width:50px'>";
  h += "</div><button onclick='runRead()'>Read</button>";
  h += "<p class='small'>Slave ID 0 or 250 = broadcast (some sensors, e.g. SM7779, use 250). Reply comes from the sensor's real address, shown below.</p>";
  h += "<div id='readResult' class='small' style='margin-top:8px'></div></div>";

  h += "<h3>Register Write (FC06)</h3><div class='card'>";
  h += "<p class='small'><b>Caution:</b> writes directly to sensor flash/registers.</p>";
  h += "<label>Slave ID</label><input id='wrSid' type='number' value='1' min='0' max='250' style='width:60px'> ";
  h += "<label>Register</label><input id='wrReg' value='0x0000' style='width:80px'> ";
  h += "<label>Value</label><input id='wrVal' value='0' style='width:80px'>";
  h += "<br><button onclick='runWrite()'>&#9888; Write</button>";
  h += "<div id='writeResult' class='small' style='margin-top:8px'></div></div>";

  h += "<h3>Raw Hex Send</h3><div class='card'>";
  h += "<p class='small'>No Modbus framing assumed — sends exactly these bytes, DE toggled around TX. Space or comma separated hex, e.g. <code>01 03 00 00 00 01 84 0A</code></p>";
  h += "<input id='rawHex' style='width:70%' placeholder='01 03 00 00 00 01 84 0A'>";
  h += "<button onclick='runRaw()'>Send</button>";
  h += "<div id='rawResult' class='small' style='margin-top:8px'></div></div>";

  h += "<h3>Live Traffic Log</h3><div class='card'>";
  h += "<label><input type='checkbox' id='logPause'> Pause</label>";
  h += "<div style='max-height:400px;overflow-y:auto;margin-top:8px'><table class='mono'><thead><tr>";
  h += "<th>Age (s)</th><th>Type</th><th>TX</th><th>RX</th><th>Result</th></tr></thead><tbody id='logBody'></tbody></table></div></div>";

  h += R"(
<script>
function j(url, opts){ return fetch(url, opts).then(r=>r.json()); }
function applySerial(){
  let b=document.getElementById('baud').value, p=document.getElementById('parity').value, s=document.getElementById('stopbits').value;
  let box=document.getElementById('serialResult'); box.textContent='Applying...';
  fetch('/api/serial?baud='+b+'&parity='+p+'&stopbits='+s,{method:'POST'}).then(r=>r.json()).then(d=>{
    box.innerHTML = '<span class="ok">Applied</span> — now ' + d.baud + ' 8' + d.parity + d.stopBits;
  }).catch(e=>{ box.textContent='Failed: '+e; });
}
function runScan(){
  let max=document.getElementById('scanMax').value;
  let box=document.getElementById('scanResult'); box.textContent='Scanning 1-'+max+'...';
  j('/api/scan?max='+max).then(d=>{
    if(d.found.length===0){ box.textContent='No slaves responded.'; return; }
    box.innerHTML = 'Found: <b>' + d.found.map(f=>f.addr+' (FC'+f.fc+')').join(', ') + '</b>';
  }).catch(e=>{ box.textContent='Failed: '+e; });
}
function runRead(){
  let sid=document.getElementById('rdSid').value, fc=document.getElementById('rdFc').value;
  let reg=document.getElementById('rdReg').value, cnt=document.getElementById('rdCount').value;
  let box=document.getElementById('readResult'); box.textContent='Reading...';
  j('/api/read?slaveId='+sid+'&fc='+fc+'&reg='+reg+'&count='+cnt).then(d=>{
    if(d.ok){
      let regs = d.regs.map((v,i)=>'#'+i+'='+v+' (0x'+v.toString(16).toUpperCase().padStart(4,'0')+')').join(', ');
      box.innerHTML = '<span class="ok">OK</span> from slave ' + d.actualSlaveId + ': ' + regs +
        '<br>TX: ' + d.tx + '<br>RX: ' + d.rx;
    } else {
      box.innerHTML = '<span class="err">FAILED</span> — ' + d.error + '<br>TX: ' + d.tx + (d.rx?'<br>RX: '+d.rx:'');
    }
  }).catch(e=>{ box.textContent='Request failed: '+e; });
}
function runWrite(){
  let sid=document.getElementById('wrSid').value, reg=document.getElementById('wrReg').value, val=document.getElementById('wrVal').value;
  if(!confirm('Write '+val+' to register '+reg+' on slave '+sid+'?')) return;
  let box=document.getElementById('writeResult'); box.textContent='Writing...';
  fetch('/api/write?slaveId='+sid+'&reg='+reg+'&value='+val,{method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.ok) box.innerHTML = '<span class="ok">OK</span> — ' + (d.error||'confirmed') + '<br>TX: '+d.tx+'<br>RX: '+d.rx;
    else box.innerHTML = '<span class="err">FAILED</span> — ' + d.error + '<br>TX: '+d.tx+(d.rx?'<br>RX: '+d.rx:'');
  }).catch(e=>{ box.textContent='Request failed: '+e; });
}
function runRaw(){
  let hex=document.getElementById('rawHex').value;
  let box=document.getElementById('rawResult'); box.textContent='Sending...';
  fetch('/api/raw?hex='+encodeURIComponent(hex),{method:'POST'}).then(r=>r.json()).then(d=>{
    box.innerHTML = 'RX (' + (d.rxLen||0) + ' bytes): <b>' + (d.rx||'(nothing)') + '</b>';
  }).catch(e=>{ box.textContent='Request failed: '+e; });
}
function fetchLog(){
  if(document.getElementById('logPause').checked) return;
  j('/api/log').then(d=>{
    let s='';
    d.forEach(e=>{
      s+='<tr><td>'+(e.ageMs/1000).toFixed(1)+'</td><td>'+e.label+'</td><td class="mono">'+e.tx+'</td>'+
         '<td class="mono">'+e.rx+'</td><td>'+e.result+'</td></tr>';
    });
    document.getElementById('logBody').innerHTML = s || '<tr><td colspan=5>No traffic yet.</td></tr>';
  });
}
setInterval(fetchLog, 1500); fetchLog();
</script>
)";
  h += htmlFooter();
  server.send(200, "text/html", h);
}

void handleWifiSetupPage() {
  String h = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Sensor Debug Setup</title><style>body{font-family:system-ui,sans-serif;padding:20px;max-width:400px;margin:0 auto}";
  h += "input{width:100%;padding:8px;margin:4px 0;box-sizing:border-box}button{width:100%;padding:10px;margin-top:10px}</style></head><body>";
  h += "<h2>Sensor Debug Tool — WiFi Setup</h2>";
  h += "<form action='/wifi-save' method='POST'>";
  h += "<label>SSID</label><input name='ssid' required>";
  h += "<label>Password</label><input name='pass' type='password'>";
  h += "<button type='submit'>Save & Reboot</button>";
  h += "</form></body></html>";
  server.send(200, "text/html", h);
}

void handleWifiSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  saveWifiCreds(ssid, pass);
  server.send(200, "text/html", "<html><body><h2>Saved. Rebooting...</h2></body></html>");
  delay(500);
  ESP.restart();
}

// --- API handlers ---
static uint16_t parseHexOrDec(const String& s) { return (uint16_t)strtol(s.c_str(), nullptr, 0); }

void handleApiSerial() {
  uint32_t baud = server.arg("baud").toInt();
  char parity = server.arg("parity").length() ? server.arg("parity")[0] : 'N';
  int stopBits = server.arg("stopbits").toInt();
  if (stopBits != 2) stopBits = 1;
  if (baud == 0) baud = 9600;
  dbgSerialApply(baud, parity, stopBits);
  String out = "{\"baud\":" + String(baud) + ",\"parity\":\"" + String(parity) + "\",\"stopBits\":" + String(stopBits) + "}";
  server.send(200, "application/json", out);
}

void handleApiScan() {
  int max = server.arg("max").toInt();
  if (max < 1) max = 20;
  if (max > 247) max = 247;
  String hits = dbgScanBus(max);
  server.send(200, "application/json", "{\"found\":" + hits + "}");
}

void handleApiRead() {
  uint8_t sid = (uint8_t)server.arg("slaveId").toInt();
  uint8_t fc = (uint8_t)server.arg("fc").toInt();
  uint16_t reg = parseHexOrDec(server.arg("reg"));
  uint8_t count = (uint8_t)server.arg("count").toInt();
  if (count < 1) count = 1;
  ModbusResult r = dbgReadRegs(sid, fc, reg, count);
  String out = "{\"ok\":" + String(r.ok ? "true" : "false");
  out += ",\"actualSlaveId\":" + String(r.actualSlaveId);
  out += ",\"tx\":\"" + r.txHex + "\",\"rx\":\"" + r.rxHex + "\"";
  if (r.ok) {
    out += ",\"regs\":[";
    for (int i = 0; i < r.regCount; i++) { if (i) out += ","; out += String(r.regs[i]); }
    out += "]";
  } else {
    out += ",\"error\":\"" + r.error + "\"";
  }
  out += "}";
  server.send(200, "application/json", out);
}

void handleApiWrite() {
  uint8_t sid = (uint8_t)server.arg("slaveId").toInt();
  uint16_t reg = parseHexOrDec(server.arg("reg"));
  uint16_t val = parseHexOrDec(server.arg("value"));
  ModbusResult r = dbgWriteReg(sid, reg, val);
  String out = "{\"ok\":" + String(r.ok ? "true" : "false");
  out += ",\"tx\":\"" + r.txHex + "\",\"rx\":\"" + r.rxHex + "\"";
  out += ",\"error\":\"" + r.error + "\"}";
  server.send(200, "application/json", out);
}

void handleApiRaw() {
  String hex = server.arg("hex");
  String rx = dbgRawHexSend(hex, 800);
  int rxLen = rx.length() ? (rx.length() + 1) / 3 : 0;
  server.send(200, "application/json", "{\"rx\":\"" + rx + "\",\"rxLen\":" + String(rxLen) + "}");
}

void handleApiLog() {
  server.send(200, "application/json", dbgGetLogJson(40));
}

// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[SensorDebug] Booting...");

  WiFi.persistent(false); // same NVS-corruption defense as the production firmware

  dbgSerialInit(RS485_RXD, RS485_TXD, RS485_DE, 9600, 'N', 1);

  loadWifiCreds();
  bool connected = tryConnectWifi();
  if (!connected) {
    Serial.println("\n[WiFi] Could not connect — starting setup AP");
    startSetupAP();
  } else {
    Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
  }

  if (apMode) {
    server.on("/", handleWifiSetupPage);
    server.on("/wifi-save", HTTP_POST, handleWifiSave);
  } else {
    server.on("/", handleRoot);
    server.on("/api/serial", HTTP_POST, handleApiSerial);
    server.on("/api/scan", HTTP_GET, handleApiScan);
    server.on("/api/read", HTTP_GET, handleApiRead);
    server.on("/api/write", HTTP_POST, handleApiWrite);
    server.on("/api/raw", HTTP_POST, handleApiRaw);
    server.on("/api/log", HTTP_GET, handleApiLog);
    // Also allow reconfiguring WiFi from the main tool if needed
    server.on("/wifi", handleWifiSetupPage);
    server.on("/wifi-save", HTTP_POST, handleWifiSave);
  }
  server.begin();
  if (apMode) {
    Serial.println("[Web] Server started (setup mode, 192.168.4.1)");
  } else {
    Serial.println("[Web] Server started at http://" + WiFi.localIP().toString() + "/");
  }
}

void loop() {
  server.handleClient();
}
