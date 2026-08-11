// =============================================================================
// sensor-debug.ino — Standalone RS485/Modbus debugging tool
//
// No setup page, no login screen, no captive portal. It connects straight
// to the bench-test hotspot (rig-test-ap) with hardcoded credentials and
// serves a plain, no-auth web page at its IP — just open the IP in a
// browser and click/type. Everything is also available over USB Serial
// (115200 baud) if WiFi isn't available.
//
// Built after the SM7779 radar sensor got stuck outputting garbage
// following experimental register writes. Separate sketch, doesn't
// touch or depend on the "real" firmware (waveshare-s3-sensors/) at all
// — flash this to the same board (or a spare) when you need to dig into
// a misbehaving sensor, flash the real firmware back when done.
//
// Hardware: Waveshare ESP32-S3-RS485-CAN
//   RS485: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21 (SP3485, HIGH=transmit)
//
// Arduino IDE board settings: same as waveshare-s3-sensors/
//   Board: "ESP32S3 Dev Module", USB CDC On Boot: Enabled,
//   Flash Size: 16MB, PSRAM: OPI PSRAM,
//   Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
//
// WiFi: connects to WIFI_SSID/WIFI_PASS below (defaults to the bench
// rig-test-ap hotspot on drill-pi-1, 10.42.0.x). To point it at a
// different network, edit the two #defines and re-flash — no runtime
// config page, no login screen. Once connected, watch Serial at boot for
// the IP, or check the Pi's DHCP leases (10.42.0.x) — then just open
// that IP in a browser.
//
// USB SERIAL COMMANDS (open Serial Monitor, 115200 baud, line ending
// "Newline") — same functionality as the web page, useful if WiFi is
// down or you don't know the IP yet:
//
//   help                          — show this list
//   baud <n>                      — set RS485 baud (e.g. baud 9600)
//   parity <n|e|o>                — set parity: none/even/odd
//   stop <1|2>                    — set stop bits
//   status                        — show current serial config
//   scan [maxAddr]                — scan slave addresses 1..maxAddr (default 20)
//   read <slaveId> <fc> <reg> [count]
//                                 — FC03/FC04 read, e.g.: read 1 3 0x0000 2
//   write <slaveId> <reg> <value> — FC06 write single register (asks to confirm)
//   write! <slaveId> <reg> <value>— FC06 write, no confirmation
//   raw <hex bytes>               — send exact bytes, show exact reply
//                                    e.g.: raw 01 03 00 00 00 01 84 0A
//   log [n]                       — show last n traffic log entries (default 15)
//   sniff <seconds>               — listen passively, print any unsolicited
//                                    bytes seen on the bus (no TX at all)
// =============================================================================

#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include "modbus_debug.h"

#define RS485_TXD 17
#define RS485_RXD 18
#define RS485_DE  21

// Bench-test hotspot (drill-pi-1 / rig128), 10.42.0.1 gateway.
// Edit + re-flash to point this at a different network — no config page.
#define WIFI_SSID "rig-test-ap"
#define WIFI_PASS "7804991970"

WebServer server(80);
String inputLine = "";

// =============================================================================
// SERIAL (USB) COMMAND INTERFACE
// =============================================================================

void printHelp() {
  Serial.println(F(
    "\n--- sensor-debug commands ---\n"
    "  help                          show this list\n"
    "  baud <n>                      set RS485 baud, e.g. baud 9600\n"
    "  parity <n|e|o>                set parity: none/even/odd\n"
    "  stop <1|2>                    set stop bits\n"
    "  status                        show current serial config\n"
    "  scan [maxAddr]                scan slave addresses 1..maxAddr (default 20)\n"
    "  read <slaveId> <fc> <reg> [count]\n"
    "                                FC03/FC04 read, e.g.: read 1 3 0x0000 2\n"
    "  write <slaveId> <reg> <value> FC06 write (asks to confirm with 'y')\n"
    "  write! <slaveId> <reg> <value> FC06 write, no confirmation\n"
    "  raw <hex bytes>               send exact bytes, e.g.: raw 01 03 00 00 00 01 84 0A\n"
    "  log [n]                       show last n traffic log entries (default 15)\n"
    "  sniff <seconds>               listen passively for unsolicited bus traffic\n"
  ));
}

std::vector<String> splitArgs(const String& s) {
  std::vector<String> out;
  int i = 0, len = s.length();
  while (i < len) {
    while (i < len && s[i] == ' ') i++;
    int start = i;
    while (i < len && s[i] != ' ') i++;
    if (i > start) out.push_back(s.substring(start, i));
  }
  return out;
}

uint32_t parseNum(const String& s) { return strtoul(s.c_str(), nullptr, 0); }

void printRegs(ModbusResult& r) {
  for (int i = 0; i < r.regCount; i++) {
    uint16_t v = r.regs[i];
    Serial.printf("  #%d = %u (0x%04X)\n", i, v, v);
  }
}

void doScan(std::vector<String>& args) {
  int maxAddr = args.size() > 1 ? parseNum(args[1]) : 20;
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;
  Serial.printf("Scanning slave addresses 1-%d at current serial config...\n", maxAddr);
  int found = 0;
  for (int addr = 1; addr <= maxAddr; addr++) {
    ModbusResult r4 = dbgReadRegs((uint8_t)addr, 4, 0x0000, 1, 300);
    if (r4.ok) { Serial.printf("  addr %d responded to FC04\n", addr); found++; continue; }
    ModbusResult r3 = dbgReadRegs((uint8_t)addr, 3, 0x0000, 1, 300);
    if (r3.ok) { Serial.printf("  addr %d responded to FC03\n", addr); found++; }
  }
  Serial.printf("Scan done. %d address(es) responded.\n", found);
}

void doRead(std::vector<String>& args) {
  if (args.size() < 4) { Serial.println("Usage: read <slaveId> <fc> <reg> [count]"); return; }
  uint8_t sid = (uint8_t)parseNum(args[1]);
  uint8_t fc = (uint8_t)parseNum(args[2]);
  uint16_t reg = (uint16_t)parseNum(args[3]);
  uint8_t count = args.size() > 4 ? (uint8_t)parseNum(args[4]) : 1;
  Serial.printf("Reading slave %d, FC%02d, reg 0x%04X, count %d...\n", sid, fc, reg, count);
  ModbusResult r = dbgReadRegs(sid, fc, reg, count);
  if (r.ok) {
    Serial.printf("OK — reply from slave %d:\n", r.actualSlaveId);
    printRegs(r);
  } else {
    Serial.println("FAILED — " + r.error);
  }
  Serial.println("  TX: " + r.txHex);
  if (r.rxHex.length()) Serial.println("  RX: " + r.rxHex);
}

void doWriteReg(std::vector<String>& args, bool skipConfirm) {
  if (args.size() < 4) { Serial.println("Usage: write <slaveId> <reg> <value>"); return; }
  uint8_t sid = (uint8_t)parseNum(args[1]);
  uint16_t reg = (uint16_t)parseNum(args[2]);
  uint16_t val = (uint16_t)parseNum(args[3]);

  if (!skipConfirm) {
    Serial.printf("About to write reg 0x%04X = %u (0x%04X) on slave %d.\n", reg, val, val, sid);
    Serial.println("Type 'y' + Enter within 10s to confirm, anything else cancels.");
    unsigned long deadline = millis() + 10000;
    String confirm = "";
    while (millis() < deadline) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') break;
        if (c != '\r') confirm += c;
      }
    }
    if (confirm != "y") { Serial.println("Cancelled."); return; }
  }

  ModbusResult r = dbgWriteReg(sid, reg, val);
  if (r.ok) Serial.println("OK — " + (r.error.length() ? r.error : String("confirmed")));
  else Serial.println("FAILED — " + r.error);
  Serial.println("  TX: " + r.txHex);
  if (r.rxHex.length()) Serial.println("  RX: " + r.rxHex);
}

void doRaw(const String& fullLine) {
  int sp = fullLine.indexOf(' ');
  if (sp < 0) { Serial.println("Usage: raw <hex bytes>, e.g. raw 01 03 00 00 00 01 84 0A"); return; }
  String hex = fullLine.substring(sp + 1);
  Serial.println("Sending raw: " + hex);
  String rx = dbgRawHexSend(hex, 800);
  if (rx.length()) Serial.println("Reply: " + rx);
  else Serial.println("No reply.");
}

void doLog(std::vector<String>& args) {
  int n = args.size() > 1 ? parseNum(args[1]) : 15;
  Serial.println("--- last " + String(n) + " log entries (newest first) ---");
  Serial.print(dbgLogHumanText(n));
}

void doStatus() {
  SerialCfg sc = dbgGetSerialCfg();
  Serial.printf("Current RS485 config: %u baud, 8%c%d\n", sc.baud, sc.parity, sc.stopBits);
}

void doBaud(std::vector<String>& args) {
  if (args.size() < 2) { Serial.println("Usage: baud <n>"); return; }
  uint32_t b = parseNum(args[1]);
  SerialCfg sc = dbgGetSerialCfg();
  dbgSerialApply(b, sc.parity, sc.stopBits);
  doStatus();
}

void doParity(std::vector<String>& args) {
  if (args.size() < 2) { Serial.println("Usage: parity <n|e|o>"); return; }
  char p = toupper(args[1][0]);
  if (p != 'N' && p != 'E' && p != 'O') { Serial.println("Parity must be n, e, or o"); return; }
  SerialCfg sc = dbgGetSerialCfg();
  dbgSerialApply(sc.baud, p, sc.stopBits);
  doStatus();
}

void doStop(std::vector<String>& args) {
  if (args.size() < 2) { Serial.println("Usage: stop <1|2>"); return; }
  int s = parseNum(args[1]);
  if (s != 1 && s != 2) { Serial.println("Stop bits must be 1 or 2"); return; }
  SerialCfg sc = dbgGetSerialCfg();
  dbgSerialApply(sc.baud, sc.parity, s);
  doStatus();
}

void doSniff(std::vector<String>& args) {
  int secs = args.size() > 1 ? parseNum(args[1]) : 5;
  Serial.printf("Sniffing bus passively for %d seconds (no TX)...\n", secs);
  unsigned long deadline = millis() + (unsigned long)secs * 1000;
  uint8_t buf[64];
  int n = 0;
  unsigned long lastByte = 0;
  while (millis() < deadline) {
    if (dbgSerialAvailable()) {
      if (n < (int)sizeof(buf)) buf[n++] = dbgSerialReadByte();
      else dbgSerialReadByte();
      lastByte = millis();
    } else if (n > 0 && millis() - lastByte > 20) {
      Serial.println("  RX: " + bytesToHex(buf, n));
      n = 0;
    }
  }
  if (n > 0) Serial.println("  RX: " + bytesToHex(buf, n));
  Serial.println("Sniff done.");
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  std::vector<String> args = splitArgs(line);
  String cmd = args[0];
  cmd.toLowerCase();

  if (cmd == "help" || cmd == "?") printHelp();
  else if (cmd == "status") doStatus();
  else if (cmd == "baud") doBaud(args);
  else if (cmd == "parity") doParity(args);
  else if (cmd == "stop") doStop(args);
  else if (cmd == "scan") doScan(args);
  else if (cmd == "read") doRead(args);
  else if (cmd == "write") doWriteReg(args, false);
  else if (cmd == "write!") doWriteReg(args, true);
  else if (cmd == "raw") doRaw(line);
  else if (cmd == "log") doLog(args);
  else if (cmd == "sniff") doSniff(args);
  else Serial.println("Unknown command '" + cmd + "'. Type 'help' for the list.");
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      Serial.println();
      handleCommand(inputLine);
      inputLine = "";
      Serial.print("> ");
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}

// =============================================================================
// WEB INTERFACE — plain HTTP, no login, no auth. Open the board's IP in
// a browser and use it directly.
// =============================================================================

const char PAGE_HEAD[] =
  "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>sensor-debug</title><style>"
  "body{font-family:monospace;background:#111;color:#ddd;margin:0;padding:12px;}"
  "h1{color:#6cf;font-size:18px;margin:0 0 10px;}"
  "h2{color:#6cf;font-size:14px;margin:16px 0 6px;border-bottom:1px solid #333;padding-bottom:4px;}"
  "form{margin:0 0 10px;padding:8px;background:#1a1a1a;border-radius:6px;}"
  "input,select{font-family:monospace;background:#222;color:#ddd;border:1px solid #444;border-radius:4px;padding:4px 6px;margin:2px;}"
  "button{font-family:monospace;background:#2a5;color:#fff;border:none;border-radius:4px;padding:6px 12px;margin:2px;cursor:pointer;}"
  "button.danger{background:#a33;}"
  "pre{background:#000;color:#8f8;padding:8px;border-radius:6px;overflow-x:auto;white-space:pre-wrap;word-break:break-all;}"
  "a{color:#6cf;}"
  ".row{display:flex;flex-wrap:wrap;gap:6px;align-items:center;}"
  "</style></head><body>"
  "<h1>sensor-debug \xe2\x80\x94 RS485/Modbus tool</h1>";

const char PAGE_FOOT[] = "</body></html>";

String htmlEscape(const String& s) {
  String out;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '&') out += "&amp;";
    else out += c;
  }
  return out;
}

void handleRoot() {
  SerialCfg sc = dbgGetSerialCfg();
  String h = PAGE_HEAD;

  h += "<div>IP: " + WiFi.localIP().toString() + " &nbsp; RSSI: " + String(WiFi.RSSI()) + " dBm &nbsp; "
       "<a href='/log'>traffic log</a></div>";

  h += "<h2>Serial config: " + String(sc.baud) + " baud, 8" + String(sc.parity) + String(sc.stopBits) + "</h2>";
  h += "<form action='/config' method='GET'><div class='row'>"
       "<select name='baud'>";
  for (uint32_t b : {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200}) {
    h += "<option value='" + String(b) + "'" + (b == sc.baud ? " selected" : "") + ">" + String(b) + "</option>";
  }
  h += "</select>"
       "<select name='parity'>"
       "<option value='N'" + String(sc.parity == 'N' ? " selected" : "") + ">None</option>"
       "<option value='E'" + String(sc.parity == 'E' ? " selected" : "") + ">Even</option>"
       "<option value='O'" + String(sc.parity == 'O' ? " selected" : "") + ">Odd</option>"
       "</select>"
       "<select name='stop'>"
       "<option value='1'" + String(sc.stopBits == 1 ? " selected" : "") + ">1 stop</option>"
       "<option value='2'" + String(sc.stopBits == 2 ? " selected" : "") + ">2 stop</option>"
       "</select>"
       "<button type='submit'>Apply</button>"
       "</div></form>";

  h += "<h2>Bus scan</h2>"
       "<form action='/scan' method='GET'><div class='row'>"
       "Max address: <input type='number' name='max' value='20' min='1' max='247' style='width:60px'>"
       "<button type='submit'>Scan</button>"
       "</div></form>";

  h += "<h2>Passive sniff (no TX)</h2>"
       "<form action='/sniff' method='GET'><div class='row'>"
       "Seconds: <input type='number' name='secs' value='5' min='1' max='30' style='width:60px'>"
       "<button type='submit'>Sniff</button>"
       "</div></form>";

  h += "<h2>Read registers</h2>"
       "<form action='/read' method='GET'><div class='row'>"
       "Slave: <input name='sid' value='1' style='width:50px'>"
       "FC: <select name='fc'><option value='3'>03</option><option value='4'>04</option></select>"
       "Reg: <input name='reg' value='0x0000' style='width:70px'>"
       "Count: <input name='count' value='1' style='width:50px'>"
       "<button type='submit'>Read</button>"
       "</div></form>";

  h += "<h2>Write register (FC06)</h2>"
       "<form action='/write' method='GET'><div class='row'>"
       "Slave: <input name='sid' value='1' style='width:50px'>"
       "Reg: <input name='reg' value='0x0000' style='width:70px'>"
       "Value: <input name='val' value='0' style='width:70px'>"
       "<button type='submit' class='danger' onclick=\"return confirm('Write this register?');\">Write</button>"
       "</div></form>";

  h += "<h2>Raw hex send</h2>"
       "<form action='/raw' method='GET'><div class='row'>"
       "<input name='hex' value='01 03 00 00 00 01 84 0A' style='width:260px'>"
       "<button type='submit'>Send</button>"
       "</div></form>";

  h += PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleConfig() {
  SerialCfg sc = dbgGetSerialCfg();
  uint32_t baud = server.hasArg("baud") ? server.arg("baud").toInt() : sc.baud;
  char parity = server.hasArg("parity") ? server.arg("parity")[0] : sc.parity;
  int stop = server.hasArg("stop") ? server.arg("stop").toInt() : sc.stopBits;
  dbgSerialApply(baud, parity, stop);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleScan() {
  int maxAddr = server.hasArg("max") ? server.arg("max").toInt() : 20;
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;

  String out = "Scanning slave addresses 1-" + String(maxAddr) + "...\n";
  int found = 0;
  for (int addr = 1; addr <= maxAddr; addr++) {
    ModbusResult r4 = dbgReadRegs((uint8_t)addr, 4, 0x0000, 1, 300);
    if (r4.ok) { out += "  addr " + String(addr) + " responded to FC04\n"; found++; continue; }
    ModbusResult r3 = dbgReadRegs((uint8_t)addr, 3, 0x0000, 1, 300);
    if (r3.ok) { out += "  addr " + String(addr) + " responded to FC03\n"; found++; }
  }
  out += "Scan done. " + String(found) + " address(es) responded.";

  String h = String(PAGE_HEAD) + "<h2>Scan result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleSniff() {
  int secs = server.hasArg("secs") ? server.arg("secs").toInt() : 5;
  if (secs < 1) secs = 1;
  if (secs > 30) secs = 30;

  String out = "Sniffing passively for " + String(secs) + "s (no TX)...\n";
  unsigned long deadline = millis() + (unsigned long)secs * 1000;
  uint8_t buf[64];
  int n = 0;
  unsigned long lastByte = 0;
  while (millis() < deadline) {
    if (dbgSerialAvailable()) {
      if (n < (int)sizeof(buf)) buf[n++] = dbgSerialReadByte();
      else dbgSerialReadByte();
      lastByte = millis();
    } else if (n > 0 && millis() - lastByte > 20) {
      out += "  RX: " + bytesToHex(buf, n) + "\n";
      n = 0;
    }
  }
  if (n > 0) out += "  RX: " + bytesToHex(buf, n) + "\n";
  out += "Sniff done.";

  String h = String(PAGE_HEAD) + "<h2>Sniff result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleReadWeb() {
  uint8_t sid = server.hasArg("sid") ? (uint8_t)parseNum(server.arg("sid")) : 1;
  uint8_t fc = server.hasArg("fc") ? (uint8_t)parseNum(server.arg("fc")) : 3;
  uint16_t reg = server.hasArg("reg") ? (uint16_t)parseNum(server.arg("reg")) : 0;
  uint8_t count = server.hasArg("count") ? (uint8_t)parseNum(server.arg("count")) : 1;

  String out = "Reading slave " + String(sid) + ", FC" + String(fc) + ", reg 0x" + String(reg, HEX) + ", count " + String(count) + "...\n";
  ModbusResult r = dbgReadRegs(sid, fc, reg, count);
  if (r.ok) {
    out += "OK \xe2\x80\x94 reply from slave " + String(r.actualSlaveId) + ":\n";
    for (int i = 0; i < r.regCount; i++) {
      out += "  #" + String(i) + " = " + String(r.regs[i]) + " (0x" + String(r.regs[i], HEX) + ")\n";
    }
  } else {
    out += "FAILED \xe2\x80\x94 " + r.error + "\n";
  }
  out += "TX: " + r.txHex + "\n";
  if (r.rxHex.length()) out += "RX: " + r.rxHex;

  String h = String(PAGE_HEAD) + "<h2>Read result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleWriteWeb() {
  uint8_t sid = server.hasArg("sid") ? (uint8_t)parseNum(server.arg("sid")) : 1;
  uint16_t reg = server.hasArg("reg") ? (uint16_t)parseNum(server.arg("reg")) : 0;
  uint16_t val = server.hasArg("val") ? (uint16_t)parseNum(server.arg("val")) : 0;

  String out = "Writing slave " + String(sid) + ", reg 0x" + String(reg, HEX) + " = " + String(val) + " (0x" + String(val, HEX) + ")...\n";
  ModbusResult r = dbgWriteReg(sid, reg, val);
  if (r.ok) out += "OK \xe2\x80\x94 " + (r.error.length() ? r.error : String("confirmed")) + "\n";
  else out += "FAILED \xe2\x80\x94 " + r.error + "\n";
  out += "TX: " + r.txHex + "\n";
  if (r.rxHex.length()) out += "RX: " + r.rxHex;

  String h = String(PAGE_HEAD) + "<h2>Write result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleRawWeb() {
  String hex = server.hasArg("hex") ? server.arg("hex") : "";
  String out = "Sending raw: " + hex + "\n";
  String rx = dbgRawHexSend(hex, 800);
  out += rx.length() ? ("Reply: " + rx) : String("No reply.");

  String h = String(PAGE_HEAD) + "<h2>Raw send result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleLogWeb() {
  String out = dbgLogHumanText(80);
  String h = String(PAGE_HEAD) + "<h2>Traffic log (last 80, newest first)</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/scan", handleScan);
  server.on("/sniff", handleSniff);
  server.on("/read", handleReadWeb);
  server.on("/write", handleWriteWeb);
  server.on("/raw", handleRawWeb);
  server.on("/log", handleLogWeb);
  server.begin();
  Serial.println("[Web] Server started, no login required.");
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  dbgSerialInit(RS485_RXD, RS485_TXD, RS485_DE, 9600, 'N', 1);
  Serial.println("\n=== sensor-debug: RS485/Modbus debugging tool ===");
  printHelp();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  Serial.printf("[WiFi] Connecting to \"%s\"...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long deadline = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected. Open http://%s/ in a browser — no login needed.\n",
      WiFi.localIP().toString().c_str());
    setupWebServer();
  } else {
    Serial.println("[WiFi] Could not connect. Web page unavailable — USB serial commands still work.");
    Serial.println("[WiFi] Edit WIFI_SSID/WIFI_PASS at the top of sensor-debug.ino and re-flash to change network.");
  }

  Serial.print("> ");
}

void loop() {
  pollSerial();
  if (WiFi.status() == WL_CONNECTED) server.handleClient();
}
