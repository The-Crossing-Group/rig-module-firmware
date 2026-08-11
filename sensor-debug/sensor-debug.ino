// =============================================================================
// sensor-debug.ino — Standalone RS485/Modbus debugging tool
//
// The board is its own WiFi access point — no router/hotspot needed,
// nothing to join beforehand. On boot it broadcasts an open SSID
// ("sensor-debug", no password) and the web page at 192.168.4.1 is
// available immediately with zero login/setup screen — just connect
// your phone/laptop to that WiFi network like any open AP, then open
// http://192.168.4.1/ in a browser. No captive portal, no credentials
// form, nothing gating the actual debugging tools. Everything is also
// available over USB Serial (115200 baud) if you'd rather skip WiFi
// entirely.
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
// WiFi: SSID/password are set by the two #defines below (open network
// by default — set AP_PASS to a real password if you want one, must be
// 8+ chars or WiFi.softAP() silently falls back to open anyway). Edit
// + re-flash to change. Once powered on, connect to the AP and open
// http://192.168.4.1/ (the fixed default IP for ESP32 softAP) — no
// login page in between.
//
// USB SERIAL COMMANDS (open Serial Monitor, 115200 baud, line ending
// "Newline") — same functionality as the web page, useful if you'd
// rather skip WiFi entirely:
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
#include "bitscope.h"

#define RS485_TXD 17
#define RS485_RXD 18
#define RS485_DE  21

// The board's own access point. Open network (no password) by default
// so there's nothing to type before you can see the debugging tools.
#define AP_SSID "sensor-debug"
#define AP_PASS ""

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
    "  autosniff <on|off>            toggle continuous background traffic capture\n"
    "  bitscope [ms]                 raw electrical capture, bypasses UART framing\n"
    "                                entirely - measures real bit period + brute-\n"
    "                                forces every byte alignment against Modbus CRC\n"
    "                                (default 500ms window, use during a known burst)\n"
    "  recover                       SM7779 recovery sweep: broadcasts 0x0068/0x0069\n"
    "                                reset writes across all parity/stop combos at\n"
    "                                9600, checking for a live response after each\n"
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
  Serial.printf("Current RS485 config: %lu baud, 8%c%d\n", sc.baud, sc.parity, sc.stopBits);
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
  bool wasAutoOn = dbgAutoSniffGetEnabled();
  dbgAutoSniffSetEnabled(false);
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
  dbgAutoSniffSetEnabled(wasAutoOn);
}

void doBitscope(std::vector<String>& args) {
  int ms = args.size() > 1 ? (int)parseNum(args[1]) : 6000;
  if (ms < 50) ms = 50;
  if (ms > 15000) ms = 15000;

  bool wasAutoOn = dbgAutoSniffGetEnabled();
  dbgAutoSniffSetEnabled(false);

  Serial.printf("Bitscope: capturing raw edges on RX pin for %dms (make sure the sensor is transmitting during this window - if it auto-reports every few seconds, use a window longer than its period)...\n", ms);
  bitscopeCapture(ms);
  BitscopeResult r = bitscopeAnalyze();

  Serial.printf("Edges captured: %d\n", r.edgeCount);
  if (r.minPulseUs > 0) {
    Serial.printf("Shortest pulse: %luus  ->  estimated real baud: %lu\n", (unsigned long)r.minPulseUs, (unsigned long)r.estimatedBaud);
    Serial.println("Raw bitstream (first up to 400 bit-units):");
    Serial.println("  " + r.bitstream);
  }
  Serial.println("Byte-alignment sweep (checked against Modbus CRC16 at every bit offset 0-10):");
  Serial.print(r.bestDecodeReport);

  dbgAutoSniffSetEnabled(wasAutoOn);
}

// =============================================================================
// RECOVERY SWEEP — built specifically for the SM7779 "stuck outputting
// garbage after writes to 0x0068/0x0069" scenario. Baud is left alone
// (that register was never touched, so 9600 almost certainly still
// applies) - only parity/stop bits are swept, since a "comm mode" write
// is a plausible way to have changed the sensor's own serial framing.
// At each of the 6 combos: broadcast (slave 250, no reply expected)
// FC06 writes of 1 -> 0x0068 and 1 -> 0x0069 (the documented defaults),
// then switch back to standard 9600 8N1 and check whether slave 1 (or
// anything on 1-5) responds normally. First combo that produces a live
// response wins and the sweep stops there, leaving the port at 9600 8N1
// so the sensor can be read/used immediately.
// =============================================================================
String dbgRecoverySweep() {
  String report;
  SerialCfg orig = dbgGetSerialCfg();
  const char parities[] = {'N', 'E', 'O'};
  const int stops[] = {1, 2};
  bool found = false;

  for (int s = 0; s < 2 && !found; s++) {
    for (int p = 0; p < 3 && !found; p++) {
      char parity = parities[p];
      int stopBits = stops[s];
      dbgSerialApply(9600, parity, stopBits);
      delay(30);

      ModbusResult w1 = dbgWriteReg(250, 0x0068, 1, 300); // comm mode -> default
      delay(80);
      ModbusResult w2 = dbgWriteReg(250, 0x0069, 1, 300); // protocol type -> default
      delay(300); // give the sensor a moment to apply a mode change before we probe it

      dbgSerialApply(9600, 'N', 1); // Modbus RTU standard framing to test with
      delay(30);

      report += "9600 8" + String(parity) + String(stopBits) + ": reset writes sent (0x68 "
        + (w1.ok ? "ok" : "no-ack") + ", 0x69 " + (w2.ok ? "ok" : "no-ack") + "). Checking for a response at 8N1... ";

      ModbusResult r = dbgReadRegs(1, 3, 0x0000, 3, 400);
      if (r.ok) {
        report += "RESPONSE from slave " + String(r.actualSlaveId) + ": ";
        for (int i = 0; i < r.regCount; i++) report += String(r.regs[i]) + " ";
        report += "\n\n*** Sensor is back! Left at 9600 8N1 - it should work normally now. ***\n";
        found = true;
        break;
      }
      // Slave ID register (0x0066) was never touched, but check a small
      // range anyway in case something else shifted it.
      bool anyHit = false;
      for (int addr = 1; addr <= 5 && !anyHit; addr++) {
        ModbusResult rr = dbgReadRegs((uint8_t)addr, 3, 0x0000, 1, 300);
        if (rr.ok) {
          report += "no reply from addr 1, but addr " + String(addr) + " responded!\n\n*** Sensor is back, now at address " + String(addr) + ", left at 9600 8N1. ***\n";
          anyHit = true;
          found = true;
        }
      }
      if (!anyHit) report += "no response.\n";
    }
  }

  if (!found) {
    dbgSerialApply(orig.baud, orig.parity, orig.stopBits);
    report += "\nNo framing combo produced a response after the reset writes. Possible next steps: "
      "the writes may not have landed in ANY of these framings (try 'sniff 10' right after this to "
      "see if the sensor's auto-report burst changed at all), a baud change may be needed too even "
      "though 0x0067 wasn't touched, or the sensor may need a hardware/power-cycle reset.";
  }
  return report;
}

void doAutoSniff(std::vector<String>& args) {
  if (args.size() < 2) {
    Serial.println(String("Auto traffic capture is currently ") + (dbgAutoSniffGetEnabled() ? "ON" : "OFF") + ". Usage: autosniff <on|off>");
    return;
  }
  String v = args[1];
  v.toLowerCase();
  if (v == "on" || v == "1") { dbgAutoSniffSetEnabled(true); Serial.println("Auto traffic capture ON — unsolicited bytes will print with [Auto] prefix."); }
  else if (v == "off" || v == "0") { dbgAutoSniffSetEnabled(false); Serial.println("Auto traffic capture OFF."); }
  else Serial.println("Usage: autosniff <on|off>");
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
  else if (cmd == "autosniff") doAutoSniff(args);
  else if (cmd == "bitscope") doBitscope(args);
  else if (cmd == "recover") { Serial.println("Starting recovery sweep (6 framing combos, ~3-4s)..."); Serial.print(dbgRecoverySweep()); }
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

  h += "<div>AP: " + String(AP_SSID) + " &nbsp; IP: " + WiFi.softAPIP().toString() + " &nbsp; "
       "Clients: " + String(WiFi.softAPgetStationNum()) + " &nbsp; "
       "<a href='/log'>traffic log</a></div>";

  h += "<h2>Auto traffic capture: " + String(dbgAutoSniffGetEnabled() ? "ON" : "OFF") + "</h2>"
       "<div class='row'>Every unsolicited byte on the bus is captured and logged automatically, no button needed - "
       "check <a href='/log'>traffic log</a> or the Serial Monitor ([Auto] lines). "
       "<a href='/autosniff?on=" + String(dbgAutoSniffGetEnabled() ? "0" : "1") + "'>"
       "<button type='button'>" + String(dbgAutoSniffGetEnabled() ? "Pause" : "Resume") + "</button></a></div>";

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

  h += "<h2>Bitscope (raw electrical capture, bypasses UART framing)</h2>"
       "<div class='row'>Measures the real bit period straight off the wire and brute-forces "
       "every byte alignment against Modbus CRC. Window must be longer than the sensor's "
       "auto-report period or it may capture zero edges (nothing sent yet).</div>"
       "<form action='/bitscope' method='GET'><div class='row'>"
       "Window (ms): <input type='number' name='ms' value='6000' min='50' max='15000' style='width:70px'>"
       "<button type='submit'>Capture</button>"
       "</div></form>";

  h += "<h2>SM7779 Recovery Sweep</h2>"
       "<div class='row'>For a sensor stuck outputting garbage after writes to 0x0068/0x0069. "
       "Cycles all 6 parity/stop combos at 9600 baud, broadcasting reset writes (1 -&gt; 0x0068, "
       "1 -&gt; 0x0069) at each, then checks for a live response at standard 8N1. Stops at the "
       "first combo that works and leaves the port there. Takes a few seconds.</div>"
       "<form action='/recover' method='GET'><div class='row'>"
       "<button type='submit' class='danger' onclick=\"return confirm('Broadcast reset writes across all framing combos?');\">Run Recovery Sweep</button>"
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
  Serial.printf("[Web] Config applied: %lu baud, 8%c%d\n", baud, parity, stop);
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
  Serial.println("[Web] " + out);

  String h = String(PAGE_HEAD) + "<h2>Scan result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleAutosniff() {
  bool on = server.hasArg("on") ? server.arg("on").toInt() != 0 : true;
  dbgAutoSniffSetEnabled(on);
  Serial.println(String("[Web] Auto traffic capture ") + (on ? "resumed" : "paused"));
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSniff() {
  int secs = server.hasArg("secs") ? server.arg("secs").toInt() : 5;
  if (secs < 1) secs = 1;
  if (secs > 30) secs = 30;

  // Manual timed sniff and continuous auto-capture both read the same
  // serial port - pause auto-capture for the duration so bytes aren't
  // split between the two, then restore whatever state it was in.
  bool wasAutoOn = dbgAutoSniffGetEnabled();
  dbgAutoSniffSetEnabled(false);

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
  Serial.println("[Web] " + out);
  dbgAutoSniffSetEnabled(wasAutoOn);

  String h = String(PAGE_HEAD) + "<h2>Sniff result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleBitscopeWeb() {
  int ms = server.hasArg("ms") ? server.arg("ms").toInt() : 6000;
  if (ms < 50) ms = 50;
  if (ms > 15000) ms = 15000;

  bool wasAutoOn = dbgAutoSniffGetEnabled();
  dbgAutoSniffSetEnabled(false);

  String out = "Capturing raw edges for " + String(ms) + "ms...\n";
  bitscopeCapture(ms);
  BitscopeResult r = bitscopeAnalyze();

  out += "Edges captured: " + String(r.edgeCount) + "\n";
  if (r.minPulseUs > 0) {
    out += "Shortest pulse: " + String(r.minPulseUs) + "us  ->  estimated real baud: " + String(r.estimatedBaud) + "\n";
    out += "Raw bitstream (first up to 400 bit-units):\n  " + r.bitstream + "\n";
  }
  out += "Byte-alignment sweep (checked against Modbus CRC16 at every bit offset 0-10):\n" + r.bestDecodeReport;
  Serial.println("[Web] Bitscope:\n" + out);

  dbgAutoSniffSetEnabled(wasAutoOn);

  String h = String(PAGE_HEAD) + "<h2>Bitscope result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleRecoverWeb() {
  String out = "Running SM7779 recovery sweep (6 framing combos)...\n\n";
  out += dbgRecoverySweep();
  Serial.println("[Web] Recovery sweep:\n" + out);

  String h = String(PAGE_HEAD) + "<h2>Recovery sweep result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
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
  Serial.println("[Web] " + out);

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
  Serial.println("[Web] " + out);

  String h = String(PAGE_HEAD) + "<h2>Write result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
  server.send(200, "text/html", h);
}

void handleRawWeb() {
  String hex = server.hasArg("hex") ? server.arg("hex") : "";
  String out = "Sending raw: " + hex + "\n";
  String rx = dbgRawHexSend(hex, 800);
  out += rx.length() ? ("Reply: " + rx) : String("No reply.");
  Serial.println("[Web] " + out);

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
  server.on("/autosniff", handleAutosniff);
  server.on("/bitscope", handleBitscopeWeb);
  server.on("/recover", handleRecoverWeb);
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
  bitscopeInit(RS485_RXD);
  Serial.println("\n=== sensor-debug: RS485/Modbus debugging tool ===");
  printHelp();

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  bool apOk = strlen(AP_PASS) > 0
    ? WiFi.softAP(AP_SSID, AP_PASS)
    : WiFi.softAP(AP_SSID);

  if (apOk) {
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[WiFi] Access point \"%s\" up. Connect to it, then open http://%s/ — no login needed.\n",
      AP_SSID, ip.toString().c_str());
    setupWebServer();
  } else {
    Serial.println("[WiFi] Access point failed to start. Web page unavailable — USB serial commands still work.");
  }

  Serial.print("> ");
}

void loop() {
  pollSerial();
  server.handleClient();
  dbgAutoSniffPoll();
}
