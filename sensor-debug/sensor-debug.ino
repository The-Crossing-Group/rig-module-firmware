// =============================================================================
// sensor-debug.ino — Standalone RS485/Modbus debugging tool (USB SERIAL ONLY)
//
// No WiFi, no web server, no setup AP. Plug the board into USB, open the
// Serial Monitor (115200 baud), type commands, read the results. That's
// the whole tool.
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
// COMMANDS (type into Serial Monitor, press Enter, line ending "Newline"):
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
#include "modbus_debug.h"

#define RS485_TXD 17
#define RS485_RXD 18
#define RS485_DE  21

String inputLine = "";

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

// --- small helpers ---
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
  dbgPrintLogHuman(n);
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

void setup() {
  Serial.begin(115200);
  delay(500);
  dbgSerialInit(RS485_RXD, RS485_TXD, RS485_DE, 9600, 'N', 1);
  Serial.println("\n=== sensor-debug: RS485/Modbus debugging tool ===");
  Serial.println("USB serial only — no WiFi.");
  printHelp();
  Serial.print("> ");
}

void loop() {
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
