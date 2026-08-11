// =============================================================================
// sensor-debug-lilygo.ino — Standalone RS485/Modbus debugging tool
// (LilyGo T-CAN485 variant)
//
// Same tool as sensor-debug/ (Waveshare ESP32-S3-RS485-CAN variant), just
// re-pinned for the LilyGo T-CAN485 board (ESP32, not ESP32-S3) which
// production rig-module-firmware.ino targets. Two boards, two sets of
// pins/enable-lines, same debugging logic (modbus_debug.h / bitscope.h
// are byte-for-byte copies from sensor-debug/).
//
// The board is its own WiFi access point — no router/hotspot needed,
// nothing to join beforehand. On boot it broadcasts an open SSID
// ("sensor-debug-lilygo", no password) and the web page at 192.168.4.1
// is available immediately with zero login/setup screen — just connect
// your phone/laptop to that WiFi network like any open AP, then open
// http://192.168.4.1/ in a browser. No captive portal, no credentials
// form, nothing gating the actual debugging tools. Everything is also
// available over USB Serial (115200 baud) if you'd rather skip WiFi
// entirely.
//
// Built after the SM7779 radar sensor got stuck outputting garbage
// following experimental register writes. Separate sketch, doesn't
// touch or depend on the "real" firmware (rig-module-firmware.ino) at
// all — flash this to the LilyGo board when you need to dig into a
// misbehaving sensor, flash the real firmware back when done.
//
// Hardware: LilyGo T-CAN485 (verified against official LilyGo example:
//   https://github.com/Xinyuan-LilyGO/T-CAN485/blob/main/example/Arduino/RS485/config.h)
//   RS485: TX=GPIO22, RX=GPIO21, DE/RE=GPIO17 (MAX13487, HIGH=transmit)
//   PIN_5V_EN=GPIO16 (5V booster enable, must be HIGH or RS485 has no
//     power at all — this board has NO onboard 3.3V RS485 supply like
//     the Waveshare does)
//   RS485_SE=GPIO19 (/SHDN pin on the MAX13487 chip, must be HIGH to
//     un-shutdown the transceiver — also unique to this board)
//
// Arduino IDE board settings: same as rig-module-firmware.ino —
//   Board: "ESP32 Dev Module" (NOT ESP32S3), default partition scheme
//   is fine, no PSRAM option (this is a plain ESP32, not ESP32-S3).
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
//   qdw90a [slaveId]              — labeled probe of all 7 QDW90A/QDY30A
//                                    registers (address, baud, unit, decimals,
//                                    value, zero point, full-scale), default
//                                    slave 1, at whatever baud/parity/stop is
//                                    currently active
// =============================================================================

#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include "modbus_debug.h"
#include "bitscope.h"

#define RS485_TXD 22
#define RS485_RXD 21
#define RS485_DE  17
#define PIN_5V_EN 16   // 5V booster enable — must be HIGH or RS485 has no power
#define RS485_SE  19   // RS485 /SHDN (shutdown pin — must be HIGH to enable chip)

// The board's own access point. Open network (no password) by default
// so there's nothing to type before you can see the debugging tools.
#define AP_SSID "sensor-debug-lilygo"
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
    "  recover                       SM7779 recovery sweep: writes 0x0068/0x0069\n"
    "                                reset values across all parity/stop combos at\n"
    "                                9600, checking for a real ack after each. If\n"
    "                                nothing acks anywhere, auto-runs a 10s sniff too\n"
    "  qdw90a [slaveId]              labeled read of all 7 QDW90A/QDY30A registers\n"
    "                                (default slave 1)\n"
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

// =============================================================================
// QDW90A / QDY30A PRESSURE SENSOR — labeled 7-register probe. Confirmed
// register map (from the production waveshare-s3-sensors firmware's
// Multi-Register Read preset, sourced from the HA community thread on
// this OEM family): 7 holding registers (FC03) starting at 0x0000:
//   0: slave address (1-255)
//   1: baud code (0=1200 1=2400 2=4800 3=9600 4=19200 5=38400 6=57600 7=115200)
//   2: pressure unit code (0=none 1=CM 2=MM 3=MPa 4=Pa 5=KPa 6=mA - labels
//      sometimes unreliable per HA community reports, verify empirically)
//   3: decimal places code (0=#### 1=###.# 2=##.## 3=#.###)
//   4: measured value (signed, apply decimal code - e.g. code 2 + raw 123 = 1.23)
//   5: range zero point
//   6: range full-scale point
// Same wire-format facts apply as the production preset: FC03 holding
// registers, default slave ID 1, default baud 9600 8N1 (same family as
// the SM7779, hence the address collision history when both are on the
// bus at once with factory defaults).
// =============================================================================
static const char* QDW90A_LABELS[7] = {
  "Slave address (1-255)",
  "Baud code (0=1200 1=2400 2=4800 3=9600 4=19200 5=38400 6=57600 7=115200)",
  "Pressure unit code (0=none 1=CM 2=MM 3=MPa 4=Pa 5=KPa 6=mA - verify empirically)",
  "Decimal places code (0=#### 1=###.# 2=##.## 3=#.###)",
  "Measured value (signed, apply decimal code)",
  "Range zero point",
  "Range full-scale point"
};

// Applies the decimal-places code (register 3) to the raw measured value
// (register 4) so the printed output shows an actual number, not just
// raw counts - e.g. decimals=2, raw=123 -> "1.23".
String qdw90aScaledValue(uint16_t rawValue, uint16_t decimalsCode) {
  int16_t signedVal = (int16_t)rawValue; // register 4 is documented signed
  int decimals = (decimalsCode <= 3) ? (int)decimalsCode : 0;
  char buf[24];
  dtostrf(signedVal / pow(10, decimals), 0, decimals, buf);
  return String(buf);
}

// Shared report builder used by both the USB command and the web page -
// one code path, two output surfaces (same pattern as dbgSniffCapture).
String qdw90aReport(uint8_t sid) {
  SerialCfg sc = dbgGetSerialCfg();
  String out = "Probing QDW90A/QDY30A at slave " + String(sid) + ", FC03, reg 0x0000, count 7 ("
    + String(sc.baud) + " baud 8" + String(sc.parity) + String(sc.stopBits) + ")...\n";
  ModbusResult r = dbgReadRegs(sid, 3, 0x0000, 7, 500);
  if (!r.ok) {
    out += "FAILED - " + r.error + "\n";
    out += "  TX: " + r.txHex + "\n";
    if (r.rxHex.length()) out += "  RX: " + r.rxHex + "\n";
    out += "No reply at all usually means: wrong baud/framing, no 24V power to the sensor, "
      "or A/B (data pair) wires swapped/miswired. A single stray byte back (not a full 7-register "
      "reply) usually means power+wiring are close but framing/baud is still off.\n";
    return out;
  }
  out += "OK - reply from slave " + String(r.actualSlaveId) + ":\n";
  for (int i = 0; i < r.regCount && i < 7; i++) {
    char line[160];
    snprintf(line, sizeof(line), "  #%d = %u (0x%04X)  %s\n", i, r.regs[i], r.regs[i], QDW90A_LABELS[i]);
    out += line;
  }
  if (r.regCount >= 5) {
    String scaled = qdw90aScaledValue(r.regs[4], r.regs[3]);
    out += "  -> Decoded pressure reading: " + scaled + " (unit per register #2's code above)\n";
  }
  out += "  TX: " + r.txHex + "\n";
  out += "  RX: " + r.rxHex + "\n";
  return out;
}

void doQdw90a(std::vector<String>& args) {
  uint8_t sid = args.size() > 1 ? (uint8_t)parseNum(args[1]) : 1;
  Serial.print(qdw90aReport(sid));
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

// Shared passive-listen implementation used by the USB command, the web
// page, and appended automatically at the end of the recovery sweep -
// one code path, three callers. Pure receive, no TX, so it's always
// safe to call regardless of what mode the port is otherwise in.
String dbgSniffCapture(int secs) {
  bool wasAutoOn = dbgAutoSniffGetEnabled();
  dbgAutoSniffSetEnabled(false);
  String out;
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
  if (out.length() == 0) out = "  (nothing captured)\n";
  dbgAutoSniffSetEnabled(wasAutoOn);
  return out;
}

void doSniff(std::vector<String>& args) {
  int secs = args.size() > 1 ? parseNum(args[1]) : 5;
  Serial.printf("Sniffing bus passively for %d seconds (no TX)...\n", secs);
  Serial.print(dbgSniffCapture(secs));
  Serial.println("Sniff done.");
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
//
// IMPORTANT: this uses UNICAST writes to address 1 as the primary probe,
// not broadcast. dbgWriteReg() reports ok=true for ANY broadcast address
// with zero reply by design (that's correct behavior for normal traffic
// logging - Modbus broadcast slaves must not reply - but it's a useless
// signal here, since it can't distinguish "reset landed and worked" from
// "wrong framing, sensor never even saw it"). A unicast write gets a real
// ack (echo) or exception code back, which actually proves the framing
// matched and the sensor is alive - false positives aren't possible.
//
// At each of the 6 combos: unicast (slave 1) FC06 writes of 1 -> 0x0068
// and 1 -> 0x0069 (documented defaults). If either gets a real ack/
// exception, that framing is confirmed live - stop there. Only if
// unicast-to-1 gets nothing at every combo do we fall back to a best-
// effort broadcast pass (both address 0, the actual Modbus-standard
// broadcast, and 250) purely to try landing the reset blind, since we
// have no better option left.
// =============================================================================
String dbgRecoverySweep() {
  String report;
  SerialCfg orig = dbgGetSerialCfg();
  const char parities[] = {'N', 'E', 'O'};
  const int stops[] = {1, 2};
  bool found = false;

  report += "--- Pass 1: unicast to address 1 (real ack = proof the framing matched) ---\n";
  for (int s = 0; s < 2 && !found; s++) {
    for (int p = 0; p < 3 && !found; p++) {
      char parity = parities[p];
      int stopBits = stops[s];
      dbgSerialApply(9600, parity, stopBits);
      delay(30);

      ModbusResult w1 = dbgWriteReg(1, 0x0068, 1, 400); // comm mode -> default
      delay(80);
      ModbusResult w2 = dbgWriteReg(1, 0x0069, 1, 400); // protocol type -> default

      report += "9600 8" + String(parity) + String(stopBits) + ": 0x68 write -> "
        + (w1.ok ? "ACK (confirmed)" : w1.error) + " | 0x69 write -> "
        + (w2.ok ? "ACK (confirmed)" : w2.error) + "\n";

      if (w1.ok || w2.ok) {
        delay(300); // let a mode change actually apply before re-probing
        dbgSerialApply(9600, 'N', 1);
        delay(30);
        ModbusResult r = dbgReadRegs(1, 3, 0x0000, 3, 400);
        if (r.ok) {
          report += "  -> Read-back OK from slave " + String(r.actualSlaveId) + ": ";
          for (int i = 0; i < r.regCount; i++) report += String(r.regs[i]) + " ";
          report += "\n\n*** Sensor confirmed alive and acked the reset at 9600 8" + String(parity) + String(stopBits)
            + ". Left at 9600 8N1. ***\n";
        } else {
          report += "\n*** Sensor ACKed the write at 9600 8" + String(parity) + String(stopBits)
            + " (framing confirmed correct) but a follow-up read at 8N1 didn't respond - "
            + "leaving the port at 9600 8" + String(parity) + String(stopBits) + " since that's the framing that worked. "
            + "Try reading it manually from here. ***\n";
          dbgSerialApply(9600, parity, stopBits);
        }
        found = true;
        break;
      }
    }
  }

  if (!found) {
    report += "\nNo unicast ack at any framing combo - the sensor never confirmed receiving anything.\n";
    report += "\n--- Pass 2: best-effort broadcast (blind, no ack possible, last resort) ---\n";
    for (int s = 0; s < 2 && !found; s++) {
      for (int p = 0; p < 3 && !found; p++) {
        char parity = parities[p];
        int stopBits = stops[s];
        dbgSerialApply(9600, parity, stopBits);
        delay(30);
        dbgWriteReg(0, 0x0068, 1, 200);
        delay(50);
        dbgWriteReg(0, 0x0069, 1, 200);
        delay(50);
        dbgWriteReg(250, 0x0068, 1, 200);
        delay(50);
        dbgWriteReg(250, 0x0069, 1, 200);
        delay(300);

        dbgSerialApply(9600, 'N', 1);
        delay(30);
        report += "9600 8" + String(parity) + String(stopBits) + " blind broadcast sent, checking 8N1 for a response... ";
        ModbusResult r = dbgReadRegs(1, 3, 0x0000, 3, 400);
        if (r.ok) {
          report += "RESPONSE from slave " + String(r.actualSlaveId) + "!\n\n*** Sensor is back, left at 9600 8N1. ***\n";
          found = true;
          break;
        }
        report += "no response.\n";
      }
    }
  }

  if (!found) {
    dbgSerialApply(orig.baud, orig.parity, orig.stopBits);
    report += "\nNothing acked or responded at any framing/method. This means either:\n"
      "  1. The sensor's receiver isn't at 9600 in ANY of these parity/stop combos (baud itself may\n"
      "     have changed too, despite 0x0067 not being an intentional target)\n"
      "  2. It needs a power cycle before register writes take effect\n"
      "  3. Those registers control something other than serial framing (comm mode may mean RS232\n"
      "     vs RS485, or half/full duplex, not parity/stop bits)\n"
      "Auto-sniff below shows whether the ~5.2s auto-burst pattern changed at all (even a byte-count or "
      "timing change would prove something landed). If it's byte-for-byte identical to before, try a "
      "power cycle next - some sensors only apply config writes on boot.\n";

    report += "\n--- Auto-sniff (10s, no TX, checking the sensor's own auto-burst pattern) ---\n";
    report += dbgSniffCapture(10);
  }
  // If a working framing was already confirmed above, that report block
  // says exactly where the port was left and why - don't touch serial
  // config again here or we'd undo it.

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
  else if (cmd == "recover") { Serial.println("Starting recovery sweep (6 framing combos + auto-sniff if nothing acks, ~4-15s)..."); Serial.print(dbgRecoverySweep()); }
  else if (cmd == "qdw90a") doQdw90a(args);
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
       "Cycles all 6 parity/stop combos at 9600 baud, writing reset values (1 -&gt; 0x0068, "
       "1 -&gt; 0x0069) to address 1 at each and checking for a real ack. Stops at the first combo "
       "that acks. If nothing acks anywhere, automatically runs a 10s passive sniff too so you can "
       "see if the sensor's auto-burst changed at all. Takes up to ~15s.</div>"
       "<form action='/recover' method='GET'><div class='row'>"
       "<button type='submit' class='danger' onclick=\"return confirm('Broadcast reset writes across all framing combos?');\">Run Recovery Sweep</button>"
       "</div></form>";

  h += "<h2>QDW90A / QDY30A Pressure Sensor Probe</h2>"
       "<div class='row'>Labeled read of all 7 registers (address, baud, unit, decimals, value, "
       "zero point, full-scale) at whatever baud/parity/stop is set above. Same family/register map "
       "as the production firmware's Multi-Register Read QDW90A preset. Needs genuine 24V power - "
       "won't respond at 12V.</div>"
       "<form action='/qdw90a' method='GET'><div class='row'>"
       "Slave: <input name='sid' value='1' style='width:50px'>"
       "<button type='submit'>Probe QDW90A</button>"
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

void handleQdw90aWeb() {
  uint8_t sid = server.hasArg("sid") ? (uint8_t)parseNum(server.arg("sid")) : 1;
  String out = qdw90aReport(sid);
  Serial.println("[Web] " + out);

  String h = String(PAGE_HEAD) + "<h2>QDW90A probe result</h2><pre>" + htmlEscape(out) + "</pre><p><a href='/'>&larr; back</a></p>" + PAGE_FOOT;
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

// Plain-text endpoint the /log page polls via JS fetch() so it can
// live-update in place - no full page reload, no manual F5 needed.
void handleLogText() {
  server.send(200, "text/plain", dbgLogHumanText(80));
}

void handleLogWeb() {
  String out = dbgLogHumanText(80);
  String h = String(PAGE_HEAD) +
    "<h2>Traffic log (last 80, newest first) - live, updates every second</h2>"
    "<div class='row'>"
    "<button type='button' onclick='toggleLogPoll()' id='logPollBtn'>Pause</button> "
    "<a href='/'>&larr; back</a>"
    "</div>"
    "<pre id='logbox'>" + htmlEscape(out) + "</pre>"
    "<script>"
    "let logPolling=true;"
    "function toggleLogPoll(){logPolling=!logPolling;document.getElementById('logPollBtn').textContent=logPolling?'Pause':'Resume';}"
    "function pollLog(){"
      "if(logPolling){"
        "fetch('/log.txt').then(r=>r.text()).then(t=>{document.getElementById('logbox').textContent=t;});"
      "}"
      "setTimeout(pollLog,1000);"
    "}"
    "pollLog();"
    "</script>"
    + PAGE_FOOT;
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
  server.on("/qdw90a", handleQdw90aWeb);
  server.on("/read", handleReadWeb);
  server.on("/write", handleWriteWeb);
  server.on("/raw", handleRawWeb);
  server.on("/log", handleLogWeb);
  server.on("/log.txt", handleLogText);
  server.begin();
  Serial.println("[Web] Server started, no login required.");
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // LilyGo T-CAN485-specific: unlike the Waveshare board, RS485 has no
  // power at all until these two lines are driven HIGH.
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);   // enable 5V rail for RS485 transceiver
  pinMode(RS485_SE, OUTPUT);
  digitalWrite(RS485_SE, HIGH);    // un-shutdown the MAX13487 RS485 chip
  delay(20);

  dbgSerialInit(RS485_RXD, RS485_TXD, RS485_DE, 9600, 'N', 1);
  bitscopeInit(RS485_RXD);
  Serial.println("\n=== sensor-debug-lilygo: RS485/Modbus debugging tool (LilyGo T-CAN485) ===");
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
