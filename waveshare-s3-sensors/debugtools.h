// =============================================================================
// debugtools.h — On-board RS485/Modbus debugging tools, ported from the
// standalone LilyGo sensor-debug tool so you don't need a second board
// wired in parallel to dig into a misbehaving sensor.
//
// Shares the SAME Serial2/_mbSerial/_RS485_DE_PIN as modbus.h (already
// initialized by modbusInit() in setup()) — this file does NOT open a
// second serial port. Every function here that touches the bus MUST be
// called with modbusBusMutex already held by the caller (webui.h's /debug
// handlers all do this), same rule as every other modbus.h function.
//
// WRITE WARNING: unlike modbus.h's normal read path, this file DOES
// include FC06 register write + the SM7779 recovery sweep (which also
// writes). That capability was deliberately left OFF the rest of this
// production firmware after a write to 0x0068/0x0069 permanently
// corrupted a sensor on 2026-08-10/11 — it's back here ONLY on this one
// diagnostic page, at Sarah's explicit request, because she has three
// more of the same sensor type on this exact bus. Every write action on
// the /debug page requires an explicit confirm click. Be careful.
//
// Changing baud/parity/stop bits here changes them for _mbSerial, which
// is the SAME port the live poll task uses — the /debug page always
// shows a prominent "restore normal operation" control that puts the
// port back to the module's configured baud at 8N1 (parity 0, stop 1),
// since every sensor slot in cfg.sensors assumes 8N1 framing.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "modbus.h"

// Current serial framing the debug tools have set (starts matching
// whatever modbusInit() used — 8N1 — until something on /debug changes
// it). Parity: 'N'/'E'/'O'. This is intentionally separate bookkeeping
// from ModuleConfig — debug framing changes are never persisted to NVS.
struct DebugSerialCfg {
  uint32_t baud = 9600;
  char     parity = 'N';
  int      stopBits = 1;
};
static DebugSerialCfg _dbgCfg;

static uint32_t _dbgSerialConfigWord(char parity, int stopBits) {
  // Arduino ESP32 core SERIAL_8[N/E/O][1/2] constants.
  if (parity == 'E') return (stopBits == 2) ? SERIAL_8E2 : SERIAL_8E1;
  if (parity == 'O') return (stopBits == 2) ? SERIAL_8O2 : SERIAL_8O1;
  return (stopBits == 2) ? SERIAL_8N2 : SERIAL_8N1;
}

// RX/TX pins, set once from setup() via debugToolsInit() — needed because
// changing parity/stop bits requires a full Serial2.begin(), not just
// updateBaudRate(), and begin() needs the pin numbers again.
static int _dbgRxPin = -1;
static int _dbgTxPin = -1;
void debugToolsInit(int rxPin, int txPin) {
  _dbgRxPin = rxPin;
  _dbgTxPin = txPin;
}

// Re-applies baud/parity/stop to the shared _mbSerial. Caller must hold
// modbusBusMutex. Note: HardwareSerial doesn't have an updateBaudRate()
// overload that also changes frame config, so a parity/stop change needs
// a full begin() — this briefly drops any partially-received byte, which
// is fine for a diagnostic tool (never called from the live poll path).
void debugSerialApply(uint32_t baud, char parity, int stopBits) {
  _dbgCfg.baud = baud;
  _dbgCfg.parity = parity;
  _dbgCfg.stopBits = stopBits;
  _mbSerial->begin(baud, _dbgSerialConfigWord(parity, stopBits), _dbgRxPin, _dbgTxPin);
  delay(20);
}

// Puts the bus back to the module's normal configured framing (8N1 at
// cfg.modbusBaud) — the "restore normal operation" button on /debug.
void debugSerialRestore(long normalBaud) {
  debugSerialApply((uint32_t)normalBaud, 'N', 1);
}

DebugSerialCfg debugGetSerialCfg() { return _dbgCfg; }

// =============================================================================
// RESULT STRUCT — shared by every debug op (read/write/raw/scan) so the
// /debug page can render one consistent "TX / RX / decoded / error" block
// regardless of which tool produced it.
// =============================================================================
struct DebugResult {
  bool ok = false;
  String error;
  uint8_t actualSlaveId = 0;
  int regCount = 0;
  uint16_t regs[16] = {0};
  String txHex;
  String rxHex;
};

static String _dbgBytesToHex(const uint8_t* buf, int len) {
  String out;
  for (int i = 0; i < len; i++) {
    char h[4]; snprintf(h, 4, "%02X ", buf[i]);
    out += h;
  }
  out.trim();
  return out;
}

// Low-level send+receive at whatever framing is currently active, fully
// independent of modbus.h's modbusReadRegs() (which hardcodes verbose
// bool + fixed error taxonomy) — this collects raw TX/RX hex regardless
// of outcome, which the read-only modbusReadRegs() path doesn't expose.
static int _dbgSendReceive(const uint8_t* req, int reqLen, uint8_t* resp, int maxResp, int timeoutMs) {
  modbusFlushRx();
  digitalWrite(_RS485_DE_PIN, HIGH);
  delayMicroseconds(100);
  _mbSerial->write(req, reqLen);
  _mbSerial->flush();
  delayMicroseconds(100);
  digitalWrite(_RS485_DE_PIN, LOW);

  unsigned long deadline = millis() + timeoutMs;
  int n = 0;
  while (millis() < deadline && n < maxResp) {
    if (_mbSerial->available()) {
      resp[n++] = _mbSerial->read();
      deadline = millis() + 20;
    }
  }
  return n;
}

// FC03/FC04 read — same tolerant parsing as modbus.h's modbusReadRegs()
// (accepts whatever function code/byte-count the reply actually says,
// doesn't require it to match the request) but returns full TX/RX hex
// and a DebugResult regardless of success/failure, for display.
DebugResult debugReadRegs(uint8_t slaveId, uint8_t funcCode, uint16_t startAddr, uint8_t count, int timeoutMs = 600) {
  DebugResult r;
  if (count < 1) count = 1;
  if (count > 16) count = 16;

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = funcCode;
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;
  r.txHex = _dbgBytesToHex(req, 8);

  uint8_t resp[64];
  int n = _dbgSendReceive(req, 8, resp, sizeof(resp), timeoutMs);
  if (n > 0) r.rxHex = _dbgBytesToHex(resp, n);

  if (n < 3) { r.error = "timeout — no response"; modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_TIMEOUT); return r; }

  if (resp[1] & 0x80) {
    if (n < 5) { r.error = "timeout (partial exception frame)"; modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_TIMEOUT); return r; }
    uint16_t rxCrc = resp[3] | ((uint16_t)resp[4] << 8);
    if (rxCrc != modbusCRC(resp, 3)) { r.error = "CRC error on exception frame"; modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_CRC_ERROR); return r; }
    r.error = "exception response, code " + String(resp[2]);
    r.actualSlaveId = resp[0];
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return r;
  }

  if (resp[1] != 0x03 && resp[1] != 0x04) { r.error = "unexpected function code " + String(resp[1], HEX); r.actualSlaveId = resp[0]; modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE); return r; }

  uint8_t byteCount = resp[2];
  int frameLen = 3 + byteCount + 2;
  if (frameLen > (int)sizeof(resp) || n < frameLen) { r.error = "timeout — got " + String(n) + " bytes, frame needs " + String(frameLen); modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_TIMEOUT); return r; }

  uint16_t rxCrc = resp[frameLen-2] | ((uint16_t)resp[frameLen-1] << 8);
  if (rxCrc != modbusCRC(resp, frameLen-2)) { r.error = "CRC error"; modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_CRC_ERROR); return r; }

  r.actualSlaveId = resp[0];
  int regsInResponse = byteCount / 2;
  r.regCount = min(regsInResponse, 16);
  for (int i = 0; i < r.regCount; i++) r.regs[i] = ((uint16_t)resp[3+i*2] << 8) | resp[3+i*2+1];
  r.ok = true;
  modbusLogTransaction(slaveId, funcCode, req, 8, resp, frameLen, MB_OK);
  return r;
}

// FC06 — write single holding register. NOT present anywhere else in
// this firmware (see file header). timeoutMs default matches the
// original LilyGo tool.
DebugResult debugWriteReg(uint8_t slaveId, uint16_t reg, uint16_t value, int timeoutMs = 500) {
  DebugResult r;
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x06;
  req[2] = reg >> 8;
  req[3] = reg & 0xFF;
  req[4] = value >> 8;
  req[5] = value & 0xFF;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;
  r.txHex = _dbgBytesToHex(req, 8);

  bool broadcast = modbusIsBroadcastAddr(slaveId);
  uint8_t resp[16];
  int n = _dbgSendReceive(req, 8, resp, sizeof(resp), timeoutMs);
  if (n > 0) r.rxHex = _dbgBytesToHex(resp, n);

  if (broadcast) {
    // Broadcast writes get no reply by design — that's success, not a
    // timeout, but it also means we have NO confirmation the write
    // actually landed anywhere. Caller should read back to check.
    r.ok = true;
    r.error = "broadcast — sent, no reply expected/possible";
    modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_OK);
    return r;
  }

  if (n < 3) { r.error = "timeout — no response"; modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_TIMEOUT); return r; }
  if (resp[1] & 0x80) {
    if (n >= 5) {
      uint16_t rxCrc = resp[3] | ((uint16_t)resp[4] << 8);
      if (rxCrc == modbusCRC(resp, 3)) { r.error = "exception response, code " + String(resp[2]); r.actualSlaveId = resp[0]; modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_BAD_RESPONSE); return r; }
    }
    r.error = "malformed exception response";
    modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_BAD_RESPONSE);
    return r;
  }
  if (n < 8) { r.error = "timeout — got " + String(n) + " bytes, echo needs 8"; modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_TIMEOUT); return r; }
  uint16_t rxCrc = resp[6] | ((uint16_t)resp[7] << 8);
  if (rxCrc != modbusCRC(resp, 6) || resp[1] != 0x06) { r.error = "bad echo response"; modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_BAD_RESPONSE); return r; }
  r.actualSlaveId = resp[0];
  r.ok = true;
  r.error = "confirmed";
  modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_OK);
  return r;
}

// Sends exact raw bytes (space-separated hex, e.g. "01 03 00 00 00 01 84 0A")
// and returns whatever comes back as hex, no interpretation at all. Also
// toggles DE around the send same as everything else. Useful for probing
// non-standard function codes or malformed-on-purpose frames.
String debugRawHexSend(const String& hexStr, int timeoutMs = 800) {
  uint8_t buf[64];
  int n = 0;
  int i = 0, len = hexStr.length();
  while (i < len && n < (int)sizeof(buf)) {
    while (i < len && hexStr[i] == ' ') i++;
    int start = i;
    while (i < len && hexStr[i] != ' ') i++;
    if (i > start) {
      buf[n++] = (uint8_t)strtoul(hexStr.substring(start, i).c_str(), nullptr, 16);
    }
  }
  if (n == 0) return "";

  uint8_t resp[64];
  int rn = _dbgSendReceive(buf, n, resp, sizeof(resp), timeoutMs);
  if (rn == 0) return "";
  return _dbgBytesToHex(resp, rn);
}

// Bus scan — same tolerant FC04-then-FC03 probe as modbus.h's
// modbusScanSlaves(), but at whatever framing debugSerialApply() has set
// (not necessarily the module's normal 8N1) — useful when hunting for a
// sensor that may have landed on a non-default parity/stop combo.
String debugScan(int maxAddr, int timeoutMs = 300) {
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;
  String out = "Scanning slave addresses 1-" + String(maxAddr) + " at "
    + String(_dbgCfg.baud) + " 8" + String(_dbgCfg.parity) + String(_dbgCfg.stopBits) + "...\n";
  int found = 0;
  for (int addr = 1; addr <= maxAddr; addr++) {
    DebugResult r4 = debugReadRegs((uint8_t)addr, 4, 0x0000, 1, timeoutMs);
    if (r4.ok) { out += "  addr " + String(addr) + " responded to FC04\n"; found++; continue; }
    DebugResult r3 = debugReadRegs((uint8_t)addr, 3, 0x0000, 1, timeoutMs);
    if (r3.ok) { out += "  addr " + String(addr) + " responded to FC03\n"; found++; }
  }
  out += "Scan done. " + String(found) + " address(es) responded.";
  return out;
}

// Passive sniff — pure receive, no TX at all, always safe regardless of
// what mode the port is in. Caller must still hold modbusBusMutex (it's
// reading the same Serial2 the poll task uses).
String debugSniffCapture(int secs) {
  String out;
  unsigned long deadline = millis() + (unsigned long)secs * 1000;
  uint8_t buf[64];
  int n = 0;
  unsigned long lastByte = 0;
  while (millis() < deadline) {
    if (_mbSerial->available()) {
      if (n < (int)sizeof(buf)) buf[n++] = _mbSerial->read();
      else _mbSerial->read();
      lastByte = millis();
    } else if (n > 0 && millis() - lastByte > 20) {
      out += "  RX: " + _dbgBytesToHex(buf, n) + "\n";
      n = 0;
    }
  }
  if (n > 0) out += "  RX: " + _dbgBytesToHex(buf, n) + "\n";
  if (out.length() == 0) out = "  (nothing captured)\n";
  return out;
}

// Baud sweep — tries one register read at 8N1 across every standard
// baud, stopping at the first clean reply. Leaves the port at whichever
// baud worked (still 8N1); restores original framing if nothing answers.
String debugBaudSweep(uint8_t slaveId, uint8_t fc, uint16_t reg, uint8_t count) {
  DebugSerialCfg orig = _dbgCfg;
  static const uint32_t bauds[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  String out = "Baud sweep: slave " + String(slaveId) + ", FC" + String(fc) + ", reg 0x" + String(reg, HEX)
    + ", count " + String(count) + ", 8N1, across standard bauds...\n";
  bool found = false;
  uint32_t foundBaud = 0;

  for (int i = 0; i < 8; i++) {
    debugSerialApply(bauds[i], 'N', 1);
    delay(30);
    DebugResult r = debugReadRegs(slaveId, fc, reg, count, 350);
    if (r.ok) {
      out += "  " + String(bauds[i]) + " baud: OK — reply from slave " + String(r.actualSlaveId) + ": ";
      for (int j = 0; j < r.regCount; j++) out += String(r.regs[j]) + " ";
      out += "\n";
      if (!found) { found = true; foundBaud = bauds[i]; }
    } else {
      out += "  " + String(bauds[i]) + " baud: " + r.error;
      if (r.rxHex.length()) out += " (raw: " + r.rxHex + ")";
      out += "\n";
    }
  }

  if (found) {
    debugSerialApply(foundBaud, 'N', 1);
    out += "\n*** Working baud found: " + String(foundBaud) + " 8N1. Port left at this baud. ***\n";
  } else {
    debugSerialApply(orig.baud, orig.parity, orig.stopBits);
    out += "\nNo baud got a clean reply at 8N1. Port restored to previous framing.\n"
      "If you're seeing garbled-but-nonzero replies at every baud, try the recovery sweep "
      "or double-check A/B polarity and power.\n";
  }
  return out;
}

// SM7779 recovery sweep — for a sensor stuck outputting garbage after
// writes to 0x0068/0x0069. Baud is left alone (9600 assumed still
// correct); only parity/stop bits are swept, using UNICAST writes to
// address 1 as the primary probe (a real ack/exception proves the
// framing matched — broadcast gives no such signal). Falls back to a
// best-effort broadcast pass if unicast gets nothing anywhere, then an
// auto-sniff so you can at least see if the sensor's own auto-burst
// pattern changed.
String debugRecoverySweep() {
  String report;
  DebugSerialCfg orig = _dbgCfg;
  const char parities[] = {'N', 'E', 'O'};
  const int stops[] = {1, 2};
  bool found = false;

  report += "--- Pass 1: unicast to address 1 (real ack = proof the framing matched) ---\n";
  for (int s = 0; s < 2 && !found; s++) {
    for (int p = 0; p < 3 && !found; p++) {
      char parity = parities[p];
      int stopBits = stops[s];
      debugSerialApply(9600, parity, stopBits);
      delay(30);

      DebugResult w1 = debugWriteReg(1, 0x0068, 1, 400);
      delay(80);
      DebugResult w2 = debugWriteReg(1, 0x0069, 1, 400);

      report += "9600 8" + String(parity) + String(stopBits) + ": 0x68 write -> "
        + (w1.ok ? "ACK (confirmed)" : w1.error) + " | 0x69 write -> "
        + (w2.ok ? "ACK (confirmed)" : w2.error) + "\n";

      if (w1.ok || w2.ok) {
        delay(300);
        debugSerialApply(9600, 'N', 1);
        delay(30);
        DebugResult r = debugReadRegs(1, 3, 0x0000, 3, 400);
        if (r.ok) {
          report += "  -> Read-back OK from slave " + String(r.actualSlaveId) + ": ";
          for (int i = 0; i < r.regCount; i++) report += String(r.regs[i]) + " ";
          report += "\n\n*** Sensor confirmed alive and acked the reset at 9600 8" + String(parity) + String(stopBits)
            + ". Left at 9600 8N1. ***\n";
        } else {
          report += "\n*** Sensor ACKed the write at 9600 8" + String(parity) + String(stopBits)
            + " (framing confirmed correct) but a follow-up read at 8N1 didn't respond - "
            + "leaving the port at 9600 8" + String(parity) + String(stopBits) + " since that's the framing that worked. ***\n";
          debugSerialApply(9600, parity, stopBits);
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
        debugSerialApply(9600, parity, stopBits);
        delay(30);
        debugWriteReg(0, 0x0068, 1, 200);
        delay(50);
        debugWriteReg(0, 0x0069, 1, 200);
        delay(50);
        debugWriteReg(250, 0x0068, 1, 200);
        delay(50);
        debugWriteReg(250, 0x0069, 1, 200);
        delay(300);

        debugSerialApply(9600, 'N', 1);
        delay(30);
        report += "9600 8" + String(parity) + String(stopBits) + " blind broadcast sent, checking 8N1 for a response... ";
        DebugResult r = debugReadRegs(1, 3, 0x0000, 3, 400);
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
    debugSerialApply(orig.baud, orig.parity, orig.stopBits);
    report += "\nNothing acked or responded at any framing/method. This means either:\n"
      "  1. The sensor's receiver isn't at 9600 in ANY of these parity/stop combos\n"
      "  2. It needs a power cycle before register writes take effect\n"
      "  3. Those registers control something other than serial framing\n"
      "Auto-sniff below shows whether the auto-burst pattern changed at all.\n";
    report += "\n--- Sniff (10s, no TX) ---\n";
    report += debugSniffCapture(10);
  }
  return report;
}

// =============================================================================
// STATIC REFERENCE DATA — documented register maps for the two sensor
// families in the field, so this info is on the page instead of buried
// in datasheets. Purely informational; the table renders this in webui.h.
// =============================================================================
struct DebugRegDoc {
  const char* reg;
  const char* name;
  const char* notes;
};

// SM7779 / XM7779 80GHz radar level sensor (Sonbest). Always replies
// FC03/3-register fixed block [distance, level, status] regardless of
// requested register/count — see modbus.h's respRegOffset mechanism.
static const DebugRegDoc SM7779_REGS[] = {
  { "0x0000", "Liquid level (air gap)", "r/o, mm. Distance from sensor to surface — goes DOWN as tank fills. reg0+reg1=3000 always." },
  { "0x0001", "Level", "r/o, mm. Second word of the fixed 3-register reply block." },
  { "0x0002", "Status", "r/o. Third word of the fixed 3-register reply block, normally 0." },
  { "0x0064", "Model code", "r/w. Device model identifier." },
  { "0x0065", "Measuring points", "r/w, 1-20. Max=20 gives ~6s measurement cycle — hardware ceiling, no faster mode." },
  { "0x0066", "Device address", "r/w, 1-249. CAUTION: breaks comms immediately — sensor stops answering at the old address." },
  { "0x0067", "Baud rate", "r/w. 1=2400 2=4800 3=9600 4=19200 5=38400 6=115200. CAUTION: breaks comms immediately." },
  { "0x0068", "Comm mode", "r/w, experimental, 1-4. WARNING: writing this corrupted a sensor on 2026-08-10. Only touch with the recovery sweep as a deliberate, isolated test." },
  { "0x0069", "Protocol type", "r/w, experimental, 1-10. Same corruption warning as 0x0068." },
  { "0x006B", "Calibration offset", "r/w. 0-1000 adds, 64535-65535 subtracts." },
};
static const int SM7779_REGS_COUNT = sizeof(SM7779_REGS) / sizeof(SM7779_REGS[0]);

// QDW90A / QDY30A / QDW50A / QDF70B pressure sensor family (Anhui Qidian).
// FC03 holding registers, 7 registers starting at 0x0000. Requires
// genuine 24V power — will not respond at 12V.
static const DebugRegDoc QDW90A_REGS[] = {
  { "0x0000", "Device address", "r/w, 1-255. Default 1 — collides with SM7779's default, watch for this on a shared bus." },
  { "0x0001", "Baud rate code", "r/w. 0=1200 1=2400 2=4800 3=9600 4=19200 5=38400 6=57600 7=115200. Default 9600." },
  { "0x0002", "Pressure unit code", "r/o or r/w depending on model. 0=none 1=CM 2=MM 3=MPa 4=Pa 5=KPa 6=mA — labels sometimes unreliable per HA community reports, verify empirically." },
  { "0x0003", "Decimal point position", "r/w. 0=#### 1=###.# 2=##.## 3=#.### — apply to register 0x0004's raw value." },
  { "0x0004", "Pressure value (live reading)", "r/o, signed. Apply the decimal code from 0x0003, e.g. code=2 raw=123 -> 1.23." },
  { "0x0005", "Zero point", "r/w. Range zero-point calibration." },
  { "0x0006", "Full-scale point", "r/w. Range full-scale calibration." },
};
static const int QDW90A_REGS_COUNT = sizeof(QDW90A_REGS) / sizeof(QDW90A_REGS[0]);

// =============================================================================
// TRAFFIC LOG TEXT — formats the ring buffer already kept by modbus.h
// (_mbLog/_mbLogHead/_mbLogCount). debugReadRegs()/debugWriteReg() above
// log into this same buffer via modbusLogTransaction() (see the calls
// added at the end of each), so /debug's log shows real production
// traffic (from the live poll task's normal sensor reads) interleaved
// with anything you do manually on this page — exactly what you want
// when hunting a bus-contention issue.
// =============================================================================
static const char* _dbgResultName(int result) {
  switch (result) {
    case MB_OK: return "OK";
    case MB_TIMEOUT: return "TIMEOUT";
    case MB_CRC_ERROR: return "CRC_ERROR";
    case MB_BAD_RESPONSE: return "BAD_RESPONSE";
    default: return "?";
  }
}

String debugLogHumanText(int n) {
  if (n > _mbLogCount) n = _mbLogCount;
  if (n <= 0) return "(no traffic logged yet)\n";
  String out;
  unsigned long nowMs = millis();
  for (int i = 0; i < n; i++) {
    int idx = (_mbLogHead - 1 - i + MODBUS_LOG_SIZE * 4) % MODBUS_LOG_SIZE;
    ModbusRawLogEntry& e = _mbLog[idx];
    unsigned long ago = nowMs - e.ms;
    out += "[-" + String(ago) + "ms] slave=" + String(e.slaveId) + " fc=" + String(e.funcCode)
      + " -> " + String(_dbgResultName(e.result))
      + "  TX: " + _dbgBytesToHex(e.tx, e.txLen);
    if (e.rxLen > 0) out += "  RX: " + _dbgBytesToHex(e.rx, e.rxLen);
    out += "\n";
  }
  return out;
}