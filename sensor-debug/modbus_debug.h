// =============================================================================
// modbus_debug.h — Raw RS485/Modbus primitives for the sensor-debug tool.
//
// Deliberately dumber and more permissive than the "real" firmware's
// modbus.h: no config persistence, no auto-detect-and-enable, no per-
// sensor scaling. Just: send bytes, read bytes, log everything, let the
// human look at the raw wire data and decide what's going on. Built after
// the SM7779 radar sensor ended up stuck outputting garbage following a
// couple of experimental register writes (0x0068 comm mode, 0x0069
// protocol type) — the working theory is those writes may have changed
// the sensor's own SERIAL framing (parity/stop bits), not just baud. The
// production firmware only ever varies baud; this tool varies parity and
// stop bits too, and lets you fire raw hex at the bus with zero Modbus
// framing assumptions at all.
// =============================================================================
#pragma once
#include <Arduino.h>

static int _dbgDePin = -1;
static HardwareSerial* _dbgSerial = nullptr;

// Current live serial config, tracked here since HardwareSerial doesn't
// expose parity/stopbits getters.
struct SerialCfg {
  uint32_t baud = 9600;
  char parity = 'N';   // 'N', 'E', 'O'
  int stopBits = 1;    // 1 or 2
};
static SerialCfg _dbgCfg;

// Maps parity+stopbits to the ESP32 core's SERIAL_8xN config constant.
// Deliberately fixed at 8 data bits — every Modbus RTU sensor in the
// field uses 8N1/8E1/8O1 (or the 2-stop-bit variants); 7-bit framing
// isn't worth the UI complexity here.
static uint32_t dbgSerialConfig(char parity, int stopBits) {
  if (parity == 'E') return (stopBits == 2) ? SERIAL_8E2 : SERIAL_8E1;
  if (parity == 'O') return (stopBits == 2) ? SERIAL_8O2 : SERIAL_8O1;
  return (stopBits == 2) ? SERIAL_8N2 : SERIAL_8N1;
}

void dbgSerialInit(int rxPin, int txPin, int dePin, uint32_t baud, char parity = 'N', int stopBits = 1) {
  _dbgDePin = dePin;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin, LOW); // receive by default

  _dbgSerial = &Serial2;
  _dbgSerial->begin(baud, dbgSerialConfig(parity, stopBits), rxPin, txPin);
  _dbgCfg.baud = baud;
  _dbgCfg.parity = parity;
  _dbgCfg.stopBits = stopBits;
  Serial.printf("[Debug] Serial2 RX=%d TX=%d DE=%d baud=%u parity=%c stop=%d\n",
    rxPin, txPin, dePin, baud, parity, stopBits);
}

// Re-applies serial config live (no persistence anywhere — this whole
// tool is meant to be reconfigured every session).
void dbgSerialApply(uint32_t baud, char parity, int stopBits) {
  if (!_dbgSerial) return;
  _dbgSerial->flush();
  _dbgSerial->end();
  delay(20);
  _dbgSerial->begin(baud, dbgSerialConfig(parity, stopBits), 18 /*RX*/, 17 /*TX*/);
  _dbgCfg.baud = baud;
  _dbgCfg.parity = parity;
  _dbgCfg.stopBits = stopBits;
  delay(20);
  Serial.printf("[Debug] Serial config applied: baud=%u parity=%c stop=%d\n", baud, parity, stopBits);
}

SerialCfg dbgGetSerialCfg() { return _dbgCfg; }

// =============================================================================
// LIVE TRAFFIC LOG — every TX/RX, raw or framed, newest first. Bigger
// buffer than the production tool since this is the whole point of this
// firmware.
// =============================================================================
struct DebugLogEntry {
  unsigned long ms;
  String label;     // e.g. "RAW", "FC03 read", "FC06 write", "SCAN"
  String tx;         // hex string
  String rx;         // hex string
  String result;     // human-readable outcome
};
#define DEBUG_LOG_SIZE 80
static DebugLogEntry _dbgLog[DEBUG_LOG_SIZE];
static int _dbgLogHead = 0;
static int _dbgLogCount = 0;

static String bytesToHex(const uint8_t* buf, int len) {
  String s;
  s.reserve(len * 3);
  for (int i = 0; i < len; i++) {
    if (i) s += ' ';
    char tmp[4];
    snprintf(tmp, sizeof(tmp), "%02X", buf[i]);
    s += tmp;
  }
  return s;
}

static void dbgLog(const String& label, const uint8_t* tx, int txLen,
                    const uint8_t* rx, int rxLen, const String& result) {
  DebugLogEntry& e = _dbgLog[_dbgLogHead];
  e.ms = millis();
  e.label = label;
  e.tx = tx ? bytesToHex(tx, txLen) : "";
  e.rx = rx ? bytesToHex(rx, rxLen) : "";
  e.result = result;
  _dbgLogHead = (_dbgLogHead + 1) % DEBUG_LOG_SIZE;
  if (_dbgLogCount < DEBUG_LOG_SIZE) _dbgLogCount++;
}

// Returns up to maxCount entries, newest first, as a JSON array string.
String dbgGetLogJson(int maxCount) {
  int n = min(maxCount, _dbgLogCount);
  String out = "[";
  for (int i = 0; i < n; i++) {
    int idx = (_dbgLogHead - 1 - i + DEBUG_LOG_SIZE * 2) % DEBUG_LOG_SIZE;
    DebugLogEntry& e = _dbgLog[idx];
    if (i) out += ",";
    out += "{\"ageMs\":" + String(millis() - e.ms) +
           ",\"label\":\"" + e.label + "\"" +
           ",\"tx\":\"" + e.tx + "\"" +
           ",\"rx\":\"" + e.rx + "\"" +
           ",\"result\":\"" + e.result + "\"}";
  }
  out += "]";
  return out;
}

// =============================================================================
// RAW WIRE ACCESS — no Modbus framing assumed at all. Sends exactly the
// bytes given, toggles DE around the transmission, and captures whatever
// comes back for up to timeoutMs (extending on each new byte by an
// inter-byte gap, same defense as the production tool's modbusFlushRx).
// =============================================================================
void dbgFlushRx(int quietMs = 8, int hardCapMs = 80) {
  if (!_dbgSerial) return;
  unsigned long quietUntil = millis() + quietMs;
  unsigned long hardDeadline = millis() + hardCapMs;
  while (millis() < quietUntil && millis() < hardDeadline) {
    if (_dbgSerial->available()) {
      _dbgSerial->read();
      quietUntil = millis() + quietMs;
    }
  }
}

int dbgRawSend(const uint8_t* tx, int txLen, uint8_t* rxBuf, int rxMaxLen, int timeoutMs) {
  if (!_dbgSerial) return 0;
  dbgFlushRx();

  digitalWrite(_dbgDePin, HIGH);
  delayMicroseconds(100);
  _dbgSerial->write(tx, txLen);
  _dbgSerial->flush();
  delayMicroseconds(100);
  digitalWrite(_dbgDePin, LOW);

  unsigned long deadline = millis() + timeoutMs;
  int n = 0;
  while (millis() < deadline && n < rxMaxLen) {
    if (_dbgSerial->available()) {
      rxBuf[n++] = _dbgSerial->read();
      deadline = millis() + 20; // inter-byte gap extends the window
    }
  }
  return n;
}

// =============================================================================
// MODBUS FRAMING — same hard-won tolerant parsing as the production
// firmware (accept FC03/FC04 replies regardless of exact funcCode
// requested, size the frame off the response's own byte-count field,
// treat 0/250 as broadcast addresses whose reply legitimately comes back
// from a different address).
// =============================================================================
static uint16_t dbgCRC(const uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

static bool dbgIsBroadcastAddr(uint8_t addr) { return addr == 0 || addr == 250; }

struct ModbusResult {
  bool ok = false;
  uint8_t actualSlaveId = 0;
  uint8_t funcCode = 0;
  uint16_t regs[16] = {0};
  int regCount = 0;
  String error;
  String txHex, rxHex;
};

// Reads registers via FC03/FC04, tolerant of sensors that reply with a
// different function code / register count than requested (confirmed
// field behavior on at least one radar level sensor) — same relaxed
// parsing as the production firmware's modbusReadRegs, but standalone
// here so this tool has zero dependency on the "real" firmware files.
ModbusResult dbgReadRegs(uint8_t slaveId, uint8_t funcCode, uint16_t startAddr, uint8_t count, int timeoutMs = 600) {
  ModbusResult r;
  if (count < 1) count = 1;
  if (count > 16) count = 16;

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = funcCode;
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  uint16_t crc = dbgCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;
  r.txHex = bytesToHex(req, 8);

  uint8_t resp[64];
  int n = dbgRawSend(req, 8, resp, sizeof(resp), timeoutMs);
  r.rxHex = n > 0 ? bytesToHex(resp, n) : "";

  if (n < 3) { r.error = "timeout (" + String(n) + " bytes)"; dbgLog("FC" + String(funcCode) + " read", req, 8, resp, n, "timeout"); return r; }

  if (resp[1] & 0x80) {
    r.error = "exception code " + String(resp[2]);
    dbgLog("FC" + String(funcCode) + " read", req, 8, resp, n, r.error);
    return r;
  }

  if (resp[1] != 0x03 && resp[1] != 0x04) {
    r.error = "unexpected function code 0x" + String(resp[1], HEX);
    dbgLog("FC" + String(funcCode) + " read", req, 8, resp, n, r.error);
    return r;
  }

  uint8_t byteCount = resp[2];
  int frameLen = 3 + byteCount + 2;
  if (frameLen > (int)sizeof(resp) || n < frameLen) {
    r.error = "short/oversized frame (got " + String(n) + ", need " + String(frameLen) + ")";
    dbgLog("FC" + String(funcCode) + " read", req, 8, resp, n, r.error);
    return r;
  }

  uint16_t rxCrc = resp[frameLen-2] | ((uint16_t)resp[frameLen-1] << 8);
  uint16_t calcCrc = dbgCRC(resp, frameLen-2);
  if (rxCrc != calcCrc) {
    r.error = "CRC mismatch (got " + String(rxCrc, HEX) + ", calc " + String(calcCrc, HEX) + ")";
    dbgLog("FC" + String(funcCode) + " read", req, 8, resp, n, r.error);
    return r;
  }

  bool addrMismatch = (resp[0] != slaveId);
  r.actualSlaveId = resp[0];
  r.funcCode = resp[1];
  if (addrMismatch && !dbgIsBroadcastAddr(slaveId)) {
    r.error = "address mismatch: asked " + String(slaveId) + ", got reply from " + String(resp[0]) + " (data below is real, just from a different address)";
    dbgLog("FC" + String(funcCode) + " read", req, 8, resp, n, r.error);
    return r;
  }

  uint8_t regsInResponse = byteCount / 2;
  int copyCount = min((int)count, (int)regsInResponse);
  for (int i = 0; i < copyCount; i++) {
    r.regs[i] = ((uint16_t)resp[3 + i*2] << 8) | resp[4 + i*2];
  }
  r.regCount = copyCount;
  r.ok = true;
  dbgLog("FC" + String(funcCode) + " read", req, 8, resp, frameLen, addrMismatch ? "ok (broadcast reply)" : "ok");
  return r;
}

// FC06 write single register.
ModbusResult dbgWriteReg(uint8_t slaveId, uint16_t regAddr, uint16_t value, int timeoutMs = 600) {
  ModbusResult r;
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x06;
  req[2] = regAddr >> 8;
  req[3] = regAddr & 0xFF;
  req[4] = value >> 8;
  req[5] = value & 0xFF;
  uint16_t crc = dbgCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;
  r.txHex = bytesToHex(req, 8);

  uint8_t resp[16];
  int n = dbgRawSend(req, 8, resp, sizeof(resp), timeoutMs);
  r.rxHex = n > 0 ? bytesToHex(resp, n) : "";

  if (dbgIsBroadcastAddr(slaveId) && n == 0) {
    // Broadcast writes may legitimately get no reply at all on some
    // sensors (standard Modbus RTU broadcast semantics: slaves must not
    // reply to address 0). Report as "sent, no reply expected/received"
    // rather than a timeout error, since that's a normal outcome here.
    r.ok = true;
    r.error = "broadcast sent, no reply (normal for some sensors)";
    dbgLog("FC06 write", req, 8, resp, n, "broadcast, no reply");
    return r;
  }

  if (n < 3) { r.error = "timeout (" + String(n) + " bytes)"; dbgLog("FC06 write", req, 8, resp, n, "timeout"); return r; }

  if (resp[1] & 0x80) {
    if (n >= 5) {
      uint16_t rxCrc = resp[3] | ((uint16_t)resp[4] << 8);
      uint16_t calcCrc = dbgCRC(resp, 3);
      if (rxCrc == calcCrc) {
        r.error = "exception code " + String(resp[2]);
        dbgLog("FC06 write", req, 8, resp, n, r.error);
        return r;
      }
    }
    r.error = "exception-shaped response, bad CRC";
    dbgLog("FC06 write", req, 8, resp, n, r.error);
    return r;
  }

  if (n < 8) { r.error = "short response (" + String(n) + " bytes)"; dbgLog("FC06 write", req, 8, resp, n, r.error); return r; }

  uint16_t rxCrc = resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t calcCrc = dbgCRC(resp, 6);
  if (rxCrc != calcCrc) {
    r.error = "CRC mismatch";
    dbgLog("FC06 write", req, 8, resp, n, r.error);
    return r;
  }

  bool addrMismatch = (resp[0] != slaveId);
  r.actualSlaveId = resp[0];
  if (addrMismatch && !dbgIsBroadcastAddr(slaveId)) {
    r.error = "address mismatch: asked " + String(slaveId) + ", got reply from " + String(resp[0]);
    dbgLog("FC06 write", req, 8, resp, n, r.error);
    return r;
  }
  if (resp[1] != 0x06) {
    r.error = "unexpected function code in reply: 0x" + String(resp[1], HEX);
    dbgLog("FC06 write", req, 8, resp, n, r.error);
    return r;
  }

  r.ok = true;
  dbgLog("FC06 write", req, 8, resp, n, addrMismatch ? "ok (broadcast reply)" : "ok");
  return r;
}

// Sends raw hex bytes verbatim, no Modbus framing/CRC assumed at all —
// for when you don't even know if the sensor speaks Modbus RTU, or want
// to hand-craft a frame the built-in tools don't cover.
String dbgRawHexSend(const String& hexIn, int timeoutMs) {
  uint8_t tx[64];
  int txLen = 0;
  String cleaned = hexIn;
  cleaned.replace(",", " ");
  int i = 0;
  while (i < (int)cleaned.length() && txLen < (int)sizeof(tx)) {
    while (i < (int)cleaned.length() && cleaned[i] == ' ') i++;
    if (i >= (int)cleaned.length()) break;
    String byteStr = "";
    while (i < (int)cleaned.length() && cleaned[i] != ' ' && byteStr.length() < 2) {
      byteStr += cleaned[i];
      i++;
    }
    if (byteStr.length() == 0) break;
    tx[txLen++] = (uint8_t)strtol(byteStr.c_str(), nullptr, 16);
  }

  uint8_t resp[64];
  int n = dbgRawSend(tx, txLen, resp, sizeof(resp), timeoutMs);
  String rxHex = n > 0 ? bytesToHex(resp, n) : "";
  dbgLog("RAW", tx, txLen, resp, n, n > 0 ? (String(n) + " bytes back") : "no reply");
  return rxHex;
}

// Scans slave addresses 1..maxAddr with a basic FC03/FC04 register-0
// probe at whatever baud/parity/stopbits is currently active. Returns a
// JSON array of hits: [{"addr":1,"fc":3}, ...].
String dbgScanBus(int maxAddr, int timeoutMs = 400) {
  String out = "[";
  bool first = true;
  for (int addr = 1; addr <= maxAddr; addr++) {
    ModbusResult r4 = dbgReadRegs((uint8_t)addr, 4, 0x0000, 1, timeoutMs);
    int fc = 0;
    if (r4.ok) fc = 4;
    else {
      ModbusResult r3 = dbgReadRegs((uint8_t)addr, 3, 0x0000, 1, timeoutMs);
      if (r3.ok) fc = 3;
    }
    if (fc) {
      if (!first) out += ",";
      out += "{\"addr\":" + String(addr) + ",\"fc\":" + String(fc) + "}";
      first = false;
    }
  }
  out += "]";
  return out;
}
