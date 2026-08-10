// =============================================================================
// modbus.h — Generic Modbus RTU MASTER over HardwareSerial (RS485)
//
// Unlike the adapter-board variant (waveshare-s3/), this talks to whatever
// slave ID / register / function code a SensorConfig says to — no fixed
// board assumption at all. Direct Modbus RTU sensors (pressure, temp,
// whatever) each get their own slave address on the bus.
//
// Uses direct HardwareSerial with DE/RE pin toggling — no external Modbus
// library needed, avoids DE timing issues (same approach proven on the
// LilyGo + adapter-board Waveshare variants).
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

static int _RS485_DE_PIN = -1;
static HardwareSerial* _mbSerial = nullptr;

// CRC16 for Modbus RTU
static uint16_t modbusCRC(const uint8_t* buf, int len) {
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

// Init RS485. baud is configurable from the webUI since different sensors
// ship with different factory-default baud rates.
void modbusInit(int rxPin, int txPin, int dePin, uint32_t baud) {
  _RS485_DE_PIN = dePin;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin, LOW); // receive mode by default

  _mbSerial = &Serial2;
  _mbSerial->begin(baud, SERIAL_8N1, rxPin, txPin);
  Serial.printf("[Modbus] Init on Serial2 RX=%d TX=%d DE=%d baud=%u\n", rxPin, txPin, dePin, baud);
}

// Send bytes, toggle DE high during TX
static void modbusSend(const uint8_t* buf, int len, bool verbose) {
  if (verbose) {
    Serial.printf("[RS485] TX (%d bytes):", len);
    for (int i = 0; i < len; i++) Serial.printf(" %02X", buf[i]);
    Serial.println();
  }

  digitalWrite(_RS485_DE_PIN, HIGH);
  delayMicroseconds(100); // DE propagation delay
  _mbSerial->write(buf, len);
  _mbSerial->flush(); // wait for TX to complete
  delayMicroseconds(100);
  digitalWrite(_RS485_DE_PIN, LOW); // back to receive
}

// Read response with timeout (ms)
static int modbusReceive(uint8_t* buf, int maxLen, int timeoutMs, bool verbose) {
  unsigned long deadline = millis() + timeoutMs;
  int n = 0;
  while (millis() < deadline && n < maxLen) {
    if (_mbSerial->available()) {
      buf[n++] = _mbSerial->read();
      deadline = millis() + 20; // inter-byte timeout 20ms
    }
  }
  if (verbose) {
    if (n > 0) {
      Serial.printf("[RS485] RX (%d bytes):", n);
      for (int i = 0; i < n; i++) Serial.printf(" %02X", buf[i]);
      Serial.println();
    } else {
      Serial.println("[RS485] RX: no bytes received");
    }
  }
  return n;
}

// Generic register read — works for FC03 (Read Holding Registers) or
// FC04 (Read Input Registers), any slave, any start address, any count
// (1-16 registers is plenty for any sensor we'll meet; capped at 16 to
// keep the response buffer small).
// Returns a status code: 0=ok, 1=timeout, 2=crc error, 3=bad response
static const int MB_OK = 0, MB_TIMEOUT = 1, MB_CRC_ERROR = 2, MB_BAD_RESPONSE = 3;

// =============================================================================
// RAW TRAFFIC LOG — every request/response byte, for the web UI's live
// "RS485 Raw Traffic" panel (/diag). Unlike Serial Monitor output, this
// captures EVERY transaction, including normal background polling, not
// just manual probes — so you can watch what a sensor is actually saying
// without a USB cable plugged in.
// =============================================================================
struct ModbusRawLogEntry {
  unsigned long ms;
  uint8_t slaveId;
  uint8_t funcCode;
  uint8_t txLen;
  uint8_t tx[8];
  uint8_t rxLen;
  uint8_t rx[40];
  int result; // MB_OK / MB_TIMEOUT / MB_CRC_ERROR / MB_BAD_RESPONSE
};
#define MODBUS_LOG_SIZE 40
static ModbusRawLogEntry _mbLog[MODBUS_LOG_SIZE];
static int _mbLogHead = 0;
static int _mbLogCount = 0;

static void modbusLogTransaction(uint8_t slaveId, uint8_t funcCode,
                                  const uint8_t* tx, int txLen,
                                  const uint8_t* rx, int rxLen, int result) {
  ModbusRawLogEntry& e = _mbLog[_mbLogHead];
  e.ms = millis();
  e.slaveId = slaveId;
  e.funcCode = funcCode;
  e.txLen = (uint8_t)min(txLen, 8);
  memcpy(e.tx, tx, e.txLen);
  e.rxLen = (uint8_t)min(rxLen, 40);
  memcpy(e.rx, rx, e.rxLen);
  e.result = result;
  _mbLogHead = (_mbLogHead + 1) % MODBUS_LOG_SIZE;
  if (_mbLogCount < MODBUS_LOG_SIZE) _mbLogCount++;
}

// Copies the most recent N transactions out for JSON serialization,
// newest first. Returns how many were actually copied.
int modbusGetRecentLog(ModbusRawLogEntry* out, int maxCount) {
  int n = min(maxCount, _mbLogCount);
  for (int i = 0; i < n; i++) {
    int idx = (_mbLogHead - 1 - i + MODBUS_LOG_SIZE * 2) % MODBUS_LOG_SIZE;
    out[i] = _mbLog[idx];
  }
  return n;
}

// Drains any bytes sitting in the RX buffer, but doesn't stop at the
// first empty check — waits until the bus has been quiet for a few ms
// before returning. A one-shot "while(available()) read()" can miss the
// tail end of a PREVIOUS response that's still trickling in (slow
// sensor + fixed timeout can leave late bytes arriving just as the next
// query is about to fire) — those leftover bytes would otherwise get
// misread as the response to the new query, corrupting it and failing
// CRC even though the wiring/baud are actually fine. Capped so a noisy/
// continuously-chattering bus can't hang this forever.
static void modbusFlushRx() {
  unsigned long quietUntil = millis() + 8;
  unsigned long hardDeadline = millis() + 80;
  while (millis() < quietUntil && millis() < hardDeadline) {
    if (_mbSerial->available()) {
      _mbSerial->read();
      quietUntil = millis() + 8; // saw a byte, reset the quiet timer
    }
  }
}

int modbusReadRegs(uint8_t slaveId, uint8_t funcCode, uint16_t startAddr,
                    uint8_t count, uint16_t* regValues, bool verbose = false,
                    int timeoutMs = 600) {
  if (!_mbSerial) return MB_TIMEOUT;
  if (count < 1) count = 1;
  if (count > 16) count = 16;

  modbusFlushRx(); // drain any late bytes left over from a previous, slower-than-expected response

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = funcCode; // 3 or 4
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  modbusSend(req, 8, verbose);

  // IMPORTANT: don't size the read off of what WE asked for. Some real
  // sensors (confirmed in the field: an RS485 radar level sensor) don't
  // echo back the function code / register count we requested at all —
  // they just always answer with whatever fixed register block they
  // have (e.g. we ask FC04/1 register, they reply FC03/3 registers,
  // every single time, regardless of the query). Sizing the read to our
  // own request truncated their longer replies and CRC-checked the
  // wrong slice — that's what caused "constant CRC errors" even though
  // the sensor's actual replies were perfectly well-formed. Read up to
  // the full buffer instead and use the RESPONSE's own byte-count field
  // (byte index 2 of any FC03/FC04 reply) to figure out the real frame
  // length after the fact.
  uint8_t resp[40];
  int n = modbusReceive(resp, sizeof(resp), timeoutMs, verbose);

  if (n < 3) {
    if (verbose) Serial.printf("[Modbus] Timeout: got %d bytes (need >=3 for a header)\n", n);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_TIMEOUT);
    return MB_TIMEOUT;
  }

  // Exception response: slave, (funcCode|0x80), exceptionCode, CRC(2) — fixed 5 bytes
  if (resp[1] & 0x80) {
    if (n < 5) {
      modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_TIMEOUT);
      return MB_TIMEOUT;
    }
    uint16_t rxCrc = resp[3] | ((uint16_t)resp[4] << 8);
    uint16_t calcCrc = modbusCRC(resp, 3);
    if (rxCrc != calcCrc) {
      modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_CRC_ERROR);
      return MB_CRC_ERROR;
    }
    if (verbose) Serial.printf("[Modbus] Exception response: code %02X\n", resp[2]);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }

  // Only FC03/FC04 are read-register replies we know how to parse.
  if (resp[1] != 0x03 && resp[1] != 0x04) {
    if (verbose) Serial.printf("[Modbus] Bad response: unexpected function code %02X\n", resp[1]);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }

  uint8_t byteCount = resp[2];
  int frameLen = 3 + byteCount + 2;
  if (frameLen > (int)sizeof(resp)) {
    if (verbose) Serial.printf("[Modbus] Bad response: byte count %d implies an oversized frame\n", byteCount);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }
  if (n < frameLen) {
    if (verbose) Serial.printf("[Modbus] Timeout: got %d bytes, frame needs %d (byteCount=%d)\n", n, frameLen, byteCount);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_TIMEOUT);
    return MB_TIMEOUT;
  }

  uint16_t rxCrc = resp[frameLen-2] | ((uint16_t)resp[frameLen-1] << 8);
  uint16_t calcCrc = modbusCRC(resp, frameLen-2);
  if (rxCrc != calcCrc) {
    if (verbose) Serial.printf("[Modbus] CRC error: got %04X, calc %04X\n", rxCrc, calcCrc);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_CRC_ERROR);
    return MB_CRC_ERROR;
  }

  if (resp[0] != slaveId) {
    if (verbose) Serial.printf("[Modbus] Bad response: slave=%02X (expected %02X)\n", resp[0], slaveId);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }
  // Deliberately NOT checking resp[1] == funcCode here — some sensors
  // (see comment above modbusReceive call) reply with a different
  // function code than what was requested but otherwise valid,
  // CRC-correct data. We only care that it was SOME valid read-register
  // reply, already confirmed above (0x03 or 0x04, no exception bit).

  uint8_t regsInResponse = byteCount / 2;
  if (regsInResponse < count) {
    // Sensor sent back fewer registers than we asked for — genuinely
    // can't satisfy the request, unlike the "sent more than asked"
    // case which we just take the front slice of below.
    if (verbose) Serial.printf("[Modbus] Bad response: got %d registers, need %d\n", regsInResponse, count);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }

  // regValues[] is sized by the CALLER for `count` registers — even if
  // the sensor sent back more (e.g. always answers with a fixed 3-reg
  // block regardless of what was requested), only copy out `count` of
  // them so we never write past the caller's buffer.
  for (int i = 0; i < count; i++) {
    regValues[i] = ((uint16_t)resp[3 + i*2] << 8) | resp[4 + i*2];
  }
  modbusLogTransaction(slaveId, funcCode, req, 8, resp, frameLen, MB_OK);
  return MB_OK;
}

// Writes a single 16-bit register (FC06 - Write Single Register). Used
// for sensor-side config registers that a datasheet documents but our
// firmware has no dedicated UI for (e.g. a "fast measurement mode"
// switch, a response-time/filter setting, a range/blind-zone value).
// Standard Modbus RTU: request and a well-formed reply both echo back
// the register address + value written, so success is "got back exactly
// what we sent, CRC-correct" — nothing to decode.
// Returns MB_OK/MB_TIMEOUT/MB_CRC_ERROR/MB_BAD_RESPONSE, same codes as
// modbusReadRegs.
int modbusWriteReg(uint8_t slaveId, uint16_t regAddr, uint16_t value,
                    bool verbose = false, int timeoutMs = 600) {
  if (!_mbSerial) return MB_TIMEOUT;

  modbusFlushRx(); // same late-byte defense as modbusReadRegs

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x06; // Write Single Register
  req[2] = regAddr >> 8;
  req[3] = regAddr & 0xFF;
  req[4] = value >> 8;
  req[5] = value & 0xFF;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  modbusSend(req, 8, verbose);

  uint8_t resp[16];
  int n = modbusReceive(resp, sizeof(resp), timeoutMs, verbose);

  if (n < 3) {
    if (verbose) Serial.printf("[Modbus] Write timeout: got %d bytes\n", n);
    modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_TIMEOUT);
    return MB_TIMEOUT;
  }

  // Exception response: slaveId, funcCode|0x80, exceptionCode, CRC(2) = 5 bytes.
  // Unlike modbusReadRegs, this used to only return early when the
  // exception frame's own CRC checked out — a CRC mismatch fell through
  // into the normal-response parsing below instead of being reported as
  // an error, a small but real gap versus modbusReadRegs' equivalent
  // check. Now matches that pattern: any CRC failure on what looks like
  // an exception frame is reported immediately, not silently ignored.
  if ((resp[1] & 0x80) && n >= 5) {
    uint16_t rxCrc = resp[3] | ((uint16_t)resp[4] << 8);
    uint16_t calcCrc = modbusCRC(resp, 3);
    if (rxCrc == calcCrc) {
      if (verbose) Serial.printf("[Modbus] Write exception response: code %02X\n", resp[2]);
      modbusLogTransaction(slaveId, 0x06, req, 8, resp, 5, MB_BAD_RESPONSE);
      return MB_BAD_RESPONSE;
    }
    if (verbose) Serial.printf("[Modbus] Write: exception-shaped response failed CRC (got %04X, calc %04X)\n", rxCrc, calcCrc);
    modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_CRC_ERROR);
    return MB_CRC_ERROR;
  }

  // Normal FC06 reply is always exactly 8 bytes: slaveId, 0x06, regHi,
  // regLo, valHi, valLo, CRC(2) — same length as the request.
  if (n < 8) {
    if (verbose) Serial.printf("[Modbus] Write: short response (%d bytes)\n", n);
    modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }

  uint16_t rxCrc = resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t calcCrc = modbusCRC(resp, 6);
  if (rxCrc != calcCrc) {
    if (verbose) Serial.printf("[Modbus] Write CRC error: got %04X, calc %04X\n", rxCrc, calcCrc);
    modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_CRC_ERROR);
    return MB_CRC_ERROR;
  }

  if (resp[0] != slaveId || resp[1] != 0x06) {
    if (verbose) Serial.printf("[Modbus] Write: bad response slave=%02X fc=%02X\n", resp[0], resp[1]);
    modbusLogTransaction(slaveId, 0x06, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }

  modbusLogTransaction(slaveId, 0x06, req, 8, resp, 8, MB_OK);
  return MB_OK;
}

// Decodes 1 or 2 raw registers into a float per the sensor's configured
// data type + word order. This is the whole point of making sensors
// generic: any Modbus sensor is "read N registers, interpret as type X".
float modbusDecodeValue(uint16_t* regs, uint8_t dataType, uint8_t wordOrder) {
  switch (dataType) {
    case MB_UINT16:
      return (float)regs[0];
    case MB_INT16:
      return (float)(int16_t)regs[0];
    case MB_UINT32: {
      uint32_t hi = (wordOrder == MB_WORD_HIGH_FIRST) ? regs[0] : regs[1];
      uint32_t lo = (wordOrder == MB_WORD_HIGH_FIRST) ? regs[1] : regs[0];
      uint32_t v = (hi << 16) | lo;
      return (float)v;
    }
    case MB_INT32: {
      uint32_t hi = (wordOrder == MB_WORD_HIGH_FIRST) ? regs[0] : regs[1];
      uint32_t lo = (wordOrder == MB_WORD_HIGH_FIRST) ? regs[1] : regs[0];
      int32_t v = (int32_t)((hi << 16) | lo);
      return (float)v;
    }
    case MB_FLOAT32: {
      uint32_t hi = (wordOrder == MB_WORD_HIGH_FIRST) ? regs[0] : regs[1];
      uint32_t lo = (wordOrder == MB_WORD_HIGH_FIRST) ? regs[1] : regs[0];
      uint32_t bits = (hi << 16) | lo;
      float f;
      memcpy(&f, &bits, sizeof(f));
      return f;
    }
    default:
      return (float)regs[0];
  }
}

// How many registers a given data type spans (1 for 16-bit types, 2 for
// 32-bit types) — used both for the actual poll and for the diagnostics
// "probe register" tool on the web UI.
uint8_t modbusRegCount(uint8_t dataType) {
  switch (dataType) {
    case MB_UINT32: case MB_INT32: case MB_FLOAT32: return 2;
    default: return 1;
  }
}

// Convenience: poll one SensorConfig fully — read the right number of
// registers, decode per its data type/word order, apply scale+offset.
// Returns MB_OK/MB_TIMEOUT/MB_CRC_ERROR/MB_BAD_RESPONSE; on MB_OK, fills
// rawOut (decoded value before scale/offset) and valueOut (after).
int modbusPollSensor(SensorConfig& s, float& rawOut, float& valueOut, bool verbose = false) {
  uint8_t n = modbusRegCount(s.dataType);
  uint16_t regs[2] = {0, 0};
  int rc = modbusReadRegs(s.slaveId, s.funcCode, s.regAddr, n, regs, verbose);
  if (rc != MB_OK) return rc;
  float raw = modbusDecodeValue(regs, s.dataType, s.wordOrder);
  rawOut = raw;
  valueOut = raw * s.scale + s.offset;
  return MB_OK;
}

// =============================================================================
// BAUD RATE AUTO-DETECTION / BUS SCAN (diagnostics)
//
// With direct sensors, there's no single "board" to auto-ID — different
// sensors on the same bus can be different models/brands entirely. What IS
// useful: a bus-wide slave scan at the configured baud, so a new sensor's
// slave ID can be confirmed without a laptop + separate Modbus tool.
// probeSlaves() tries FC03 register 0 (1 register) against every address
// 1-247 in turn and reports which ones actually answer (with a valid CRC)
// — that's "something is alive at address N", the fastest way to sanity-
// check a fresh sensor before writing config
// =============================================================================
static const uint32_t MODBUS_AUTODETECT_BAUDS[] = {
  9600, 19200, 4800, 38400, 2400, 57600, 1200, 115200
};
static const int MODBUS_AUTODETECT_BAUDS_COUNT =
  sizeof(MODBUS_AUTODETECT_BAUDS) / sizeof(MODBUS_AUTODETECT_BAUDS[0]);

// Tries each standard baud against a specific slave ID + register (FC03
// then FC04), used by the "Auto-Detect Baud" button while adding/editing
// one sensor. Restores originalBaud if nothing answers. Returns the
// working baud, or -1 if none found.
long modbusAutoDetectBaud(uint8_t slaveId, uint32_t originalBaud) {
  if (!_mbSerial) return -1;
  Serial.println("[Modbus] ---- Auto-detecting baud rate ----");
  for (int i = 0; i < MODBUS_AUTODETECT_BAUDS_COUNT; i++) {
    uint32_t tryBaud = MODBUS_AUTODETECT_BAUDS[i];
    Serial.printf("[Modbus]   Trying %u baud...\n", tryBaud);
    _mbSerial->flush();
    _mbSerial->updateBaudRate(tryBaud);
    delay(20);

    uint16_t regs[1];
    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
      ok = (modbusReadRegs(slaveId, 4, 0x0000, 1, regs) == MB_OK) ||
           (modbusReadRegs(slaveId, 3, 0x0000, 1, regs) == MB_OK);
    }
    if (ok) {
      Serial.printf("[Modbus] Auto-detect SUCCESS at %u baud\n", tryBaud);
      return (long)tryBaud;
    }
  }
  Serial.println("[Modbus] Auto-detect found nothing at any baud — restoring original");
  _mbSerial->flush();
  _mbSerial->updateBaudRate(originalBaud);
  delay(20);
  return -1;
}

// Scans slave addresses 1..maxAddr for anything that responds to a basic
// FC04 (then FC03) probe of register 0. Calls onFound(addr, funcCode) for
// each hit, funcCode being whichever (4 or 3) actually got a valid reply.
// This is a synchronous, blocking scan (a few hundred ms per address at
// worst) — the caller (web handler) should only invoke it on demand, not
// from the poll loop, and should hold modbusBusMutex for the whole call.
//
// timeoutMs per probe defaults to 400 — some sensors (radar/ultrasonic
// level sensors especially) take noticeably longer than a simple
// pressure/temp transducer to answer a query. Too short a timeout here
// doesn't just slow the scan down, it can cause outright MISSED
// detections (sensor's real answer arrives after we've already given up
// and moved to the next address) — worth the extra time per address.
template<typename FoundFn>
void modbusScanSlaves(int maxAddr, FoundFn onFound, int timeoutMs = 400) {
  uint16_t regs[1];
  for (int addr = 1; addr <= maxAddr; addr++) {
    if (modbusReadRegs((uint8_t)addr, 4, 0x0000, 1, regs, false, timeoutMs) == MB_OK) {
      onFound(addr, 4);
    } else if (modbusReadRegs((uint8_t)addr, 3, 0x0000, 1, regs, false, timeoutMs) == MB_OK) {
      onFound(addr, 3);
    }
  }
}

// =============================================================================
// AUTO-DETECT & ENABLE — scans the bus and automatically fills in/enables
// sensor config slots for any slave that responds and isn't already
// configured. This is what makes "just wire up a sensor and it shows up"
// work without visiting the web UI at all. New slots get conservative
// defaults (func code = whichever answered, register 0, uint16, scale 1)
// — good enough to prove the sensor is alive; the actual register/type/
// scale for a real reading still needs to be dialed in by hand (every
// sensor's register map is different), but this closes the "nothing shows
// up until I use the diagnostics page" gap entirely.
//
// Also tries other standard baud rates if the current one finds nothing
// AND no sensor is enabled yet — see modbusAutoDetectAndEnable() below for
// why that condition matters (shared-bus baud constraint).
// =============================================================================

// Fast path: scan addresses 1..maxAddr at whatever baud is CURRENTLY
// active and fill in/enable slots for new hits. Does not touch any slot
// that's already enabled (so on-purpose disabled sensors that still
// happen to be wired up and answering aren't silently re-enabled/reset).
// Returns how many new slots were filled.
static int _mbScanAndFillAtCurrentBaud(ModuleConfig& cfg, int maxAddr) {
  // Which slave IDs are already configured (enabled or not) so we don't
  // double-assign the same physical sensor to two slots on repeat scans.
  bool alreadyConfigured[248] = { false };
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (cfg.sensors[i].enabled) {
      uint8_t sid = cfg.sensors[i].slaveId;
      if (sid <= 247) alreadyConfigured[sid] = true;
    }
  }

  int newCount = 0;
  modbusScanSlaves(maxAddr, [&](int addr, int fc) {
    if (addr < 1 || addr > 247) return;
    if (alreadyConfigured[addr]) return; // already have a slot for this slave

    // Find the first free (disabled) slot.
    int slot = -1;
    for (int i = 0; i < MAX_SENSORS; i++) {
      if (!cfg.sensors[i].enabled) { slot = i; break; }
    }
    if (slot < 0) return; // no free slots left, nothing more we can do

    SensorConfig& s = cfg.sensors[slot];
    s.enabled   = true;
    s.name      = "Sensor " + String(addr) + " (auto)";
    s.kind      = "";
    s.unit      = "";
    s.slaveId   = (uint8_t)addr;
    s.funcCode  = (uint8_t)fc;
    s.regAddr   = 0;
    s.dataType  = MB_UINT16;
    s.wordOrder = MB_WORD_HIGH_FIRST;
    s.scale     = 1.0f;
    s.offset    = 0.0f;
    s.decimals  = 2;

    alreadyConfigured[addr] = true;
    newCount++;
    Serial.printf("[AutoDetect] New sensor found: slave=%d fc=%d baud=%u -> slot %d (enabled, needs register/type tuning)\n",
      addr, fc, _mbSerial ? _mbSerial->baudRate() : 0, slot);
  });

  return newCount;
}

// Full auto-detect: scans the CURRENT baud first (the common case — an
// existing bus with sensors already talking at the configured rate, plus
// a freshly-added sensor that happens to match). If that finds nothing
// AND no sensor is enabled yet at all, it's reasonable to assume the bus
// itself might just be at a different baud than the default 9600 — so it
// tries every other standard baud too, and if one of THOSE finds
// something, adopts that baud as the module's new modbusBaud (persisted
// by the caller) since every sensor on a shared RS485 bus must run the
// same baud anyway.
//
// Once at least one sensor is enabled, the module's baud is committed —
// we do NOT keep hopping bauds looking for stray sensors at a different
// rate, since that would require switching the whole bus's baud (and
// with it, silence the sensor(s) already configured and working). Add
// a second sensor at a different baud on a genuinely different physical
// bus? Not supported by this variant — one shared Serial2, one baud.
//
// cfg.modbusBaud is updated in place if a different baud is adopted;
// caller is responsible for persisting config afterward (matches the
// existing convention used by modbusAutoDetectBaud()/handleConfig()).
// Caller must hold modbusBusMutex for the whole call. maxAddr bounds how
// many slave addresses get probed per baud attempt (keeps worst-case scan
// time sane — every extra baud tried multiplies the cost by ~maxAddr*2
// timeouts if the bus is genuinely empty).
int modbusAutoDetectAndEnable(ModuleConfig& cfg, int maxAddr = 16) {
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;

  int newCount = _mbScanAndFillAtCurrentBaud(cfg, maxAddr);
  if (newCount > 0) return newCount;

  // Nothing at the current baud. Only worth trying other bauds if we
  // don't already have sensors relying on this one, AND the user hasn't
  // explicitly locked the baud in via /config or the per-sensor
  // Auto-Detect Baud button. Without this second check, a noisy/
  // colliding bus (e.g. two sensors sharing a slave address, or any
  // other source of garbled traffic) could produce a false-positive CRC
  // match at some other baud during a routine background scan and
  // silently overwrite a baud the user just deliberately set — that bug
  // bit hard on 2026-08-10 (manual baud fix kept reverting because every
  // reboot + periodic scan re-ran the full baud hunt regardless).
  bool anyEnabled = false;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (cfg.sensors[i].enabled) { anyEnabled = true; break; }
  }
  if (anyEnabled || cfg.baudManuallySet || !_mbSerial) return 0;

  uint32_t originalBaud = (uint32_t)cfg.modbusBaud;
  for (int i = 0; i < MODBUS_AUTODETECT_BAUDS_COUNT; i++) {
    uint32_t tryBaud = MODBUS_AUTODETECT_BAUDS[i];
    if (tryBaud == originalBaud) continue; // already tried above
    Serial.printf("[AutoDetect] Nothing at %u baud, trying %u...\n", originalBaud, tryBaud);
    _mbSerial->flush();
    _mbSerial->updateBaudRate(tryBaud);
    delay(20);

    newCount = _mbScanAndFillAtCurrentBaud(cfg, maxAddr);
    if (newCount > 0) {
      Serial.printf("[AutoDetect] Found sensor(s) at %u baud — adopting as module baud rate\n", tryBaud);
      cfg.modbusBaud = (long)tryBaud;
      return newCount;
    }
  }

  // Found nothing anywhere — restore the original baud so we don't leave
  // the bus configured at some random rate from the last failed attempt.
  Serial.println("[AutoDetect] No sensors found at any standard baud — restoring original");
  _mbSerial->flush();
  _mbSerial->updateBaudRate(originalBaud);
  delay(20);
  return 0;
}