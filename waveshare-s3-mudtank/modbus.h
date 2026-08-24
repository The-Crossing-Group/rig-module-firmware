// =============================================================================
// modbus.h — Modbus RTU over HardwareSerial (RS485), merged for the
// combined mudtank variant.
//
// This file provides BOTH:
//   1) A GENERIC register read (modbusReadRegs) that works for any slave/
//      register/function code — the foundation for independent RS485
//      sensors (SensorConfig) AND for the fixed adapter board convenience
//      wrappers below, which are now built on top of it rather than a
//      separate FC04-only implementation. This unifies what used to be
//      two different Modbus client implementations (waveshare-s3/ used a
//      simpler FC04-only reader; waveshare-s3-sensors/ used a flexible
//      FC03/FC04 reader with response-length-from-reply-byte-count
//      handling) into the one that's more robust — every fixed-board read
//      now also benefits from the same "read up to the full buffer and
//      trust the reply's own byte count" logic that fixed the SM7779
//      radar sensor's CRC errors.
//   2) BoardProfile auto-detection (Product ID register 0x00F7) for the
//      fixed adapter board(s), and modbusScanSlaves()/modbusAutoDetectAndEnable()
//      for independent sensors — both coexist since they serve different
//      purposes (one fixed board profile vs. N arbitrary sensor slots).
//
// NOTE: There is deliberately no modbusWriteReg()/FC06 write support
// anywhere in this file (HARD RULE, see MEMORY.md) — writing to a
// sensor's own config registers corrupted the SM7779 radar sensor's
// internal state during 2026-08-10/11 debugging. FC16 (write multiple
// holding registers) IS retained for the fixed board's mode-3 write
// (Waveshare-specific channel mode convention) — that's a documented,
// safe, standard convention for that specific board, not an experimental
// write to an unknown sensor's config space.
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

// Init RS485. baud is configurable from the webUI since different boards/
// sensors ship with different factory-default baud rates.
void modbusInit(int rxPin, int txPin, int dePin, uint32_t baud) {
  _RS485_DE_PIN = dePin;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin, LOW); // receive mode by default

  _mbSerial = &Serial2;
  _mbSerial->begin(baud, SERIAL_8N1, rxPin, txPin);
  Serial.printf("[Modbus] Init on Serial2 RX=%d TX=%d DE=%d baud=%u\n", rxPin, txPin, dePin, baud);
}

// Send bytes, toggle DE high during TX. `verbose` prints the hex dump;
// callers that hammer this at high rate (nothing currently does in this
// merged variant, but kept for API parity) can pass false.
static void modbusSend(const uint8_t* buf, int len, bool verbose = true) {
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

// Read response with timeout (ms).
static int modbusReceive(uint8_t* buf, int maxLen, int timeoutMs, bool verbose = true) {
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

// Status codes for modbusReadRegs()
static const int MB_OK = 0, MB_TIMEOUT = 1, MB_CRC_ERROR = 2, MB_BAD_RESPONSE = 3;

// =============================================================================
// RAW TRAFFIC LOG — every request/response byte, kept in a small ring
// buffer. Retained internally only (no web UI page currently reads it
// back out in this variant — the standalone sensor-debug tools cover deep
// RS485 diagnostics separately). Recording it here is harmless (SRAM
// only, never written back to a sensor).
// =============================================================================
struct ModbusRawLogEntry {
  unsigned long ms;
  uint8_t slaveId;
  uint8_t funcCode;
  uint8_t txLen;
  uint8_t tx[8];
  uint8_t rxLen;
  uint8_t rx[40];
  int result;
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

// Drains any bytes sitting in the RX buffer, but waits until the bus has
// been quiet for a few ms before returning — a one-shot drain can miss
// the tail end of a PREVIOUS response still trickling in (slow sensor +
// fixed timeout), which would otherwise get misread as the response to
// the NEXT query and fail CRC even though wiring/baud are fine. Capped so
// a noisy/chattering bus can't hang this forever.
static void modbusFlushRx() {
  unsigned long quietUntil = millis() + 8;
  unsigned long hardDeadline = millis() + 80;
  while (millis() < quietUntil && millis() < hardDeadline) {
    if (_mbSerial->available()) {
      _mbSerial->read();
      quietUntil = millis() + 8;
    }
  }
}

// Modbus broadcast addresses: standard RTU broadcast is 0, but several
// real sensors (SM7779 confirmed) instead use 0xFA/250 as their broadcast
// listen address. Treat both as "no real reply address is expected back."
static bool modbusIsBroadcastAddr(uint8_t addr) { return addr == 0 || addr == 250; }

// Generic register read — works for FC03 (Read Holding Registers) or
// FC04 (Read Input Registers), any slave, any start address, any count
// (1-16 registers, capped to keep the response buffer small).
int modbusReadRegs(uint8_t slaveId, uint8_t funcCode, uint16_t startAddr,
                    uint8_t count, uint16_t* regValues, bool verbose = false,
                    int timeoutMs = 600, uint8_t* actualSlaveIdOut = nullptr,
                    // How many registers into the SLAVE'S REPLY to skip
                    // before extracting `count` values — needed for
                    // sensors (confirmed: SM7779 radar) that always answer
                    // with the same fixed multi-register block regardless
                    // of startAddr. Defaults to 0 = take the front of the
                    // reply (matches every "normal" sensor's behavior).
                    uint8_t respRegOffset = 0) {
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
  // have. Sizing the read to our own request truncated their longer
  // replies and CRC-checked the wrong slice. Read up to the full buffer
  // instead and use the RESPONSE's own byte-count field (byte index 2 of
  // any FC03/FC04 reply) to figure out the real frame length after the fact.
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

  // Address mismatch is only fatal for a NON-broadcast query — a
  // broadcast query is SUPPOSED to get a reply from the slave's own
  // real address, not the broadcast address, and a sensor whose address
  // drifted from what we assumed should still surface its real reading
  // (with the actual address reported via actualSlaveIdOut) rather than
  // being thrown away and mislabeled MB_BAD_RESPONSE.
  bool addrMismatch = (resp[0] != slaveId);
  if (actualSlaveIdOut) *actualSlaveIdOut = resp[0];
  if (addrMismatch && !modbusIsBroadcastAddr(slaveId)) {
    if (verbose) Serial.printf("[Modbus] Bad response: slave=%02X (expected %02X) — sensor answered as a DIFFERENT address than queried; data may still be valid, just came from address %02X instead\n", resp[0], slaveId, resp[0]);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }
  // Deliberately NOT checking resp[1] == funcCode here — some sensors
  // reply with a different function code than requested but otherwise
  // valid, CRC-correct data (confirmed: SM7779 always replies FC03 even
  // when FC04 is requested). We only care that it was SOME valid
  // read-register reply, already confirmed above.

  uint8_t regsInResponse = byteCount / 2;
  if (regsInResponse < (int)respRegOffset + count) {
    if (verbose) Serial.printf("[Modbus] Bad response: got %d registers, need offset %d + count %d\n",
      regsInResponse, respRegOffset, count);
    modbusLogTransaction(slaveId, funcCode, req, 8, resp, n, MB_BAD_RESPONSE);
    return MB_BAD_RESPONSE;
  }

  // regValues[] is sized by the CALLER for `count` registers — even if
  // the sensor sent back more (e.g. a fixed 3-reg block regardless of
  // what was requested), only copy out `count` of them, STARTING at
  // respRegOffset registers into the reply.
  int base = 3 + (int)respRegOffset * 2;
  for (int i = 0; i < count; i++) {
    regValues[i] = ((uint16_t)resp[base + i*2] << 8) | resp[base + i*2 + 1];
  }
  modbusLogTransaction(slaveId, funcCode, req, 8, resp, frameLen, MB_OK);
  return MB_OK;
}

// Decodes 1 or 2 raw registers into a float per the sensor's configured
// data type + word order — the core of making independent sensors
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

// Convenience: poll one independent SensorConfig fully — read the right
// number of registers, decode per its data type/word order, apply
// scale+offset. Returns MB_OK/MB_TIMEOUT/MB_CRC_ERROR/MB_BAD_RESPONSE; on
// MB_OK, fills rawOut (decoded value before scale/offset) and valueOut
// (after).
int modbusPollSensor(SensorConfig& s, float& rawOut, float& valueOut, bool verbose = false) {
  uint8_t n = modbusRegCount(s.dataType);
  uint16_t regs[2] = {0, 0};
  int rc = modbusReadRegs(s.slaveId, s.funcCode, s.regAddr, n, regs, verbose, 600, nullptr, s.respRegOffset);
  if (rc != MB_OK) return rc;
  float raw = modbusDecodeValue(regs, s.dataType, s.wordOrder);
  rawOut = raw;
  valueOut = raw * s.scale + s.offset;
  return MB_OK;
}

// =============================================================================
// FIXED ADAPTER BOARD — convenience wrappers built on modbusReadRegs()/
// FC01/FC02/FC05/FC16 for the primary board's analog channels + AMIDJ14
// digital I/O. FC04 is used for the analog channels (matches every board
// this firmware has targeted: Waveshare 8AI reports on FC04; the AMIDJ14
// also answers FC04). Board auto-detection reads the Product ID register
// via FC03.
// =============================================================================

// FC16 — Write Multiple Holding Registers (used only for the Waveshare
// board's documented mode-3/4-20mA channel-mode convention — NOT a
// generic sensor-config write path; see file header re: FC06 ban).
bool modbusWriteMultiple(uint8_t slaveId, uint16_t startAddr, uint8_t count, uint16_t* values) {
  if (!_mbSerial) return false;

  while (_mbSerial->available()) _mbSerial->read();

  int pduLen = 7 + count * 2;
  uint8_t req[32];
  req[0] = slaveId;
  req[1] = 0x10;             // FC16
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  req[6] = count * 2;        // byte count
  for (int i = 0; i < count; i++) {
    req[7 + i*2]     = values[i] >> 8;
    req[7 + i*2 + 1] = values[i] & 0xFF;
  }
  uint16_t crc = modbusCRC(req, pduLen);
  req[pduLen]     = crc & 0xFF;
  req[pduLen + 1] = crc >> 8;

  modbusSend(req, pduLen + 2);

  // Response: slaveId + 0x10 + startAddr(2) + count(2) + CRC(2) = 8 bytes
  uint8_t resp[16];
  int n = modbusReceive(resp, 8, 300);

  if (n < 8) {
    Serial.printf("[Modbus] FC16 timeout: got %d\n", n);
    return false;
  }

  uint16_t rxCrc   = resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t calcCrc = modbusCRC(resp, 6);
  if (rxCrc != calcCrc || resp[0] != slaveId || resp[1] != 0x10) {
    Serial.println("[Modbus] FC16 bad response");
    return false;
  }
  return true;
}

// FC02 — Read Discrete Inputs. `quiet`/`timeoutMs` used by the DI pulse-
// counter fast-poll loop (pulse.h "Pulse Counter Mode"), which hammers a
// single DI as fast as possible without flooding Serial, with a shorter
// timeout than the normal 300ms default.
bool modbusReadDiscreteInputs(uint8_t slaveId, uint16_t startAddr, uint8_t count, bool* bits,
                               bool quiet = false, int timeoutMs = 300) {
  if (!_mbSerial) return false;

  while (_mbSerial->available()) _mbSerial->read();

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x02;             // FC02
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  modbusSend(req, 8, !quiet);

  // Response: slaveId + 0x02 + byteCount + packed bits + CRC(2)
  uint8_t byteCount = (count + 7) / 8;
  int expectedLen = 3 + byteCount + 2;
  uint8_t resp[16];
  int n = modbusReceive(resp, expectedLen, timeoutMs, !quiet);

  if (n < expectedLen) {
    if (!quiet) Serial.printf("[Modbus] FC02 timeout: got %d, expected %d\n", n, expectedLen);
    return false;
  }

  uint16_t rxCrc   = resp[n-2] | ((uint16_t)resp[n-1] << 8);
  uint16_t calcCrc = modbusCRC(resp, n-2);
  if (rxCrc != calcCrc || resp[0] != slaveId || resp[1] != 0x02) {
    if (!quiet) Serial.printf("[Modbus] FC02 bad response: slave=%02X fc=%02X\n", resp[0], resp[1]);
    return false;
  }

  for (int i = 0; i < count; i++) {
    uint8_t byteIdx = i / 8;
    uint8_t bitIdx  = i % 8;
    bits[i] = (resp[3 + byteIdx] >> bitIdx) & 0x01;
  }
  return true;
}

// FC01 — Read Coils.
bool modbusReadCoils(uint8_t slaveId, uint16_t startAddr, uint8_t count, bool* bits) {
  if (!_mbSerial) return false;

  while (_mbSerial->available()) _mbSerial->read();

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x01;             // FC01
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  modbusSend(req, 8);

  uint8_t byteCount = (count + 7) / 8;
  int expectedLen = 3 + byteCount + 2;
  uint8_t resp[16];
  int n = modbusReceive(resp, expectedLen, 300);

  if (n < expectedLen) {
    Serial.printf("[Modbus] FC01 timeout: got %d, expected %d\n", n, expectedLen);
    return false;
  }

  uint16_t rxCrc   = resp[n-2] | ((uint16_t)resp[n-1] << 8);
  uint16_t calcCrc = modbusCRC(resp, n-2);
  if (rxCrc != calcCrc || resp[0] != slaveId || resp[1] != 0x01) {
    Serial.printf("[Modbus] FC01 bad response: slave=%02X fc=%02X\n", resp[0], resp[1]);
    return false;
  }

  for (int i = 0; i < count; i++) {
    uint8_t byteIdx = i / 8;
    uint8_t bitIdx  = i % 8;
    bits[i] = (resp[3 + byteIdx] >> bitIdx) & 0x01;
  }
  return true;
}

// FC05 — Write Single Coil. value: true=ON (0xFF00), false=OFF (0x0000)
bool modbusWriteCoil(uint8_t slaveId, uint16_t coilAddr, bool value) {
  if (!_mbSerial) return false;

  while (_mbSerial->available()) _mbSerial->read();

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x05;             // FC05
  req[2] = coilAddr >> 8;
  req[3] = coilAddr & 0xFF;
  req[4] = value ? 0xFF : 0x00;
  req[5] = 0x00;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  modbusSend(req, 8);

  // Echo response: slaveId + 0x05 + addr(2) + value(2) + CRC(2) = 8 bytes
  uint8_t resp[16];
  int n = modbusReceive(resp, 8, 300);

  if (n < 8) {
    Serial.printf("[Modbus] FC05 timeout: got %d\n", n);
    return false;
  }

  uint16_t rxCrc   = resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t calcCrc = modbusCRC(resp, 6);
  if (rxCrc != calcCrc || resp[0] != slaveId || resp[1] != 0x05) {
    Serial.printf("[Modbus] FC05 bad response: slave=%02X fc=%02X\n", resp[0], resp[1]);
    return false;
  }
  return true;
}

// Convenience: read the fixed board's analog input channels via FC04.
// numChannels lets a board with fewer real channels (e.g. AMIDJ14 has 6,
// vs. 8 on Waveshare) be read without requesting registers past what it
// actually implements. Any channels beyond numChannels are zero-filled
// (reads as "open circuit", same as a real unwired channel).
bool modbusReadAll(uint8_t slaveId, uint16_t* raw8, int numChannels = 8) {
  if (numChannels > 8) numChannels = 8;
  if (numChannels < 1) numChannels = 1;
  for (int i = numChannels; i < 8; i++) raw8[i] = 0;
  return modbusReadRegs(slaveId, 0x04, 0x0000, (uint8_t)numChannels, raw8, false, 300) == MB_OK;
}

// AMIDJ14 digital I/O addresses — confirmed against real Modbus Poll
// register captures for this board. DI is 0-based, DO is 1-based (not a
// typo — the board's own coil numbering genuinely starts at 1 for outputs).
static const uint16_t AMIDJ14_DI_START = 0; // DI1=0, DI2=1, DI3=2, DI4=3 (FC02)
static const uint16_t AMIDJ14_DO_START = 1; // DO1=1, DO2=2, DO3=3, DO4=4 (FC01/FC05)

bool modbusReadAllDI(uint8_t slaveId, bool* din4) {
  return modbusReadDiscreteInputs(slaveId, AMIDJ14_DI_START, 4, din4);
}

// Convenience: read a SINGLE digital input — used by the DI pulse-
// counter fast-poll loop (pulse.h "Pulse Counter Mode"), which only
// cares about one DI's transitions at a time and wants each round-trip
// as short as possible since round-trip time directly caps the highest
// RPM this method can measure without aliasing.
bool modbusReadOneDI(uint8_t slaveId, int diIndex, bool* state, bool quiet = true, int timeoutMs = 60) {
  return modbusReadDiscreteInputs(slaveId, AMIDJ14_DI_START + diIndex, 1, state, quiet, timeoutMs);
}

bool modbusReadAllDO(uint8_t slaveId, bool* dout4) {
  return modbusReadCoils(slaveId, AMIDJ14_DO_START, 4, dout4);
}

bool modbusWriteDO(uint8_t slaveId, int doIndex, bool value) {
  if (doIndex < 0 || doIndex > 3) return false;
  return modbusWriteCoil(slaveId, AMIDJ14_DO_START + doIndex, value);
}

// =============================================================================
// BOARD AUTO-DETECTION (fixed adapter board)
//
// Different analog-to-Modbus boards use a different raw-value scale and a
// different channel count, but ALL of them we support expose the same
// "Product ID" special-function register at 0x00F7 (247) — reading it via
// FC03 identifies the connected board with no jumpers, dropdown, or manual
// selection needed at all:
//   0 (or read fails)  -> unknown/legacy -> assume Waveshare (safe default)
//   2308               -> Waveshare 8AI (B):  8ch, raw is µA          (/1000)
//   2814               -> Eletechsup AMIDJ14: 6ch, raw is 0.01mA      (/100)
// Add more SKUs here as needed — this table is the ONLY place board-
// specific behavior needs to be taught to the firmware.
// =============================================================================
struct BoardProfile {
  const char* name;
  int   numChannels;
  float rawDivisor;    // raw register value / rawDivisor = mA
  // AMIDJ14 also exposes 4 digital inputs (FC02, addr 0-3) and 4 digital
  // outputs (FC01/FC05, addr 1-4) alongside its 6 analog channels — the
  // Waveshare 8AI board has no such hardware.
  bool  hasDigitalIO;
};

static const BoardProfile BOARD_WAVESHARE_8AI  = { "Waveshare 8AI (B)",  8, 1000.0f, false };
static const BoardProfile BOARD_ELETECHSUP_AMIDJ14 = { "Eletechsup AMIDJ14", 6, 100.0f, true };

// Resolves an ExtraBoardConfig.boardType string ("amidj14"/"waveshare") to
// its BoardProfile. Used for extra/advanced boards, which are explicitly
// typed by whoever wired them up rather than auto-probed like the primary
// board.
BoardProfile boardProfileForType(const String& boardType) {
  if (boardType == "waveshare") return BOARD_WAVESHARE_8AI;
  return BOARD_ELETECHSUP_AMIDJ14; // default
}

// Used when the Product ID probe gets no/bad response at all (bus not
// wired up yet, wrong baud, board unpowered) — deliberately shows
// EVERYTHING (8 analog channels + digital I/O) rather than silently
// defaulting to Waveshare's narrower profile, so "unwired/unplugged"
// doesn't look identical to "this firmware doesn't support your board".
static const BoardProfile BOARD_UNKNOWN = { "Unknown (no response — check wiring)", 8, 1000.0f, true };

// Reads special-function register 0x00F7 (Product ID) via FC03. Returns
// the matching BoardProfile, or BOARD_UNKNOWN if the read fails or
// returns an ID we don't recognize yet.
BoardProfile modbusDetectBoard(uint8_t slaveId) {
  if (!_mbSerial) return BOARD_UNKNOWN;
  uint16_t regs[1];
  int rc = modbusReadRegs(slaveId, 0x03, 0x00F7, 1, regs, false, 300);
  if (rc != MB_OK) {
    Serial.println("[Modbus] Board ID probe: no/bad response — check wiring/baud/power. Showing all channels + digital I/O so nothing's hidden while you sort it out.");
    return BOARD_UNKNOWN;
  }
  uint16_t productId = regs[0];
  Serial.printf("[Modbus] Board ID probe: Product ID register = %u\n", productId);
  switch (productId) {
    case 2308: return BOARD_WAVESHARE_8AI;
    case 2814: return BOARD_ELETECHSUP_AMIDJ14;
    default:
      Serial.printf("[Modbus] Unrecognized Product ID %u — showing all channels + digital I/O\n", productId);
      return BOARD_UNKNOWN;
  }
}

// =============================================================================
// BAUD RATE AUTO-DETECTION / BUS SCAN (shared bus — used by BOTH the
// fixed board and independent sensors, since they all live on the same
// Serial2/RS485 wire and must agree on one baud).
// =============================================================================
static const uint32_t MODBUS_AUTODETECT_BAUDS[] = {
  9600, 19200, 4800, 38400, 2400, 57600, 1200, 115200
};
static const int MODBUS_AUTODETECT_BAUDS_COUNT =
  sizeof(MODBUS_AUTODETECT_BAUDS) / sizeof(MODBUS_AUTODETECT_BAUDS[0]);

// Tries each standard baud against a specific slave ID + register (FC04
// then FC03), used by "Auto-Detect Baud" buttons (fixed board's Config
// page, and per-sensor on /sensors). Restores originalBaud if nothing
// answers. Returns the working baud, or -1 if none found.
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
// each hit. Synchronous/blocking — caller (web handler) should hold
// modbusBusMutex for the whole call.
//
// timeoutMs defaults to 700 (SM7779 radar sensors have a ~6s internal
// measurement cycle and only answer promptly on SOME polls; too short a
// timeout here can cause outright MISSED detections). Retries each
// address up to 2x before giving up.
template<typename FoundFn>
void modbusScanSlaves(int maxAddr, FoundFn onFound, int timeoutMs = 700, bool verbose = true) {
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;
  uint16_t regs[1];
  for (int addr = 1; addr <= maxAddr; addr++) {
    if (verbose) Serial.printf("[Scan] Probing addr %d...\n", addr);
    bool found = false;
    int foundFc = 4;
    for (int attempt = 0; attempt < 2 && !found; attempt++) {
      if (modbusReadRegs((uint8_t)addr, 4, 0x0000, 1, regs, verbose, timeoutMs) == MB_OK) {
        found = true; foundFc = 4;
      } else if (modbusReadRegs((uint8_t)addr, 3, 0x0000, 1, regs, verbose, timeoutMs) == MB_OK) {
        found = true; foundFc = 3;
      }
    }
    if (found) onFound(addr, foundFc);
  }
}

// =============================================================================
// AUTO-DETECT & ENABLE (independent sensors) — scans the bus and
// automatically fills in/enables sensor config slots for any slave that
// responds and isn't already configured (by an independent sensor slot
// OR by the fixed board's own slave ID / any extra board's slave ID —
// skipping those prevents auto-detect from stomping a slot that's
// actually the fixed adapter board, not a loose sensor). New slots get
// conservative defaults (func code = whichever answered, register 0,
// uint16, scale 1) — good enough to prove the sensor is alive; the
// actual register/type/scale for a real reading still needs to be dialed
// in by hand.
// =============================================================================

// Fast path: scan addresses 1..maxAddr at whatever baud is CURRENTLY
// active and fill in/enable slots for new hits. Does not touch any slot
// that's already enabled, nor any address already claimed by the fixed
// board or an extra board. Returns how many new slots were filled.
static int _mbScanAndFillAtCurrentBaud(ModuleConfig& cfg, int maxAddr) {
  bool alreadyConfigured[248] = { false };
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (cfg.sensors[i].enabled) {
      uint8_t sid = cfg.sensors[i].slaveId;
      if (sid <= 247) alreadyConfigured[sid] = true;
    }
  }
  // Also skip the fixed board's own slave ID and any enabled extra
  // board's slave ID — those addresses are the adapter board(s), not a
  // loose independent sensor, even though they'll answer a basic FC04/
  // FC03 register-0 probe just like a real sensor would.
  if (cfg.modbusSlaveId >= 1 && cfg.modbusSlaveId <= 247) {
    alreadyConfigured[cfg.modbusSlaveId] = true;
  }
  for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
    if (cfg.extraBoards[i].enabled && cfg.extraBoards[i].slaveId >= 1 && cfg.extraBoards[i].slaveId <= 247) {
      alreadyConfigured[cfg.extraBoards[i].slaveId] = true;
    }
  }

  int newCount = 0;
  modbusScanSlaves(maxAddr, [&](int addr, int fc) {
    if (addr < 1 || addr > 247) return;
    if (alreadyConfigured[addr]) return;

    int slot = -1;
    for (int i = 0; i < MAX_SENSORS; i++) {
      if (!cfg.sensors[i].enabled) { slot = i; break; }
    }
    if (slot < 0) return; // no free slots left

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

// Full auto-detect: scans the CURRENT baud first. If that finds nothing
// AND no sensor is enabled yet AND the baud hasn't been manually locked
// in (cfg.baudManuallySet), it's reasonable to assume the bus itself
// might be at a different baud than the default — so it tries every
// other standard baud too, adopting whichever finds something. Once at
// least one sensor is enabled OR the baud is locked, this stops hopping
// bauds — see config.h ModuleConfig.baudManuallySet for the full 2026-08-10
// incident history behind this guard.
int modbusAutoDetectAndEnable(ModuleConfig& cfg, int maxAddr = 16) {
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;

  int newCount = _mbScanAndFillAtCurrentBaud(cfg, maxAddr);
  if (newCount > 0) return newCount;

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

  Serial.println("[AutoDetect] No sensors found at any standard baud — restoring original");
  _mbSerial->flush();
  _mbSerial->updateBaudRate(originalBaud);
  delay(20);
  return 0;
}

// =============================================================================
// SLAVE ID BUS SCAN (Advanced / hidden — /advanced page) — sanity-check
// tool for the fixed-board side: probes addresses 1..maxAddr and reports
// which ones answer, plus their Product ID (0x00F7) if it identifies as a
// known board. Distinct from modbusScanSlaves() above (which is generic
// "something answered FC03/FC04") — this one also tries to identify a
// FIXED BOARD specifically via the Product ID register, for the Advanced
// page's multi-board setup workflow.
// =============================================================================
template<typename FoundFn>
void modbusScanSlavesWithBoardId(int maxAddr, FoundFn onFound, int timeoutMs = 300) {
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;
  uint16_t regs[1];
  for (int addr = 1; addr <= maxAddr; addr++) {
    if (modbusReadRegs((uint8_t)addr, 0x04, 0x0000, 1, regs, false, timeoutMs) == MB_OK) {
      BoardProfile p = modbusDetectBoard((uint8_t)addr);
      int productId = -1;
      if (String(p.name) == BOARD_WAVESHARE_8AI.name) productId = 2308;
      else if (String(p.name) == BOARD_ELETECHSUP_AMIDJ14.name) productId = 2814;
      onFound(addr, productId);
    }
  }
}