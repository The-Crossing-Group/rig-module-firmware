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

int modbusReadRegs(uint8_t slaveId, uint8_t funcCode, uint16_t startAddr,
                    uint8_t count, uint16_t* regValues, bool verbose = false) {
  if (!_mbSerial) return MB_TIMEOUT;
  if (count < 1) count = 1;
  if (count > 16) count = 16;

  while (_mbSerial->available()) _mbSerial->read(); // flush stale RX

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

  int expectedLen = 3 + count * 2 + 2;
  uint8_t resp[40];
  int n = modbusReceive(resp, expectedLen, 300, verbose);

  if (n < expectedLen) {
    if (verbose) Serial.printf("[Modbus] Timeout: got %d, expected %d\n", n, expectedLen);
    return MB_TIMEOUT;
  }

  uint16_t rxCrc = resp[n-2] | ((uint16_t)resp[n-1] << 8);
  uint16_t calcCrc = modbusCRC(resp, n-2);
  if (rxCrc != calcCrc) {
    if (verbose) Serial.printf("[Modbus] CRC error: got %04X, calc %04X\n", rxCrc, calcCrc);
    return MB_CRC_ERROR;
  }

  if (resp[0] != slaveId || resp[1] != funcCode) {
    if (verbose) Serial.printf("[Modbus] Bad response: slave=%02X fc=%02X\n", resp[0], resp[1]);
    return MB_BAD_RESPONSE;
  }

  for (int i = 0; i < count; i++) {
    regValues[i] = ((uint16_t)resp[3 + i*2] << 8) | resp[4 + i*2];
  }
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
// FC04 (then FC03) probe of register 0. Calls onFound(addr) for each hit.
// This is a synchronous, blocking scan (a few hundred ms per address at
// worst) — the caller (web handler) should only invoke it on demand, not
// from the poll loop, and should hold modbusBusMutex for the whole call.
template<typename FoundFn>
void modbusScanSlaves(int maxAddr, FoundFn onFound) {
  uint16_t regs[1];
  for (int addr = 1; addr <= maxAddr; addr++) {
    bool found = (modbusReadRegs((uint8_t)addr, 4, 0x0000, 1, regs) == MB_OK) ||
                 (modbusReadRegs((uint8_t)addr, 3, 0x0000, 1, regs) == MB_OK);
    if (found) onFound(addr);
  }
}
