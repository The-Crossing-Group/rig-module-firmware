// =============================================================================
// modbus.h — Manual Modbus RTU over HardwareSerial (RS485)
//
// Uses direct HardwareSerial with DE/RE pin toggling.
// No external Modbus library needed — avoids DE timing issues.
// =============================================================================
#pragma once
#include <Arduino.h>

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

// Init RS485
// baud: configurable from the webUI (/config, "RS485 Baud Rate") — different
// analog-to-Modbus boards ship with different factory defaults (Waveshare
// 8AI (B) = 9600bps, SDSIN SN-3002 clone = 4800bps), so this is NOT hardcoded.
void modbusInit(int rxPin, int txPin, int dePin, uint32_t baud) {
  _RS485_DE_PIN = dePin;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin, LOW); // receive mode by default

  _mbSerial = &Serial2;
  _mbSerial->begin(baud, SERIAL_8N1, rxPin, txPin);
  Serial.printf("[Modbus] Init on Serial2 RX=%d TX=%d DE=%d baud=%u\n", rxPin, txPin, dePin, baud);
}

// Send bytes, toggle DE high during TX
static void modbusSend(const uint8_t* buf, int len) {
  Serial.printf("[RS485] TX (%d bytes):", len);
  for (int i = 0; i < len; i++) Serial.printf(" %02X", buf[i]);
  Serial.println();

  digitalWrite(_RS485_DE_PIN, HIGH);
  delayMicroseconds(100); // DE propagation delay
  _mbSerial->write(buf, len);
  _mbSerial->flush(); // wait for TX to complete
  delayMicroseconds(100);
  digitalWrite(_RS485_DE_PIN, LOW); // back to receive
}

// Read response with timeout (ms)
static int modbusReceive(uint8_t* buf, int maxLen, int timeoutMs) {
  unsigned long deadline = millis() + timeoutMs;
  int n = 0;
  while (millis() < deadline && n < maxLen) {
    if (_mbSerial->available()) {
      buf[n++] = _mbSerial->read();
      deadline = millis() + 20; // inter-byte timeout 20ms
    }
  }
  if (n > 0) {
    Serial.printf("[RS485] RX (%d bytes):", n);
    for (int i = 0; i < n; i++) Serial.printf(" %02X", buf[i]);
    Serial.println();
  } else {
    Serial.println("[RS485] RX: no bytes received");
  }
  return n;
}

// FC04 — Read Input Registers
// Returns true on success, fills regValues[count]
bool modbusReadInputRegs(uint8_t slaveId, uint16_t startAddr, uint8_t count, uint16_t* regValues) {
  if (!_mbSerial) return false;

  // Flush RX
  while (_mbSerial->available()) _mbSerial->read();

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x04;             // FC04
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  modbusSend(req, 8);

  // Expected response: slaveId + 0x04 + byteCount + (count*2 bytes) + CRC
  int expectedLen = 3 + count * 2 + 2;
  uint8_t resp[64];
  int n = modbusReceive(resp, expectedLen, 300);

  if (n < expectedLen) {
    Serial.printf("[Modbus] FC04 timeout: got %d, expected %d\n", n, expectedLen);
    return false;
  }

  // Validate CRC
  uint16_t rxCrc = resp[n-2] | ((uint16_t)resp[n-1] << 8);
  uint16_t calcCrc = modbusCRC(resp, n-2);
  if (rxCrc != calcCrc) {
    Serial.printf("[Modbus] CRC error: got %04X, calc %04X\n", rxCrc, calcCrc);
    return false;
  }

  // Validate function code
  if (resp[0] != slaveId || resp[1] != 0x04) {
    Serial.printf("[Modbus] Bad response: slave=%02X fc=%02X\n", resp[0], resp[1]);
    return false;
  }

  // Extract register values
  for (int i = 0; i < count; i++) {
    regValues[i] = ((uint16_t)resp[3 + i*2] << 8) | resp[4 + i*2];
  }
  return true;
}

// FC16 — Write Multiple Holding Registers
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

// Convenience: read the board's analog input channels. numChannels lets a
// board with fewer real channels (e.g. the Eletechsup AMIDJ14 only has 6,
// vs. 8 on the original board this firmware targeted) be read without
// requesting registers past what it actually implements — asking for more
// registers than a board has causes it to return a Modbus exception or no
// response at all, which would otherwise fail the ENTIRE read (all 8
// channels going stale) just because channels 6/7 don't exist on that
// hardware. Any channels beyond numChannels are zero-filled (reads as
// "open circuit", same as a real unwired channel — a reasonable default
// for "this channel doesn't exist on this board").
bool modbusReadAll(uint8_t slaveId, uint16_t* raw8, int numChannels = 8) {
  if (numChannels > 8) numChannels = 8;
  if (numChannels < 1) numChannels = 1;
  for (int i = numChannels; i < 8; i++) raw8[i] = 0;
  return modbusReadInputRegs(slaveId, 0x0000, (uint8_t)numChannels, raw8);
}

// =============================================================================
// BOARD AUTO-DETECTION
//
// Different analog-to-Modbus boards use a different raw-value scale and a
// different channel count, but ALL of them we support expose the same
// "Product ID" special-function register at 0x00F7 (247) — reading it via
// FC03 identifies the connected board with no jumpers, dropdown, or manual
// selection needed at all:
//   0 (or read fails)  -> unknown/legacy -> assume Waveshare (safe default,
//                         matches every board this firmware originally
//                         targeted before Product ID detection existed)
//   2308               -> Waveshare 8AI (B):  8ch, raw is µA          (/1000)
//   2814               -> Eletechsup AMIDJ14: 6ch, raw is 0.01mA      (/100)
// Add more SKUs here as needed — this table is the ONLY place board-
// specific behavior needs to be taught to the firmware; scaleChannel() and
// the poll task just consume the resulting BoardProfile.
// =============================================================================
struct BoardProfile {
  const char* name;
  int   numChannels;
  float rawDivisor;    // raw register value / rawDivisor = mA
};

static const BoardProfile BOARD_WAVESHARE_8AI  = { "Waveshare 8AI (B)",  8, 1000.0f };
static const BoardProfile BOARD_ELETECHSUP_AMIDJ14 = { "Eletechsup AMIDJ14", 6, 100.0f };

// Reads special-function register 0x00F7 (Product ID) via FC03. Returns the
// matching BoardProfile, or Waveshare as the safe fallback if the read
// fails (unplugged bus, board without this register, wrong baud, etc.) or
// returns an ID we don't recognize yet — the original 8-channel/µA
// behavior is that fallback, so any board this firmware worked with
// before Product-ID detection existed keeps working identically.
BoardProfile modbusDetectBoard(uint8_t slaveId) {
  if (!_mbSerial) return BOARD_WAVESHARE_8AI;

  while (_mbSerial->available()) _mbSerial->read();

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x03;             // FC03 — read holding/special-function registers
  req[2] = 0x00;
  req[3] = 0xF7;             // register 247 = Product ID
  req[4] = 0x00;
  req[5] = 0x01;              // read 1 register
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  modbusSend(req, 8);

  uint8_t resp[16];
  int n = modbusReceive(resp, 7, 300); // slave+fc+len+2 data+2 CRC = 7 bytes

  if (n < 7) {
    Serial.println("[Modbus] Board ID probe: no/short response — defaulting to Waveshare");
    return BOARD_WAVESHARE_8AI;
  }
  uint16_t rxCrc   = resp[n-2] | ((uint16_t)resp[n-1] << 8);
  uint16_t calcCrc = modbusCRC(resp, n-2);
  if (rxCrc != calcCrc || resp[0] != slaveId || resp[1] != 0x03) {
    Serial.println("[Modbus] Board ID probe: bad response — defaulting to Waveshare");
    return BOARD_WAVESHARE_8AI;
  }

  uint16_t productId = ((uint16_t)resp[3] << 8) | resp[4];
  Serial.printf("[Modbus] Board ID probe: Product ID register = %u\n", productId);

  switch (productId) {
    case 2308: return BOARD_WAVESHARE_8AI;
    case 2814: return BOARD_ELETECHSUP_AMIDJ14;
    default:
      Serial.printf("[Modbus] Unrecognized Product ID %u — defaulting to Waveshare\n", productId);
      return BOARD_WAVESHARE_8AI;
  }
}

// =============================================================================
// BAUD RATE AUTO-DETECTION
//
// Instead of guessing, actually probe the bus: try each standard baud rate
// in turn, sending a real FC04 read at each and checking for a CRC-valid,
// correctly-addressed response. Whichever baud gets a real answer from the
// board is the right one. Order is most-likely-first based on the boards we
// support (Waveshare 8AI (B) = 9600, SDSIN SN-3002 clone = 4800), covering
// the full standard Modbus RTU range so any board just works.
//
// Reuses the already-open Serial2 (no re-init/pin changes) via
// updateBaudRate() — safe to call any time after modbusInit(). Restores the
// original baud if nothing answers, so a failed scan never leaves the bus
// worse off than before.
// =============================================================================
static const uint32_t MODBUS_AUTODETECT_BAUDS[] = {
  9600, 4800, 19200, 2400, 38400, 1200, 57600, 115200
};
static const int MODBUS_AUTODETECT_BAUDS_COUNT =
  sizeof(MODBUS_AUTODETECT_BAUDS) / sizeof(MODBUS_AUTODETECT_BAUDS[0]);

long modbusAutoDetectBaud(uint8_t slaveId, uint32_t originalBaud) {
  if (!_mbSerial) return -1;
  Serial.println("[Modbus] ---- Auto-detecting baud rate ----");
  for (int i = 0; i < MODBUS_AUTODETECT_BAUDS_COUNT; i++) {
    uint32_t tryBaud = MODBUS_AUTODETECT_BAUDS[i];
    Serial.printf("[Modbus]   Trying %u baud...\n", tryBaud);
    _mbSerial->flush();
    _mbSerial->updateBaudRate(tryBaud);
    delay(20); // let the UART settle at the new rate before probing

    // Read just 1 register — enough to confirm a real Modbus device is
    // answering, cheaper/faster than a full 8-register probe per baud.
    uint16_t regs[1];
    bool ok = false;
    // A couple of quick attempts per baud — occasionally the first probe
    // right after a rate change gets missed by the board.
    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
      ok = modbusReadInputRegs(slaveId, 0x0000, 1, regs);
    }
    if (ok) {
      Serial.printf("[Modbus] Auto-detect SUCCESS at %u baud (reg0=%u)\n", tryBaud, regs[0]);
      return (long)tryBaud;
    }
  }
  Serial.println("[Modbus] Auto-detect found nothing at any baud — restoring original");
  _mbSerial->flush();
  _mbSerial->updateBaudRate(originalBaud);
  delay(20);
  return -1;
}
