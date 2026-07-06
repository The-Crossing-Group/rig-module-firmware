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
void modbusInit(int rxPin, int txPin, int dePin) {
  _RS485_DE_PIN = dePin;
  pinMode(dePin, OUTPUT);
  digitalWrite(dePin, LOW); // receive mode by default

  _mbSerial = &Serial2;
  _mbSerial->begin(9600, SERIAL_8N1, rxPin, txPin);
  Serial.printf("[Modbus] Init on Serial2 RX=%d TX=%d DE=%d\n", rxPin, txPin, dePin);
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

// Convenience: read all 8 input registers
bool modbusReadAll(uint8_t slaveId, uint16_t* raw8) {
  return modbusReadInputRegs(slaveId, 0x0000, 8, raw8);
}
