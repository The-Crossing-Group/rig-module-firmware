// =============================================================================
// bitscope.h — Raw electrical-level RS485 capture, bypassing the UART
// peripheral's framing assumptions entirely.
//
// Every tool so far (dbgReadRegs, dbgRawSend, the passive sniff) relies
// on the ESP32's hardware UART to decode bytes for us — which means it
// silently ASSUMES the baud/parity/stopbits we told it are correct. If
// the sensor's actual framing changed (e.g. from those old writes to
// comm-mode/protocol-type registers) and our assumption is wrong in a
// way none of the standard baud/parity/stop combos catch, the hardware
// UART will just keep handing us garbage with no way to tell WHY.
//
// This module skips the UART decoder completely. It attaches a raw GPIO
// interrupt directly to the RX pin and timestamps every single voltage
// transition (every edge) with microsecond precision. From that list of
// edges we can:
//   1. Measure the REAL bit period straight off the wire (the shortest
//      pulse width tells us the true baud, even if it's some non-
//      standard value no dropdown would ever have listed)
//   2. Reconstruct the actual 1/0 bitstream with zero framing
//      assumptions at all
//   3. Try decoding that bitstream as 8N1 UART bytes starting at every
//      possible bit offset (0-9), and check each resulting byte
//      sequence against the Modbus CRC16 — if the sensor is sending
//      valid Modbus RTU but our receiver has been misinterpreting the
//      byte boundaries, this will find it
//
// This is the "does the ESP32's own GPIO interrupt conflict with the
// UART peripheral also using that pin" question — it doesn't. The
// ESP32's GPIO matrix lets any number of peripherals AND a plain GPIO
// interrupt all read the same input pad simultaneously; attachInterrupt
// here doesn't disturb Serial2's own use of the pin at all.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <vector>

#define BITSCOPE_MAX_EDGES 4000
static volatile uint32_t _bsEdgeTimeUs[BITSCOPE_MAX_EDGES];
static volatile uint8_t  _bsEdgeLevel[BITSCOPE_MAX_EDGES];   // level AFTER this edge
static volatile int      _bsEdgeCount = 0;
static int               _bsPin = -1;
static bool              _bsCapturing = false;

// ISR: record timestamp + resulting level for every edge. digitalRead()
// is a little slow for an ISR but at RS485 sensor baud rates (a few kHz
// of edges at most) there's plenty of headroom - this is a diagnostic
// tool, not a production hot path.
void IRAM_ATTR _bsOnEdge() {
  int idx = _bsEdgeCount;
  if (idx < BITSCOPE_MAX_EDGES) {
    _bsEdgeTimeUs[idx] = micros();
    _bsEdgeLevel[idx] = digitalRead(_bsPin);
    _bsEdgeCount = idx + 1;
  }
}

void bitscopeInit(int rxPin) {
  _bsPin = rxPin;
  // Pin is already an INPUT (shared with Serial2's RX use) - just attach
  // our own interrupt on top of it, don't touch pinMode so we don't
  // fight whatever Serial2/UART already configured on it.
}

// Blocks for captureMs milliseconds recording every edge on the RX
// line. Call with nothing transmitting on the bus you want to hear
// (though it works fine picking up unsolicited/auto-report traffic too
// - that's the whole point here).
void bitscopeCapture(int captureMs) {
  _bsEdgeCount = 0;
  _bsCapturing = true;
  attachInterrupt(digitalPinToInterrupt(_bsPin), _bsOnEdge, CHANGE);
  delay(captureMs);
  detachInterrupt(digitalPinToInterrupt(_bsPin));
  _bsCapturing = false;
}

struct BitscopeResult {
  int edgeCount = 0;
  uint32_t minPulseUs = 0;      // shortest pulse seen (candidate unit bit period)
  uint32_t estimatedBaud = 0;
  String bitstream;             // reconstructed raw 0/1 string, one char per bit-unit
  String bestDecodeReport;      // human-readable summary of best-guess byte decode(s)
};

static uint16_t _bsCRC16(const uint8_t* buf, int len) {
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

// Tries every start offset in the bit array for standard 8N1 framing
// (start=0, 8 data bits LSB-first, 1 stop) and reports any decode whose
// resulting bytes contain a valid Modbus CRC16 anywhere in them. This is
// the brute-force "is this actually valid Modbus, just misaligned"
// check - the whole reason this tool exists.
static String _bsTryDecodeAllOffsets(const std::vector<uint8_t>& bits) {
  String report;
  int n = bits.size();
  bool anyCrcHit = false;

  for (int offset = 0; offset < n && offset < 11; offset++) {
    std::vector<uint8_t> bytes;
    int i = offset;
    while (i + 9 <= n) {  // need at least start+8data+stop = 10 bit-slots; but we don't strictly validate start/stop values, just chop
      // Expect bits[i] == start(0) ideally, but don't require it - just decode whatever's there and let CRC be the judge.
      uint8_t val = 0;
      for (int k = 0; k < 8; k++) val |= (bits[i + 1 + k] << k);  // LSB first, skip assumed start bit at i
      bytes.push_back(val);
      i += 10; // start + 8 data + 1 stop
    }
    if (bytes.size() < 4) continue;

    // Scan this byte sequence for any valid Modbus CRC16 frame at any sub-offset/length.
    for (size_t o = 0; o < bytes.size(); o++) {
      for (size_t flen = 4; o + flen <= bytes.size(); flen++) {
        uint16_t crc = _bsCRC16(&bytes[o], flen - 2);
        uint8_t lo = crc & 0xFF, hi = (crc >> 8) & 0xFF;
        if (bytes[o + flen - 2] == lo && bytes[o + flen - 1] == hi) {
          report += "  *** VALID MODBUS CRC *** bit-offset=" + String(offset) + " byte-range=[" + String(o) + ".." + String(o + flen - 1) + "]: ";
          for (size_t k = 0; k < flen; k++) { char h[4]; snprintf(h, 4, "%02X ", bytes[o + k]); report += h; }
          report += "\n";
          anyCrcHit = true;
        }
      }
    }

    // Always show the raw decode for this offset too, for manual eyeballing.
    report += "  offset " + String(offset) + ": ";
    for (uint8_t b : bytes) { char h[4]; snprintf(h, 4, "%02X ", b); report += h; }
    report += "\n";
  }

  if (!anyCrcHit) report = "  (no valid Modbus CRC found at any bit offset 0-10)\n" + report;
  return report;
}

BitscopeResult bitscopeAnalyze() {
  BitscopeResult r;
  int n = _bsEdgeCount;
  r.edgeCount = n;
  if (n < 4) {
    r.bestDecodeReport = "Not enough edges captured (" + String(n) + ") - is the sensor actually transmitting right now?";
    return r;
  }

  // Snapshot volatile arrays into local copies (capture is already stopped by now).
  std::vector<uint32_t> times(n);
  std::vector<uint8_t> levels(n);
  for (int i = 0; i < n; i++) { times[i] = _bsEdgeTimeUs[i]; levels[i] = _bsEdgeLevel[i]; }

  // Find the shortest pulse width above a noise floor (20us - filters
  // electrical glitches/reflections, well below any real UART bit at
  // even 115200 baud which is ~8.7us... note: if the real baud is above
  // ~115200 this floor would clip it, but that's not realistic for an
  // RS485 sensor). This shortest pulse is our best estimate of one bit
  // period, since a single-bit pulse is by definition the smallest
  // interval the signal can hold before the next transition.
  uint32_t minPulse = UINT32_MAX;
  for (int i = 1; i < n; i++) {
    uint32_t d = times[i] - times[i - 1];
    if (d > 15 && d < minPulse) minPulse = d;
  }
  if (minPulse == UINT32_MAX || minPulse == 0) {
    r.bestDecodeReport = "Could not determine a bit period from captured edges (all pulses were noise-floor or zero-width).";
    return r;
  }
  r.minPulseUs = minPulse;
  r.estimatedBaud = 1000000UL / minPulse;

  // Reconstruct the bitstream: between edge i-1 and edge i, the line
  // held at levels[i-1] for (times[i]-times[i-1]) us, which is
  // round(duration/minPulse) bit-units of that level.
  std::vector<uint8_t> bits;
  bits.reserve(n * 4);
  for (int i = 1; i < n; i++) {
    uint32_t d = times[i] - times[i - 1];
    int units = (int)((d + minPulse / 2) / minPulse);  // round to nearest
    if (units < 1) units = 1;
    if (units > 200) units = 200; // cap a single idle gap from blowing up the buffer
    for (int u = 0; u < units; u++) bits.push_back(levels[i - 1]);
  }

  // Build a compact bitstream string for manual inspection (cap length
  // for readability - full detail available via the offset decode below).
  String bs;
  int showBits = min((int)bits.size(), 400);
  for (int i = 0; i < showBits; i++) bs += bits[i] ? '1' : '0';
  if ((int)bits.size() > showBits) bs += "...";
  r.bitstream = bs;

  r.bestDecodeReport = _bsTryDecodeAllOffsets(bits);
  return r;
}
