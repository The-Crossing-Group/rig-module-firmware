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

// Reconstructs the bit sequence for an ASSUMED bit period, anchoring
// every edge's bit-index to elapsed time since the FIRST edge (not to
// the previous edge). This matters a lot: naive per-interval rounding
// (round each gap independently, divide by period) accumulates error
// every single edge - if the period is off by even 4%, a 90-bit frame
// drifts by ~3.6 bit-widths by the end, which silently corrupts bytes
// partway through a frame while the first byte or two still happen to
// look right (exactly the "matches for 2 bytes then garbage" pattern
// this tool exists to catch). Anchoring to absolute elapsed time makes
// each bit-boundary decision independent, so errors don't compound.
static std::vector<uint8_t> _bsReconstructBits(const std::vector<uint32_t>& times, const std::vector<uint8_t>& levels, double periodUs) {
  std::vector<uint8_t> bits;
  int n = times.size();
  bits.reserve(n * 4);
  long cumulativeBits = 0;
  for (int i = 1; i < n; i++) {
    double elapsed = (double)(times[i] - times[0]);
    long expected = (long)round(elapsed / periodUs);
    long count = expected - cumulativeBits;
    if (count < 1) count = 1;          // guarantee forward progress even if noise briefly puts us behind
    if (count > 200) count = 200;      // cap one trailing idle gap from blowing up the buffer
    for (long u = 0; u < count; u++) bits.push_back(levels[i - 1]);
    cumulativeBits += count;
  }
  return bits;
}

BitscopeResult bitscopeAnalyze() {
  BitscopeResult r;
  int n = _bsEdgeCount;
  r.edgeCount = n;
  if (n < 4) {
    r.bestDecodeReport = "Not enough edges captured (" + String(n) + ") - is the sensor actually transmitting right now? If it auto-reports periodically, make sure the capture window is longer than its report interval.";
    return r;
  }

  // Snapshot volatile arrays into local copies (capture is already stopped by now).
  std::vector<uint32_t> times(n);
  std::vector<uint8_t> levels(n);
  for (int i = 0; i < n; i++) { times[i] = _bsEdgeTimeUs[i]; levels[i] = _bsEdgeLevel[i]; }

  // Find the shortest pulse width above a noise floor (20us - filters
  // electrical glitches/reflections). This is our rough baud estimate,
  // but it's noisy (ISR/digitalRead latency, timer granularity) - good
  // enough to report, NOT good enough to reconstruct a whole frame
  // against without drift. The real decode below tests exact standard
  // baud periods instead of trusting this measurement directly.
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

  // Build a compact raw bitstream string (quantized against the noisy
  // measured period, just for eyeballing) - cap length for readability.
  std::vector<uint8_t> roughBits = _bsReconstructBits(times, levels, (double)minPulse);
  String bs;
  int showBits = min((int)roughBits.size(), 400);
  for (int i = 0; i < showBits; i++) bs += roughBits[i] ? '1' : '0';
  if ((int)roughBits.size() > showBits) bs += "...";
  r.bitstream = bs;

  // Now the real test: reconstruct against every EXACT standard baud
  // period (not the noisy measurement) using drift-free anchoring, and
  // CRC-sweep each one. If the sensor is really running one of these
  // standard bauds, this finds a clean decode even though the measured
  // minPulse was off by a few percent.
  struct BaudCandidate { const char* name; double periodUs; };
  BaudCandidate candidates[] = {
    {"2400",   1000000.0 / 2400.0},
    {"4800",   1000000.0 / 4800.0},
    {"9600",   1000000.0 / 9600.0},
    {"19200",  1000000.0 / 19200.0},
    {"38400",  1000000.0 / 38400.0},
    {"57600",  1000000.0 / 57600.0},
    {"115200", 1000000.0 / 115200.0},
  };

  String report = "Measured (noisy) estimate: " + String(r.estimatedBaud) + " baud - testing exact standard bauds with drift-free reconstruction instead:\n\n";
  bool anyWin = false;
  for (auto& c : candidates) {
    std::vector<uint8_t> bits = _bsReconstructBits(times, levels, c.periodUs);
    String sub = _bsTryDecodeAllOffsets(bits);
    bool hit = sub.indexOf("VALID MODBUS CRC") >= 0;
    if (hit) anyWin = true;
    report += String(hit ? "  [BAUD " : "  [baud ") + c.name + (hit ? " *** HIT ***]\n" : "]\n");
    if (hit) report += sub + "\n";
  }
  if (!anyWin) {
    report += "\nNo standard baud produced a valid CRC anywhere. Full decode at each standard baud (bit offset 0, for eyeballing):\n";
    for (auto& c : candidates) {
      std::vector<uint8_t> bits = _bsReconstructBits(times, levels, c.periodUs);
      String line = "  " + String(c.name) + ": ";
      int i = 0, shown = 0;
      while (i + 9 < (int)bits.size() && shown < 16) {
        uint8_t val = 0;
        for (int k = 0; k < 8; k++) val |= (bits[i + 1 + k] << k);
        char h[4]; snprintf(h, 4, "%02X ", val); line += h;
        i += 10;
        shown++;
      }
      report += line + "\n";
    }
  }

  r.bestDecodeReport = report;
  return r;
}
