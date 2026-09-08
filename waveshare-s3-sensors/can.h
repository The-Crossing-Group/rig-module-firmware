// =============================================================================
// can.h — CAN bus via ESP32-S3 native TWAI controller
//
// Three jobs:
//   1) Raw frame capture — a small ring buffer of the most recent frames
//      (ID, DLC, data bytes, timestamp) for the live diagnostics page.
//      This is the "look at the bus before you know what's on it" tool.
//   2) Signal extraction — CanSignalConfig entries pull a byte range out
//      of frames matching a given ID and decode it into a value, same
//      spirit as a Modbus sensor but sourced from CAN instead of RS485.
//   3) CANopen bring-up + raw relay (2026-09-08, "CANopen Bridge" mode) —
//      this board moved from being wired directly into the ditchwitch-
//      logger PC's InnoMaker USB-CAN adapter to sitting on the encoder's
//      CAN wire itself and relaying frames over WiFi instead (easier/
//      safer wiring — one CAN run to a nearby ESP32 instead of all the
//      way to the panel PC). That means THIS board now has to do the
//      CANopen bring-up (SYNC/baud-detect burst + NMT Start) that used
//      to happen on the PC's side — confirmed necessary to wake the
//      encoder at all, see ditchwitch-logger's can_listener.py
//      _canopen_bringup() (this is a direct port of that same sequence,
//      same COB-IDs, same timing). Decode of the resulting frames stays
//      on the PC (J1939-shaped Proprietary B payload, no confirmed byte
//      layout yet — decoding it here would mean guessing, which this
//      whole project's rule forbids) — this board just forwards the raw
//      frames captured by job #1 above, reusing that same ring buffer.
//
// Uses the ESP-IDF TWAI driver directly (esp_driver_twai) rather than a
// third-party Arduino CAN library — it's built into the ESP32 Arduino
// core already (no extra library to install) and is the same driver
// underlying every ESP32 CAN example.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "config.h"

static bool _canStarted = false;
static bool _canListenOnly = true; // tracks which mode canStart() brought the driver up in

// Ring buffer of raw captured frames for the diagnostics page.
struct CanFrameLog {
  uint32_t id;
  bool     extended;
  uint8_t  dlc;
  uint8_t  data[8];
  unsigned long ms; // millis() at capture
};
#define CAN_LOG_SIZE 60
static CanFrameLog _canLog[CAN_LOG_SIZE];
static int _canLogHead = 0;   // next write index
static int _canLogCount = 0;  // how many valid entries (caps at CAN_LOG_SIZE)
static unsigned long _canFrameTotal = 0; // total frames seen since CAN start
static unsigned long _canLastFrameMs = 0;

// Starts the TWAI controller on the given pins at the given bitrate.
// Returns true on success. Safe to call again after canStop() to change
// bitrate (requires driver uninstall/reinstall, done here).
//
// listenOnly=true (default/original behavior): we never transmit on this
// bus — purely a passive tap, safest option (a mis-wired/mis-timed board
// can't possibly disrupt drill CAN traffic if it physically cannot send).
//
// listenOnly=false ("CANopen Bridge" mode, cfg.canopenBridge): needed to
// actually transmit the CANopen bring-up sequence (see canopenBringup()
// below) — required because the encoder this board is bridging for is
// confirmed silent until it receives that bring-up, see this file's
// header comment. Only use this on a bus you're SUPPOSED to be sending
// on (i.e. your own encoder's dedicated wire, not tapped onto shared
// drill CAN traffic you don't own).
bool canStart(int txPin, int rxPin, long bitrate, bool listenOnly = true) {
  if (_canStarted) {
    twai_stop();
    twai_driver_uninstall();
    _canStarted = false;
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)txPin, (gpio_num_t)rxPin,
    listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  _canListenOnly = listenOnly;

  twai_timing_config_t t_config;
  switch (bitrate) {
    case 125000:  t_config = TWAI_TIMING_CONFIG_125KBITS();  break;
    case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS();  break;
    case 500000:  t_config = TWAI_TIMING_CONFIG_500KBITS();  break;
    case 1000000: t_config = TWAI_TIMING_CONFIG_1MBITS();    break;
    default:      t_config = TWAI_TIMING_CONFIG_250KBITS();  break; // most common default (J1939/drill CAN)
  }
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("[CAN] Driver install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[CAN] Start failed");
    twai_driver_uninstall();
    return false;
  }
  _canStarted = true;
  _canLogHead = 0;
  _canLogCount = 0;
  _canFrameTotal = 0;
  Serial.printf("[CAN] Started, TX=%d RX=%d bitrate=%ld (%s)\n", txPin, rxPin, bitrate,
    listenOnly ? "listen-only" : "NORMAL, can transmit");
  return true;
}

bool canIsListenOnly() { return _canListenOnly; }

// Sends a raw CAN frame. No-op (returns false) if the driver is in
// listen-only mode — TWAI_MODE_LISTEN_ONLY physically cannot transmit,
// this isn't a soft check that could be bypassed.
bool canSend(uint32_t id, bool extended, const uint8_t* data, uint8_t len) {
  if (!_canStarted || _canListenOnly) return false;
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = extended ? 1 : 0;
  msg.data_length_code = len;
  memcpy(msg.data, data, min((int)len, 8));
  return twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK;
}

// =============================================================================
// CANopen bring-up — SYNC/baud-detect burst + NMT Start.
//
// Direct port of ditchwitch-logger's src/can_listener.py
// _canopen_bringup() (same COB-IDs, same ~2s baud-detect duration, same
// NMT Start payload) — confirmed on real bench hardware (2026-09-03/04,
// see that project's README + MEMORY.md) to be what wakes the EPC
// CANopen encoder from its factory-silent state, even though what it
// then broadcasts turns out to be J1939-shaped rather than real CiA 301
// PDOs (see this file's header comment — that's why this board doesn't
// try to decode position itself).
//
// Blocking for ~2 seconds (the baud-detect burst) — called once at boot
// and again by canopenBringupIfDue()'s self-heal retry, both from
// loop(), same tradeoff the PC-side Python listener makes (it blocks its
// own dedicated CAN thread the same way).
// =============================================================================
#define CANOPEN_ID_NMT               0x000
#define CANOPEN_ID_SYNC_BAUDDETECT   0x080
#define CANOPEN_NMT_CS_START         0x01

void canopenBringup(uint8_t targetNodeId, bool targetSpecificNode) {
  if (_canListenOnly) {
    Serial.println("[CANopen] Bring-up skipped — CAN is in listen-only mode");
    return;
  }
  Serial.println("[CANopen] Bring-up: sending baud-detect burst (0x080)...");
  uint8_t zeros[8] = {0};
  unsigned long deadline = millis() + 2000UL;
  while (millis() < deadline) {
    canSend(CANOPEN_ID_SYNC_BAUDDETECT, false, zeros, 8);
    delay(50);
  }

  // Target byte: 0x00 = "all nodes" (confirmed working on bench, the
  // safe default). Real CiA 301 also allows targeting a specific node
  // ID in this byte — exposed as an opt-in matching the ditchwitch-
  // logger Sensor Configuration page's same option, for consistency.
  uint8_t targetByte = targetSpecificNode ? (targetNodeId & 0x7F) : 0x00;
  uint8_t nmt[2] = { CANOPEN_NMT_CS_START, targetByte };
  Serial.printf("[CANopen] Sending NMT 'Start' (0x000, [0x01, 0x%02X])...\n", targetByte);
  canSend(CANOPEN_ID_NMT, false, nmt, 2);
}

void canStop() {
  if (_canStarted) {
    twai_stop();
    twai_driver_uninstall();
    _canStarted = false;
    Serial.println("[CAN] Stopped");
  }
}

bool canIsRunning() { return _canStarted; }

// Call this frequently (e.g. every loop() iteration or from a dedicated
// task) — drains any pending RX frames, logs them to the ring buffer, and
// runs them through every enabled CanSignalConfig for extraction.
// nonBlockingTimeoutMs=0 means "don't block if nothing's waiting".
void canPoll(ModuleConfig& cfg, CanSignalReading* canReadings, SemaphoreHandle_t mtx) {
  if (!_canStarted) return;

  twai_message_t msg;
  // Drain everything currently queued, but cap iterations per call so a
  // CAN flood can't starve the rest of loop() (WiFi/HTTP/web server).
  for (int i = 0; i < 64; i++) {
    if (twai_receive(&msg, 0) != ESP_OK) break; // nothing waiting, done for this call

    _canFrameTotal++;
    _canLastFrameMs = millis();

    // Log raw frame
    CanFrameLog& slot = _canLog[_canLogHead];
    slot.id = msg.identifier;
    slot.extended = msg.extd;
    slot.dlc = msg.data_length_code;
    memcpy(slot.data, msg.data, min((int)msg.data_length_code, 8));
    slot.ms = millis();
    _canLogHead = (_canLogHead + 1) % CAN_LOG_SIZE;
    if (_canLogCount < CAN_LOG_SIZE) _canLogCount++;

    // Signal extraction — check every enabled signal for a matching ID
    if (xSemaphoreTake(mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
      for (int s = 0; s < MAX_CAN_SIGNALS; s++) {
        CanSignalConfig& sig = cfg.canSignals[s];
        if (!sig.enabled) continue;
        if (sig.canId != msg.identifier || sig.extended != (bool)msg.extd) continue;
        if (sig.byteOffset + sig.byteLen > msg.data_length_code) continue; // not enough data in this frame

        uint32_t raw = 0;
        if (sig.bigEndian) {
          for (int b = 0; b < sig.byteLen; b++) raw = (raw << 8) | msg.data[sig.byteOffset + b];
        } else {
          for (int b = sig.byteLen - 1; b >= 0; b--) raw = (raw << 8) | msg.data[sig.byteOffset + b];
        }

        float decoded;
        if (sig.signedVal) {
          // Sign-extend based on byteLen
          int32_t sraw;
          if (sig.byteLen == 1) sraw = (int8_t)raw;
          else if (sig.byteLen == 2) sraw = (int16_t)raw;
          else sraw = (int32_t)raw;
          decoded = (float)sraw;
        } else {
          decoded = (float)raw;
        }

        canReadings[s].rawValue = decoded;
        canReadings[s].value = decoded * sig.scale + sig.offset;
        canReadings[s].hasValue = true;
        canReadings[s].status = "ok";
        canReadings[s].lastSeenMs = millis();
      }
      xSemaphoreGive(mtx);
    }
  }
}

// Copies the most recent N frames (up to CAN_LOG_SIZE) out for JSON
// serialization, newest first. Returns how many were actually copied.
int canGetRecentFrames(CanFrameLog* out, int maxCount) {
  int n = min(maxCount, _canLogCount);
  for (int i = 0; i < n; i++) {
    int idx = (_canLogHead - 1 - i + CAN_LOG_SIZE * 2) % CAN_LOG_SIZE;
    out[i] = _canLog[idx];
  }
  return n;
}

unsigned long canGetFrameTotal() { return _canFrameTotal; }
unsigned long canGetLastFrameMs() { return _canLastFrameMs; }

// Rough bus activity rate — frames seen in roughly the last second, based
// on ring buffer contents. Good enough for a "is anything talking" gauge
// on the diagnostics page; not a precise bus-load calculation.
int canGetRecentFrameRate() {
  if (_canLogCount == 0) return 0;
  unsigned long now = millis();
  int count = 0;
  for (int i = 0; i < _canLogCount; i++) {
    int idx = (_canLogHead - 1 - i + CAN_LOG_SIZE * 2) % CAN_LOG_SIZE;
    if (now - _canLog[idx].ms <= 1000) count++;
    else break; // log is newest-first-ish per insertion order; older entries won't be within 1s either
  }
  return count;
}

// =============================================================================
// CANopen Bridge self-heal (2026-09-08)
//
// Bring-up is a one-shot burst (~2s of TX), not a persistent state — if
// the encoder wasn't powered yet, or a bus glitch/reboot on the encoder
// side happens later, this board would otherwise stay silently waiting
// forever with no way to recover short of a manual reboot. Same
// philosophy as this firmware's other self-heals (WiFi NVS erase,
// baud-detect retry) — if nothing's been seen in a while, just try
// again; harmless if the encoder was fine all along (bring-up is a
// no-op from its perspective once already OPERATIONAL).
//
// Call this from loop() every cycle — internally rate-limited so it only
// actually re-sends every RETRY_INTERVAL_MS.
bool canopenBringupIfDue(uint8_t targetNodeId, bool targetSpecificNode) {
  static unsigned long lastAttempt = 0;
  const unsigned long RETRY_INTERVAL_MS = 30000UL; // don't hammer the bus faster than this
  unsigned long now = millis();

  if (lastAttempt == 0) {
    // First call ever (post-boot) — always try once, whether or not any
    // frame has been seen yet (there can't have been one before boot).
    lastAttempt = now;
    canopenBringup(targetNodeId, targetSpecificNode);
    return true;
  }
  if (now - lastAttempt < RETRY_INTERVAL_MS) return false;
  if (_canFrameTotal > 0 && (now - _canLastFrameMs) < RETRY_INTERVAL_MS) return false; // traffic is flowing, leave it alone

  lastAttempt = now;
  Serial.println("[CANopen] No CAN traffic seen recently — re-running bring-up (self-heal)");
  canopenBringup(targetNodeId, targetSpecificNode);
  return true;
}

// =============================================================================
// Serializes recent raw frames (from the ring buffer job #1 keeps
// anyway) as a JSON array for the Pi POST payload — see this file's
// header comment for why decode stays on the PC side (unconfirmed J1939
// byte layout) and this board only relays raw bytes. Only sends frames
// newer than `sinceMs` (this board's own millis(), not wall-clock) so
// the same frame isn't resent every poll cycle; the PC times frames by
// its own receipt time anyway (see local_server.py's
// _api_rig_module_post()), so this board's millis() is only used
// locally to dedupe, never transmitted.
//
// maxCount caps how many go in one POST body (bandwidth/heap), oldest-
// first so the PC sees them in the order they actually happened.
unsigned long canSerializeRecentFrames(JsonArray& arr, unsigned long sinceMs, int maxCount) {
  CanFrameLog buf[CAN_LOG_SIZE];
  int n = canGetRecentFrames(buf, CAN_LOG_SIZE); // newest-first
  unsigned long newestMs = sinceMs;
  int added = 0;
  // Walk newest-first but only keep frames after sinceMs, then emit in
  // reverse (oldest-first) so the array reads chronologically.
  int keep = 0;
  for (int i = 0; i < n; i++) {
    if ((long)(buf[i].ms - sinceMs) <= 0) break; // ring buffer is newest-first; once we hit
                                                   // one at/before sinceMs, everything after
                                                   // it is older still — safe to stop
    keep++;
  }
  if (keep > maxCount) keep = maxCount; // if more arrived than fit, keep the NEWEST `maxCount`
  for (int i = keep - 1; i >= 0; i--) {
    JsonObject f = arr.createNestedObject();
    f["id"] = buf[i].id;
    f["extended"] = buf[i].extended;
    f["dlc"] = buf[i].dlc;
    char hex[25]; // 8 bytes * 3 chars ("XX ") + null
    hex[0] = '\0';
    for (int b = 0; b < buf[i].dlc && b < 8; b++) {
      char byteStr[4];
      snprintf(byteStr, sizeof(byteStr), "%02X ", buf[i].data[b]);
      strcat(hex, byteStr);
    }
    f["data_hex"] = String(hex);
    f["ms"] = buf[i].ms;
    if (buf[i].ms > newestMs) newestMs = buf[i].ms;
    added++;
  }
  return newestMs;
}
