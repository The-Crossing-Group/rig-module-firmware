// =============================================================================
// can.h — CAN bus via ESP32-S3 native TWAI controller
//
// Two jobs:
//   1) Raw frame capture — a small ring buffer of the most recent frames
//      (ID, DLC, data bytes, timestamp) for the live diagnostics page.
//      This is the "look at the bus before you know what's on it" tool.
//   2) Signal extraction — CanSignalConfig entries pull a byte range out
//      of frames matching a given ID and decode it into a value, same
//      spirit as a Modbus sensor but sourced from CAN instead of RS485.
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
bool canStart(int txPin, int rxPin, long bitrate) {
  if (_canStarted) {
    twai_stop();
    twai_driver_uninstall();
    _canStarted = false;
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)txPin, (gpio_num_t)rxPin, TWAI_MODE_LISTEN_ONLY);
  // LISTEN_ONLY: we never transmit on this bus — purely a passive tap,
  // which is both simpler and safer (a mis-wired/mis-timed board can't
  // possibly disrupt drill CAN traffic if it physically cannot send).

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
  Serial.printf("[CAN] Started, TX=%d RX=%d bitrate=%ld (listen-only)\n", txPin, rxPin, bitrate);
  return true;
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
